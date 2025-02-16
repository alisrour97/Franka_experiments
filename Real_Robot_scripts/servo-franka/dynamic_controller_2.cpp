#include <iostream>
#include <thread>
#include <chrono>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpColVector.h>


#if defined(VISP_HAVE_FRANKA)

// Define functions here
////////////////////////////////////Not used //////////////////////////////////////////
vpColVector getFriction(const vpColVector &dq) {
  const double FI[] = {0.54615, 0.87224, 0.64068, 1.2794, 0.83904, 0.30301, 0.56489,
                       5.1181, 9.0657, 10.136, 5.5903, 8.3469, 17.133, 10.336,
                       0.039533, 0.025882, -0.04607, 0.036194, 0.026226, -0.021047, 0.0035526};

  const int njoints = 7;

  vpColVector tau_f(njoints, 0);

  for (int i = 0; i < njoints; ++i) {
    double exp_term = exp(-FI[i + njoints] * (dq[i] + FI[i + 2 * njoints]));
    double TAU_F = FI[i] / (1 + exp_term);
    double TAU_F_CONST = FI[i]/(1 + exp (-FI[i + njoints] * FI[i + 2 * njoints]));
    tau_f[i] = TAU_F - TAU_F_CONST;
  }

  return tau_f;
}


class Time
{
public:
  std::chrono::time_point<std::chrono::system_clock> start;
  std::string s;

  void tic() { start = std::chrono::high_resolution_clock::now(); }
  void toc()
  {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "block " + s + " : ";
    std::cout << "elapsed Time in milliseconds = " << duration.count() << "\n\n";
  }
  Time(std::string s_)
  {
    s = s_;
    tic();
  }
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


void task_gripper_open(franka::Gripper gripper, double opt_grasping_width, double opt_grasping_speed, Time timer)
{ 
  timer.tic();
  gripper.move(opt_grasping_width + 0.025, opt_grasping_speed);
  timer.toc();

}


vpMatrix
Ta( const vpHomogeneousMatrix &edMe ) // Lw = I - 0.5θS(u) + (1 - sinc(θ)/sinc²(θ/2))*S(u)*S(u)
{
  vpMatrix Lx( 6, 6 ), Lw( 3, 3 ), skew_u( 3, 3 );

  Lx.eye(); // initialize Lx to I
  vpThetaUVector tu( edMe ); // orientation axis angle

  vpColVector u;
  double theta;

  tu.extract( theta, u ); // Extract the rotation angle and the rotation axis u
  skew_u = vpColVector::skew( u );
  Lw.eye();
  if ( theta != 0.0 )
  {
    Lw -= 0.5 * theta * skew_u;
    Lw += ( 1 - ( ( vpMath::sinc( theta ) ) / ( vpMath::sqr( vpMath::sinc( theta * 0.5 ) ) ) ) ) * skew_u * skew_u;
  }

  // equal identity if there was no rotation ( theta = 0 )
  Lx.insert( Lw, 3, 3 ); 

  return Lx;
}


// Function to read trajectory from CSV file
std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>>
readTrajectory(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file) {
        std::cout << "Failed to open " << filePath << std::endl;
        return {};
    }

    // Read the header row
    std::string line;
    if (!std::getline(file, line)) {
        std::cout << "Failed to read the header row" << std::endl;
        return {};
    }

    // Split the header row into column names
    std::vector<std::string> columnNames;
    std::string columnName;
    std::istringstream iss(line);
    while (std::getline(iss, columnName, ',')) {
        columnNames.push_back(columnName);
    }

    // Determine the number of groups
    int numGroups = columnNames.size() / 6;

    // Create data structures to store the groups
    std::vector<std::vector<double>> posGroup, velGroup, accGroup;

    // Read and store the data for each group
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        for (int groupIndex = 0; groupIndex < numGroups; ++groupIndex) {
            std::vector<double> groupData;
            for (int columnIndex = groupIndex * 6; columnIndex < (groupIndex + 1) * 6; ++columnIndex) {
                std::string value;
                std::getline(iss, value, ',');
                groupData.push_back(std::stod(value));
            }
            if (groupIndex == 0) {
                posGroup.push_back(groupData);
            } else if (groupIndex == 1) {
                velGroup.push_back(groupData);
            } else if (groupIndex == 2) {
                accGroup.push_back(groupData);
            }
        }
    }

    return std::make_tuple(posGroup, velGroup, accGroup);
}




int main(int argc, char **argv)
{
  // By default
  std::string opt_robot_ip = "192.168.30.10";
  std::string opt_traj_name = "traj1";
  // std::string opt_traj_type = "NL";
  std::string opt_traj_num = "0";
  std::string opt_traj_k = "nki";
  std::string opt_traj_wait = "nw";



  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      opt_robot_ip = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--traj" && i + 1 < argc) {
      opt_traj_name = std::string(argv[i + 1]);
    } 
    else if (std::string(argv[i]) == "--gain" && i + 1 < argc) {
      opt_traj_k = std::string(argv[i + 1]);
    }
    else if(std::string(argv[i]) == "--num" && i + 1 < argc){
      opt_traj_num = std::string(argv[i + 1]);
    }    else if(std::string(argv[i]) == "--wait" && i + 1 < argc){
      opt_traj_wait = std::string(argv[i + 1]);
    }else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Move Panda robot to a position specified from a file." << std::endl;
      std::cout << argv[0] << " [--ip <default " << opt_robot_ip << ">] [--read <position file name>] [--help] [-h]\n"
                << std::endl;
      std::cout << "Example:\n" << argv[0] << " --ip 192.168.100.1 --read position.pos\n" << std::endl;

      return EXIT_SUCCESS;
    }
  }

  // CSV file for outputing data after executing a motion task [time, q, dq, torque, ex, ev]

  std::string filePath;

  if (opt_traj_k == "ki")
    {filePath = "../Data_video/log_dyn_" + opt_traj_name + "_" + opt_traj_k + "_" + opt_traj_num +  ".csv" , std::ofstream::out;}
  else{
    filePath = "../Data_video/log_dyn_" + opt_traj_name + "_" + opt_traj_num + ".csv" , std::ofstream::out;}

  std::ofstream ofs(filePath);

  // Header names
  std::vector<std::string> headerNames = {"Time", 
                                          "q1", "q2", "q3", "q4", "q5", "q6", "q7",
                                          "dq1", "dq2", "dq3", "dq4", "dq5", "dq6", "dq7",
                                          "t1", "t2", "t3", "t4", "t5", "t6", "t7",
                                          "ex1", "ex2", "ex3", "ex4", "ex5", "ex6",
                                          "ev1", "ev2", "ev3", "ev4", "ev5", "ev6",
                                          "xd", "yd", "zd","uxd", "uyd", "uzd",
                                           "x", "y", "z", "ux", "uy", "uz",
                                          "vxd", "vyd", "vzd", "vx", "vy", "vz"};
  // Data writing to a specific CSV file
  for (const auto& header : headerNames) {
      ofs << header;
      if (&header != &headerNames.back()) {
          ofs << ",";
      }
  }
  ofs << "\n";


  // Read Bezier curve trajectory from a CSV file to apply for the robot
  auto [posGroup, velGroup, accGroup] = readTrajectory("../trajectories/" + opt_traj_name + ".csv");


  vpRobotFranka robot;
  



  try 
  {
    robot.connect(opt_robot_ip);
    
    franka::Gripper gripper(opt_robot_ip);
    // double opt_grasping_width = 0.015;
    double opt_grasping_width = 0.04;
    double opt_grasping_speed = 2;
    double opt_grasping_force = 80;
    double opt_releasing_index = 500;
    gripper.grasp(0.075, opt_grasping_speed, 60);
    vpTime::sleepMs(1000);
    gripper.grasp(opt_grasping_width, opt_grasping_speed, 60);
    vpTime::sleepMs(2000);


    std::thread t1;

    Time timer ("elapsed time gripper");



    float Ts = 1; // 1ms sampling for the trajectory

    vpColVector x_init(6, 0);
    vpHomogeneousMatrix eeMdesired;
    //q_init_mod

    vpColVector q_init( {0.1075775274, -0.774082241, -0.9161245859, -1.456263293,  -0.6251063798,  1.070128464,  -0.2568718399} );

    // q_init 1
    // vpColVector q_init( {1.691173, -1.225535, -1.537874, -0.899814, -1.191407, 1.228254, -1.302819} );






    // q_init 2
    // vpColVector q_init( {1.691173, -1.225535, -1.537874, -0.899814, -1.191407, 1.228254, -1.302819} );

    // Get initial position of the robot from coppeliasim
    // robot.getPosition(vpRobot::END_EFFECTOR_FRAME, x_init);
    // robot.getPosition(vpRobot::JOINT_STATE, q_init);
    // std::cout<<x_init.t()<<std::endl;
    // std::cout<<q_init.t()<<std::endl;
    // vpTime::wait(10000);


    robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
    robot.setPosition( vpRobot::JOINT_STATE, q_init );


    // Get initial position of the robot from coppeliasim
    robot.getPosition(vpRobot::END_EFFECTOR_FRAME, x_init);
    robot.getPosition(vpRobot::JOINT_STATE, q_init);




    vpColVector x(6, 0), xdot(6, 0), q( 7, 0 ), dq( 7, 0 ), tau_d( 7, 0 ), G(7, 0), C( 7, 0 ), F( 7, 0 ), tau_d0( 7, 0 ), tau_cmd( 7, 0 ),
        x_e( 6, 0 ), x_e_i(6, 0), dx_e( 6, 0 ), dx_ed( 6, 0 ), ddx_ed( 6, 0 );
    vpMatrix fJe( 6, 7 ), Ja( 6, 7 ), dJa( 6, 7 ), Ja_old( 6, 7 ), B( 7, 7 ), I7( 7, 7 ), Ja_pinv_B_t( 6, 7 );
    vpColVector pose_err_norm( 2, 0 ), tau( 7, 0 );

    vpMatrix JJT (7, 7);

    vpRotationMatrix R_ref, Ro;
    vpPoseVector pos_ref, pos_cur;
    vpQuaternionVector q_cur, q_ref;
    vpColVector qd_sub(3, 0), Q_sub(3, 0);
    double a;
    vpColVector b(3, 0);
    vpColVector q_err_eval(4, 0);

    vpColVector q_0_dot (7, 0);
    vpColVector e_x(6, 0), xd(6, 0), xd_dot(6, 0), qdot(7, 0);
    vpPoseVector x_ed;


    vpHomogeneousMatrix fMed, fMed0, fMe_cur;
    robot.getPosition( vpRobot::JOINT_STATE, q );
    fMed0 = robot.get_fMe(q);
    fMed  = fMed0;
    // std::cout<<fMed0<<std::endl;

    robot.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL );
    vpColVector torque_max( {87, 87, 87, 87, 12, 12, 12} );

    vpMatrix K( 6, 6 ), D( 6, 6 ), edVf( 6, 6 ), K_v (6, 6), KI(6, 6);


    double wp = 20;
    double wo = 23;
    // double wp = 25;
    // double wo = 20;
    K.diag( { 400, 400, 400, 400, 400, 400} );
    D.diag( { 40, 40, 40, 40, 40,  40} );
    KI.diag( { 50, 50, 50, 50, 50,  50} );

    K = 1.2*K;
    D = 1.5*D;

    if (opt_traj_k == "ki")
      KI = KI * 1;
    else
      KI = KI * 0;
    
    std::cout<<"This is KI"<<std::endl<<KI<<std::endl;



    // K.diag( { 160, 160, 160 , 500 , 500, 500 } );
    // D.diag( { 16, 16, 16, 25, 25, 25} );

    // K = 2.2*K;
    // D = 2*D;



    I7.eye();
    double mu = 4;
    double dt = 0;

    double time_start_trajectory, time_prev, time_cur, time_stamp;

    int i = 0;
    double param = posGroup.size();
    
    time_prev = vpTime::measureTimeMs();
    time_start_trajectory = time_prev;
    const double conv = 0.001;

    // For csv data handling [time, q, dq, torque, ex, ev]
    const unsigned int n_q = q.size();
    const unsigned int n_dq = dq.size();
    const unsigned int n_t = tau.size();
    const unsigned int n_ex = x_e.size();
    const unsigned int n_ev = dx_e.size();
    const unsigned int n_pos_d = 6;
    const unsigned int n_pos_cur = 6;
    const unsigned int n_vel_d = 3;
    const unsigned int n_vel_cur = 3;

    vpColVector data_log( n_q + n_dq + n_t + n_ex + n_ev + n_pos_d + n_pos_cur + n_vel_d + n_vel_cur + 1);
    bool first_time = true;



    while (i < param)

    { 
      time_stamp = vpTime::measureTimeMs();
      time_cur = time_stamp - time_start_trajectory;
      

      robot.getPosition( vpRobot::END_EFFECTOR_FRAME, x );
      robot.getPosition( vpRobot::JOINT_STATE, q );
      robot.getVelocity( vpRobot::JOINT_STATE, dq );
      robot.getMass( B );
      robot.getCoriolis( C );
      robot.get_fJe( fJe );
      robot.getForceTorque( vpRobot::JOINT_STATE, tau );

      F = getFriction(dq);
      
  

      xdot = fJe * dq;

      // collect the reference and current poses
      pos_ref = {posGroup[i][0], posGroup[i][1], posGroup[i][2], posGroup[i][3], posGroup[i][4], posGroup[i][5]};
      pos_cur = {x[0], x[1], x[2], x[3], x[4], x[5]};
      // collect desired velocity, acc of the end-effector
      dx_ed = {velGroup[i][0], velGroup[i][1], velGroup[i][2], velGroup[i][3], velGroup[i][4], velGroup[i][5]};
      ddx_ed ={accGroup[i][0], accGroup[i][1], accGroup[i][2], accGroup[i][3], accGroup[i][4], accGroup[i][5]};

      // calculation of the ref, current  Rotational matrices
      R_ref.buildFrom(pos_ref);
      Ro.buildFrom(pos_cur);
      // collect the Homogeneous transformation Reference, velocity, acceleration references
      fMed.insert(R_ref);
      fMed[0][3] = posGroup[i][0];
      fMed[1][3] = posGroup[i][1];
      fMed[2][3] = posGroup[i][2];

      fMe_cur.buildFrom(pos_cur);


      // //build quaternion from rotational matrix
      // q_ref.buildFrom(R_ref);
      // q_cur.buildFrom(Ro);

      // // Error vector calculations based on the quaternion method
      // qd_sub = {q_ref[1], q_ref[2], q_ref[3]};
      // Q_sub = {q_cur[1], q_cur[2], q_cur[3]};
      // a = q_ref[0] * q_cur[0] + qd_sub.t() * Q_sub;
      // b = q_cur[0] * qd_sub - q_ref[0] * Q_sub - vpColVector::cross(qd_sub, Q_sub);
      // q_err_eval[0] = a;
      // q_err_eval.insert(1, b);

      // x_e = {posGroup[i][0] - x[0], posGroup[i][1] - x[1], posGroup[i][2] - x[2], b[0], b[1], b[2]};


      edVf.insert( fMed.getRotationMatrix().t(), 0, 0 );
      edVf.insert( fMed.getRotationMatrix().t(), 3, 3 );


      x_e = (vpColVector)vpPoseVector( fMed.inverse() * robot.get_fMe(q) ); // edMe // pose error [ t, theta * u ] using homogeneous transformations


      // std::cout<<"This is ex1:  "<<x_e.t()<<std::endl;
      Ja  = Ta( fMed.inverse() * robot.get_fMe(q) ) * edVf * fJe; //equation 2.12 -- Analytical jacobian
      dx_e = Ta( fMed.inverse() * robot.get_fMe(q) ) * edVf * ( dx_ed - xdot );


      dt = time_cur - time_prev;
      // if - else logic for calculating analytical jacobian derivation, first time dja = 0
      if ( i != 0 )
      {
        dJa = ( Ja - Ja_old ) / (conv * dt);
        x_e_i = (conv*dt) * dx_e + x_e;

        
      
      }
      else 
      { 
        dJa = 0;
      }

      Ja_old = Ja;
      Ja_pinv_B_t = ( Ja * B.inverseByCholesky() * Ja.t() ).inverseByCholesky() * Ja * B.inverseByCholesky();

      // JaT = Ja.transpose();
      // JaPinv = (lambda * I + Ja *JaT).inverse() * JaT;

      // tau_d = B * Ja.pseudoInverse() * ( ddx_ed  - K * ( x_e ) + D * (dx_e) - dJa * dq ) + C + F - ( I7 - Ja.t() * Ja_pinv_B_t ) * B * dq * 100;
      // JJT = Ja * Ja.transpose();
      // std::cout<<" counter i:"<< i<<std::endl;
      // std::cout<< "det:"<< JJT.det()<<std::endl;

      // tau_d = B * Ja.pseudoInverse() * ( ddx_ed  - K * ( x_e ) + D * (dx_e) - dJa * dq ) + C + F - ( I7 - Ja.t() * Ja_pinv_B_t ) * B * dq * 100;
      tau_d = B * Ja.pseudoInverse() * ( ddx_ed   - K * ( x_e ) - KI * (x_e_i) + D * (dx_e) - dJa * dq ) + C + F - ( I7 - Ja.t() * Ja_pinv_B_t ) * B * dq * 100;
      // tau_d = B * Ja.pseudoInverse() * ( - dJa * dq ) + C + F ;


      if (first_time == true)
      {
        tau_d0 = tau_d;
        first_time = false;
      }

      tau_cmd = tau_d - tau_d0 *  std::exp( -mu * ( time_cur * conv ) ) ;

      

      // tau_d = robot.saturateVelocities(tau_d, torque_max, true);
      // std::cout<<tau_d.t()<<std::endl;

      robot.setForceTorque( vpRobot::JOINT_STATE, tau_d);

      i++;

      // if ( i == 1800)
      // {
      //   // gripper.move(opt_grasping_width + 0.01, opt_grasping_speed);
      //   // // gripper.stop();
      //   // robot.gripperRelease();
      //   t1 = std::thread (task_gripper_open, std::move(gripper), std::move(opt_grasping_width), std::move(opt_grasping_speed), std::move(timer));
      // }
      

      time_prev = time_cur;
      // std::cout<<"Time current::: "<<(vpTime::measureTimeMs()-time_start_trajectory)/1000<<std::endl;
      // std::cout<<x_e.t()<<std::endl;

      //Data writing to specific csv file
      data_log[0] = time_cur/1000; // convert to seconds
      for (unsigned int d=0; d <n_q; d++)
        data_log[d+1] = q[d];
      for (unsigned int d=0; d <n_dq; d++)
        data_log[n_q + d + 1] = dq[d];
      for (unsigned int d=0; d <n_t; d++)
        data_log[n_q + n_dq + d + 1] = tau[d];
      for (unsigned int d=0; d <n_ex; d++)
        data_log[n_q + n_dq + n_t + d + 1] = x_e[d];
      for (unsigned int d=0; d <n_ev; d++)
        data_log[n_q + n_dq + n_t + n_ex + d + 1] = dx_e[d];
      for (unsigned int d=0; d <n_pos_d; d++)
        data_log[n_q + n_dq + n_t + n_ex + n_ev + d + 1] = pos_ref[d];
      for (unsigned int d=0; d <n_pos_cur; d++)
        data_log[n_q + n_dq + n_t + n_ex + n_ev + n_pos_d + d + 1] = pos_cur[d];
      for (unsigned int d=0; d <n_vel_d; d++)
        data_log[n_q + n_dq + n_t + n_ex + n_ev + n_pos_d +n_pos_cur+ d + 1] = dx_ed[d];
      for (unsigned int d=0; d <n_vel_cur; d++)
        data_log[n_q + n_dq + n_t + n_ex + n_ev + n_pos_d +n_pos_cur + n_vel_d + d + 1] = xdot[d];

      // write all the data
      data_log.t().csvPrint(ofs);

      vpTime::wait(time_stamp, Ts);


    }

    // Close the file to save your data
    ofs.close();
    // stop the robot
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState( vpRobot::STATE_STOP );
    // robot.gripperOpen();

    // gripper.stop();




  } catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
    return EXIT_FAILURE;
  } catch (const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. "
              << std::endl;
    return EXIT_FAILURE;
  } catch (const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_FRANKA)
  std::cout << "Install libfranka." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
