#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpColVector.h>




#if defined(VISP_HAVE_FRANKA)


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


double euclideanNorm(const vpColVector& v) {
    double norm = 0.0;
    for (int i = 0; i < v.size(); ++i) {
        norm += v[i] * v[i];
    }
    return std::sqrt(norm);
}


//Function to convert angle axis to Rotation Matrix

vpMatrix angleAxisToRotationMatrix(const vpColVector& angleAxis) {
    // Ensure angleAxis is a column vector
    vpColVector axis = angleAxis / euclideanNorm(angleAxis);
    double angle = euclideanNorm(angleAxis);

    double s = std::sin(angle);
    double c = std::cos(angle);
    double t = 1.0 - c;

    vpMatrix R(3, 3);
    R[0][0] = t * axis[0] * axis[0] + c;
    R[1][1] = t * axis[1] * axis[1] + c;
    R[2][2] = t * axis[2] * axis[2] + c;
    R[0][1] = t * axis[0] * axis[1] - s * axis[2];
    R[0][2] = t * axis[0] * axis[2] + s * axis[1];
    R[1][0] = t * axis[0] * axis[1] + s * axis[2];
    R[1][2] = t * axis[1] * axis[2] - s * axis[0];
    R[2][0] = t * axis[0] * axis[2] - s * axis[1];
    R[2][1] = t * axis[1] * axis[2] + s * axis[0];

    return R;
}

vpColVector rotm2quat(const vpMatrix& R) {
    double trace_R = R[0][0] + R[1][1] + R[2][2];
    double eta = 0.5 * std::sqrt(trace_R + 1.0);

    vpColVector q(4, 0);
    q[0] = eta;
    q[1] = (R[2][1] - R[1][2]) / (4.0 * eta);
    q[2] = (R[0][2] - R[2][0]) / (4.0 * eta);
    q[3] = (R[1][0] - R[0][1]) / (4.0 * eta);

    return q;
}


int main(int argc, char **argv)
{
  std::string opt_robot_ip = "192.168.30.10";
  std::string opt_position_filename = "";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      opt_robot_ip = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--read" && i + 1 < argc) {
      opt_position_filename = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Move Panda robot to a position specified from a file." << std::endl;
      std::cout << argv[0] << " [--ip <default " << opt_robot_ip << ">] [--read <position file name>] [--help] [-h]\n"
                << std::endl;
      std::cout << "Example:\n" << argv[0] << " --ip 192.168.100.1 --read position.pos\n" << std::endl;

      return EXIT_SUCCESS;
    }
  }

    // CSV file for outputing data after executing a motion task [time, q, dq, torque, ex, ev]
  std::ofstream ofs("../Data/log_kin.csv", std::ofstream::out);
  // Header names
  std::vector<std::string> headerNames = {"Time", 
                                          "q1", "q2", "q3", "q4", "q5", "q6", "q7",
                                          "dq1", "dq2", "dq3", "dq4", "dq5", "dq6", "dq7",
                                          "t1", "t2", "t3", "t4", "t5", "t6", "t7",
                                          "ex1", "ex2", "ex3", "ex4", "ex5", "ex6",
                                          "ev1", "ev2", "ev3", "ev4", "ev5", "ev6"};
  // Data writing to a specific CSV file
  for (const auto& header : headerNames) {
      ofs << header;
      if (&header != &headerNames.back()) {
          ofs << ",";
      }
  }
  ofs << "\n";

  // Read Bezier curve trajectory from a CSV file to apply for the robot
  auto [posGroup, velGroup, accGroup] = readTrajectory("../trajectories/traj1.csv");


  vpRobotFranka robot;

  try 
  {
    robot.connect(opt_robot_ip);
    float Ts = 1; // 5ms sampling for the trajectory


    vpColVector x_init(6, 0);
    vpHomogeneousMatrix eeMdesired;

    // vpColVector q_init( {-0.4159054989, -1.590290421, -1.381977526, -2.000352023, -1.520563276, 1.323665423, -0.7684260877 } );
    vpColVector q_init( { 0, vpMath::rad( -45 ), 0, vpMath::rad( -135 ), 0, vpMath::rad( 90 ), vpMath::rad( 45 ) } );

    robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
    robot.setPosition( vpRobot::JOINT_STATE, q_init );

    // Get initial position of the robot from coppeliasim
    robot.getPosition(vpRobot::END_EFFECTOR_FRAME, x_init);
    robot.getPosition(vpRobot::JOINT_STATE, q_init);
    std::cout<< " This is x_init: "<< std::endl << x_init.t()<<std::endl;



    vpColVector x(6, 0), xdot(6, 0), q( 7, 0 ), dq( 7, 0 ), tau_d( 7, 0 ), C( 7, 0 ), F( 7, 0 ), tau_d0( 7, 0 ), tau_cmd( 7, 0 ),
        x_e( 6, 0 ), v_e( 6, 0 ), a_e( 6, 0 ), dx_e( 6, 0 ), dx_ed( 6, 0 ), ddx_ed( 6, 0 ), ddq_cmd(7, 0), dq_cmd (7, 0), dq_cmd_old(7, 0);
    vpMatrix fJe( 6, 7 ), Ja( 6, 7 ), dJa( 6, 7 ), Ja_old( 6, 7 ), B( 7, 7 ), I7( 7, 7 ), Ja_pinv_B_t( 6, 7 );
    vpColVector pose_err_norm( 2, 0 ), tau( 7, 0 );


    vpRotationMatrix R_ref, Ro;
    vpPoseVector pos_ref, pos_cur;
    vpQuaternionVector q_cur, q_ref;
    vpColVector qd_sub(3, 0), Q_sub(3, 0);
    double a;
    vpColVector b(3, 0);
    vpColVector q_err_eval(4, 0);

    vpColVector e_x(6, 0), xd(6, 0), xd_dot(6, 0), qdot(7, 0);

    vpMatrix K( 6, 6 ), D(6, 6), edVf( 6, 6 );

    vpHomogeneousMatrix fMed;
  
    robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );



    double time_start_trajectory, time_prev, time_cur, time_stamp1, time_stamp2, time_stamp3, time_stamp4;



    int i = 0;
    double dt;
    double param = posGroup.size();
    
    time_prev = vpTime::measureTimeMs();
    time_start_trajectory = time_prev;


    // min and max of joint velocities dq
    double dq_max[7] = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
    double dq_min[7] = {-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100};

    vpColVector q_min ({-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973});
    vpColVector q_max ({2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973});
    vpColVector q_median = (q_max + q_min)/2 ;
    vpColVector q_0_dot (7, 0);


    const double conv = 0.001;


    // K.diag( { 0.5, 0.5, 0.6 , 0.25 , 0.5 , 0.5 } );
    // D.diag( { 5, 15, 7, 5, 5, 5 } );


    double wp = 10;
    double wo = 5;
    K.diag( { wp * wp, wp * wp, wp * wp, wo * wo, wo * wo, wo * wo } );
    D.diag( { 2 * wp, 2 * wp, 2 * wp, 2 * wo, 2 * wo, 2 * wo } );



    I7.eye();
    // For csv data handling [time, q, dq, torque, ex, ev]
    const unsigned int n_q = q.size();
    const unsigned int n_dq = dq.size();
    const unsigned int n_t = tau.size();
    const unsigned int n_ex = x_e.size();
    const unsigned int n_ev = dx_e.size();

    vpColVector data_log( n_q + n_dq + n_t + n_ex + n_ev + 1);


    while (i < param)

    { 

      time_stamp1 = vpTime::measureTimeMs();
      time_cur = time_stamp1 - time_start_trajectory;
            
      robot.getPosition( vpRobot::JOINT_STATE, q);
      robot.getPosition( vpRobot::END_EFFECTOR_FRAME, x );
      robot.getVelocity( vpRobot::JOINT_STATE, dq );
      robot.get_fJe( fJe );

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
      //Homogeneous transformation desired
      fMed.insert(R_ref);
      fMed[0][3] = posGroup[i][0];
      fMed[1][3] = posGroup[i][1];
      fMed[2][3] = posGroup[i][2];


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
      // //Calculation of the error in position, analytical jacobian, error in velocity
      // x_e = {posGroup[i][0] - x[0], posGroup[i][1] - x[1], posGroup[i][2] - x[2], b[0], b[1], b[2]};


      x_e = (vpColVector)vpPoseVector( fMed.inverse() * robot.get_fMe(q) ); // edMe // pose error [ t, theta * u ] using homogeneous transformations


      edVf.insert( fMed.getRotationMatrix().t(), 0, 0 );
      edVf.insert( fMed.getRotationMatrix().t(), 3, 3 );

      Ja  =  Ta( fMed.inverse() * robot.get_fMe(q) ) * edVf * fJe; //equation 2.12 -- Analytical jacobian
      dx_e = Ta( fMed.inverse() * robot.get_fMe(q) ) * edVf * ( dx_ed - xdot ) ;



      // time step
      dt = time_cur - time_prev;

      //if - else logic for calculating analytical jacobian derivation, first time dja = 0
      if ( i != 0 )
      { 
        dJa = ( Ja - Ja_old ) / (conv * dt);
        // ddq_cmd =  Ja.pseudoInverse() * (ddx_ed  + K * ( x_e ) + D * (dx_e) - dJa * dq );
        ddq_cmd =  Ja.pseudoInverse() * ( ddx_ed - K * ( x_e ) + D * (dx_e) - dJa * dq);
        // dJa.pseudoInverse().print(std::cout, 10, "dja: ");
        dq_cmd = dq_cmd_old + conv * dt * ddq_cmd;
        
      }
      else 
      { 
        dJa = 0;
        dq_cmd_old = 0;
        dq_cmd = 0;
      }



      Ja_old = Ja;
      dq_cmd_old = dq_cmd;


      // std::cout<<"e_x  "<<x_e.t()<<std::endl;
      // apply control law

      // qdot =  fJe.transpose()*( dx_ed  + K * x_e);
      // std::cout<<"dq2  "<<qdot.t()<<std::endl;
      robot.setVelocity(vpRobot::JOINT_STATE, dq_cmd);

      


      // std::cout<<"time:   "<< time_cur <<"  error:    "<< x_e.t() <<std::endl;
      time_prev = time_cur;
      i++;
      vpTime::wait(time_stamp1, Ts);

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

      // write all the data
      data_log.t().csvPrint(ofs);

    }

    // Close the file to save your data
    ofs.close();

    // stop the robot
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState( vpRobot::STATE_STOP );



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
