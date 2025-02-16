#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpColVector.h>


#if defined(VISP_HAVE_FRANKA)

// Define functions here

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
  std::ofstream ofs("../Data/log.csv", std::ofstream::out);
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
    float Ts = 1; // 3ms sampling for the trajectory

    std::cout<<"Current time before starting the trajectory: "<<std::endl<<vpTime::measureTimeMs()<<std::endl;

    vpColVector x_init(6, 0);
    vpHomogeneousMatrix eeMdesired;

    // vpColVector q_init( {-0.4159054989, -1.590290421, -1.381977526, -2.000352023, -1.520563276, 1.323665423, -0.7684260877 } );
    vpColVector q_init( { 0, vpMath::rad( -45 ), 0, vpMath::rad( -135 ), 0, vpMath::rad( 90 ), vpMath::rad( 45 ) } );

    robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
    robot.setPosition( vpRobot::JOINT_STATE, q_init );

    std::cout<<"This is q_init :"<< q_init.t()<<std::endl;

    // Get initial position of the robot from coppeliasim
    robot.getPosition(vpRobot::END_EFFECTOR_FRAME, x_init);
    robot.getPosition(vpRobot::JOINT_STATE, q_init);
    std::cout<< " This is x_init: "<< std::endl << x_init.t()<<std::endl;

    // Samuel added this for gui visualization
    vpImage<unsigned char> IIIII(640, 640, 0);
    vpDisplayX display;
    display.init(IIIII);
    vpDisplay::display(IIIII);
    vpDisplay::flush(IIIII);


    vpPlot *plotter = nullptr;

    plotter = new vpPlot( 4, 600, 600, 10, 10, "Real time curves plotter" );
    plotter->setTitle( 0, "Joint positions [rad]" );
    plotter->initGraph( 0, 7 );
    plotter->setLegend( 0, 0, "q1" );
    plotter->setLegend( 0, 1, "q2" );
    plotter->setLegend( 0, 2, "q3" );
    plotter->setLegend( 0, 3, "q4" );
    plotter->setLegend( 0, 4, "q5" );
    plotter->setLegend( 0, 5, "q6" );
    plotter->setLegend( 0, 6, "q7" );

    plotter->setTitle( 1, "Joint torques measure [Nm]" );
    plotter->initGraph( 1, 7 );
    plotter->setLegend( 1, 0, "Tau1" );
    plotter->setLegend( 1, 1, "Tau2" );
    plotter->setLegend( 1, 2, "Tau3" );
    plotter->setLegend( 1, 3, "Tau4" );
    plotter->setLegend( 1, 4, "Tau5" );
    plotter->setLegend( 1, 5, "Tau6" );
    plotter->setLegend( 1, 6, "Tau7" );

    plotter->setTitle( 2, "Cartesian EE pose error [m] - [rad]" );
    plotter->initGraph( 2, 6 );
    plotter->setLegend( 2, 0, "e_x" );
    plotter->setLegend( 2, 1, "e_y" );
    plotter->setLegend( 2, 2, "e_z" );
    plotter->setLegend( 2, 3, "e_tu_x" );
    plotter->setLegend( 2, 4, "e_tu_y" );
    plotter->setLegend( 2, 5, "e_tu_z" );

    plotter->setTitle( 3, "Pose error norm [m] - [rad]" );
    plotter->initGraph( 3, 2 );
    plotter->setLegend( 3, 0, "||e_p||" );
    plotter->setLegend( 3, 1, "||e_o||" );

    vpColVector x(6, 0), xdot(6, 0), q( 7, 0 ), dq( 7, 0 ), tau_d( 7, 0 ), G(7, 0), C( 7, 0 ), F( 7, 0 ), tau_d0( 7, 0 ), tau_cmd( 7, 0 ),
        x_e( 6, 0 ), dx_e( 6, 0 ), dx_ed( 6, 0 ), ddx_ed( 6, 0 );
    vpMatrix fJe( 6, 7 ), Ja( 6, 7 ), dJa( 6, 7 ), Ja_old( 6, 7 ), B( 7, 7 ), I7( 7, 7 ), Ja_pinv_B_t( 6, 7 );
    vpColVector pose_err_norm( 2, 0 ), tau( 7, 0 );



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
    std::cout<<"This is the q: "<<std::endl<<q.t()<<std::endl;
    fMed0 = robot.get_fMe(q);
    fMed  = fMed0;
    // std::cout<<fMed0<<std::endl;

    robot.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL );

    vpMatrix K( 6, 6 ), D( 6, 6 ), edVf( 6, 6 ), K_v (6, 6);

    // double wp = 15;
    // double wo = 15;
    // double wd = 30;

    // K.diag( { wp * wp, wp * wp, wp * wp , wo * wo , wo * wo , wo * wo } );
    // D.diag( { wd, wd, wd, wd, wd, wd } );

    // double wp = 12;
    // double wo = 8;
    // double wd = 70;

    // K.diag( { wp * wp, wp * wp, wp * wp , wo * wo , wo * wo , wo * wo } );
    // D.diag( { wd, wd, wd, wd, wd, wd } );

    // double wp = 12;
    // double wo = 8;
    // double wd = 20;

    // K.diag( { 450, 450, 400 , 600 , 600, 950 } );
    // D.diag( { 15, 45, 45, 45, 45, 60} );

    K.diag( { 160, 160, 160 , 500 , 500, 500 } );
    D.diag( { 16, 16, 16, 25, 25, 25} );

    //quaternion
    // K = 1*K;
    // D = 1*D;


    // K.diag( { 4, 4, 4 , 4 , 4 , 4 } );
    // D.diag( { 20, 20, 20, 20, 20, 20 } );

    K = 2.2*K;
    D = 2*D;




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

    vpColVector data_log( n_q + n_dq + n_t + n_ex + n_ev + 1);


    while (i < param)

    { 
      time_stamp = vpTime::measureTimeMs();
      time_cur = time_stamp - time_start_trajectory;
      

      robot.getPosition( vpRobot::END_EFFECTOR_FRAME, x );
      // robot.getVelocity( vpRobot::END_EFFECTOR_FRAME, xdot );
      robot.getPosition( vpRobot::JOINT_STATE, q );
      robot.getVelocity( vpRobot::JOINT_STATE, dq );
      robot.getMass( B );
      robot.getGravity(G);
      robot.getCoriolis( C );
      robot.get_fJe( fJe );
      robot.getForceTorque( vpRobot::JOINT_STATE, tau );

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
      // std::cout<<q_err_eval.t()<<std::endl;

      // x_e = {posGroup[i][0] - x[0], posGroup[i][1] - x[1], posGroup[i][2] - x[2], b[0], b[1], b[2]};

      // std::cout<<"This is ex1:  "<<x_e.t()<<std::endl;


      edVf.insert( fMed.getRotationMatrix().t(), 0, 0 );
      edVf.insert( fMed.getRotationMatrix().t(), 3, 3 );


      x_e = (vpColVector)vpPoseVector( fMed.inverse() * robot.get_fMe(q) ); // edMe // pose error [ t, theta * u ] using homogeneous transformations


      // std::cout<<"This is ex1:  "<<x_e.t()<<std::endl;
      Ja  = Ta( fMed.inverse() * robot.get_fMe(q) ) * edVf * fJe; //equation 2.12 -- Analytical jacobian
      dx_e = Ta( fMed.inverse() * robot.get_fMe(q) ) * edVf * ( dx_ed - xdot );


      dt = time_cur - time_prev;
      // if - else logic for calculating analytical jacobian derivation, first time dja = 0
      if ( i != 0 ){dJa = ( Ja - Ja_old ) / (conv * dt);}
      else {dJa = 0;}


      Ja_old = Ja;
      Ja_pinv_B_t = ( Ja * B.inverseByCholesky() * Ja.t() ).inverseByCholesky() * Ja * B.inverseByCholesky();

      // Compute the control law
      // tau_d = B * Ja.pseudoInverse() * ( ddx_ed  - K * ( x_e ) + D * (dx_e) - dJa * dq  ) + C -
      //         ( I7 - Ja.t() * Ja_pinv_B_t ) * B * dq * 100; // equation 2.57 + 2.6 + 2.59

      tau_d = B * Ja.pseudoInverse() * ( ddx_ed  - K * ( x_e ) + D * (dx_e) - dJa * dq  ) + C;
    


      
      tau_d0 = tau_d;
      // tau_cmd = tau_d;
      tau_cmd = tau_d - tau_d0 * std::exp( -mu * ( time_cur ) );


      robot.setForceTorque( vpRobot::JOINT_STATE, tau_cmd );

      plotter->plot( 0, time_cur, q );
      plotter->plot( 1, time_cur, tau_cmd );
      plotter->plot( 2, time_cur, x_e );
      pose_err_norm[0] = sqrt( x_e.extract( 0, 3 ).sumSquare() );
      pose_err_norm[1] = sqrt( x_e.extract( 3, 3 ).sumSquare() );
      plotter->plot( 3, time_cur, pose_err_norm );
              
      i++;

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

      // write all the data
      data_log.t().csvPrint(ofs);

      // std::cout<<"e_x  "<<x_e.t()<<std::endl;

      vpTime::wait(time_stamp, Ts);

    }

    // Close the file to save your data
    ofs.close();
    // stop the robot
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState( vpRobot::STATE_STOP );



    // if ( plotter != nullptr )
    // {
    //   delete plotter;
    //   plotter = nullptr;
    // }

    // plotter->saveData( 0, "../Data/joint-position.txt", "% " );
    // plotter->saveData( 1, "../Data/torques.txt", "% " );
    // plotter->saveData( 2, "../Data/cartesian_EE_error.txt", "%" );
    // plotter->saveData( 3, "../Data/pose_error_norm.txt", " % " );


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
