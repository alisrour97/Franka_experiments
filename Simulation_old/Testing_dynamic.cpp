/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2021 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *****************************************************************************/

//! \example tutorial-franka-coppeliasim-cartesian-impedance-control.cpp

#include <iostream>

#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>
//#include <matplot/matplot.h>

#include <visp3/gui/vpPlot.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

// Trajectory Functions bezier curves
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;



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




MatrixXf blkdiag(const MatrixXf& a, int count)
{
  MatrixXf bdm = MatrixXf::Zero(a.rows() * count, a.cols() * count);
  for (int i = 0; i < count; ++i)
  {
      bdm.block(i * a.rows(), i * a.cols(), a.rows(), a.cols()) = a;
  }

  return bdm;
} 



float fact(int n) {
   if (n == 0 || n == 1)
   return 1;
   else
   return n * fact(n - 1);
}


// Combination function
int comb(int n, int k) {

    if (n < k) return 0;
    if (k == 0) return 1;


    int result = n;
    for (int i = 2; i <= k; ++i) {
        result *= (n - i +1);
        result /= i;
    }
    return result;
}



// //--------------------------5 order--------------------------------------------
// // AX = B
// // A = inv(X)*B - here we compute matrix A
MatrixXf computeQuinticCoeff(float t0, float tf, int N, std::vector<float> vec_x0, std::vector<float> vec_xf)
{
    MatrixXf M(6, 6);
    MatrixXf M_big(18, 18);
    MatrixXf B(18, 1);

    // Define delta T
    float DT = tf-t0;
    // Define Explicitely the Matrix M
    M(0, 0) = 1;
    M(0, 1) = 0;
    M(0, 2) = 0;
    M(0, 3) = 0;
    M(0, 4) = 0;
    M(0, 5) = 0;

    M(1, 0) = 0;
    M(1, 1) = 0;
    M(1, 2) = 0;
    M(1, 3) = 0;
    M(1, 4) = 0;
    M(1, 5) = 1;

    M(2, 0) = -N / DT;
    M(2, 1) =  N / DT;
    M(2, 2) = 0;
    M(2, 3) = 0;
    M(2, 4) = 0;
    M(2, 5) = 0;


    M(3, 0) = 0;
    M(3, 1) = 0;
    M(3, 2) = 0;
    M(3, 3) = 0;
    M(3, 4) = -N / DT;
    M(3, 5) =  N / DT;

    M(4, 0) = N*(N-1)/(std::pow(DT, 2));
    M(4, 1) = -2*N*(N-1)/(std::pow(DT, 2));
    M(4, 2) = N*(N-1)/(std::pow(DT, 2));
    M(4, 3) = 0;
    M(4, 4) = 0;
    M(4, 5) = 0;

    M(5, 0) = 0;
    M(5, 1) = 0;
    M(5, 2) = 0;
    M(5, 3) = N*(N-1)/(std::pow(DT, 2));
    M(5, 4) = -2*N*(N-1)/(std::pow(DT, 2));
    M(5, 5) = N*(N-1)/(std::pow(DT, 2));

    //Define the initial and final positions of the waypoint B

    B(0, 0) = vec_x0[0];
    B(1, 0) = vec_xf[0];
    B(2, 0) = vec_x0[3];
    B(3, 0) = vec_xf[3];
    B(4, 0) = vec_x0[6];
    B(5, 0) = vec_xf[6];
    B(6, 0) = vec_x0[1];
    B(7, 0) = vec_xf[1];
    B(8, 0) = vec_x0[4];
    B(9, 0) = vec_xf[4];
    B(10, 0) = vec_x0[7];
    B(11, 0) = vec_xf[7];
    B(12, 0) = vec_x0[2];
    B(13, 0) = vec_xf[2];
    B(14, 0) = vec_x0[5];
    B(15, 0) = vec_xf[5];
    B(16, 0) = vec_x0[8];
    B(17, 0) = vec_xf[8];


    // Concatenate the M matrix 3 times for three trajectories (pos, Vel, Acc)

    M_big = blkdiag (M, 3);
    return (M_big.inverse() * B);
   
}



void Construct_Trajectory(MatrixXf& position, MatrixXf& velocity, MatrixXf& acceleration,
                         float t0, float tf, std::vector<float> wp_t, int N, int N_waypoints, float Ts,
                          std::vector<std::vector<float>> wp_vec)
{
    
    MatrixXf a; // coefficient of bezier trajectory
    
    // define the time steps: ex :  for 5s trajetory 5000 steps
    MatrixXf bp_rd(1, 6), bp_vd(1, 6), bp_ad(1, 6);
    MatrixXf tmp(1, 3);
    float DT; // delta time
    float s; // between 0 and 1 for each segment
    float t = 0; //initialize time
    int l = 0; // used for loop steps as counter
    int steps = (tf -t0)/Ts ; // Depends on the frequency
    int select = 0; // to select which part of the trajectory

   


    while(l < steps)
    {   
      if( l == 0) // enter this only at the start
      {   
          a = computeQuinticCoeff(wp_t[select], wp_t[select+1], N, wp_vec[select], wp_vec[select+1]);
          a.resize(6, 3);
                


      }

      if (wp_t[select + 1] < t )
      {  
          select = select + 1; // next section
          if (select == N_waypoints -1)
          {
              break;
          }
          a = computeQuinticCoeff(wp_t[select], wp_t[select+1], N, wp_vec[select], wp_vec[select+1]);
          a.resize(6, 3);
          
      }

      //define s
      DT = wp_t[select+1] - wp_t[select];
      s = (t - wp_t[select])/DT;

      for(int i = 0; i<N; i++)
      {
          bp_rd(0, i)= std::pow(s, i) * std::pow(1-s, N-1-i) * (fact(N-1)/(fact(i)*fact(N-1-i)));

          bp_vd(0, i)= (i * std::pow(s, i-1) * std::pow(1 - s, N-1-i) - (N-1-i) * std::pow(s, i) * 
                          std::pow(1-s, N-2-i)) * (fact(N-1)/(fact(i)*fact(N-1-i)))/DT ;

          bp_ad(0, i)= (i * (i-1) * std::pow(s, i-2) * std::pow(1-s, N-1-i) - 2 * i * (N-1-i) *
                          std::pow(s, i-1) * std::pow(1-s, N-2-i) + (N-1-i) * (N-i-2)* 
                          std::pow(s, i) * std::pow(1-s, N-3-i)) * (fact(N-1)/(fact(i)*fact(N-1-i)))/(std::pow(DT, 2)) ;


      }

      tmp = bp_rd * a;
      
      position(l, 0) = tmp(0);
      position(l, 1) = tmp(1);
      position(l, 2) = tmp(2);

      tmp = bp_vd * a;

      velocity(l, 0) = tmp(0);
      velocity(l, 1) = tmp(1);
      velocity(l, 2) = tmp(2);

      tmp = bp_ad * a;

      acceleration(l, 0) = tmp(0);
      acceleration(l, 1) = tmp(1);
      acceleration(l, 2) = tmp(2);

      // at the end of iteration , update t
      t = t + Ts;
      l = l + 1;
    }
    position(0, 0) = wp_vec[0][0];position(0, 1) = wp_vec[0][1];position(0, 2) = wp_vec[0][2];
    velocity(0, 0) = 0 ; velocity(0, 1) = 0;  velocity(0, 2) = 0; //instead of nan
    acceleration(0, 0) = 0; acceleration(0, 1) = 0;  acceleration(0, 2) = 0; //instead of nan


}


//takes control points as vector 
VectorXf  bspline_deriv(MatrixXf a, double t, double ti, 
double tf, int deriv, int degree, int N_dim) 
{
    double dt = tf - ti;
    VectorXf point(N_dim);
    point.setZero();
    float tmp;
    for (int k = 0; k <= deriv; k++)
     {
        for (int j = deriv - k; j <= degree - k; j++) 
        {
            tmp = (fact(degree) * comb(deriv, k) * std::pow(-1, k) * std::pow(t - ti, j - deriv + k) * std::pow(tf - t, degree - j - k)) /
                        (std::pow(dt, degree) * fact(j - deriv + k) * fact(degree - j - k)) ;
            point += tmp * a.col(j);  
        }
    }

    return point;
}


void interpolate_waypoint_at(int N_jc, int N_dim, int degree, 
                            int N, int index, float t, std::vector<float>& wp_t, 
                            std::vector<std::vector<float>>& wp_vec)

{   
    MatrixXf a; // coefficient of bezier trajectory
    MatrixXf a_new; // coefficient of bezier trajectory
    MatrixXf waypoint(N_jc, N_dim);
    std::vector<float> wp;
    float ti = wp_t[index - 1];
    float tf = wp_t[index];
    a = computeQuinticCoeff(ti, tf, N, wp_vec[index - 1], wp_vec[index]);
    a.resize(6, N_dim);
    a_new = a.transpose();


    // std::cout<<"This is a: "<<a<< std::endl;

    for (int deriv = 0; deriv < N_jc; deriv++) 
    {
        waypoint.row(deriv) = bspline_deriv(a_new, t, ti, tf, deriv, degree, N_dim) ;
    }

    //now just put the waypoint matrix in vector wp
    for (int i = 0; i < N_jc; i++ )
    {
        for (int j = 0; j < N_dim; j++ )
        {
            wp.push_back(waypoint(i, j)) ;
        }
    }

    wp_vec.insert(wp_vec.begin()+index, wp);
    wp_t.insert(wp_t.begin()+ index, t);


}


double solveQuadratic(double a, double b, double c) 
{
  double discriminant = b*b - 4*a*c;

  if (discriminant > 0) 
  {
    double x1 = (-b + sqrt(discriminant)) / (2*a);
    double x2 = (-b - sqrt(discriminant)) / (2*a);
    std::cout << "The solutions are: " << x1 << " and " << x2 << std::endl;
    if(x1 >0) return x1;
    if(x2>0) return x2;

  } 
  
  else if (discriminant == 0) 
  {
    double x = -b / (2*a);
    std::cout << "The solution is: " << x << std::endl;
    return x;
  } 
  
  std::cout << "No real solutions." << std::endl;
      
  return 0;
}

int
main( int argc, char **argv )
{
  bool opt_coppeliasim_sync_mode = false;
  bool opt_verbose               = false;
  bool opt_save_data             = false;

  for ( int i = 1; i < argc; i++ )
  {
    if ( std::string( argv[i] ) == "--enable-coppeliasim-sync-mode" )
    {
      opt_coppeliasim_sync_mode = true;
    }
    else if ( std::string( argv[i] ) == "--verbose" || std::string( argv[i] ) == "-v" )
    {
      opt_verbose = true;
    }
    else if ( std::string( argv[i] ) == "--save" )
    {
      opt_save_data = true;
    }
    else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
    {
      std::cout << argv[0] << "[--enable-coppeliasim-sync-mode]"
                << " [--save]"
                << " [--verbose] [-v] "
                << " [--help] [-h]" << std::endl;
      return EXIT_SUCCESS;
    }
  }


  rclcpp::init( argc, argv );
  auto node = std::make_shared< rclcpp::Node >( "frankasim_testing" );
  rclcpp::WallRate loop_rate( 100 );
  rclcpp::spin_some( node );

  vpROSRobotFrankaCoppeliasim robot;

  try
  {

    robot.setVerbose( opt_verbose );
    robot.connect();

    const auto gripper_pub  = node->create_publisher< std_msgs::msg::Int32 >( "/gripper", 1 );
    std_msgs::msg::Int32 gripper_state;

    std::cout << "Coppeliasim sync mode enabled: " << ( opt_coppeliasim_sync_mode ? "yes" : "no" ) << std::endl;
    robot.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
    robot.setCoppeliasimSyncMode( false );
    robot.coppeliasimStartSimulation();


    /*#############################################################################################################*/
    // trajectory calculations
    //using namespace matplot;
    int N = 6; // N_coefficients of bezier curve
    int N_waypoints = 4; // number of cartesian waypoints
    int N_pieces = N_waypoints - 1;
    float Ts = 0.003; //10ms sampling time
    float t0 = 0; // initial time
    float tf = 5; // final time
    int N_dim = 3; // --> considering x, y and z --> dimension is 3
    int N_jc = 3; // -->  conditions position, velocity and acceleration
    int degree = 2 * N_jc -1; // --> degree of the polynomial
    int param = (tf-t0)/Ts; // number of samples (time points)

    std::cout<<param<<std::endl;


    
    MatrixXf position(param, 3);
    MatrixXf velocity(param, 3);
    MatrixXf acceleration(param, 3);


    // this is used only to plot the trajectory in 3D using matplotplusplus
    std::vector <float> xt_d(param);
    std::vector <float> yt_d(param);
    std::vector <float> zt_d(param);

    std::vector <float> vxt_d(param);
    std::vector <float> vyt_d(param);
    std::vector <float> vzt_d(param);
    std::vector <float> t(param);


    // actual traj in 3D performed by the manipulator
    std::vector <float> xt(param);
    std::vector <float> yt(param);
    std::vector <float> zt(param);

    std::vector <float> vxt(param);
    std::vector <float> vyt(param);
    std::vector <float> vzt(param);


    // First initialize 2 waypoints "initial & Final"
    std::vector<float> wp_0{0.5, -0.4, 0.2, 0, 0, 0, 0, 0, 0};
    // std::vector<float> wp_1{-0.35, 0, 0.35, 0.5, 0, 0.5, 0, 0, 0};
    std::vector<float> wp_1{0, -0.4, 0.5, 0.5, -0.2, 0.5, 0, 0, 0};
    std::vector<float> wp_f{0.3, -0.3, 0.5, 0, 0, 0, 0, 0, 0};



    //Initialize Vector of waypoints and vector of timing for the waypoints
    //Initialization contains only information First & Final waypoint

    std::vector<std::vector<float>> wp_vec{wp_0, wp_1, wp_f};
    std::vector<float> wp_t{t0, 4*tf/5, tf};

    // determine final position of the projectile
    double l = wp_vec.size() - 2 ;
    // double h = 0.8; // increment z by this value to transfer to world coordinate
    double b = 0; // value of the final altitude desired
    double h = 0.8; // height augmentation in coppeliasim for the base of the robot

    double tf_projectile = solveQuadratic(-0.5*9.81, wp_vec[l][5], wp_vec[l][2] + h -b);
 

    vpColVector object_Final (3, 0);
    object_Final[0] = wp_vec[l][0] + wp_vec[l][3] * tf_projectile;
    object_Final[1] = wp_vec[l][1] + wp_vec[l][4] * tf_projectile;
    object_Final[2] = wp_vec[l][2] + h + wp_vec[l][5] * tf_projectile - 0.5 * (9.81) * std::pow(tf_projectile, 2) ;

    
    Construct_Trajectory(position, velocity, acceleration, t0, tf, wp_t, N, N_waypoints, Ts, wp_vec);


   
    

    for(int i =0; i<param; i++)
    {
        xt_d[i] = position(i, 0);
        yt_d[i] = position(i, 1);
        zt_d[i] = position(i, 2);

        if (i <= wp_t[wp_vec.size() - 2]/Ts)
        {
          vxt_d[i] = velocity(i, 0);
          vyt_d[i] = velocity(i, 1);
          vzt_d[i] = velocity(i, 2);
        }


        t[i] = 0 + (i*Ts);
    }

  
   
    // std::cout<<"############## Trajectory Constructed ############"<<std::endl;
    std::cout<<"Current time before Starting trajectory: "<< std::endl<<robot.getCoppeliasimSimulationTime()<<std::endl;



    // Trajectory Constructed
    /*#############################################################################################################*/


    // Here we can write a code in which it would enable us to calculte joint angles given EE desired position " IK problem"
    // // Define the desired end-effector position
    //initial homogenious transformation base --> EE
    vpColVector q_init(7, 0), x_init(6, 0);
    vpHomogeneousMatrix eeMdesired;
    eeMdesired = robot.get_fMe();

    eeMdesired[0][3] = wp_0[0]; // x-coordinate
    eeMdesired[1][3] = wp_0[1]; // y-coordinate
    eeMdesired[2][3] = wp_0[2]; // z-coordinate

    q_init = robot.solveIK(eeMdesired);
    robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
    robot.setPosition( vpRobot::JOINT_STATE, q_init );
    //vpTime::wait( 2000 );
    
    std::cout<<"This is q_init :"<<q_init.t()<<std::endl;
    q_init.rad2deg();
    std::cout<<"This is q_init :"<<q_init.t()<<std::endl;

    // Get initial position of the robot from coppeliasim
    robot.getPosition(vpRobot::END_EFFECTOR_FRAME, x_init);
    robot.getPosition(vpRobot::JOINT_STATE, q_init);
    std::cout<< " This is x_init: "<< std::endl << x_init.t()<<std::endl;




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

    vpColVector x(6, 0), xdot(6, 0), q( 7, 0 ), dq( 7, 0 ), tau_d( 7, 0 ), C( 7, 0 ), F( 7, 0 ), tau_d0( 7, 0 ), tau_cmd( 7, 0 ),
        x_e( 6, 0 ), dx_e( 6, 0 ), dx_ed( 6, 0 ), ddx_ed( 6, 0 );
    vpMatrix fJe( 6, 7 ), Ja( 6, 7 ), dJa( 6, 7 ), Ja_old( 6, 7 ), B( 7, 7 ), I7( 7, 7 ), Ja_pinv_B_t( 6, 7 );
    vpColVector pose_err_norm( 2, 0 ), tau( 7, 0 );


    // min and max of joint velocities dq
    double dq_max[7] = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
    double dq_min[7] = {-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100};

    vpColVector q_min ({-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973});
    vpColVector q_max ({2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973});
    vpColVector q_median = (q_max + q_min)/2 ;
    vpColVector q_0_dot (7, 0);

    vpColVector e_x(6, 0), xd(6, 0), xd_dot(6, 0), qdot(7, 0);




    robot.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL );
    std::cout<< " Controller Chosen: Torque"<<std::endl;

    robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );



    vpHomogeneousMatrix fMed, fMed0;
    fMed0 = robot.get_fMe();
    fMed  = fMed0;
    std::cout<<fMed0<<std::endl;

    bool final_quit       = false;
    bool first_time       = false;
    bool start_trajectory = false;

    vpMatrix K( 6, 6 ), D( 6, 6 ), edVf( 6, 6 ), K_v (6, 6);

    double wp = 50;
    double wo = 20;
    K.diag( { wp * wp, wp * wp, wp * wp, wo * wo, wo * wo, wo * wo } );
    D.diag( { 2 * wp, 2 * wp, 2 * wp, 2 * wo, 2 * wo, 2 * wo } );
    I7.eye();

    double mu = 4;
    double dt = 0;

    double time_start_trajectory, time_prev, time_cur;

    int i = 0;
    double tmp_gripper_open = 4*param/5; // time opening the gripper
    
    std::cout<< " -->executing Bezier Trajectory"<<std::endl;
    time_start_trajectory = time_cur;

    while(i < param)

    {

      time_cur = robot.getCoppeliasimSimulationTime();
      robot.getPosition( vpRobot::END_EFFECTOR_FRAME, x );
      robot.getVelocity( vpRobot::END_EFFECTOR_FRAME, xdot );
      robot.getPosition( vpRobot::JOINT_STATE, q );
      robot.getVelocity( vpRobot::JOINT_STATE, dq );
      robot.getMass( B );
      robot.getCoriolis( C );
      robot.getFriction( F );
      robot.get_fJe( fJe );
      robot.getForceTorque( vpRobot::JOINT_STATE, tau );


      if (tmp_gripper_open < i)
        {
          gripper_state.data = 1;
          gripper_pub->publish(gripper_state);
        }

      // Compute Cartesian trajectories
      // clang-format off
      fMed[0][3] = position(i, 0) ;
      fMed[1][3] = position(i, 1) ;
      fMed[2][3] = position(i, 2) ;

      dx_ed[0] = velocity(i, 0) ;
      dx_ed[1] = velocity(i, 1);
      dx_ed[2] = velocity(i, 2);

      ddx_ed[0] = acceleration(i, 0) ;
      ddx_ed[1] = acceleration(i, 1) ;
      ddx_ed[2] = acceleration(i, 2);


      // clang-format on

      edVf.insert( fMed.getRotationMatrix().t(), 0, 0 );
      edVf.insert( fMed.getRotationMatrix().t(), 3, 3 );


      x_e = (vpColVector)vpPoseVector( fMed.inverse() * robot.get_fMe() ); // edMe // pose error [ t, theta * u ] using homogeneous transformations
      Ja  = Ta( fMed.inverse() * robot.get_fMe() ) * edVf * fJe; //equation 2.12 -- Analytical jacobian

      dx_e = Ta( fMed.inverse() * robot.get_fMe() ) * edVf * ( dx_ed - fJe * dq );


      dt = time_cur - time_prev;

      if ( dt != 0 )
      {
        dJa = ( Ja - Ja_old ) / dt;
      }
      else
      {
        dJa = 0;
      }
      Ja_old = Ja;

      Ja_pinv_B_t = ( Ja * B.inverseByCholesky() * Ja.t() ).inverseByCholesky() * Ja * B.inverseByCholesky();

      // Compute the control law
      tau_d = B * Ja.pseudoInverse() * ( ddx_ed  - K * ( x_e ) + D * (dx_e) - dJa * dq  ) + C + F -
              ( I7 - Ja.t() * Ja_pinv_B_t ) * B * dq * 100; // equation 2.57 + 2.6 + 2.59

      
      tau_d0 = tau_d;


      tau_cmd = tau_d - tau_d0 * std::exp( -mu * ( time_cur - time_start_trajectory ) );


      robot.setForceTorque( vpRobot::JOINT_STATE, tau_cmd );
      
      // these are for plotting the actual path in 3D
      

      xt[i] = x[0];
      yt[i] = x[1];
      zt[i] = x[2];

      if (i <= wp_t[wp_vec.size() - 2]/Ts)
      {
        vxt[i] = xdot[0];
        vyt[i] = xdot[1];
        vzt[i] = -xdot[2];
      }


      plotter->plot( 0, time_cur, q );
      plotter->plot( 1, time_cur, tau );
      plotter->plot( 2, time_cur, x_e );
      pose_err_norm[0] = sqrt( x_e.extract( 0, 3 ).sumSquare() );
      pose_err_norm[1] = sqrt( x_e.extract( 3, 3 ).sumSquare() );
      plotter->plot( 3, time_cur, pose_err_norm );
              
      i++;

      time_prev = time_cur;
      robot.wait( time_cur, Ts ); // Sync loop at 1000 Hz (1 ms)

    }

    std::cout<<"Hello"<<std::endl;



    robot.coppeliasimStopSimulation();


    vpMouseButton::vpMouseButtonType button;
    if ( vpDisplay::getClick( plotter->I, button, false ) )
    {
      if ( button == vpMouseButton::button3 )
      {
        final_quit = true;
        std::cout << "Stop the robot " << std::endl;
        robot.setRobotState( vpRobot::STATE_STOP );
      }
    }


    if ( opt_save_data )
    {
      plotter->saveData( 0, "sim-cart-joint-position.txt", "# " );
      plotter->saveData( 1, "sim-cart-joint-torques.txt", "# " );
      plotter->saveData( 2, "sim-cart-pose-error.txt", "# " );
      plotter->saveData( 3, "sim-cart-pose-error-norm.txt", "# " );
    }

    if ( plotter != nullptr )
    {
      delete plotter;
      plotter = nullptr;
    }
    robot.coppeliasimStopSimulation();
  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState( vpRobot::STATE_STOP );
    return EXIT_FAILURE;
  }

  return 0;
}
