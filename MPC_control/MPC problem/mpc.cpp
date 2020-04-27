/*
File Name: main.cpp

Created by bowen wang on 2/4/20.

Copyright © 2020 bowen wang. All rights reserved.
*/

/*////////////////////////////////////////////////////////////////
Usage:
Build the MPC problem for the two-wheeled turtlebot.
///////////////////////////////////////////////////////////////*/


#include <acado_code_generation.hpp>
#include <math.h>
USING_NAMESPACE_ACADO

#define PI 3.1415926

int main( )
{

    // State
    DifferentialState   x;  // the turtlebot position (x, y)
    DifferentialState   y;  // the turtlebot position (x, y)
    DifferentialState   v;  // the turtlebot linear velocity
    DifferentialState   theta;  // the excitation angle
    DifferentialState   omega;  // the angular velocity
    Control             Tl;  // left torque of the wheel
    Control             Tr; // right torque of the wheel

    //const value
    const double     ts = 0.1;  // time interval
    const double     mB = 6.96;  //Mass of pendulum body
    const double     mW = 0.3;   //Mass of wheel
    const double     l = 0.208;  //lenght of pendulum
    const double     r = 0.031;  //Radius of wheel
    const double     d = 0.257;  //Distance between wheels(tread)
    const double     I1 = 0, I2 = 0, I3 = 4.348*1e-7; // MOI of pendulum body
    const double     K = 1.89*1e-10,  J = 3.78*1e-10; //MOI of wheel
    
    //Intermedia value
    double miu1 = (mB+2*mW+2*J/(r*r))*(I2+mB*l*l) - (mB*mB)*(l*l);
    double miu2 = I3 + 2*K + 2*(mW+J/(r*r))*(d*d);
    double acc_omega = (d/(r*miu2))*(Tl-Tr)*57.2958;
    double phi = (1/2)*acc_omega*(ts*ts) + omega*ts;
    double A = (90 - theta + phi/2)*PI/180;
    double acc = (((I2+mB*l*l)/r + mB*l)/miu1)*(Tl+Tr)*100;
    
    // Model equations:
    DifferentialEquation f;

    f << dot( x ) == ((1/2)*acc*ts + v)*cos(A);
    f << dot( v ) == acc;
    f << dot( theta ) == phi;
    f << dot( omega ) == acc_omega;

    // Reference functions and weighting matrices:
    Function h, hN;
    h << x << y << v << theta << omega << Tl << Tr;
    hN << x << y << v << theta << omega;

    // Provide defined weighting matrices:
    DMatrix W = eye<double>( h.getDim() );
    DMatrix WN = eye<double>( hN.getDim() );
    //WN *= 5;

    // Or provide sparsity patterns for the weighting matrices
//    BMatrix W = eye<bool>( h.getDim() );
//    BMatrix WN = eye<bool>( hN.getDim() );

    //
    // Optimal Control Problem
    //
    OCP ocp(0.0, 3.0, 10);

    ocp.subjectTo( f );

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    //contraints
    ocp.subjectTo( -1.0 <= Tl <= 1.0 );
    ocp.subjectTo( -1.0 <= Tr <= 1.0 );
    ocp.subjectTo( 0.0 <= v <= 10.0 );

    // Export the code:
    OCPexport mpc( ocp );

    mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
    mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
    mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
    mpc.set( NUM_INTEGRATOR_STEPS,        30              );

    mpc.set( QP_SOLVER,                   QP_QPOASES      );
//     mpc.set( HOTSTART_QP,                 YES             );
//     mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
    mpc.set( GENERATE_TEST_FILE,          YES             );
    mpc.set( GENERATE_MAKE_FILE,          YES             );
    mpc.set( GENERATE_MATLAB_INTERFACE,   YES             );
    mpc.set( GENERATE_SIMULINK_INTERFACE, YES             );

//     mpc.set( USE_SINGLE_PRECISION,        YES             );

    if (mpc.exportCode( "MPC_for_turtlebot" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}
