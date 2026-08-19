#include "model.h"
namespace mpcc{
Model::Model()
:Ts_(1.0)
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Model::Model(double Ts,const PathToJson &path)
:Ts_(Ts),param_(Param(path.param_path))
{
}

double Model::getSlipAngleFront(const State &x) const
{
    // compute slip angels given current state
    return -std::atan2(x.vy+x.r*param_.lf,x.vx) + x.delta;
}

double Model::getSlipAngleRear(const State &x) const
{
    // compute slip angels given current state
    return -std::atan2(x.vy-x.r*param_.lr,x.vx);
}

TireForces Model::getForceFront(const State &x) const
{

    const double F_zf = getForceNormal().F_N_front; // normal force at front tire

    const double alpha_f = getSlipAngleFront(x);
    const double F_y = F_zf * param_.Df * std::sin(param_.Cf * std::atan(param_.Bf * alpha_f - param_.Ef * (param_.Bf * alpha_f - std::atan(param_.Bf * alpha_f))));
    const double F_x = 0.0;

    return {F_y,F_x};
}

TireForces Model::getForceRear(const State &x) const
{
    const double F_zr = getForceNormal().F_N_rear; // normal force at front tire


    const double alpha_r = getSlipAngleRear(x);
    const double F_y = F_zr * param_.Dr * std::sin(param_.Cr * std::atan(param_.Br * alpha_r - param_.Er * (param_.Br * alpha_r - std::atan(param_.Br * alpha_r))));
    // const double F_x = param_.Cm1*x.D - param_.Cm2*x.D*x.vx;// - param_.Cr0 - param_.Cr2*std::pow(x.vx,2.0);
    const double F_x = param_.Cm1*x.D - param_.Cm2*x.D*x.vx - param_.Cr0 - param_.Cr2*std::pow(x.vx,2.0);

    return {F_y,F_x};
}

// double Model::getForceFriction(const State &x) const
// {
//     return -param_.Cr0 - param_.Cr2*std::pow(x.vx,2.0);
// }

NormalForces Model::getForceNormal() const
{
    // at this point aero forces could be modeled
    const double f_n_front = param_.m*param_.g*param_.lr/(param_.lf + param_.lr);
    const double f_n_rear = param_.m*param_.g*param_.lf/(param_.lf + param_.lr);
    return {f_n_front,f_n_rear};
}

TireForcesDerivatives Model::getForceFrontDerivatives(const State &x) const
{
    const double alpha_f = getSlipAngleFront(x);
    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double delta = x.delta;

    const double F_zf = getForceNormal().F_N_front;

    const double B = param_.Bf;
    const double C = param_.Cf;
    const double D = param_.Df;
    const double E = param_.Ef;

    const double tan_term = std::atan(B * alpha_f);
    const double inner = B * alpha_f - E * (B * alpha_f - tan_term);
    const double outer = C * std::atan(inner);

    const double dFy_dAlpha = F_zf * D * C * std::cos(outer) *
        (B * (1. - E) + E * B / (1. + std::pow(B * alpha_f, 2))) / (1. + std::pow(inner, 2));

    // d alpha_f / dx terms
    const double denom = vx*vx + std::pow(vy + r * param_.lf, 2);
    const double dAlpha_dvx = (vy + r * param_.lf) / denom;
    const double dAlpha_dvy = -vx / denom;
    const double dAlpha_dr  = -vx * param_.lf / denom;
    const double dAlpha_ddelta = 1.0;

    // dF_y
    const double dF_y_vx    = dFy_dAlpha * dAlpha_dvx;
    const double dF_y_vy    = dFy_dAlpha * dAlpha_dvy;
    const double dF_y_r     = dFy_dAlpha * dAlpha_dr;
    const double dF_y_D     = 0.0;
    const double dF_y_delta = dFy_dAlpha * dAlpha_ddelta;

    return {dF_y_vx, dF_y_vy, dF_y_r, dF_y_D, dF_y_delta,
            0.0, 0.0, 0.0, 0.0, 0.0}; // dF_x_* = 0
}

// TireForcesDerivatives Model::getForceRearDerivatives(const State &x) const
// {
//     const double alpha_r = getSlipAngleRear(x);
//     const double vx = x.vx;
//     const double vy = x.vy;
//     const double r  = x.r;
//     const double D  = x.D;

//     const double F_zr = getForceNormal().F_N_rear;

//     const double B = param_.Br;
//     const double C = param_.Cr;
//     const double Dp = param_.Dr;
//     const double E = param_.Er;

//     const double tan_term = std::atan(B * alpha_r);
//     const double inner = B * alpha_r - E * (B * alpha_r - tan_term);
//     const double outer = C * std::atan(inner);

//     const double dFy_dAlpha = F_zr * Dp * C * std::cos(outer) *
//         (B * (1. - E) + E * B / (1. + std::pow(B * alpha_r, 2))) / (1. + std::pow(inner, 2));

//     // d alpha_r / dx terms
//     const double denom = vx*vx + std::pow(vy - r * param_.lr, 2);
//     const double dAlpha_dvx = (vy - r * param_.lr) / denom;
//     const double dAlpha_dvy = -vx / denom;
//     const double dAlpha_dr  = vx * param_.lr / denom;

//     // dF_y
//     const double dF_y_vx    = dFy_dAlpha * dAlpha_dvx;
//     const double dF_y_vy    = dFy_dAlpha * dAlpha_dvy;
//     const double dF_y_r     = dFy_dAlpha * dAlpha_dr;
//     const double dF_y_D     = 0.0;
//     const double dF_y_delta = 0.0;

//     // F_rx
//     const double dF_x_vx    = -param_.Cm2*D;
//     const double dF_x_vy    = 0.0;
//     const double dF_x_r     = 0.0;
//     const double dF_x_D     = param_.Cm1 - param_.Cm2*vx;
//     const double dF_x_delta = 0.0;

//     return {dF_y_vx, dF_y_vy, dF_y_r, dF_y_D, dF_y_delta,
//             dF_x_vx, dF_x_vy, dF_x_r, dF_x_D, dF_x_delta};
// }

TireForcesDerivatives Model::getForceRearDerivatives(const State &x) const
{
    const double alpha_r = getSlipAngleRear(x);
    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double D  = x.D;

    const double F_zr = getForceNormal().F_N_rear;

    const double B = param_.Br;
    const double C = param_.Cr;
    const double Dp = param_.Dr;
    const double E = param_.Er;

    const double tan_term = std::atan(B * alpha_r);
    const double inner = B * alpha_r - E * (B * alpha_r - tan_term);
    const double outer = C * std::atan(inner);

    const double dFy_dAlpha = F_zr * Dp * C * std::cos(outer) *
        (B * (1. - E) + E * B / (1. + std::pow(B * alpha_r, 2))) / (1. + std::pow(inner, 2));

    // d alpha_r / dx terms
    const double denom = vx*vx + std::pow(vy - r * param_.lr, 2);
    const double dAlpha_dvx = (vy - r * param_.lr) / denom;
    const double dAlpha_dvy = -vx / denom;
    const double dAlpha_dr  = vx * param_.lr / denom;

    // dF_y
    const double dF_y_vx    = dFy_dAlpha * dAlpha_dvx;
    const double dF_y_vy    = dFy_dAlpha * dAlpha_dvy;
    const double dF_y_r     = dFy_dAlpha * dAlpha_dr;
    const double dF_y_D     = 0.0;
    const double dF_y_delta = 0.0;

    // UPDATED F_rx derivatives with -Cr0 - Cr2*vx^2
    // F_x = Cm1*D - Cm2*D*vx - Cr0 - Cr2*vx^2
    const double dF_x_vx    = -param_.Cm2 * D - 2.0 * param_.Cr2 * vx;
    const double dF_x_vy    = 0.0;
    const double dF_x_r     = 0.0;
    const double dF_x_D     = param_.Cm1 - param_.Cm2 * vx;
    const double dF_x_delta = 0.0;

    return {dF_y_vx, dF_y_vy, dF_y_r, dF_y_D, dF_y_delta,
            dF_x_vx, dF_x_vy, dF_x_r, dF_x_D, dF_x_delta};
}

// FrictionForceDerivatives Model::getForceFrictionDerivatives(const State &x) const
// {
//     return {-2.0*param_.Cr2*x.vx,0.0,0.0,0.0,0.0};
// }

StateVector Model::getF(const State &x,const Input &u) const
{
    const double phi = x.phi;
    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double D = x.D;
    const double delta = x.delta;
    const double vs = x.vs;

    const double dD = u.dD;
    const double dDelta = u.dDelta;
    const double dVs = u.dVs;

    const TireForces tire_forces_front = getForceFront(x);
    const TireForces tire_forces_rear  = getForceRear(x);
    // const double friction_force = getForceFriction(x);

    StateVector f;
    f(0) = vx*std::cos(phi) - vy*std::sin(phi);
    f(1) = vy*std::cos(phi) + vx*std::sin(phi);
    f(2) = r;
    // f(3) = 1.0/param_.m*(tire_forces_rear.F_x + friction_force - tire_forces_front.F_y*std::sin(delta) + param_.m*vy*r);
    f(3) = 1.0/param_.m*(tire_forces_rear.F_x - tire_forces_front.F_y*std::sin(delta) + param_.m*vy*r);
    f(4) = 1.0/param_.m*(tire_forces_rear.F_y + tire_forces_front.F_y*std::cos(delta) - param_.m*vx*r);
    f(5) = 1.0/param_.Iz*(tire_forces_front.F_y*param_.lf*std::cos(delta) - tire_forces_rear.F_y*param_.lr);
    f(6) = vs;
    f(7) = dD;
    f(8) = dDelta;
    f(9) = dVs;

    return f;
}

LinModelMatrix Model::getModelJacobian(const State &x, const Input &u) const
{
    // compute jacobian of the model
    // state values
    const double phi = x.phi;
    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double D = x.D;
    const double delta = x.delta;

//    LinModelMatrix lin_model_c;
    A_MPC A_c = A_MPC::Zero();
    B_MPC B_c = B_MPC::Zero();
    g_MPC g_c = g_MPC::Zero();

    const StateVector f = getF(x,u);

    const TireForces F_front = getForceFront(x);
//    TireForces F_rear  = getForceRear(x);

    const TireForcesDerivatives dF_front = getForceFrontDerivatives(x);
    const TireForcesDerivatives dF_rear  = getForceRearDerivatives(x);
    // const FrictionForceDerivatives dF_fric = getForceFrictionDerivatives(x);

    // Derivatives of function
    // f1 = v_x*std::cos(phi) - v_y*std::sin(phi)
    const double df1_dphi = -vx*std::sin(phi) - vy*std::cos(phi);
    const double df1_dvx  = std::cos(phi);
    const double df1_dvy  = -std::sin(phi);

    // f2 = v_y*std::cos(phi) + v_x*std::sin(phi);
    const double df2_dphi = -vy*std::sin(phi) + vx*std::cos(phi);
    const double df2_dvx  = std::sin(phi);
    const double df2_dvy  = std::cos(phi);

    // f3 = r;
    const double df3_dr = 1.0;

    // f4 = 1/param_.m*(F_rx + F_fric - F_fy*std::sin(delta) + param_.m*v_y*r);
    // const double df4_dvx     = 1.0/param_.m*(dF_rear.dF_x_vx + dF_fric.dF_f_vx - dF_front.dF_y_vx*std::sin(delta));
    const double df4_dvx = 1.0/param_.m*(dF_rear.dF_x_vx /* + dF_fric.dF_f_vx */ - dF_front.dF_y_vx*std::sin(delta));
    const double df4_dvy     = 1.0/param_.m*(                - dF_front.dF_y_vy*std::sin(delta)    + param_.m*r);
    const double df4_dr      = 1.0/param_.m*(                - dF_front.dF_y_r*std::sin(delta)     + param_.m*vy);
    const double df4_dD      = 1.0/param_.m* dF_rear.dF_x_D;
    const double df4_ddelta  = 1.0/param_.m*(                - dF_front.dF_y_delta*std::sin(delta) - F_front.F_y*std::cos(delta));

    // f5 = 1/param_.m*(F_ry + F_fy*std::cos(delta) - param_.m*v_x*r);
    const double df5_dvx     = 1.0/param_.m*(dF_rear.dF_y_vx  + dF_front.dF_y_vx*std::cos(delta)    - param_.m*r);
    const double df5_dvy     = 1.0/param_.m*(dF_rear.dF_y_vy  + dF_front.dF_y_vy*std::cos(delta));
    const double df5_dr      = 1.0/param_.m*(dF_rear.dF_y_r   + dF_front.dF_y_r*std::cos(delta)     - param_.m*vx);
    const double df5_ddelta  = 1.0/param_.m*(                   dF_front.dF_y_delta*std::cos(delta) - F_front.F_y*std::sin(delta));

    // f6 = 1/param_.Iz*(F_fy*l_f*std::cos(delta)- F_ry*l_r)
    const double df6_dvx     = 1.0/param_.Iz*(dF_front.dF_y_vx*param_.lf*std::cos(delta)    - dF_rear.dF_y_vx*param_.lr);
    const double df6_dvy     = 1.0/param_.Iz*(dF_front.dF_y_vy*param_.lf*std::cos(delta)    - dF_rear.dF_y_vy*param_.lr);
    const double df6_dr      = 1.0/param_.Iz*(dF_front.dF_y_r*param_.lf*std::cos(delta)     - dF_rear.dF_y_r*param_.lr);
    const double df6_ddelta  = 1.0/param_.Iz*(dF_front.dF_y_delta*param_.lf*std::cos(delta) - F_front.F_y*param_.lf*std::sin(delta));

    // Jacobians
    // Matrix A
    // Column 1
    // all zero
    // Column 2
    // all zero
    // Column 3
    A_c(0,2) = df1_dphi;
    A_c(1,2) = df2_dphi;
    // Column 4
    A_c(0,3) = df1_dvx;
    A_c(1,3) = df2_dvx;
    A_c(3,3) = df4_dvx;
    A_c(4,3) = df5_dvx;
    A_c(5,3) = df6_dvx;
    // Column 5
    A_c(0,4) = df1_dvy;
    A_c(1,4) = df2_dvy;
    A_c(3,4) = df4_dvy;
    A_c(4,4) = df5_dvy;
    A_c(5,4) = df6_dvy;
    // Column 6
    A_c(2,5) = df3_dr;
    A_c(3,5) = df4_dr;
    A_c(4,5) = df5_dr;
    A_c(5,5) = df6_dr;
    // Column 7
    // all zero
    // Column 8
    A_c(3,7) = df4_dD;
    // Column 9
    A_c(3,8) = df4_ddelta;
    A_c(4,8) = df5_ddelta;
    A_c(5,8) = df6_ddelta;
    // Column 10
    A_c(6,9) = 1.0;

    // Matrix B
    // Column 1
    B_c(7,0) = 1.0;
    // Column 2
    B_c(8,1) = 1.0;
    // Column 3
    B_c(9,2) = 1.0;

    //zero order term
    g_c = f - A_c*stateToVector(x) - B_c*inputToVector(u);

    return {A_c,B_c,g_c};
}

LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c) const
{
    // disctetize the continuous time linear model \dot x = A x + B u + g using ZHO
    Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp = Eigen::Matrix<double,NX+NU+1,NX+NU+1>::Zero();
    // building matrix necessary for expm
    // temp = Ts*[A,B,g;zeros]
    temp.block<NX,NX>(0,0) = lin_model_c.A;
    temp.block<NX,NU>(0,NX) = lin_model_c.B;
    temp.block<NX,1>(0,NX+NU) = lin_model_c.g;
    temp = temp*Ts_;
    // take the matrix exponential of temp
    const Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp_res = temp.exp();
    // extract dynamics out of big matrix
    // x_{k+1} = Ad x_k + Bd u_k + gd
    //temp_res = [Ad,Bd,gd;zeros]
    const A_MPC A_d = temp_res.block<NX,NX>(0,0);
    const B_MPC B_d = temp_res.block<NX,NU>(0,NX);
    const g_MPC g_d = temp_res.block<NX,1>(0,NX+NU);

    return {A_d,B_d,g_d};
}

LinModelMatrix Model::getLinModel(const State &x, const Input &u) const
{
    // compute linearized and discretized model
    const LinModelMatrix lin_model_c = getModelJacobian(x,u);
    // discretize the system
    return discretizeModel(lin_model_c);
}
}