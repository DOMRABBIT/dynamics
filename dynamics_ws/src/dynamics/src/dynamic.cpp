#include "dynamic.h"

// Inverse dynamics using Recursive Newton-Euler Method
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
//  qd[]: The robot's joint velocity array, which size is concided with NB,
//        mostly attain from state estimator or just different by q
// qdd[]: The robot's joint acceleration array, which size is concided with NB,
//        attain from desire joint acceleration
// Gra_offset: if true, then caluate product tau contain the gravity term; if false, vice verse
Eigen::MatrixXd inverse_dynamic(Model &model,
                                double _q[],
                                double _qd[],
                                double _qdd[],
                                bool Gra_offset,
                                Eigen::MatrixXd &v_out,
                                Eigen::MatrixXd &a_out)
{
    int NB = model.NB;
    Eigen::Matrix<double, 6, 6> *X_up;
    Eigen::Matrix<double, 6, 6> *X_q;
    Eigen::Matrix<double, 6, 1> *v;
    Eigen::Matrix<double, 6, 1> *a;
    Eigen::Matrix<double, 6, 1> *f;
    Eigen::Matrix<double, 6, 1> g;
    // MatrixXd q = Eigen::Map<MatrixXd>(_q, NB, 1);
    MatrixXd qd = Eigen::Map<MatrixXd>(_qd, NB, 1);
    MatrixXd qdd = Eigen::Map<MatrixXd>(_qdd, NB, 1);
    Eigen::MatrixXd torque = Eigen::Map<MatrixXd>(_q, NB, 1); //torque is the same size of vector(_q)
    torque.setZero();
    X_up = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
    Eigen::Matrix<double, 6, 1> vJ; // joint velocity, satisfying vJ = S(j)*qd and v(i) = v(i-1) + vJ
    v = (Eigen::Matrix<double, 6, 1> *)malloc(sizeof(Eigen::Matrix<double, 6, 1>) * NB); //space velocity of each body
    a = (Eigen::Matrix<double, 6, 1> *)malloc(sizeof(Eigen::Matrix<double, 6, 1>) * NB); // space acceleration of each body
    f = (Eigen::Matrix<double, 6, 1> *)malloc(sizeof(Eigen::Matrix<double, 6, 1>) * NB); // space force of each body
    v_out.setZero(6, NB);
    a_out.setZero(6, NB);
    // Update Robot state, X
    model.update_model(model, _q, nullptr);

    if (Gra_offset)
        g << 0, 0, 0, 0, 0, -9.81;
    else
        g << 0, 0, 0, 0, 0, 0;

    // forward pass, velocity and acceleration propagation
    for (int i = 0; i < model.NB; i++)
    {
        vJ = model.joint[i].S_body * qd(i);
        Eigen::Matrix3d R = model.Ttree[i].block(0, 0, 3, 3).transpose();
        Eigen::Vector3d p = -R * model.Ttree[i].block(0, 3, 3, 1);
        AdjointT(R, p, X_up[i]); // X_up(i) = i_X_lenda(i), i_S_parent(i) = i_X_parent(i) * parent(i)_S_parent(i)
        if (model.parent[i] == -1)
        {
            v[i] = vJ;
            a[i] = X_up[i] * (-g) + model.joint[i].S_body * qdd(i);
        }
        else
        {
            v[i] = X_up[i] * v[model.parent[i]] + vJ;
            a[i] = X_up[i] * a[model.parent[i]] + model.joint[i].S_body * qdd(i) + crm(v[i]) * vJ;
            a_out.block(0, i, 6, 1) = crm(v[i]) * vJ;
        }
        f[i] = model.I[i] * a[i] + crf(v[i]) * model.I[i] * v[i];
        v_out.block(0, i, 6, 1) = v[i];
        
    }

    for (int i = model.NB-1; i >= 0; --i)
    {
        torque(i) = model.joint[i].S_body.transpose() * f[i];
        
        if(model.parent[i] != -1)
        {
            f[model.parent[i]] = f[model.parent[i]] + X_up[i].transpose() * f[i];
        }
    }
    return torque;
        
}

// Floating base Inverse dynamics using Recursive Newton-Euler Method
// model: The robot model which must been created
// _q_base[]: Floating base orientation and position (4+3 dimension) in inertial coordinate
// _qd_base[]: Floating base twist(6 dimension) in inertial coordinate
//   q[]: The robot's joint value array, which size is concide with NB
//  qd[]: The robot's joint velocity array, which size is concided with NB,
//        mostly attain from state estimator or just different by q
// qdd[]: The robot's joint acceleration array, which size is concided with NB,
//        attain from desire joint acceleration
// Gra_offset: if true, then caluate product tau contain the gravity term; if false, vice verse
// a_base: An output vector of this function
Eigen::MatrixXd inverse_dynamic_Flt(Model &model,
                                    double _q_base[],
                                    double _qd_base[],
                                    double _q[],
                                    double _qd[],
                                    double _qdd[],
                                    bool Gra_offset,
                                    Eigen::Matrix<double,6,1> &a_base)
{
    int NB = model.NB;
    Eigen::Matrix<double, 6, 6> *X_up;
    Eigen::Matrix<double, 6, 6> *X_q;
    Eigen::Matrix<double, 6, 1> *v;
    Eigen::Matrix<double, 6, 1> *a;
    Eigen::Matrix<double, 6, 1> *f;
    Eigen::Matrix<double, 6, 1> g;
    Eigen::Matrix<double, 6, 1> f0;
    // MatrixXd q = Eigen::Map<MatrixXd>(_q, NB, 1);
    MatrixXd qd = Eigen::Map<MatrixXd>(_qd, NB, 1);
    MatrixXd qdd = Eigen::Map<MatrixXd>(_qdd, NB, 1);
    Eigen::MatrixXd torque = Eigen::Map<MatrixXd>(_q, NB, 1); // torque is the same size of vector(_q)
    torque.setZero();
    X_up = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
    Eigen::Matrix<double, 6, 1> vJ;                                                      // joint velocity, satisfying vJ = S(j)*qd and v(i) = v(i-1) + vJ
    v = (Eigen::Matrix<double, 6, 1> *)malloc(sizeof(Eigen::Matrix<double, 6, 1>) * NB); // space velocity of each body
    a = (Eigen::Matrix<double, 6, 1> *)malloc(sizeof(Eigen::Matrix<double, 6, 1>) * NB); // space acceleration of each body
    f = (Eigen::Matrix<double, 6, 1> *)malloc(sizeof(Eigen::Matrix<double, 6, 1>) * NB); // space force of each body

    Eigen::Matrix<double, 6, 6> *IC;
    IC = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
    Eigen::Matrix<double, 6, 6> IC_base;

    Eigen::Matrix<double, 6, 1> a0_r;
    // Update Robot state, X
    model.update_model(model, _q, _q_base);

    if (Gra_offset)
        g << 0, 0, 0, 0, 0, -9.81;
    else
        g << 0, 0, 0, 0, 0, 0;

    // 在floating base坐标系下表示g
    Eigen::Matrix<double, 6, 6> X_up_ref;
    Eigen::Matrix3d R = model.T_Flt.block(0, 0, 3, 3).transpose();
    Eigen::Vector3d p = -R * model.T_Flt.block(0, 3, 3, 1);
    AdjointT(R, p, X_up_ref);
    g = X_up_ref * g;
    // 在floating base坐标系下表示v_base
    MatrixXd v_base = Eigen::Map<MatrixXd>(_qd_base, 6, 1);
    v_base = X_up_ref * v_base;

    IC_base = model.I_base;
    f0 = model.I_base * (-g) + crf(v_base) * model.I_base * v_base; //
    // forward pass, velocity and acceleration propagation
    for (int i = 0; i < model.NB; i++)
    {
        vJ = model.joint[i].S_body * qd(i);
        Eigen::Matrix3d R = model.Ttree[i].block(0, 0, 3, 3).transpose();
        Eigen::Vector3d p = -R * model.Ttree[i].block(0, 3, 3, 1);
        AdjointT(R, p, X_up[i]); // X_up(i) = i_X_lenda(i), i_S_parent(i) = i_X_parent(i) * parent(i)_S_parent(i)
        if (model.parent[i] == -1)
        {
            v[i] = X_up[i] * v_base + vJ;
            a[i] = X_up[i] * (-g) + model.joint[i].S_body * qdd(i) + crm(v[i]) * vJ; //
        }
        else
        {
            v[i] = X_up[i] * v[model.parent[i]] + vJ;
            a[i] = X_up[i] * a[model.parent[i]] + model.joint[i].S_body * qdd(i) + crm(v[i]) * vJ;
        }
        f[i] = model.I[i] * a[i] + crf(v[i]) * model.I[i] * v[i];
        IC[i] = model.I[i];
        // std::cout << "v" << i << ": " << v[i].transpose() << std::endl;
        // std::cout << "a" << i << ": " << a[i].transpose() << std::endl;
    }

    for (int i = model.NB - 1; i >= 0; --i)
    {
        if (model.parent[i] != -1)
        {
            f[model.parent[i]] = f[model.parent[i]] + X_up[i].transpose() * f[i];
            IC[model.parent[i]] += X_up[i].transpose() * IC[i] * X_up[i];
        }
        else // if parent is floating base
        {
            f0 = f0 + X_up[i].transpose() * f[i];
            IC_base += X_up[i].transpose() * IC[i] * X_up[i];
            
        }
    }
    a_base = -(IC_base.inverse()) * f0;
    Eigen::Matrix<double, 6, 1> a0_t;
    for (int i = 0; i < model.NB; i++)
    {
        if(model.parent[i] == -1) //if parent is floating base
        {
            a0_t = X_up[i] * a_base;
            torque(i) = model.joint[i].S_body.transpose() * (f[i] + IC[i] * a0_t);
        }
        else
        {
            a0_t = X_up[i] * a0_t;
            torque(i) = model.joint[i].S_body.transpose() * (f[i] + IC[i] * a0_t);
        }
        
    }
    //a_base += g;  // a_base in floating base coordinate
    return torque;
}

// Calculate genralize inertial matrix via Recursive Newton-Euler Method
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
//  qd[]: The robot's joint velocity array, which size is concided with NB,
//        mostly attain from state estimator or just differented by q
// Gra_offset: if true, then caluate product tau contain the gravity term; if false, vice verse
Eigen::MatrixXd Cal_Generalize_Inertial_Matrix_RNEA(Model &model,
                                                  double _q[],
                                                  double _qd[],
                                                  Eigen::MatrixXd Bias_force)
{
    Eigen::MatrixXd H;
    Eigen::MatrixXd v;
    Eigen::MatrixXd a;
    int NB = model.NB;
    double *qdd_s;
    qdd_s = (double *)malloc(sizeof(double) * NB);
    H.setZero(NB, NB);//intialize H
    for (int i = 0; i < NB; i++)
    {
        //make the select vector qdd_s
        for (int j = 0; j < NB; j++)
        {
            if(i == j)
                qdd_s[j] = 1;
            else
                qdd_s[j] = 0;
        }
        H.block(0, i, NB, 1) = inverse_dynamic(model, _q, _qd, qdd_s, true,v,a) - Bias_force;
    }
    return H;
}

// Calculate genralize inertial matrix via Composite-Rigid-Body Algorithm
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
//        mostly attain from state estimator or just differented by q
Eigen::MatrixXd Cal_Generalize_Inertial_Matrix_CRBA(Model &model,
                                                    double _q[])
{
    int NB = model.NB;
    // intialize H
    Eigen::MatrixXd H; 
    H.setZero(NB, NB);
    // intialize IC
    Eigen::Matrix<double, 6, 6> *IC; 
    IC = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
    for (int i = 0; i < NB;i++)
    {
        IC[i] = model.I[i];
    }
    // calculate X_up
    Eigen::Matrix<double, 6, 6> *X_up;
    X_up = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
    for (int i = 0; i < model.NB; i++)
    {
        Eigen::Matrix3d R = model.Ttree[i].block(0, 0, 3, 3).transpose();
        Eigen::Vector3d p = -R * model.Ttree[i].block(0, 3, 3, 1);
        AdjointT(R, p, X_up[i]); // X_up(i) = i_X_lenda(i), i_S_parent(i) = i_X_parent(i) * parent(i)_S_parent(i)
    }
    // calculate each body's composite inertial IC
    for (int i = model.NB - 1; i >= 0; i--)
    {
        if (model.parent[i] != -1)
        {
            IC[model.parent[i]] += X_up[i].transpose() * IC[i] * X_up[i];
        }
    }
    //calculate H
    Eigen::Matrix<double, 6, 1> fh;
    int j = 0;
    for (int i = 0; i < NB; i++)
    {
        fh = IC[i] * model.joint[i].S_body; //in body i coordinate
        H(i, i) = model.joint[i].S_body.transpose() * fh;
        j = i;
        while(model.parent[j] > -1)
        {
            fh = X_up[j].transpose() * fh; // X_up(i) = i_X_lenda(i)
            j = model.parent[j];
            H(i, j) = model.joint[j].S_body.transpose() * fh;
            H(j, i) = H(i, j);
        }
    }
    return H;
}

// Calculate floating base genralize inertial matrix via Composite-Rigid-Body Algorithm
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
//        mostly attain from state estimator or just differented by q
Eigen::MatrixXd Cal_Generalize_Inertial_Matrix_CRBA_Flt(Model &model,
                                                        double _q[],
                                                        MatrixXd &H_fl,
                                                        MatrixXd &F,
                                                        Matrix<double,6,6> &I_flbase)
{
    int NB = model.NB;
    // intialize H
    Eigen::MatrixXd H;
    H.setZero(NB, NB);
    // intialize IC
    Eigen::Matrix<double, 6, 6> *IC;
    IC = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
    Eigen::Matrix<double, 6, 6> IC_base;
    IC_base = model.I_base;
    for (int i = 0; i < NB; i++)
    {
        IC[i] = model.I[i];
    }
    // calculate X_up
    Eigen::Matrix<double, 6, 6> *X_up;
    X_up = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
    for (int i = 0; i < model.NB; i++)
    {
        Eigen::Matrix3d R = model.Ttree[i].block(0, 0, 3, 3).transpose();
        Eigen::Vector3d p = -R * model.Ttree[i].block(0, 3, 3, 1);
        AdjointT(R, p, X_up[i]); // X_up(i) = i_X_lenda(i), i_S_parent(i) = i_X_parent(i) * parent(i)_S_parent(i)
    }
    // calculate each body's composite inertial IC
    for (int i = model.NB - 1; i >= 0; i--)
    {
        if (model.parent[i] != -1)
        {
            IC[model.parent[i]] += X_up[i].transpose() * IC[i] * X_up[i];
        }
        else //if parent is floating base
        {
            IC_base += X_up[i].transpose() * IC[i] * X_up[i];
        }
    }
    // calculate H
    Eigen::Matrix<double, 6, 1> *fh;
    fh = (Eigen::Matrix<double, 6, 1> *)malloc(sizeof(Eigen::Matrix<double, 6, 1>) * NB);
    int j = 0;
    for (int i = 0; i < NB; i++)
    {
        fh[i] = IC[i] * model.joint[i].S_body; // in body i coordinate
        H(i, i) = model.joint[i].S_body.transpose() * fh[i];
        j = i;
        while (model.parent[j] > -1)
        {
            fh[i] = X_up[j].transpose() * fh[i]; // X_up(i) = i_X_lenda(i)
            j = model.parent[j];
            H(i, j) = model.joint[j].S_body.transpose() * fh[i];
            H(j, i) = H(i, j);
        }
        fh[i] = X_up[j].transpose() * fh[i];
    }
    // Eigen::MatrixXd F;
    F.setZero(6, NB);
    for (int i = 0; i < NB;i++)
    {
        F.block(0, i, 6, 1) = fh[i];
    }
    Eigen::MatrixXd H_Flt;
    H_Flt.setZero(NB + 6, NB + 6);
    H_Flt.block(0, 0, 6, 6) = IC_base;
    H_Flt.block(6, 6, NB, NB) = H;
    H_Flt.block(0, 6, 6, NB) = F;
    H_Flt.block(6, 0, NB, 6) = F.transpose();
    // Eigen::MatrixXd H_fl;
    H_fl = H - F.transpose() * IC_base.inverse() * F;
    I_flbase = IC_base;
    return H_Flt;
}

// Calculate generalize bias force via Recursive Newton-Euler Method
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
//  qd[]: The robot's joint velocity array, which size is concided with NB,
//        mostly attain from state estimator or just differented by q
// Gra_offset: if true, then caluate bias force contain the gravity term; if false, vice verse
Eigen::MatrixXd Cal_Generalize_Bias_force(Model &model,
                                          double _q[],
                                          double _qd[],
                                          bool Gra_offset,
                                          Eigen::MatrixXd &v_out,
                                          Eigen::MatrixXd &a_out)
{
    Eigen::MatrixXd Bias_force;
    int NB = model.NB;
    double *qdd;
    qdd = (double *)malloc(sizeof(double) * NB);
    for (int i = 0; i < NB;i++)
    {
        qdd[i] = 0;
    }
    Bias_force = inverse_dynamic(model, _q, _qd, qdd, Gra_offset, v_out, a_out);
    return Bias_force;
}

// Calculate generalize bias force via Recursive Newton-Euler Method
// model: The robot model which must been created
// q_fltbase[]: The robot's floating base orientation and position (dimension :7) representing at body coordinate
// v_fltbase[]: The robot's floating base twist(dimension :6) representing at body coordinate
//   q[]: The robot's joint value array, which size is concide with NB
//  qd[]: The robot's joint velocity array, which size is concided with NB,
//        mostly attain from state estimator or just differented by q
// Gra_offset: if true, then caluate bias force contain the gravity term; if false, vice verse
Eigen::MatrixXd Cal_Generalize_Bias_force_Flt(Model &model,
                                              double q_fltbase[],
                                              double v_fltbase[],
                                              double _q[],
                                              double _qd[],
                                              bool Gra_offset,
                                              Eigen::MatrixXd &v_out,
                                              Eigen::MatrixXd &a_out)
{
    int NB = model.NB;
    Eigen::Matrix<double, 6, 6> *X_up;
    Eigen::Matrix<double, 6, 6> *X_q;
    Eigen::Matrix<double, 6, 1> *v;
    Eigen::Matrix<double, 6, 1> *a;
    Eigen::Matrix<double, 6, 1> *f;
    Eigen::Matrix<double, 6, 1> g;
    Eigen::Matrix<double, 6, 1> f0;
    // MatrixXd q = Eigen::Map<MatrixXd>(_q, NB, 1);
    MatrixXd qd = Eigen::Map<MatrixXd>(_qd, NB, 1);
    Eigen::MatrixXd C = Eigen::Map<MatrixXd>(_q, NB, 1); // torque is the same size of vector(_q)
    C.setZero();
    X_up = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
    Eigen::Matrix<double, 6, 1> vJ;                                                      // joint velocity, satisfying vJ = S(j)*qd and v(i) = v(i-1) + vJ
    v = (Eigen::Matrix<double, 6, 1> *)malloc(sizeof(Eigen::Matrix<double, 6, 1>) * NB); // space velocity of each body
    a = (Eigen::Matrix<double, 6, 1> *)malloc(sizeof(Eigen::Matrix<double, 6, 1>) * NB); // space acceleration of each body
    f = (Eigen::Matrix<double, 6, 1> *)malloc(sizeof(Eigen::Matrix<double, 6, 1>) * NB); // space force of each body
    a_out.setZero(6, NB);
    v_out.setZero(6, NB);
    // Update Robot state, X
    model.update_model(model, _q, q_fltbase);

    if (Gra_offset)
        g << 0, 0, 0, 0, 0, -9.81;
    else
        g << 0, 0, 0, 0, 0, 0;

    // 在floating base坐标系下表示g
    Eigen::Matrix<double, 6, 6> X_up_ref;
    Eigen::Matrix3d R = model.T_Flt.block(0, 0, 3, 3).transpose();
    Eigen::Vector3d p = -R * model.T_Flt.block(0, 3, 3, 1);
    AdjointT(R, p, X_up_ref);
    g = X_up_ref * g;
    // 在floating base坐标系下表示v_base
    MatrixXd v_base = Eigen::Map<MatrixXd>(v_fltbase, 6, 1);
    v_base = X_up_ref * v_base;

    // forward pass, velocity and acceleration propagation
    for (int i = 0; i < model.NB; i++)
    {
        vJ = model.joint[i].S_body * qd(i);
        Eigen::Matrix3d R = model.Ttree[i].block(0, 0, 3, 3).transpose();
        Eigen::Vector3d p = -R * model.Ttree[i].block(0, 3, 3, 1);
        AdjointT(R, p, X_up[i]); // X_up(i) = i_X_lenda(i), i_S_parent(i) = i_X_parent(i) * parent(i)_S_parent(i)
        if (model.parent[i] == -1)
        {
            v[i] = X_up[i] * v_base + vJ;
            a_out.block(0, i, 6, 1) = crm(v[i]) * vJ;
            a[i] = X_up[i] * (-g) + crm(v[i]) * vJ;
        }
        else
        {
            v[i] = X_up[i] * v[model.parent[i]] + vJ;
            a_out.block(0, i, 6, 1) = crm(v[i]) * vJ;
            a[i] = X_up[i] * a[model.parent[i]] + crm(v[i]) * vJ;
        }
        f[i] = model.I[i] * a[i] + crf(v[i]) * model.I[i] * v[i];
        v_out.block(0, i, 6, 1) = v[i];
    }
    f0 = model.I_base * (-g) + crf(v_base) * model.I_base * v_base;
    for (int i = model.NB - 1; i >= 0; --i)
    {
        C(i) = model.joint[i].S_body.transpose() * f[i];

        if (model.parent[i] != -1)
        {
            f[model.parent[i]] = f[model.parent[i]] + X_up[i].transpose() * f[i];
        }
        else // if parent is floating base
        {
            f0 = f0 + X_up[i].transpose() * f[i];
        }
    }

    Eigen::MatrixXd C_Flt;
    C_Flt.setZero(NB + 6, 1);
    C_Flt.block(0, 0, 6, 1) = f0;
    C_Flt.block(6, 0, NB, 1) = C;
    return C_Flt;
}

// Calculate gravity term via Recursive Newton-Euler Method
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
Eigen::MatrixXd Cal_Gravity_Term(Model &model,double _q[])
{
    Eigen::MatrixXd Gravity_Term;
    Eigen::MatrixXd v;
    Eigen::MatrixXd a;
    int NB = model.NB;
    double *qdd, *qd;
    qdd = (double *)malloc(sizeof(double) * NB);
    qd = (double *)malloc(sizeof(double) * NB);
    for (int i = 0; i < NB; i++)
    {
        qdd[i] = 0;
        qd[i] = 0;
    }
    Gravity_Term = inverse_dynamic(model, _q, qd, qdd, true,v,a);
    return Gravity_Term;
}

// Ajoint operation: calculate generalize transformation from rotation matrix and distance vector
void AdjointT(Eigen::Matrix3d R, Eigen::Vector3d p, Eigen::Matrix<double, 6, 6> &X)
{
    Eigen::Matrix3d pR;
    Eigen::Matrix3d px;
    px << 0, -p(2), p(1),
        p(2), 0, -p(0),
        -p(1), p(0), 0;
    pR = px * R;
    X.setZero(6, 6);
    X.block(0, 0, 3, 3) = R;
    X.block(3, 3, 3, 3) = R;
    X.block(3, 0, 3, 3) = pR;
}

// Calculate Geometric jacobian matrix, dimensions: 6xNB, which can choose in space form or body form
// model: The robot model which must been created
//         ib: Choose which link the Jacobian revalent to
// Coordinate: Either be INERTIAL_COORDINATE or BODY_COORDINATE
//             if choose INERTIAL_COORDINATE, then will calculte space Jacobian
//             if choose BODY_COORDINATE, then will calculte body Jacobian
Eigen::MatrixXd Cal_Geometric_Jacobain(Model &model, int ib, bool Coordinate)
{
    int NB = model.NB;
    Eigen::MatrixXd J;
    J.setZero(6, NB); //initialize Jacobian matrix

    // initial k set
    int *k,*k_temp; 
    k_temp = (int *)malloc(sizeof(int) * NB);
    k = (int *)malloc(sizeof(int) * NB);
    int j = ib;
    int num = 0;
    while(j >= 0)
    {
        k_temp[num] = j;
        j = model.parent[j];
        num++;
    }
    for (int i = num-1; i >= 0; --i)
    {
        k[num - 1 - i] = k_temp[i];
    }
    // Jacobian of body ib repesent at inertial frame
    if (Coordinate == INERTIAL_COORDINATE) //Inertial numbered -1
    {
        Eigen::Matrix<double,6,6> X_down;
        X_down.setIdentity(6, 6);
        for (int i = 0; i < num;i++)
        {
            X_down = X_down * model.Xtree[k[i]];
            J.block(0, k[i], 6, 1) = X_down * model.joint[k[i]].S_body;
        }
    }
    // Jacobian of body ib repesent at body ib coordinate
    else if (Coordinate == BODY_COORDINATE)
    {
        Eigen::Matrix<double, 6, 6> X_up;
        X_up.setIdentity(6, 6);
        J.block(0, k[num - 1], 6, 1) = model.joint[k[num - 1]].S_body;
        for (int i = num - 1; i > 0; --i)
        {
            X_up = X_up * model.X_uptree[i];
            J.block(0, k[i-1], 6, 1) = X_up * model.joint[k[i-1]].S_body;
        }
    }
    return J;
}

// Calculate matrix K which depict the constrian of the model
// model: The robot model which must been created
//     v: each moving link's spatial velocity, size(v) = [6,NB]
//   avp: each moving velocity product acceleration, size(avp) = [6,NB]
//     k: Matrix k is input, and satisfy K*qdd = k
Eigen::MatrixXd Cal_K_Flt(Model &model,
                      Eigen::MatrixXd v,
                      Eigen::MatrixXd avp,
                      Eigen::MatrixXd &k)
{
    int NB = model.NB;
    int NL = model.NL;
    int *nc;
    nc = (int *)malloc(sizeof(int) * NL);
    MatrixXd K;
    int col_K = 0, row_K = 0;
    col_K = NB + 6;
    for (int i = 0; i < NL;i++)
    {
        nc[i] = model.loopjoint[i].T.cols();
    }
    for (int i = 0; i < NL; i++)
    {
        row_K += nc[i];
    }
    K.setZero(row_K, col_K);
    k.setZero(row_K, 1);
    int row_index = 0;
    for (int nl = 0; nl < NL; nl++)
    {
        Matrix<double, 6, 6> lp_X_s;
        Matrix3d R_t;
        R_t = model.loopjoint[nl].Ts.block(0, 0, 3, 3);
        Vector3d xyz = model.loopjoint[nl].Ts.block(0, 3, 3, 1);
        R_t.transposeInPlace();
        xyz = (-R_t) * xyz;
        AdjointT(R_t, xyz, lp_X_s);

        Matrix<double, 6, 6> lp_X_p;
        R_t = model.loopjoint[nl].Tp.block(0, 0, 3, 3);
        xyz = model.loopjoint[nl].Tp.block(0, 3, 3, 1);
        R_t.transposeInPlace();
        xyz = (-R_t) * xyz;
        AdjointT(R_t, xyz, lp_X_p);

        Matrix<double, 6, 1> vs, vp, as, ap;

        Matrix<double, 6, 6> lps_X_ref;
        Matrix<double, 6, 6> lpp_X_ref; 
        Matrix<double, 6, 6> ref_X_p;
        ref_X_p.setIdentity();
        Matrix<double, 6, 6> ref_X_s;
        ref_X_s.setIdentity();
        Matrix<double, 6, 6> p_X_ref;
        p_X_ref.setIdentity();
        Matrix<double, 6, 6> s_X_ref;
        s_X_ref.setIdentity();
        // initial k set
        int *ks_set, *ks_temp;
        ks_temp = (int *)malloc(sizeof(int) * NB);
        ks_set = (int *)malloc(sizeof(int) * NB);
        int j = model.loopjoint[nl].suc;
        int num_ks = 0;
        while (j >= 0)
        {
            ks_temp[num_ks] = j;
            j = model.parent[j];
            num_ks++;
        }
        for (int i = num_ks - 1; i >= 0; --i)
        {
            ks_set[num_ks - 1 - i] = ks_temp[i];
        }
        Eigen::Matrix<double, 6, 6> X_up;
        X_up.setIdentity(6, 6);
        for (int i = 0; i < num_ks; i++)
        {
            X_up = model.X_uptree[ks_set[i]] * X_up;
        }
        s_X_ref = X_up * model.X_upFlt;
        lps_X_ref = lp_X_s * s_X_ref;
        //------------------------------------------
        int *kp_set, *kp_temp;
        kp_temp = (int *)malloc(sizeof(int) * NB);
        kp_set = (int *)malloc(sizeof(int) * NB);
        j = model.loopjoint[nl].pre;
        int num_kp = 0;
        while (j >= 0)
        {
            kp_temp[num_kp] = j;
            j = model.parent[j];
            num_kp++;
        }
        for (int i = num_kp - 1; i >= 0; --i)
        {
            kp_set[num_kp - 1 - i] = kp_temp[i];
        }
        X_up.setIdentity(6, 6);
        for (int i = 0; i < num_kp; i++)
        {
            X_up = model.X_uptree[kp_set[i]] * X_up;
        }
        p_X_ref = X_up * model.X_upFlt;
        lpp_X_ref = lp_X_p * p_X_ref;

        // --------------------------------------
        if (model.loopjoint[nl].pre == WORLD)
        {
            vp.setZero(6, 1);
            ap.setZero(6, 1);
        }
        else
        {
            vp = ref_X_p * v.block(0, model.loopjoint[nl].pre, 6, 1);
            ap = ref_X_p * avp.block(0, model.loopjoint[nl].pre, 6, 1);
        }
        if (model.loopjoint[nl].suc == WORLD)
        {
            vs.setZero(6, 1);
            as.setZero(6, 1);
        }
        else
        {
            vs = ref_X_s * v.block(0, model.loopjoint[nl].suc, 6, 1);
            as = ref_X_s * avp.block(0, model.loopjoint[nl].suc, 6, 1);
        }

        MatrixXd T_ref = lps_X_ref.transpose() * model.loopjoint[nl].T;

        if(nl!=0)
            row_index += nc[nl - 1];

        k.block(row_index, 0, nc[nl], 1) = -T_ref.transpose() * (as - ap + crm(vs) * vp);
    
        Matrix<double, 6, 6> X_down;
        X_down.setIdentity(6, 6);
        for (int i = 0; i < num_ks; i++)
        {
            X_down = X_down * model.Xtree[ks_set[i]];
            K.block(row_index, 6 + ks_set[i], nc[nl], 1) = T_ref.transpose() * X_down * model.joint[ks_set[i]].S_body;  
        }
        for (int i = 0; i < num_kp; i++)
        {
            X_down = X_down * model.Xtree[kp_set[i]];
            K.block(row_index, 6 + kp_set[i], nc[nl], 1) = T_ref.transpose() * X_down * model.joint[kp_set[i]].S_body;
        }
        K.block(row_index, 0, nc[nl], 6) = T_ref.transpose() * model.X_Flt;
    }

    return K;
}

// Calculate matrix K which depict the constrian of the model
// model: The robot model which must been created
//     v: each moving link's spatial velocity, size(v) = [6,NB]
//   avp: each moving velocity product acceleration, size(avp) = [6,NB]
//     k: Matrix k is input, and satisfy K*qdd = k
Eigen::MatrixXd Cal_K(Model &model,
                          Eigen::MatrixXd v,
                          Eigen::MatrixXd avp,
                          Eigen::MatrixXd &k)
{
    int NB = model.NB;
    int NL = model.NL;
    int *nc;
    nc = (int *)malloc(sizeof(int) * NL);
    MatrixXd K;
    int col_K = 0, row_K = 0;
    col_K = NB + 6;
    for (int i = 0; i < NL; i++)
    {
        nc[i] = model.loopjoint[i].T.cols();
    }
    for (int i = 0; i < NL; i++)
    {
        row_K += nc[i];
    }
    K.setZero(row_K, col_K);
    k.setZero(row_K, 1);

    for (int nl = 0; nl < NL; nl++)
    {
        Matrix<double, 6, 6> lp_X_s;
        Matrix3d R_t;
        R_t = model.loopjoint[nl].Ts.block(0, 0, 3, 3);
        Vector3d xyz = model.loopjoint[nl].Ts.block(0, 3, 3, 1);
        R_t.transposeInPlace();
        xyz = (-R_t) * xyz;
        AdjointT(R_t, xyz, lp_X_s);

        Matrix<double, 6, 6> lp_X_p;
        R_t = model.loopjoint[nl].Tp.block(0, 0, 3, 3);
        xyz = model.loopjoint[nl].Tp.block(0, 3, 3, 1);
        R_t.transposeInPlace();
        xyz = (-R_t) * xyz;
        AdjointT(R_t, xyz, lp_X_p);

        Matrix<double, 6, 1> vs, vp, as, ap;

        Matrix<double, 6, 6> lps_X_ref;
        Matrix<double, 6, 6> lpp_X_ref;
        Matrix<double, 6, 6> ref_X_p;
        ref_X_p.setIdentity();
        Matrix<double, 6, 6> ref_X_s;
        ref_X_s.setIdentity();
        Matrix<double, 6, 6> p_X_ref;
        p_X_ref.setIdentity();
        Matrix<double, 6, 6> s_X_ref;
        s_X_ref.setIdentity();
        // initial k set
        int *ks_set, *ks_temp;
        ks_temp = (int *)malloc(sizeof(int) * NB);
        ks_set = (int *)malloc(sizeof(int) * NB);
        int j = model.loopjoint[nl].suc;
        int num_ks = 0;
        while (j >= 0)
        {
            ks_temp[num_ks] = j;
            j = model.parent[j];
            num_ks++;
        }
        for (int i = num_ks - 1; i >= 0; --i)
        {
            ks_set[num_ks - 1 - i] = ks_temp[i];
        }
        Eigen::Matrix<double, 6, 6> X_up;
        X_up.setIdentity(6, 6);
        for (int i = 0; i < num_ks; i++)
        {
            X_up = model.X_uptree[ks_set[i]] * X_up;
        }
        s_X_ref = X_up;
        lps_X_ref = lp_X_s * s_X_ref;
        //------------------------------------------
        int *kp_set, *kp_temp;
        kp_temp = (int *)malloc(sizeof(int) * NB);
        kp_set = (int *)malloc(sizeof(int) * NB);
        j = model.loopjoint[nl].pre;
        int num_kp = 0;
        while (j >= 0)
        {
            kp_temp[num_kp] = j;
            j = model.parent[j];
            num_kp++;
        }
        for (int i = num_kp - 1; i >= 0; --i)
        {
            kp_set[num_kp - 1 - i] = kp_temp[i];
        }
        X_up.setIdentity(6, 6);
        for (int i = 0; i < num_kp; i++)
        {
            X_up = model.X_uptree[kp_set[i]] * X_up;
        }
        p_X_ref = X_up;
        lpp_X_ref = lp_X_p * p_X_ref;

        // --------------------------------------
        if (model.loopjoint[nl].pre == WORLD)
        {
            vp.setZero(6, 1);
            avp.setZero(6, 1);
        }
        else
        {
            vp = ref_X_p * v.block(0, model.loopjoint[nl].pre, 6, 1);
            ap = ref_X_p * avp.block(0, model.loopjoint[nl].pre, 6, 1);
        }
        if (model.loopjoint[nl].suc == WORLD)
        {
            vs.setZero(6, 1);
            avp.setZero(6, 1);
        }
        else
        {
            vs = ref_X_s * v.block(0, model.loopjoint[nl].suc, 6, 1);
            as = ref_X_s * avp.block(0, model.loopjoint[nl].suc, 6, 1);
        }

        MatrixXd T_ref = lps_X_ref.transpose() * model.loopjoint[nl].T;
        if (nl == 0)
        {
            k.block(0, 0, nc[nl], 1) = -T_ref.transpose() * (as - ap + crm(vs) * vp);
        }
        else
        {
            k.block(nc[nl - 1], 0, nc[nl], 1) = -T_ref.transpose() * (as - ap + crm(vs) * vp);
        }

        Matrix<double, 6, 6> X_down;
        X_down.setIdentity(6, 6);
        for (int i = 0; i < num_ks; i++)
        {
            X_down = X_down * model.Xtree[ks_set[i]];
            if (nl == 0)
            {
                K.block(0, 6 + ks_set[i], nc[nl], 1) = T_ref.transpose() * X_down * model.joint[ks_set[i]].S_body;
            }
            else
            {
                K.block(nc[nl - 1], 6 + ks_set[i], nc[nl], 1) = T_ref.transpose() * X_down * model.joint[ks_set[i]].S_body;
            }
        }
        for (int i = 0; i < num_kp; i++)
        {
            X_down = X_down * model.Xtree[kp_set[i]];
            if (nl == 0)
            {
                K.block(0, kp_set[i], nc[nl], 1) = T_ref.transpose() * X_down * model.joint[kp_set[i]].S_body;
            }
            else
            {
                K.block(nc[nl - 1], kp_set[i], nc[nl], 1) = T_ref.transpose() * X_down * model.joint[kp_set[i]].S_body;
            }
        }
    }

    return K;
}
