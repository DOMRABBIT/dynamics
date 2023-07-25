#ifndef _DYNAMIC_H_
#define _DYNAMIC_H_

#include "CRPmodel.h"

#define BODY_COORDINATE true
#define INERTIAL_COORDINATE false

Eigen::MatrixXd inverse_dynamic(Model &model,
                                double _q[],
                                double _qd[],
                                double _qdd[],
                                bool Gra_offset,
                                Eigen::MatrixXd &v_out,
                                Eigen::MatrixXd &a_out);

Eigen::MatrixXd inverse_dynamic_Flt(Model &model,
                                    double _q_base[],
                                    double _qd_base[],
                                    double _q[],
                                    double _qd[],
                                    double _qdd[],
                                    bool Gra_offset,
                                    Eigen::Matrix<double, 6, 1> &a_base);

Eigen::MatrixXd Cal_Generalize_Inertial_Matrix_RNEA(Model &model,
                                                    double _q[],
                                                    double _qd[],
                                                    Eigen::MatrixXd Bias_force);

Eigen::MatrixXd Cal_Generalize_Inertial_Matrix_CRBA(Model &model,
                                                    double _q[]);

Eigen::MatrixXd Cal_Generalize_Inertial_Matrix_CRBA_Flt(Model &model,
                                                        double _q[],
                                                        MatrixXd &H_fl,
                                                        MatrixXd &F,
                                                        Matrix<double, 6, 6> &I_flbase);

Eigen::MatrixXd Cal_Generalize_Bias_force(Model &model,
                                          double _q[],
                                          double _qd[],
                                          bool Gra_offset,
                                          Eigen::MatrixXd &v_out,
                                          Eigen::MatrixXd &a_out);

Eigen::MatrixXd Cal_Generalize_Bias_force_Flt(Model &model,
                                              double q_fltbase[],
                                              double v_fltbase[],
                                              double _q[],
                                              double _qd[],
                                              bool Gra_offset,
                                              Eigen::MatrixXd &v_out,
                                              Eigen::MatrixXd &a_out);

Eigen::MatrixXd Cal_Gravity_Term(Model &model, double _q[]);

void AdjointT(Eigen::Matrix3d R, Eigen::Vector3d p, Eigen::Matrix<double, 6, 6> &X);

Eigen::MatrixXd Cal_Geometric_Jacobain(Model &model, int ib, bool Coordinate);

Eigen::MatrixXd Cal_K_Flt(Model &model,
                          Eigen::MatrixXd v,
                          Eigen::MatrixXd avp,
                          Eigen::MatrixXd &k);

Eigen::MatrixXd Cal_K(Model &model,
                          Eigen::MatrixXd v,
                          Eigen::MatrixXd avp,
                          Eigen::MatrixXd &k);

#endif