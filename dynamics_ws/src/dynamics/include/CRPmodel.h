#ifndef _MODEL_H_
#define _MODEL_H_

#include "body.h"
#include "joint.h"
#include "spatial.h"
#include <visualization_msgs/Marker.h>
#include <math.h>

#define WORLD -2

enum _basetype
{
    Fixed,
    Floating
};

struct _LoopJoint
{
    int suc;      //loop joint's successor
    int pre;      // loop joint's predessor
    Eigen::Matrix<double, 6, 6> Xp; // loop joint's frame represent to successor link's frame
    Eigen::Matrix<double, 6, 6> Xs; // loop joint's frame represent to predessor link's frame
    Eigen::MatrixXd T; // constrian force space represent at loop joint frame
};

//构建model类
//指定model的连杆数，且在开链系统中(可动)连杆数=关节数

class Model
{
public:
    Model(int Nb)
    {
        NB = Nb;
        link = (Body *)malloc(sizeof(Body) * NB);
        joint = (Joint *)malloc(sizeof(Joint) * NB);
        I = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
        parent = (int *)malloc(sizeof(int) * NB);
        Xtree = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
        Xj = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
        Xq = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
        Ttree = (Eigen::Matrix<double, 4, 4> *)malloc(sizeof(Eigen::Matrix<double, 4, 4>) * NB);
        Tj = (Eigen::Matrix<double, 4, 4> *)malloc(sizeof(Eigen::Matrix<double, 4, 4>) * NB);
        Tq = (Eigen::Matrix<double, 4, 4> *)malloc(sizeof(Eigen::Matrix<double, 4, 4>) * NB);

        for (int i = 0; i < NB; i++)
        {
            Xtree[i] = Eigen::Matrix<double, 6, 6>::Identity(6, 6);
            Xj[i] = Xtree[i];
            Xq[i] = Xtree[i];
            parent[i] = i;
            Ttree[i] = Eigen::Matrix<double, 4, 4>::Identity(4, 4);
            Tj[i] = Ttree[i];
            Tq[i] = Ttree[i];
        }
    }
    Model(int Nb, int Nl)
    {
        NB = Nb;
        NL = Nl;
        link = (Body *)malloc(sizeof(Body) * NB);
        joint = (Joint *)malloc(sizeof(Joint) * NB);
        I = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
        parent = (int *)malloc(sizeof(int) * NB);
        Xtree = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
        Xj = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
        Xq = (Eigen::Matrix<double, 6, 6> *)malloc(sizeof(Eigen::Matrix<double, 6, 6>) * NB);
        Ttree = (Eigen::Matrix<double, 4, 4> *)malloc(sizeof(Eigen::Matrix<double, 4, 4>) * NB);
        Tj = (Eigen::Matrix<double, 4, 4> *)malloc(sizeof(Eigen::Matrix<double, 4, 4>) * NB);
        Tq = (Eigen::Matrix<double, 4, 4> *)malloc(sizeof(Eigen::Matrix<double, 4, 4>) * NB);

        loopjoint = (_LoopJoint *)malloc(sizeof(_LoopJoint) * NL);

        for (int i = 0; i < NB; i++)
        {
            Xtree[i] = Eigen::Matrix<double, 6, 6>::Identity(6, 6);
            Xj[i] = Xtree[i];
            Xq[i] = Xtree[i];
            parent[i] = i;
            Ttree[i] = Eigen::Matrix<double, 4, 4>::Identity(4, 4);
            Tj[i] = Ttree[i];
            Tq[i] = Ttree[i];
        }

        for (int i = 0; i < NL;i++)
        {
            loopjoint[i].suc = -1;
            loopjoint[i].pre = -1;
            loopjoint[i].Xp.setIdentity(6, 6);
            loopjoint[i].Xs.setIdentity(6, 6);
        }
    }

    void update_model(Model &model, double q[], double q_flt[]);

    void build_jaka(Model &jaka);
    void build_a1(Model &a1);
    static void mcI_to_rbi(double m, Eigen::Vector3d com, Eigen::Matrix3d Ic, Eigen::Matrix<double,6,6> &rbi);

    int NB;
    _basetype BaseType;
    Body *link;
    Joint *joint;
    Eigen::Matrix<double,6,6> *I;
    int *parent;
    Eigen::Matrix<double, 6, 6> *Xtree; // body(i) coordinate respect to body(i-1) coordinate
    Eigen::Matrix<double, 6, 6> *Xj;    // joint(i) coordinate respect to body(i-1) coordinate
    Eigen::Matrix<double, 6, 6> *Xq;    // body(i) coordinte respect to joint(i) coordinte
    Eigen::Matrix<double, 4, 4> *Ttree; // body(i) coordinate respect to body(i-1) coordinate
    Eigen::Matrix<double, 4, 4> *Tj;    // joint(i) coordinate respect to body(i-1) coordinate
    Eigen::Matrix<double, 4, 4> *Tq;    // body(i) coordinte respect to joint(i) coordinte

    Body link_Flt;
    Joint joint_Flt;
    Eigen::Matrix<double, 6, 6> I_base;
    Eigen::Matrix<double, 6, 6> X_Flt;
    Eigen::Matrix<double, 6, 6> Xj_Flt;
    Eigen::Matrix<double, 6, 6> Xq_Flt;
    Eigen::Matrix<double, 4, 4> T_Flt;
    Eigen::Matrix<double, 4, 4> Tj_Flt;
    Eigen::Matrix<double, 4, 4> Tq_Flt;

    int NL;
    _LoopJoint *loopjoint;
};

#endif