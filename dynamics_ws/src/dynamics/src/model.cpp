#include "ros/ros.h"
#include "CRPmodel.h"
#include "dynamic.h"


using namespace std;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "model");
    ros::NodeHandle n;
    ros::Rate r(1);
    //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    //Model jaka(6);           // jaka机械臂有6个连杆
    //jaka.build_jaka(jaka); //设置jaka机械臂的参数

    //double q[6] = {1, 1, 1, 1, 1, 1};
    //jaka.move_joint(jaka, q);
    
    while (ros::ok())
    {
        //show_model_in_rviz(jaka, marker_pub);
        r.sleep();
    }

    return 0;
}



void Model::build_jaka(Model &jaka)
{
    // number the parent set
    for (int i = 0; i < jaka.NB; i++)
    {
        jaka.parent[i] = i;
    }
    for (int i = 0; i < jaka.NB; i++) // normalize parent set
    {
        jaka.parent[i] = jaka.parent[i] - 1;
    }

    //-- 描述jaka机械臂模型，关节处于初始值时连杆i的坐标系与关节i的坐标系重合
    Eigen::Vector3d xyz;
    Eigen::Matrix3d R;

    // Joint1
    jaka.joint[0].jtype = Rz;
    jaka.joint[0].Set_S_body(Rz);
    jaka.joint[0].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, 0.0645;
    jaka.Tj[0] = Rp2T(R, xyz);
    jaka.Tq[0] = roz(jaka.joint[0].value);
    jaka.Ttree[0] = jaka.Tj[0] * jaka.Tq[0];

    R = jaka.Ttree[0].block(0, 0, 3, 1);
    xyz = jaka.Ttree[0].block(0, 3, 3, 1);
    AdjointT(R, xyz, jaka.Xtree[0]);

    // Link1
    jaka.link[0].mass = 2.32744879445344;
    jaka.link[0].com << 0, 0.00217822178706876, -0.0149669059830168;
    jaka.link[0].Ic << 0.00458147343407386, 0, 0,
        0, 0.00454359720683469, 0,
        0, 0, 0.00363214903311472;
    

    // Joint2
    jaka.joint[1].jtype = Rz;
    jaka.joint[1].Set_S_body(Rz);
    jaka.joint[1].value = 0;
    R << 1, 0, 0,
        0, 0, -1,
        0, 1, 0;
    xyz << 0, 0, 0;
    jaka.Tj[1] = Rp2T(R, xyz);
    jaka.Tq[1] = roz(jaka.joint[1].value);
    jaka.Ttree[1] = jaka.Tj[1] * jaka.Tq[1];
    R = jaka.Ttree[1].block(0, 0, 3, 1);
    xyz = jaka.Ttree[1].block(0, 3, 3, 1);
    AdjointT(R, xyz, jaka.Xtree[1]);

    // Link2
    jaka.link[1].mass = 5.95378231056391;
    jaka.link[1].com << 0.119793596970604, 0, -0.102397627385994;
    jaka.link[1].Ic << 0.0106282836293239, 0, -0.000911073241596094,
        0, 0.0804164638556798, 0,
        -0.000911073241596094, 0, 0.0779892876387226;

    // Joint3
    jaka.joint[2].jtype = Rz;
    jaka.joint[2].Set_S_body(Rz);
    jaka.joint[2].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0.246, 0, 0;
    jaka.Tj[2] = Rp2T(R, xyz);
    jaka.Tq[2] = roz(jaka.joint[2].value);
    jaka.Ttree[2] = jaka.Tj[2] * jaka.Tq[2];
    R = jaka.Ttree[2].block(0, 0, 3, 1);
    xyz = jaka.Ttree[2].block(0, 3, 3, 1);
    AdjointT(R, xyz, jaka.Xtree[2]);
    // Link3
    jaka.link[2].mass = 2.99370623962181;
    jaka.link[2].com << 0.129240666536215, 0, -0.014357405937694;
    jaka.link[2].Ic << 0.00375122418749307, 0, 0.00143595144649744,
        0, 0.0304577469052334, 0,
        0.00143595144649744, 0, 0.0293178341594524;

    // Joint4
    jaka.joint[3].jtype = Rz;
    jaka.joint[3].Set_S_body(Rz);
    jaka.joint[3].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0.228, 0, -0.113;
    jaka.Tj[3] = Rp2T(R, xyz);
    jaka.Tq[3] = roz(jaka.joint[3].value);
    jaka.Ttree[3] = jaka.Tj[3] * jaka.Tq[3];
    R = jaka.Ttree[3].block(0, 0, 3, 1);
    xyz = jaka.Ttree[3].block(0, 3, 3, 1);
    AdjointT(R, xyz, jaka.Xtree[3]);
    // Link4
    jaka.link[3].mass = 1.16925259073626;
    jaka.link[3].com << 0.000241526484683341, -0.0325144102286034, 0.00192585885719938;
    jaka.link[3].Ic << 0.00173057761809281, 0, 0,
        0, 0.000991339747095069, 0,
        0.00143595144649744, 0, 0.00170641007481186;

    // Joint5
    jaka.joint[4].jtype = Rz;
    jaka.joint[4].Set_S_body(Rz);
    jaka.joint[4].value = 0;
    R << 1, 0, 0,
        0, 0, -1,
        0, 1, 0;
    xyz << 0, -0.1175, 0;
    jaka.Tj[4] = Rp2T(R, xyz);
    jaka.Tq[4] = roz(jaka.joint[4].value);
    jaka.Ttree[4] = jaka.Tj[4] * jaka.Tq[4];
    R = jaka.Ttree[4].block(0, 0, 3, 1);
    xyz = jaka.Ttree[4].block(0, 3, 3, 1);
    AdjointT(R, xyz, jaka.Xtree[4]);
    // Link5
    jaka.link[4].mass = 1.30755746948789;
    jaka.link[4].com << 0.000197046850542781, 0.0226924862530863, -0.00183338030328768;
    jaka.link[4].Ic << 0.00285036503364272, 0, 0,
        0, 0.00109173403124772, 0,
        0, 0, 0.00282378446902515;

    // Joint6
    jaka.joint[5].jtype = Rz;
    jaka.joint[5].Set_S_body(Rz);
    jaka.joint[5].value = 0;
    R << 1, 0, 0,
        0, 0, 1,
        0, -1, 0;
    xyz << 0, 0.105, 0;
    jaka.Tj[5] = Rp2T(R, xyz);
    jaka.Tq[5] = roz(jaka.joint[5].value);
    jaka.Ttree[5] = jaka.Tj[5] * jaka.Tq[5];
    R = jaka.Ttree[5].block(0, 0, 3, 1);
    xyz = jaka.Ttree[5].block(0, 3, 3, 1);
    AdjointT(R, xyz, jaka.Xtree[5]);

    // Link6
    jaka.link[5].mass = 0.507738406221693;
    jaka.link[5].com << -0.000117639495146871, 0, -0.0132366598443958;
    jaka.link[5].Ic << 0.000208846698087936, 0, 0,
        0, 0.000211905205869407, 0,
        0, 0, 0.000360986690973954;

    cout << endl
         << "Model building completed! NB = " << jaka.NB << endl;
    cout << "The parent set is [ ";
    for (int i = 0; i < jaka.NB; i++)
        cout << jaka.parent[i] << " ";
    cout << "]" << endl;

    for (int i = 0; i < jaka.NB; i++)
    {
        mcI_to_rbi(jaka.link[i].mass, jaka.link[i].com, jaka.link[i].Ic, jaka.I[i]);
    }
}

void Model::mcI_to_rbi(double m, Eigen::Vector3d com, Eigen::Matrix3d Ic, Eigen::Matrix<double,6,6> &rbi)
{
    Matrix3d C;
    Matrix3d Ident;
    Ident.setIdentity(3,3);
    C << 0, -com(2), com(1),
        com(2), 0, -com(0),
        -com(1), com(0), 0;

    rbi.block(0, 0, 3, 3) = Ic + m * C * C.transpose();
    rbi.block(0, 3, 3, 3) = m * C;
    rbi.block(3, 0, 3, 3) = m * C.transpose();
    rbi.block(3, 3, 3, 3) = m * Ident;
}

void Model::update_model(Model &model, double q[], double q_flt[])
{
    if (q_flt != nullptr)
    {
        for (int i = 0; i < 7; i++)
        {
            model.joint_Flt.valueFlt[i] = q_flt[i];
        }

        model.Tq_Flt = Flt_Transform(model.joint_Flt.valueFlt);
        model.T_Flt = model.Tj_Flt * model.Tq_Flt;
        Eigen::Matrix3d R;
        Eigen::Vector3d xyz;
        R = model.T_Flt.block(0, 0, 3, 3);
        xyz = model.T_Flt.block(0, 3, 3, 1);
        AdjointT(R, xyz, model.X_Flt);
    }
    for (int i = 0; i < model.NB; i++)
    {
        model.joint[i].value = q[i];
        if (model.joint[i].jtype == Rz)
        {
            model.Xq[i] = rotz(model.joint[i].value);
            model.Tq[i] = roz(model.joint[i].value);
        }
        else if (model.joint[i].jtype == Rx)
        {
            model.Xq[i] = rotx(model.joint[i].value);
            model.Tq[i] = rox(model.joint[i].value);
        }
        else if (model.joint[i].jtype == Ry)
        {
            model.Xq[i] = roty(model.joint[i].value);
            model.Tq[i] = roy(model.joint[i].value);
        }

        model.Xtree[i] = model.Xj[i] * model.Xq[i];
        model.Ttree[i] = model.Tj[i] * model.Tq[i];
        Eigen::Matrix3d R;
        Eigen::Vector3d xyz;
        R = model.Ttree[i].block(0, 0, 3, 3);
        xyz = model.Ttree[i].block(0, 3, 3, 1);
        AdjointT(R, xyz, model.Xtree[i]);

        // std::cout << i << ": " << endl
        //           << model.Ttree[i] << endl;
    }
}

void Model::build_a1(Model &a1)
{
    a1.BaseType = Floating;
    int parentset[12] = {-1, 0, 1, -1, 3, 4, -1, 6, 7, -1, 9, 10};
    // number the parent set
    for (int i = 0; i < a1.NB; i++)
    {
        a1.parent[i] = parentset[i];
    }

    //-- 描述A1机械狗模型，关节处于初始值时连杆i的坐标系与关节i的坐标系重合
    Eigen::Vector3d xyz;
    Eigen::Matrix3d R;

    //Joint_floating
    a1.joint_Flt.jtype = Flt; //Floating Joint
    a1.joint_Flt.Set_S_body(Flt);
    for (int i = 0; i < 7;i++)
        a1.joint_Flt.valueFlt[i] = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, 0;
    a1.Tj_Flt = Rp2T(R, xyz);
    a1.Tq_Flt = Flt_Transform(a1.joint_Flt.valueFlt);
    a1.T_Flt = a1.Tj_Flt * a1.Tq_Flt;

    R = a1.T_Flt.block(0, 0, 3, 3);
    xyz = a1.T_Flt.block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.X_Flt);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_upFlt); 

    // Link_floating
    a1.link_Flt.mass = 6.0;
    a1.link_Flt.com << 0, 0.0041, -0.0005;
    a1.link_Flt.Ic << 0.0158533, 0, 0,
        0, 0.0377999, 0,
        0, 0, 0.0456542;
    // ------------------------------------------------
    // Joint0 FR_hip
    a1.joint[0].jtype = Rx;
    a1.joint[0].Set_S_body(Rx);
    a1.joint[0].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0.1805, -0.047, 0;
    a1.Tj[0] = Rp2T(R, xyz);
    a1.Tq[0] = roz(a1.joint[0].value);
    a1.Ttree[0] = a1.Tj[0] * a1.Tq[0];

    R = a1.Ttree[0].block(0, 0, 3, 3);
    xyz = a1.Ttree[0].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[0]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[0]);

    // Link0
    a1.link[0].mass = 0.696;
    a1.link[0].com << -0.003311, -0.000635, 3.1e-05;
    a1.link[0].Ic << 0.000469246, 9.409e-06, -3.42e-07,
        9.409e-06, 0.00080749, 4.66e-07,
        -3.42e-07, 4.66e-07, 0.000552929;
    // ------------------------------------------------
    // Joint1 FR_thigh
    a1.joint[1].jtype = Ry;
    a1.joint[1].Set_S_body(Ry);
    a1.joint[1].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, -0.0838, 0;
    a1.Tj[1] = Rp2T(R, xyz);
    a1.Tq[1] = roz(a1.joint[1].value);
    a1.Ttree[1] = a1.Tj[1] * a1.Tq[1];

    R = a1.Ttree[1].block(0, 0, 3, 3);
    xyz = a1.Ttree[1].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[1]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[1]);

    // Link1
    a1.link[1].mass = 1.013;
    a1.link[1].com << -0.003237, 0.022327, -0.027326;
    a1.link[1].Ic << 0.005529065, -4.825e-06, 0.000343869,
        -4.825e-06, 0.005139339, -2.2448e-05,
        0.000343869, -2.2448e-05, 0.001367788;

    // ------------------------------------------------
    // Joint2 FR_calf
    a1.joint[2].jtype = Ry;
    a1.joint[2].Set_S_body(Ry);
    a1.joint[2].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, -0.2;
    a1.Tj[2] = Rp2T(R, xyz);
    a1.Tq[2] = roz(a1.joint[2].value);
    a1.Ttree[2] = a1.Tj[2] * a1.Tq[2];

    R = a1.Ttree[2].block(0, 0, 3, 3);
    xyz = a1.Ttree[2].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[2]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[2]);

    // Link2
    a1.link[2].mass = 0.226;
    a1.link[2].com << 0.006435, 0.0, -0.107388;
    a1.link[2].Ic << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    // ------------------------------------------------
    // Joint3 FL_hip
    a1.joint[3].jtype = Rx;
    a1.joint[3].Set_S_body(Rx);
    a1.joint[3].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0.1805, 0.047, 0;
    a1.Tj[3] = Rp2T(R, xyz);
    a1.Tq[3] = roz(a1.joint[3].value);
    a1.Ttree[3] = a1.Tj[3] * a1.Tq[3];

    R = a1.Ttree[3].block(0, 0, 3, 3);
    xyz = a1.Ttree[3].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[3]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[3]);

    // Link3
    a1.link[3].mass = 0.696;
    a1.link[3].com << -0.003311, 0.000635, 3.1e-05;
    a1.link[3].Ic << 0.000469246, -9.409e-06, -3.42e-07,
        -9.409e-06, 0.00080749, -4.66e-07,
        -3.42e-07, -4.66e-07, 0.000552929;

    // ------------------------------------------------
    // Joint4 FL_thigh
    a1.joint[4].jtype = Ry;
    a1.joint[4].Set_S_body(Ry);
    a1.joint[4].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0.0838, 0;
    a1.Tj[4] = Rp2T(R, xyz);
    a1.Tq[4] = roz(a1.joint[4].value);
    a1.Ttree[4] = a1.Tj[4] * a1.Tq[4];

    R = a1.Ttree[4].block(0, 0, 3, 3);
    xyz = a1.Ttree[4].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[4]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[4]);

    // Link4
    a1.link[4].mass = 1.013;
    a1.link[4].com << -0.003237, -0.022327, -0.027326;
    a1.link[4].Ic << 0.005529065, 4.825e-06, 0.000343869,
        4.825e-06, 0.005139339, 2.2448e-05,
        0.000343869, 2.2448e-05, 0.001367788;

    // ------------------------------------------------
    // Joint5 FL_calf
    a1.joint[5].jtype = Ry;
    a1.joint[5].Set_S_body(Ry);
    a1.joint[5].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, -0.2;
    a1.Tj[5] = Rp2T(R, xyz);
    a1.Tq[5] = roz(a1.joint[5].value);
    a1.Ttree[5] = a1.Tj[5] * a1.Tq[5];

    R = a1.Ttree[5].block(0, 0, 3, 3);
    xyz = a1.Ttree[5].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[5]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[5]);

    // Link4
    a1.link[5].mass = 0.226;
    a1.link[5].com << 0.006435, 0.0, -0.107388;
    a1.link[5].Ic << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    // ------------------------------------------------
    // Joint6 RR_hip
    a1.joint[6].jtype = Rx;
    a1.joint[6].Set_S_body(Rx);
    a1.joint[6].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << -0.1805, -0.047, 0;
    a1.Tj[6] = Rp2T(R, xyz);
    a1.Tq[6] = roz(a1.joint[6].value);
    a1.Ttree[6] = a1.Tj[6] * a1.Tq[6];

    R = a1.Ttree[6].block(0, 0, 3, 3);
    xyz = a1.Ttree[6].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[6]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[6]);

    // Link6
    a1.link[6].mass = 0.696;
    a1.link[6].com << 0.003311, -0.000635, 3.1e-05;
    a1.link[6].Ic << 0.000469246, -9.409e-06, 3.42e-07,
        -9.409e-06, 0.00080749, 4.66e-07,
        3.42e-07, 4.66e-07, 0.000552929;

    // ------------------------------------------------
    // Joint7 RR_thigh
    a1.joint[7].jtype = Ry;
    a1.joint[7].Set_S_body(Ry);
    a1.joint[7].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, -0.0838, 0;
    a1.Tj[7] = Rp2T(R, xyz);
    a1.Tq[7] = roz(a1.joint[7].value);
    a1.Ttree[7] = a1.Tj[7] * a1.Tq[7];

    R = a1.Ttree[7].block(0, 0, 3, 3);
    xyz = a1.Ttree[7].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[7]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[7]);

    // Link7
    a1.link[7].mass = 1.013;
    a1.link[7].com << -0.003237, 0.022327, -0.027326;
    a1.link[7].Ic << 0.005529065, -4.825e-06, 0.000343869,
        -4.825e-06, 0.005139339, -2.2448e-05,
        0.000343869, -2.2448e-05, 0.001367788;

    // ------------------------------------------------
    // Joint8 RR_calf
    a1.joint[8].jtype = Ry;
    a1.joint[8].Set_S_body(Ry);
    a1.joint[8].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, -0.2;
    a1.Tj[8] = Rp2T(R, xyz);
    a1.Tq[8] = roz(a1.joint[8].value);
    a1.Ttree[8] = a1.Tj[8] * a1.Tq[8];

    R = a1.Ttree[8].block(0, 0, 3, 3);
    xyz = a1.Ttree[8].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[8]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[8]);

    // Link8
    a1.link[8].mass = 0.226;
    a1.link[8].com << 0.006435, 0.0, -0.107388;
    a1.link[8].Ic << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    // ------------------------------------------------
    // Joint9 RL_hip
    a1.joint[9].jtype = Rx;
    a1.joint[9].Set_S_body(Rx);
    a1.joint[9].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << -0.1805, 0.047, 0;
    a1.Tj[9] = Rp2T(R, xyz);
    a1.Tq[9] = roz(a1.joint[9].value);
    a1.Ttree[9] = a1.Tj[9] * a1.Tq[9];

    R = a1.Ttree[9].block(0, 0, 3, 3);
    xyz = a1.Ttree[9].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[9]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[9]);

    // Link9
    a1.link[9].mass = 0.696;
    a1.link[9].com << 0.003311, 0.000635, 3.1e-05;
    a1.link[9].Ic << 0.000469246, 9.409e-06, 3.42e-07,
        9.409e-06, 0.00080749, -4.66e-07,
        3.42e-07, -4.66e-07, 0.000552929;

    // ------------------------------------------------
    // Joint10 RL_thigh
    a1.joint[10].jtype = Ry;
    a1.joint[10].Set_S_body(Ry);
    a1.joint[10].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0.0838, 0;
    a1.Tj[10] = Rp2T(R, xyz);
    a1.Tq[10] = roz(a1.joint[10].value);
    a1.Ttree[10] = a1.Tj[10] * a1.Tq[10];

    R = a1.Ttree[10].block(0, 0, 3, 3);
    xyz = a1.Ttree[10].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[10]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[10]);

    // Link10
    a1.link[10].mass = 1.013;
    a1.link[10].com << -0.003237, -0.022327, -0.027326;
    a1.link[10].Ic << 0.005529065, 4.825e-06, 0.000343869,
        4.825e-06, 0.005139339, 2.2448e-05,
        0.000343869, 2.2448e-05, 0.001367788;

    // ------------------------------------------------
    // Joint11 RL_calf
    a1.joint[11].jtype = Ry;
    a1.joint[11].Set_S_body(Ry);
    a1.joint[11].value = 0;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, -0.2;
    a1.Tj[11] = Rp2T(R, xyz);
    a1.Tq[11] = roz(a1.joint[11].value);
    a1.Ttree[11] = a1.Tj[11] * a1.Tq[11];
    

    R = a1.Ttree[11].block(0, 0, 3, 3);
    xyz = a1.Ttree[11].block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.Xtree[11]);
    R = R.transpose();
    xyz = -R * xyz;
    AdjointT(R, xyz, a1.X_uptree[11]);

    // Link11
    a1.link[11].mass = 0.226;
    a1.link[11].com << 0.006435, 0.0, -0.107388;
    a1.link[11].Ic << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    for (int i = 0; i < a1.NB; i++)
    {
        mcI_to_rbi(a1.link[i].mass, a1.link[i].com, a1.link[i].Ic, a1.I[i]);
    }
    mcI_to_rbi(a1.link_Flt.mass, a1.link_Flt.com, a1.link_Flt.Ic, a1.I_base);

    // Setting loop joint 0
    Matrix4d Ts,Tp;
    a1.loopjoint[0].pre = WORLD;
    a1.loopjoint[0].suc = 2;
    a1.loopjoint[0].T.setZero(6, 3);
    a1.loopjoint[0].T.block(3, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, -0.2;
    Ts = Rp2T(R, xyz);
    R = Ts.block(0, 0, 3, 3);
    xyz = Ts.block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.loopjoint[0].Xs);
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, 0;
    Tp = Rp2T(R, xyz);
    R = Tp.block(0, 0, 3, 3);
    xyz = Tp.block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.loopjoint[0].Xp);

    // Setting loop joint 1
    a1.loopjoint[1].pre = WORLD;
    a1.loopjoint[1].suc = 5;
    a1.loopjoint[1].T.setZero(6, 3);
    a1.loopjoint[1].T.block(3, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, -0.2;
    Ts = Rp2T(R, xyz);
    R = Ts.block(0, 0, 3, 3);
    xyz = Ts.block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.loopjoint[1].Xs);
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, 0;
    Tp = Rp2T(R, xyz);
    R = Tp.block(0, 0, 3, 3);
    xyz = Tp.block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.loopjoint[1].Xp);

    // Setting loop joint 2
    a1.loopjoint[2].pre = WORLD;
    a1.loopjoint[2].suc = 8;
    a1.loopjoint[2].T.setZero(6, 3);
    a1.loopjoint[2].T.block(3, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, -0.2;
    Ts = Rp2T(R, xyz);
    R = Ts.block(0, 0, 3, 3);
    xyz = Ts.block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.loopjoint[2].Xs);
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, 0;
    Tp = Rp2T(R, xyz);
    R = Tp.block(0, 0, 3, 3);
    xyz = Tp.block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.loopjoint[2].Xp);

    // Setting loop joint 3
    a1.loopjoint[3].pre = WORLD;
    a1.loopjoint[3].suc = 11;
    a1.loopjoint[3].T.setZero(6, 3);
    a1.loopjoint[3].T.block(3, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, -0.2;
    Ts = Rp2T(R, xyz);
    R = Ts.block(0, 0, 3, 3);
    xyz = Ts.block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.loopjoint[3].Xs);
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz << 0, 0, 0;
    Tp = Rp2T(R, xyz);
    R = Tp.block(0, 0, 3, 3);
    xyz = Tp.block(0, 3, 3, 1);
    AdjointT(R, xyz, a1.loopjoint[3].Xp);

    cout
        << endl
        << "Model building completed! NB = " << a1.NB <<" NL = "<< a1.NL << endl;
    cout << "The parent set is [ ";
    for (int i = 0; i < a1.NB; i++)
        cout << a1.parent[i] << " ";
    cout << "]" << endl;
}

    // void Model::move_joint(Model &model, double q[])
    // {
    //     //cout << "size: " << sizeof(q[0]) << endl;

    //     for (int i = 0; i < model.NB; i++)
    //     {
    //         model.joint[i].value = q[i];
    //         //cout << "joint " << i + 1 << " :" << model.joint[i].value << endl;
    //     }
    // }
