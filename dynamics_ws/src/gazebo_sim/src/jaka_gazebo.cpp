#include "ros/ros.h"
#include "jaka_gazebo.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <iomanip>

using namespace std;

#define FR_hip   0
#define FR_thigh 1
#define FR_calf  2
#define FL_hip   3
#define FL_thigh 4
#define FL_calf  5
#define RR_hip   6
#define RR_thigh 7
#define RR_calf  8
#define RL_hip   9
#define RL_thigh 10
#define RL_calf  11

double pos[12], vel[12], tau[12];
double timer;

void JointPos_PD_controll(double q_cmd[], double qd_cmd[], double q_cur[], double qd_cur[], double tau[]);

void subscriberCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    // FR Leg
    pos[FR_hip] = msg->position[4];
    vel[FR_hip] = msg->velocity[4];
    tau[FR_hip] = msg->effort[4];
    pos[FR_thigh] = msg->position[5];
    vel[FR_thigh] = msg->velocity[5];
    tau[FR_thigh] = msg->effort[5];
    pos[FR_calf] = msg->position[3];
    vel[FR_calf] = msg->velocity[3];
    tau[FR_calf] = msg->effort[3];
    // FL Leg
    pos[FL_hip] = msg->position[1];
    vel[FL_hip] = msg->velocity[1];
    tau[FL_hip] = msg->effort[1];
    pos[FL_thigh] = msg->position[2];
    vel[FL_thigh] = msg->velocity[2];
    tau[FL_thigh] = msg->effort[2];
    pos[FL_calf] = msg->position[0];
    vel[FL_calf] = msg->velocity[0];
    tau[FL_calf] = msg->effort[0];
    // RR Leg
    pos[RR_hip] = msg->position[10];
    vel[RR_hip] = msg->velocity[10];
    tau[RR_hip] = msg->effort[10];
    pos[RR_thigh] = msg->position[11];
    vel[RR_thigh] = msg->velocity[11];
    tau[RR_thigh] = msg->effort[11];
    pos[RR_calf] = msg->position[9];
    vel[RR_calf] = msg->velocity[9];
    tau[RR_calf] = msg->effort[9];
    // RL Leg
    pos[RL_hip] = msg->position[7];
    vel[RL_hip] = msg->velocity[7];
    tau[RL_hip] = msg->effort[7];
    pos[RL_thigh] = msg->position[8];
    vel[RL_thigh] = msg->velocity[8];
    tau[RL_thigh] = msg->effort[8];
    pos[RL_calf] = msg->position[6];
    vel[RL_calf] = msg->velocity[6];
    tau[RL_calf] = msg->effort[6];
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "a1_control");
    ros::NodeHandle n;
    ros::Publisher tau1_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/FR_hip_controller_effort/command", 1);
    ros::Publisher tau2_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/FR_thigh_controller_effort/command", 1);
    ros::Publisher tau3_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/FR_calf_controller_effort/command", 1);
    ros::Publisher tau4_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/FL_hip_controller_effort/command", 1);
    ros::Publisher tau5_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/FL_thigh_controller_effort/command", 1);
    ros::Publisher tau6_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/FL_calf_controller_effort/command", 1);
    ros::Publisher tau7_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/RR_hip_controller_effort/command", 1);
    ros::Publisher tau8_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/RR_thigh_controller_effort/command", 1);
    ros::Publisher tau9_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/RR_calf_controller_effort/command", 1);
    ros::Publisher tau10_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/RL_hip_controller_effort/command", 1);
    ros::Publisher tau11_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/RL_thigh_controller_effort/command", 1);
    ros::Publisher tau12_cmd_publisher = n.advertise<std_msgs::Float64>("a1_gazebo/RL_calf_controller_effort/command", 1);
    ros::Subscriber subscriber = n.subscribe("/a1_gazebo/joint_states", 12, &subscriberCallback);
    ros::Rate r(500);
    Model a1(12,4);         // a1机器狗有12个运动连杆和4个闭环关节
    a1.build_a1(a1); // 设置a1机器狗的参数

    std_msgs::Float64 msg[12];

    double q[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double qd[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double qdd[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //desired acceleration

    //PD control
    double tau_PD[12];
    double q_cmd[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double qd_cmd[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    Eigen::Matrix<double, 12, 1> tau_ID, g;
    Eigen::MatrixXd Jaco;
    Eigen::Matrix<double, 12, 1> qdd_FD,tau_FD;
    qdd_FD << qdd[0], qdd[1], qdd[2],
        qdd[3], qdd[4], qdd[5],
        qdd[6], qdd[7], qdd[8],
        qdd[9], qdd[10], qdd[11];
    Eigen::Matrix<double, 12, 1> C;
    Eigen::Matrix<double, 18, 1> C_Flt;
    Eigen::MatrixXd H_RNEA, H_CRBA;
   
    a1.update_model(a1,q,nullptr);
    timer = 0;
    cout.precision(3);
    while (ros::ok())
    {
        timer++;
        // Update joint states
        for (int i = 0; i < 12; i++)
        {
            q[i] = pos[i]; //
            qd[i] = vel[i]; // vel[i]
            // cout << q[i]<<" ";
        }
        // cout << endl;
        //  inverse dynamics using Recusive Newton-Euler Method
        //  /*--normal test--*/
        //  tau_ID = inverse_dynamic(a1, q, qd, qdd, true);
        g = Cal_Gravity_Term(a1, q);
        //  // calculate generalize bias force(include gravity term)
        // C = Cal_Generalize_Bias_force(a1, q, qd, true);
        //  // calculate generalize inertial matrix
        //  H_RNEA = Cal_Generalize_Inertial_Matrix_RNEA(a1, q, qd, C);
        //  H_CRBA = Cal_Generalize_Inertial_Matrix_CRBA(a1, q);
        //  tau_FD = H_CRBA * qdd_FD + C; // anlysis form of dynamic equation
        //  /*--normal test--*/

        // /*--Floating base test--*/
        double q_Fltbase[7] = {0};
        double v_Fltbase[6] = {0};
        Eigen::Matrix3d R;
        Eigen::Quaterniond qua;
        R.setIdentity(3, 3);
        // R = rox(3.14159/2.0f).block(0, 0, 3, 3);
        qua = Eigen::Quaterniond(R);
        q_Fltbase[0] = qua.w();
        q_Fltbase[1] = qua.x();
        q_Fltbase[2] = qua.y();
        q_Fltbase[3] = qua.z();
        Eigen::MatrixXd v;// each link's spatial velocity
        Eigen::MatrixXd a; // each link's spatial accleration
        C_Flt = Cal_Generalize_Bias_force_Flt(a1, q_Fltbase, v_Fltbase, q, qd, true, v, a);
        // cout << "C_Flt: " << C_Flt.transpose() << endl;
        // cout << "C____: " << C.transpose() << endl;
        // cout << "p0: " << C_Flt.block(0, 0, 6, 1).transpose() << endl;
        MatrixXd H_Flt;
        MatrixXd H_fl;
        MatrixXd F;
        Matrix<double, 6, 6> I_flbase;
        H_Flt = Cal_Generalize_Inertial_Matrix_CRBA_Flt(a1, q, H_fl, F, I_flbase);
        Matrix<double, 12, 1> tau_IDFlt;
        Matrix<double, 12, 1> tau_CalFlt;
        Matrix<double, 18, 1> tau_CalFlt_big;
        Matrix<double, 6, 1> a0;
        Matrix<double, 18, 1> qdd_FD_Flt;

        Matrix<double, 12, 1> C_fl;
        C_fl = C_Flt.block(6, 0, 12, 1) - F.transpose() * I_flbase.inverse() * C_Flt.block(0,0,6,1);

        tau_IDFlt = inverse_dynamic_Flt(a1, q_Fltbase, v_Fltbase, q, qd, qdd, true , a0);
        qdd_FD_Flt.block(0, 0, 6, 1) = a0;
        qdd_FD_Flt.block(6, 0, 12, 1) = qdd_FD;

        tau_CalFlt = H_fl * qdd_FD + C_fl;
        tau_CalFlt_big = H_Flt * qdd_FD_Flt + C_Flt;
        // cout << " error: " << endl;
        // cout << I_flbase * a0 + C_Flt.block(0, 0, 6, 1) << endl;
        // cout << "C_Flt: " << endl
        //      << C_Flt.block(6,0,12,1).transpose() << endl;
        // cout << "tau_ID: "
        //      << tau_IDFlt.transpose() << endl;
        // cout << "a_base: " << a0.transpose() << endl
        //      << endl;
        // cout<< "tau_CalFlt: " << endl
        //     << tau_CalFlt.transpose() << endl;
        // cout << "tau_CalFltbig: " << endl
        //      << tau_CalFlt_big.transpose() << endl;
        // cout << "tau_error: " << endl
        //      << tau_CalFlt.transpose() - tau_IDFlt.transpose() << endl;

        // /*--Floating base test--*/

        /*-- contrains test --*/
        MatrixXd K,k;
        K = Cal_K_Flt(a1, v, a, k);

        cout << "K: " << endl
             << K << endl;
        cout << "k: " << endl
             << k.transpose() << endl;
        /*-- contrains test --*/

        // JointPos_PD_controll(q_cmd, qd_cmd, q, qd, tau_PD);

       

        // /*-- Jacobian Test --*/
        // Update Robot state, X
        // a1.update_model(a1, q,nullptr);
        // Jaco = Cal_Geometric_Jacobain(a1, 2, INERTIAL_COORDINATE);
        // cout << Jaco << endl<<endl;
        // Eigen::FullPivLU<Eigen::Matrix<double,6,6>> lu_decomp(Jaco);
        // cout << "rank is " <<lu_decomp.rank() << endl;
        // Eigen::EigenSolver<Eigen::MatrixXd> es(Jaco);
        // cout << "特征值为：" << endl;
        // cout << es.eigenvalues().real() << endl;
        // /*-- Jacobian Test --*/

        // send message
        for (int i = 0; i < 12; i++)
        {
            msg[i].data = g(i);
        }

        tau1_cmd_publisher.publish(msg[0]);
        tau2_cmd_publisher.publish(msg[1]);
        tau3_cmd_publisher.publish(msg[2]);
        tau4_cmd_publisher.publish(msg[3]);
        tau5_cmd_publisher.publish(msg[4]);
        tau6_cmd_publisher.publish(msg[5]);
        tau7_cmd_publisher.publish(msg[6]);
        tau8_cmd_publisher.publish(msg[7]);
        tau9_cmd_publisher.publish(msg[8]);
        tau10_cmd_publisher.publish(msg[9]);
        tau11_cmd_publisher.publish(msg[10]);
        tau12_cmd_publisher.publish(msg[11]);

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

void JointPos_PD_controll(double q_cmd[], double qd_cmd[], double q_cur[], double qd_cur[], double tau[])
{
    double Kp = 1.5f;
    double Kd = 0.0f;
    double Ki = 0.01f;
    static double SumError[6] = {0};
    //cout << "SumError: ";
    for (int i = 0; i < 6; i++)
    {
        SumError[i] += (q_cmd[i] - q_cur[i]);
        //cout << SumError[i] << " ";
    }
    //cout << endl;
    for (int i = 0; i < 6; i++)
    {
        tau[i] = Kp * (q_cmd[i] - q_cur[i]) + Kd * (qd_cmd[i] - qd_cur[i])+Ki*SumError[i];
    }
}

// int jaka(int argc, char *argv[])
// {

//     ros::init(argc, argv, "jaka_gazebo");
//     ros::NodeHandle n;
//     ros::Publisher tau1_cmd_publisher = n.advertise<std_msgs::Float64>("jaka_Robot/joint1_effort_controller/command", 1);
//     ros::Publisher tau2_cmd_publisher = n.advertise<std_msgs::Float64>("jaka_Robot/joint2_effort_controller/command", 1);
//     ros::Publisher tau3_cmd_publisher = n.advertise<std_msgs::Float64>("jaka_Robot/joint3_effort_controller/command", 1);
//     ros::Publisher tau4_cmd_publisher = n.advertise<std_msgs::Float64>("jaka_Robot/joint4_effort_controller/command", 1);
//     ros::Publisher tau5_cmd_publisher = n.advertise<std_msgs::Float64>("jaka_Robot/joint5_effort_controller/command", 1);
//     ros::Publisher tau6_cmd_publisher = n.advertise<std_msgs::Float64>("jaka_Robot/joint6_effort_controller/command", 1);
//     ros::Subscriber subscriber = n.subscribe("/jaka_Robot/joint_states", 6, &subscriberCallback);
//     ros::Rate r(500);
//     Model jaka(6);         // jaka机械臂有6个连杆
//     jaka.build_jaka(jaka); // 设置jaka机械臂的参数

//     std_msgs::Float64 msg[6];

//     double q[6] = {0, 0, 0, 0, 0, 0};
//     double qd[6] = {0, 0, 0, 0, 0, 0};
//     double qdd[6] = {0, 0, 0, 0, 0, 0};

//     // PD control
//     double tau_PD[6];
//     double q_cmd[6] = {1, 0, 0, 0, 0, 0};
//     double qd_cmd[6] = {0, 0, 0, 0, 0, 0};

//     Eigen::Matrix<double, 6, 1> tau_ID, g;
//     Eigen::MatrixXd Jaco;
//     Eigen::Matrix<double, 6, 1> qdd_FD, tau_FD;
//     qdd_FD << qdd[0], qdd[1], qdd[2], qdd[3], qdd[4], qdd[5];
//     Eigen::Matrix<double, 6, 1> C;
//     Eigen::MatrixXd H_RNEA, H_CRBA;
//     jaka.update_model(jaka, q);
//     timer = 0;
//     while (ros::ok())
//     {
//         timer++;
//         // Update joint states
//         for (int i = 0; i < 6; i++)
//         {
//             q[i] = pos[i];
//             qd[i] = vel[i];
//         }
//         // inverse dynamics using Recusive Newton-Euler Method
//         tau_ID = inverse_dynamic(jaka, q, qd, qdd, true);
//         g = Cal_Gravity_Term(jaka, q);
//         // calculate generalize bias force(include gravity term)
//         C = Cal_Generalize_Bias_force(jaka, q, qd, true);
//         // calculate generalize inertial matrix
//         H_RNEA = Cal_Generalize_Inertial_Matrix_RNEA(jaka, q, qd, C);
//         H_CRBA = Cal_Generalize_Inertial_Matrix_CRBA(jaka, q);
//         tau_FD = H_CRBA * qdd_FD + C; // anlysis form of dynamic equation

//         JointPos_PD_controll(q_cmd, qd_cmd, q, qd, tau_PD);

//         // send message

//         for (int i = 0; i < 6; i++)
//         {
//             msg[i].data = g(i);
//         }
//         // Update Robot state, X
//         // jaka.update_model(jaka, q);
//         // Jaco = Cal_Geometric_Jacobain(jaka, 5, BODY_COORDINATE);
//         // cout << Jaco << endl<<endl;
//         // Eigen::FullPivLU<Eigen::Matrix<double,6,6>> lu_decomp(Jaco);
//         // cout << "rank is " <<lu_decomp.rank() << endl;
//         // Eigen::EigenSolver<Eigen::MatrixXd> es(Jaco);
//         // cout << "特征值为：" << endl;
//         // cout << es.eigenvalues().real() << endl;

//         tau1_cmd_publisher.publish(msg[0]);
//         tau2_cmd_publisher.publish(msg[1]);
//         tau3_cmd_publisher.publish(msg[2]);
//         tau4_cmd_publisher.publish(msg[3]);
//         tau5_cmd_publisher.publish(msg[4]);
//         tau6_cmd_publisher.publish(msg[5]);

//         r.sleep();
//         ros::spinOnce();
//     }
//     return 0;
// }
