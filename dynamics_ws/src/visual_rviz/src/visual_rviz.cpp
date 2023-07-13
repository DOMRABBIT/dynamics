#include "visual_rviz.h"

using namespace std;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "visualization");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    Model a1(12);           // a1有12个moving连杆
    a1.build_a1(a1); // 设置jaka机械臂的参数

    double q[12] = {0.5f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};
    a1.update_model(a1, q,nullptr);
    while (ros::ok())
    {
        show_model_in_rviz(a1, marker_pub);
        r.sleep();
    }

    return 0;
}

void rviz_link(Eigen::Matrix3d R,
               Eigen::Vector3d p,
               visualization_msgs::Marker &link,
               std::string ns,
               int id,
               std::string frame,
               ros::Publisher &marker_pub)
{
    Eigen::Quaterniond wxyz(R);
    wxyz.normalize();
    link.header.frame_id = frame;
    link.header.stamp = ros::Time::now();
    link.ns = ns;
    link.id = id;

    link.type = visualization_msgs::Marker::CYLINDER;
    link.action = visualization_msgs::Marker::ADD;

    link.pose.position.x = p(0, 0);
    link.pose.position.y = p(1, 0);
    link.pose.position.z = p(2, 0);
    link.pose.orientation.x = wxyz.x();
    link.pose.orientation.y = wxyz.y();
    link.pose.orientation.z = wxyz.z();
    link.pose.orientation.w = wxyz.w();

    link.scale.x = 0.05;
    link.scale.y = 0.05;
    link.scale.z = 0.05;

    if (id % 2 == 0)
    {
        link.color.r = 1.0f;
        link.color.g = 0.0f;
        link.color.b = 0.0f;
        link.color.a = 0.4;
    }
    else
    {
        link.color.r = 0.0f;
        link.color.g = 0.0f;
        link.color.b = 1.0f;
        link.color.a = 0.5;
    }

    link.lifetime = ros::Duration();
    marker_pub.publish(link);
}

void rviz_com(Eigen::Matrix3d R,
              Eigen::Vector3d p,
              visualization_msgs::Marker &com,
              std::string ns,
              int id,
              std::string frame,
              ros::Publisher &marker_pub)
{
    Eigen::Quaterniond wxyz(R);
    wxyz.normalize();
    com.header.frame_id = frame;
    com.header.stamp = ros::Time::now();
    com.ns = ns;
    com.id = id;

    com.type = visualization_msgs::Marker::SPHERE;
    com.action = visualization_msgs::Marker::ADD;

    com.pose.position.x = p(0, 0);
    com.pose.position.y = p(1, 0);
    com.pose.position.z = p(2, 0);
    com.pose.orientation.x = wxyz.x();
    com.pose.orientation.y = wxyz.y();
    com.pose.orientation.z = wxyz.z();
    com.pose.orientation.w = wxyz.w();

    com.scale.x = 0.02;
    com.scale.y = 0.02;
    com.scale.z = 0.02;

    if (id % 2 == 0)
    {
        com.color.r = 1.0f;
        com.color.g = 0.0f;
        com.color.b = 0.0f;
        com.color.a = 1.0;
    }
    else
    {
        com.color.r = 0.0f;
        com.color.g = 0.0f;
        com.color.b = 1.0f;
        com.color.a = 1.0;
    }

    com.lifetime = ros::Duration();
    marker_pub.publish(com);
}

void show_model_in_rviz(Model &model, ros::Publisher &marker_pub)
{
    ros::Rate r1(1000);

    string frame = "base";
    Matrix3d R;
    Vector3d p;
    Matrix4d T_up;
    R.setIdentity(3, 3);
    p.setZero();
    

    // show base
    r1.sleep();
    visualization_msgs::Marker link;
    rviz_link(R, p, link, "base", 0, frame, marker_pub);

    // show link
    for (int i = 0; i < model.NB; i++)
    {
        r1.sleep();
        T_up.setIdentity(4, 4);
        int j = i;
        while (j > -1)
        {
            T_up = model.Ttree[j] * T_up;
            j = model.parent[j];
        }
        R = T_up.block(0, 0, 3, 3);
        p = T_up.block(0, 3, 3, 1);
        rviz_link(R, p, link, "base", i + 1, frame, marker_pub);
    }

    T_up.setIdentity(4, 4);
    visualization_msgs::Marker com;
    // show center of mass
    for (int i = 0; i < model.NB; i++)
    {
        r1.sleep();
        T_up.setIdentity(4, 4);
        int j = i;
        while (j > -1)
        {
            T_up = model.Ttree[j] * T_up;
            j = model.parent[j];
        }
        R = T_up.block(0, 0, 3, 3);
        p = T_up.block(0, 3, 3, 1);
        p += R * model.link[i].com;
        rviz_com(R, p, com, "base", i + model.NB + 1, frame, marker_pub);
    }
}