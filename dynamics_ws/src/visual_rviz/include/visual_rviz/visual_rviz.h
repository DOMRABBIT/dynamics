#ifndef _VISUAL_RVIZ_H_
#define _VISUAL_RVIZ_H_

#include "ros/ros.h"
#include "CRPmodel.h"

void rviz_link(Eigen::Matrix3d R,
               Eigen::Vector3d p,
               visualization_msgs::Marker &link,
               std::string ns,
               int id,
               std::string frame,
               ros::Publisher &marker_pub);

void rviz_com(Eigen::Matrix3d R,
              Eigen::Vector3d p,
              visualization_msgs::Marker &link,
              std::string ns,
              int id,
              std::string frame,
              ros::Publisher &marker_pub);

void show_model_in_rviz(Model &model, ros::Publisher &marker_pub);

#endif