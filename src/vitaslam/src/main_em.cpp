#include "utils/utils.h"
#include <boost/property_tree/ini_parser.hpp>
#include <ros/ros.h>
#include "vitaslam/experience_map.h"
#include <vitaslam/TopologicalAction.h>
#include <nav_msgs/Odometry.h>
#include <vitaslam/TopologicalMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
  
ros::Publisher pub_em;
ros::Publisher pub_metrics;
ros::Publisher pub_pose;
ros::Publisher pub_em_markers;
ros::Publisher pub_goal_path;
geometry_msgs::PoseStamped pose_output;
vitaslam::TopologicalMap em_map;
visualization_msgs::Marker em_marker;

/*! Graphics things using irrlicht to draw the top down view */
#ifdef HAVE_IRRLICHT
#include "graphics/experience_map_scene.h"
vitaslam::ExperienceMapScene *ems;
bool use_graphics;
#endif
  
int publish_metrics;

using namespace vitaslam;
void odo_callback(nav_msgs::OdometryConstPtr odo, vitaslam::ExperienceMap *em)
{
    static ros::Time prev_time(0);
   
    if (prev_time.toSec() > 0)
    {
        double time_diff = (odo->header.stamp - prev_time).toSec();
        /* For the experience map we only need to set vtrans and vrot in the 2D plane in which the robot is moving because
         * the experience map is a top down view of the position of the robot in the xy plane with angle around z axis
         * Careful!!!!: odom does not have a positions
         */
        double vtrans_x = odo->twist.twist.linear.x;
        double vtrans_y = odo->twist.twist.linear.y;
        double vrot = odo->twist.twist.angular.z;
        em->on_odo(vtrans_x, vtrans_y, vrot, time_diff);
    }
   
    static ros::Time prev_goal_update(0);
   
    // Set the path to the next goal
    if (em->get_current_goal_id() >= 0)
    {
        // (prev_goal_update.toSec() == 0 || (odo->header.stamp - prev_goal_update).toSec() > 5)
        //em->calculate_path_to_goal(odo->header.stamp.toSec());
   
        prev_goal_update = odo->header.stamp;
   
        em->calculate_path_to_goal(odo->header.stamp.toSec());
        static nav_msgs::Path path;
        if (em->get_current_goal_id() >= 0)
        {
            em->get_goal_waypoint();

            static geometry_msgs::PoseStamped pose;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "1";

            pose.header.seq = 0;
            pose.header.frame_id = "1";
            path.poses.clear();
            unsigned int trace_exp_id = em->get_goals()[0];
            while (trace_exp_id != em->get_goal_path_final_exp())
            {
                pose.pose.position.x = em->get_experience(trace_exp_id)->x_m;
                pose.pose.position.y = em->get_experience(trace_exp_id)->y_m;
                path.poses.push_back(pose);
                pose.header.seq++;
                trace_exp_id = em->get_experience(trace_exp_id)->goal_to_current;
            }
            pub_goal_path.publish(path);

            path.header.seq++;
        }
        else
        {
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "1";
        path.poses.clear();
        pub_goal_path.publish(path);

        path.header.seq++;
        }
    }
    prev_time = odo->header.stamp;
}

void action_callback(vitaslam::TopologicalActionConstPtr action, vitaslam::ExperienceMap *em, ros::Publisher * pub_metrics)
{ 
  switch (action->action)
  {
    case vitaslam::TopologicalAction::CREATE_NODE:
      //cout << "CREATE_NODE src: " << action->src_id << " dest: " << action->dest_id << " with rel rads g: " << action->relative_rad << endl;
      em->on_create_experience(action->dest_id); // Creating a new node usually at the current position in pc
      em->on_set_experience(action->dest_id, 0); // No relative angles because its the same place
      break;
  
    case vitaslam::TopologicalAction::CREATE_EDGE:
      //cout << "CREATE_EDGE src: " << action->src_id << " dest: " << action->dest_id << endl;
      em->on_create_link(action->src_id, action->dest_id, action->relative_rad);
      em->on_set_experience(action->dest_id, action->relative_rad);
      break;
  
    case vitaslam::TopologicalAction::SET_NODE:
      //cout << "SET_NODE src: " << action->src_id << " dest: " << action->dest_id << " with rel rads g: " << action->relative_rad<< endl;
      em->on_set_experience(action->dest_id, action->relative_rad);
      break;
  
  }
  
  em->iterate();
  // Get the position and orientation of the experience map and publish it
  pose_output.pose.position.x = em->get_experience(em->get_current_id())->x_m;
  pose_output.pose.position.y = em->get_experience(em->get_current_id())->y_m;

  pose_output.pose.orientation.x = 0;
  pose_output.pose.orientation.y = 0;
  pose_output.pose.orientation.z = sin(em->get_experience(em->get_current_id())->gamma_m / 2.0);
  pose_output.pose.orientation.w = cos(em->get_experience(em->get_current_id())->gamma_m / 2.0);
  pub_pose.publish(pose_output);
 
  static ros::Time prev_pub_time(0);
  // Log metrics
  if (publish_metrics)
  {
    std_msgs::Float32MultiArray em_coords;
    em_coords.data.clear();
    for (int i = 0; i < em->get_num_experiences(); i++)
    {
        em_coords.data.push_back(em->get_experience(i)->id);
        em_coords.data.push_back(em->get_experience(i)->x_m);
        em_coords.data.push_back(em->get_experience(i)->y_m);
        em_coords.data.push_back(em->get_experience(i)->gamma_m);
    }
    pub_metrics->publish(em_coords);
  }

  // Propagate the relaxation trhough all other experiences
  if (action->header.stamp - prev_pub_time > ros::Duration(30.0))
  {
    prev_pub_time = action->header.stamp;
 
    em_map.header.stamp = ros::Time::now();
    em_map.header.seq++;
    em_map.node_count = em->get_num_experiences();
    em_map.node.resize(em->get_num_experiences());

    for (int i = 0; i < em->get_num_experiences(); i++)
    {
      em_map.node[i].id = em->get_experience(i)->id;
      em_map.node[i].pose.position.x = em->get_experience(i)->x_m;
      em_map.node[i].pose.position.y = em->get_experience(i)->y_m;
      em_map.node[i].pose.orientation.x = 0;
      em_map.node[i].pose.orientation.y = 0;
      em_map.node[i].pose.orientation.z = sin(em->get_experience(i)->gamma_m / 2.0);
      em_map.node[i].pose.orientation.w = cos(em->get_experience(i)->gamma_m / 2.0);
    }
    em_map.edge_count = em->get_num_links();
    em_map.edge.resize(em->get_num_links());
    for (int i = 0; i < em->get_num_links(); i++)
    {
      em_map.edge[i].source_id = em->get_link(i)->exp_from_id;
      em_map.edge[i].destination_id = em->get_link(i)->exp_to_id;
      em_map.edge[i].duration = ros::Duration(em->get_link(i)->delta_time_s);
      // TODO is beta here correct?
      em_map.edge[i].transform.translation.x = em->get_link(i)->d * cos(em->get_link(i)->heading_rad);
      em_map.edge[i].transform.translation.y = em->get_link(i)->d * sin(em->get_link(i)->heading_rad);
      // TODO this is not done yet
      em_map.edge[i].transform.translation.z = 0;
      em_map.edge[i].transform.rotation.x = 0;
      em_map.edge[i].transform.rotation.y = 0;
      em_map.edge[i].transform.rotation.z = sin(em->get_link(i)->gamma_rad / 2.0);
      em_map.edge[i].transform.rotation.w = cos(em->get_link(i)->gamma_rad / 2.0);
    }
    pub_em.publish(em_map);
  }
  
  em_marker.header.stamp = ros::Time::now();
  em_marker.header.seq++;
  em_marker.header.frame_id = "1";
  em_marker.type = visualization_msgs::Marker::LINE_LIST;
  em_marker.points.resize(em->get_num_links() * 2);
  em_marker.action = visualization_msgs::Marker::ADD;
  em_marker.scale.x = 0.01;
  em_marker.color.a = 1;
  em_marker.ns = "em";
  em_marker.id = 0;
  em_marker.pose.orientation.x = 0;
  em_marker.pose.orientation.y = 0;
  em_marker.pose.orientation.z = 0;
  em_marker.pose.orientation.w = 1;
  for (int i = 0; i < em->get_num_links(); i++)
  
  {
    em_marker.points[i * 2].x = em->get_experience(em->get_link(i)->exp_from_id)->x_m;
    em_marker.points[i * 2].y = em->get_experience(em->get_link(i)->exp_from_id)->y_m;
    em_marker.points[i * 2].z = 0;
    em_marker.points[i * 2 + 1].x = em->get_experience(em->get_link(i)->exp_to_id)->x_m;
    em_marker.points[i * 2 + 1].y = em->get_experience(em->get_link(i)->exp_to_id)->y_m;
    em_marker.points[i * 2 + 1].z = 0;
  }
  
  pub_em_markers.publish(em_marker);
    #ifdef HAVE_IRRLICHT
    if (use_graphics)
    {
        ems->update_scene();
        ems->draw_all();
    }
    #endif
}

/*! Set a new goal pose within the experience map */
void set_goal_pose_callback(geometry_msgs::PoseStampedConstPtr pose, vitaslam::ExperienceMap * em) 
{ 
   em->add_goal(pose->pose.position.x, pose->pose.position.y);
}
int main(int argc, char * argv[])
{
    ROS_INFO_STREAM("starting vitaslam experience map");
    if (argc < 2)
    {
        ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
        exit(-1);
    }
    std::string topic_root = "";
    boost::property_tree::ptree settings, general_settings, vitaslam_settings;
    read_ini(argv[1], settings);

    get_setting_child(vitaslam_settings, settings, "vitaslam", true);
    get_setting_child(general_settings, settings, "general", true);
    get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");
    get_setting_from_ptree(publish_metrics, general_settings, "publish_metrics", 0);

    if (!ros::isInitialized()) { ros::init(argc, argv, "ViTaSLAMExperienceMap"); }

    ros::NodeHandle node;
    vitaslam::ExperienceMap * em = new vitaslam::ExperienceMap(vitaslam_settings);

    /*! Publisher 1: Complete representaiton of the experience maps as topological map consisting of a list of nodes and edges */
    pub_em = node.advertise<vitaslam::TopologicalMap>(topic_root + "/ExperienceMap/Map", 1);
    /*! Publisher 2: Marker message suitable for rendering the map within rviz */                          
    pub_em_markers = node.advertise<visualization_msgs::Marker>(topic_root + "/ExperienceMap/MapMarker", 1);
    /*! Publisher for the robot pose within the experience map (pose is based on the current experience) */
    pub_pose = node.advertise<geometry_msgs::PoseStamped>(topic_root + "/ExperienceMap/RobotPose", 1);
    pub_metrics = node.advertise<std_msgs::Float32MultiArray>(topic_root + "/log/em", 3);
    pub_goal_path = node.advertise<nav_msgs::Path>(topic_root + "/ExperienceMap/PathToGoal", 1);

    /*! Odometry subscriber */
    ros::Subscriber sub_odometry = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 0, boost::bind(odo_callback, _1, em), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    /*! Subscribe to the topological action topic */
    ros::Subscriber sub_action = node.subscribe<vitaslam::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction", 0, boost::bind(action_callback, _1, em, &pub_metrics), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    /*! Subscriber to the set a new goal pose topic. A goal is a pose to which the robot shall navigate*/  
    ros::Subscriber sub_goal = node.subscribe<geometry_msgs::PoseStamped>(topic_root + "/ExperienceMap/SetGoalPose", 0, boost::bind(set_goal_pose_callback, _1, em),  ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    /*! Set up irrlicht */
    #ifdef HAVE_IRRLICHT
    boost::property_tree::ptree draw_settings;
    get_setting_child(draw_settings, settings, "draw", true);
    get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
    if (use_graphics)
    {
      ems = new vitaslam::ExperienceMapScene(draw_settings, em);
    }
    #endif

    ros::spin();
    return 0;
}
