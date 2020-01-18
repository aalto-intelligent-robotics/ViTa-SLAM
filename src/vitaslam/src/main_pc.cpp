#include <iostream>
#include "utils/utils.h"
#include <boost/property_tree/ini_parser.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
  
// ViTa-SLAM includes
#include "vitaslam/posecell_network.h"
#include <vitaslam/TopologicalAction.h>
#include <nav_msgs/Odometry.h>
#include <vitaslam/CombinedTemplateMessage.h>


// Graphics includes
#if HAVE_IRRLICHT
#include "graphics/posecell_scene.h"
vitaslam::PosecellScene *pcs;
bool use_graphics;
#endif

using namespace vitaslam;

vitaslam::TopologicalAction pc_output;
int publish_metrics;

void odo_callback(nav_msgs::OdometryConstPtr odo, vitaslam::PosecellNetwork *pc, ros::Publisher * pub_pc, ros::Publisher * pub_log)
{
    static ros::Time prev_time(0);

    if (prev_time.toSec() > 0)
    {
        // Publish a topological action built based on the posecell network pc
        double time_diff = (odo->header.stamp - prev_time).toSec();

        pc_output.src_id = pc->get_current_exp_id();
        // The odometry used here used to come from the visual odometry node in the original ratslam
        pc->on_odo(odo->twist.twist.linear.x, odo->twist.twist.linear.y, odo->twist.twist.angular.z, time_diff);
        pc_output.action = pc->get_action();
        //ROS_INFO_STREAM("pc_output.action: " << pc_output.action);
        if (pc_output.action != vitaslam::PosecellNetwork::NO_ACTION)
        {
            pc_output.header.stamp = ros::Time::now();
            pc_output.header.seq++;
            pc_output.dest_id = pc->get_current_exp_id();
            pc_output.relative_rad = pc->get_relative_rad();
            pub_pc->publish(pc_output);
        }
        // Publish the current best estimate for the logger and to compute the metrics later
        if (publish_metrics)
        {
            // This function publishes the best estime translated by half of the posecells network size because
            // the initial energy is injected in the center of the pc network.
            // By offestting it, the robots estimated position is always (0|0|0)
            std_msgs::Float32MultiArray cur_best;
            cur_best.data.clear();
            cur_best.data.push_back((pc->x()-pc->get_pc_xyz_dim()/2) + pc->x_wrap() * pc->get_pc_xyz_dim());
            cur_best.data.push_back((pc->y()-pc->get_pc_xyz_dim()/2) + pc->y_wrap() * pc->get_pc_xyz_dim());
            cur_best.data.push_back(pc->g() * (360.0/pc->get_pc_abg_dim()));
            pub_log->publish(cur_best);
        }
        #ifdef HAVE_IRRLICHT
	    if (use_graphics)
	    {
		    pcs->update_scene();
		    pcs->draw_all();
	    }
        #endif
    }
    prev_time = odo->header.stamp;
}

void visual_template_callback(vitaslam::CombinedTemplateMessageConstPtr vt, vitaslam::PosecellNetwork *pc)
{
    pc->on_vita_template(vt->current_id, vt->relative_rad);
    #ifdef HAVE_IRRLICHT
	if (use_graphics)
	{
	    pcs->update_scene();
		pcs->draw_all();
	}
    #endif
}

int main(int argc, char * argv[])
{
    if (argc < 2)
    {
        ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
        exit(-1);
    }
    std::string topic_root = "";
    boost::property_tree::ptree settings, vitaslam_settings, general_settings;
    read_ini(argv[1], settings);
    get_setting_child(vitaslam_settings, settings, "vitaslam", true);
    get_setting_child(general_settings, settings, "general", true);
    get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");
    int PC_DIM_XYZ, PC_DIM_ABG;
    get_setting_from_ptree(PC_DIM_XYZ, vitaslam_settings, "pc_dim_xyz", 21);
    get_setting_from_ptree(PC_DIM_ABG, vitaslam_settings, "pc_dim_abg", 36);
    if (!ros::isInitialized()) { ros::init(argc, argv, "ViTaSLAMPosecellGrid"); }
    ros::NodeHandle node;
    vitaslam::PosecellNetwork * pc = new vitaslam::PosecellNetwork(vitaslam_settings);
    ros::Publisher pub_pc = node.advertise<vitaslam::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction", 0);

    get_setting_from_ptree(publish_metrics, general_settings, "publish_metrics", 0);

    ros::Publisher pub_log = node.advertise<std_msgs::Float32MultiArray>(topic_root + "/log/cur_best_pos_est", 3);
    ros::Subscriber sub_odometry = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 0, boost::bind(odo_callback, _1, pc, &pub_pc ,&pub_log), ros::VoidConstPtr(),
    ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_template = node.subscribe<vitaslam::CombinedTemplateMessage>(topic_root + "/ViTa/Template", 0, boost::bind(visual_template_callback, _1, pc), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    #ifdef HAVE_IRRLICHT
    boost::property_tree::ptree draw_settings;
    get_setting_child(draw_settings, settings, "draw", true);
    get_setting_from_ptree(use_graphics, draw_settings, "enable_pc", true);
    if (use_graphics)
    {
	    pcs = new vitaslam::PosecellScene(draw_settings, pc);
    }
    #endif

    ros::spin();
    return 0;
}

