#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "utils/utils.h"
#include <sensor_msgs/image_encodings.h>
#include <boost/property_tree/ini_parser.hpp>
#include <tf/transform_datatypes.h>   
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <vitaslam/CombinedTemplateMessage.h>
#include <ros/console.h>
   
#include <image_transport/image_transport.h>
   
#include "vitaslam/local_template_match.h"

// Show the visual templates
#if HAVE_IRRLICHT
#include "graphics/local_view_scene.h"
vitaslam::LocalViewScene *lvs = NULL;
bool render_templates; // read from config if the view templates should be rendered to screen
#endif

   
using namespace vitaslam;
vitaslam::LocalTemplateMatch * lt = NULL;
int TT_TEMPLATE_SIZE = 24+3*3*3; // 24 whiskers and a flattened 3x3 2d histogram

// Local storages for visual and tactile data
sensor_msgs::ImageConstPtr temp;
unsigned int image_width;
unsigned int image_height;
vector<double> *contacts = new vector<double>();
vector<vector<double>> *deflections = new vector<vector<double>>();
vector<vector<double>> *deflections_buffer = new vector<vector<double>>();

int SENSOR_MODE;
bool last_rcp_state = true;
bool visual_data_changed = false;
bool contact_data_changed = false;
bool init_done = false;

/*!  This function clips an angle in rads to a range from 0 to 2 rad
 *  And would technically count by how many rotations we had to clip,
 *  but this is not used right now
 */
double clip2rad(double angle)
{
    unsigned int rots = round(abs(angle) / (2.0f * M_PI));
    if (angle < 0)
    {
        angle += (2.0f * M_PI) * (rots+1);
    }
    else if (angle >= 2.0f * M_PI)
    {
        angle -= (2.0f * M_PI) * rots;
    }
    return angle;
}

void image_callback(sensor_msgs::ImageConstPtr in)
{
    temp = in;
    visual_data_changed= true;
}

void rcp_state_callback(const std_msgs::Bool::ConstPtr& rcp_state, ros::Publisher * pub_t)
{
    // Recieve the image and preprocess it, Then we receive the id which will be used in posecells_network.cpp
    // This was the old image input (which was bgr8, but the whiskeye camera gives us rgb8 images)
    //lv->on_image(&image->data[0], (image->encoding == "bgr8" ? false : true), image->width, image->height);
    // We changed the image encoding here to rgb8 because that is what the whiskeye gazebo model provides.
    // Also take the current tactile data here even though it might not have changed, but a template has to consist of both tactile and visual data
    if (visual_data_changed == true && contact_data_changed == true)
        init_done = true;
    if (rcp_state->data != last_rcp_state && (visual_data_changed == true || contact_data_changed) && init_done){
        // If the whiskers finished protracting
        if (last_rcp_state == true)
        {
            static vitaslam::CombinedTemplateMessage t_output;
            lt->on_whisk_completed(&temp->data[0], (temp->encoding == "rgb8" ? false : true), temp->width, temp->height, *contacts, *deflections_buffer);

            t_output.header.stamp = ros::Time::now();
            t_output.header.seq++;
            // Get the id of the current vita template
            t_output.current_id = lt->get_current_template();
            // Get the relative difference of the template to create to the last template
            t_output.relative_rad = lt->get_relative_rad();

            // Publish the output template
            pub_t->publish(t_output);

            // Draw the scene (the windows on the screen that shows the camera views) 
            #ifdef HAVE_IRRLICHT
            if (render_templates)
            {
                lvs->draw_all();
            }
            #endif
            // Reset the tactile data storages
            contact_data_changed = false;
            visual_data_changed = false;

        }
        // If the whisker finished retracting
        if (last_rcp_state == false)
        {
            // Empty the current deflection storage and fill it in the global deflections variable
            deflections_buffer = deflections;
            deflections= new vector<vector<double>>();
            for (int i = 0; i < 48; i ++)
            {
                deflections->push_back(vector<double>());
            }
        }
        last_rcp_state = rcp_state->data;
    }
}

/*! This callback receives the contact points of the whiskers with an
 * object in world frame and stores them for later use
 */
void contacts_callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    contacts = new std::vector<double>();
    for (int i = 0; i < array->data.size(); i++)
    {
        contacts->push_back(array->data[i]);
    }
    // When there is no contact return a zero so that the vector is not empty
    if (contacts->size() <= 0)
        contacts = new std::vector<double>(1, 0);
    contact_data_changed = true;
}

/*! This callback receives the contact points of the whiskers with an
 * object in world frame and stores them for later use
 * The incoming array contains 24 values [x0,y0,x1,y1,...,x47,y47]
 */
void deflections_callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    int data_index = 0;
    int i = 0;
    for (i = 0; i < array->data.size(); i+=2)
    {
        double x_cur = array->data[i];
        double y_cur = array->data[i+1];
        deflections->at(i).push_back(x_cur);
        deflections->at(i+1).push_back(y_cur);
    }
}

/*! This node preprocesses the current image of the camera input into a visual template and the tactile data into a tactile template
 * */ 
int main(int argc, char * argv[])
{
    if (argc < 2) { exit(-1); }
    if (!ros::isInitialized()) { ros::init(argc, argv, "ViTaSLAMTemplate"); }

    std::string topic_root = "";

    boost::property_tree::ptree settings, vitaslam_settings, general_settings;
    read_ini(argv[1], settings);

    // Read the settings from the vitaslam/config/ file
    get_setting_child(general_settings, settings, "general", true);
    get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");

    // Create new helper class local template match with the given vitaslam settings
    get_setting_child(vitaslam_settings, settings, "vitaslam", true);
    lt = new vitaslam::LocalTemplateMatch(vitaslam_settings);

    // Check the sensor mode
    get_setting_from_ptree(SENSOR_MODE, vitaslam_settings, "sensor_mode", 2);

    // Get the size of the tactile template
    get_setting_from_ptree(TT_TEMPLATE_SIZE, vitaslam_settings, "tt_template_size", 24+3*3*3);

    ros::NodeHandle node;
    // Create publisher for the visual template
    ros::Publisher pub_template = node.advertise<vitaslam::CombinedTemplateMessage>(topic_root + "/ViTa/Template", 0);

    image_transport::ImageTransport it(node);
    // Create subscriber for the camera image with the image_callback function
    // Subscribe to the camera mounted to the base of the whiskeye robot. The stereo cams on the neck are called cam0 and cam1, the one on the body cam2
    image_transport::Subscriber sub_visual = it.subscribe(topic_root + "/platform/cam0", 0, boost::bind(image_callback, _1));

    // Subscribe to the raw tactile data
    ros::Subscriber sub_contacts = node.subscribe<std_msgs::Float32MultiArray>(topic_root + "/head/contact_world", 0, boost::bind(&contacts_callback,_1));
    ros::Subscriber sub_deflections = node.subscribe<std_msgs::Float32MultiArray>(topic_root + "/head/xy", 0, boost::bind(&deflections_callback,_1));

    // Subscribe to the whisker status
    ros::Subscriber sub_rcp_state = node.subscribe<std_msgs::Bool>(topic_root + "/head/rcp_state", 0, boost::bind(&rcp_state_callback, _1, &pub_template));

    // Create the empty visual data storage variables
    image_width = 0;
    image_height = 0;

    // Initialize the deflections memory
    for (int i = 0; i < 48; i++)
    {
        deflections->push_back(vector<double>());
    }
    for (int i = 0; i < 48; i ++)
    {
        deflections_buffer->push_back(vector<double>());
    }


#ifdef HAVE_IRRLICHT
    boost::property_tree::ptree draw_settings;
    get_setting_child(draw_settings, settings, "draw", true);
    get_setting_from_ptree(render_templates, draw_settings, "enable_view_templates", true);
    if (render_templates)
      lvs = new vitaslam::LocalViewScene(draw_settings, lt);
#endif


    ros::spin();
    return 0;
}
