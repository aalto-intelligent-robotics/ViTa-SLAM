#ifndef _EXPERIENCE_MAP_H_
#define _EXPERIENCE_MAP_H_
  
#define _USE_MATH_DEFINES
#include "math.h"
  
#include <stdio.h>
#include <vector>
#include <deque>
  
#include <iostream>
  
#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;
  
#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/deque.hpp>

using namespace std;

namespace vitaslam
{
/*!
 * The Link structure describes a link
 * between two experiences.
 */
struct Link
{
    double d;
    double heading_rad;
    double gamma_rad;
    int exp_to_id;
    int exp_from_id;
    double delta_time_s;
};

/*!
 * The Experience structure describes
 * a node in the Experience_Map with the given parameters
 */
struct Experience
{
    int id; // its own id

    double x_m, y_m, gamma_m; 
    int vt_id;

    unsigned int experience_type; //describes the type of an experience (3D or 6D), this will be used later to draw the experiences in different colors

    std::vector<unsigned int> links_from; // links from this experience
    std::vector<unsigned int> links_to; // links to this experience


    // goal navigation
    double time_from_current_s;
    unsigned int goal_to_current, current_to_goal;
};

class ExperienceMap
{
    public:
        /*! ExperienceMapScene should have access to private variables */
        friend class ExperienceMapScene;
        ExperienceMap(ptree settings);
        ~ExperienceMap();

        /*! Update the current position of the experience map since the last experience. */
        void on_odo(double vtrans_x, double vtrans_y, double, double time_diff_s);
        /*! Create a new experience for a given position */
        int on_create_experience(unsigned int exp_id);
        /*! Change the current experience */
        int on_set_experience(int new_exp_id, double relative_rad_gamma);
        /*! Create a new link between to experiences based on their id and their relative angles in rad */
        bool on_create_link(int exp_id_from, int exp_id_to, double relative_rad_gamma);
        /*! Clear the deque goal_list */
        void clear_goal_list() { goal_list.clear(); }
        /*! Pops the front element of the goal_list */
        void delete_current_goal() { goal_list.pop_front(); }
        /*! Setting and handling new goals based on their position*/
        void add_goal(double x_m, double y_m);
        /*! Put a goal into the goal list (used in add_goal(double x_m, double y_m) */
        void add_goal(int id) { goal_list.push_back(id); }

        /* ======== Compuatations =========== */
        /*! Returns if a path to the goal has been found
         * \return bool: True if a path was found
         */
        bool calculate_path_to_goal(double time_s);
        /*! Computes the euclidean distance between two experiences
         */
        double exp_euclidean_m(Experience *exp1, Experience *exp2);
        /*! Update the map by relaxing the graph
         */
        bool iterate();

        /* ======== Getter Functions ======== */
        /*! Searches the experience from the experiences vector that is the current goal
         * \return bool : True if goal_list contains at least 1 element
        * */          
        bool get_goal_waypoint();                                                                                                               
        /*! This function returns a pointer to the experience with the given id */
        Experience *get_experience(int id) { return &experiences[id]; }
        /*! Returns the link with a given id */
        Link * get_link(int id) { return &links[id]; }
        /*! This function returns the id of the first element in the front queue */
        int get_current_goal_id() { return (goal_list.size() == 0) ? -1 : (int)goal_list.front(); }
        /*! Returns if a goal has been reached */
        bool get_goal_success() { return goal_success; }
        /*! Returns the distance from the subgoal */
        double get_subgoal_m() const;
        /*! Returns the angle in radians from the goal by comparing the experience and goal angle */
        double get_subgoal_rad() const;
        /*! Get the id of the current experience */
        int get_current_id() { return current_exp_id; }
        /*! Returns the final experience id of the goal path */
        unsigned int get_goal_path_final_exp() { return goal_path_final_exp_id; }
        /*! Get the number of links in the private links vector */
        int get_num_links() { return links.size(); }
        /*! Get the number of experiences in the private experiences vector */
        int get_num_experiences() { return experiences.size(); }
        /*! Returns the dequeu<int> goal_list */
        const std::deque<int> &get_goals() const { return goal_list; }

    private:
        ExperienceMap()
        {
            ;
        }
        /*! Calculate the distance between two experiences using Dijkstra's algorithm, can be slow for many experiences */
        double dijkstra_distance_between_experiences(int id1, int id2);

        struct compare
        {
            bool operator()(const Experience *exp1, const Experience *exp2)
            {
                return exp1->time_from_current_s > exp2->time_from_current_s;
            }
        };

        int EXP_LOOPS;        
        double EXP_CORRECTION;
        unsigned int MAX_GOALS;
        double EXP_INITIAL_EM_DEG;

        std::vector<Experience> experiences;
        std::vector<Link> links;
        std::deque<int> goal_list;

        int current_exp_id, prev_exp_id;
        double accum_delta_x;
        double accum_delta_y;
        double accum_delta_gamma;
        double accum_delta_time_s;
        double relative_rad;

        int waypoint_exp_id;
        bool goal_success;
        double goal_timeout_s;
        unsigned int goal_path_final_exp_id;
};
}
#endif
