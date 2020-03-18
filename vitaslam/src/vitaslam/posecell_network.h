/* ViTa-SLAM
 *
 * Copyright (C) 2020
 *
 * ViTaSLAM algorithm by
 * Oliver Struckmeier (oliver.struckmeier@aalto.fi), Kshitij Tiwari (kshitij.tiwari@aalto.fi)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * ViTa-SLAM is based on the RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au),
 * and in specific the openRatSLAM imeplementation by:
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 */

#ifndef _POSE_CELL_NETWORK_HPP                                                                                                            
#define _POSE_CELL_NETWORK_HPP
#define _USE_MATH_DEFINES
#include "math.h"
#include <stdio.h>
#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;
#include "../utils/utils.h"     
#include "posecells.h"
#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_member.hpp>

using namespace std;
     
namespace vitaslam
{   
    /*! This struct describes a visual template in the posecell network */
    struct PosecellVisualTemplate
    {
        unsigned int id;
        double pcvt_x, pcvt_y, pcvt_z, pcvt_a, pcvt_b, pcvt_g;
        double decay;
        std::vector<unsigned int> exps;
    };
    /*! This struct describes a posecell experience consisting of a visual template with vt_id and a vector of x,y,th (or more) positions */
    struct PosecellExperience {
        double pcxp_x, pcxp_y, pcxp_z, pcxp_a, pcxp_b, pcxp_g;
        int vt_id;
    };
    class PosecellNetwork{
        public:
            friend class PosecellScene;
            enum PosecellAction {NO_ACTION = 0, CREATE_NODE = 1, CREATE_EDGE = 2, SET_NODE = 3};
            PosecellNetwork(ptree settings);
            ~PosecellNetwork();

            // These functions return the best x,y,z,a,b,g and are updated by find_best()
            double x() { return best_x; } 
            double y() { return best_y; } 
            double g() { return best_g; } 

            // These variables track how often a wraparound hapened
            double x_wrap() { return pc_x_wrap; }
            double y_wrap() { return pc_y_wrap; }
            int get_pc_xyz_dim() { return PC_DIM_XYZ; }
            int get_pc_abg_dim() { return PC_DIM_ABG; }
            
            void on_odo(double vtrans_x, double vtrans_y, double vrot, double time_diff_s);
            void on_vita_template(int vt, double vt_rad);//double vt_rad_a, double vt_rad_b, double double_vt_rad_g);
            void on_tactile_template(int tt);

            PosecellAction get_action();
            unsigned int get_current_exp_id() {return current_exp; }
            double get_relative_rad() { return vt_delta_pcvt_g * 2.0 * M_PI / PC_DIM_ABG; }
            /*! This function returns the distance between two posecells to use as a measure of when to create a new posecell */
            double get_delta_pc(double x, double y, double g);
        private:
            /*! Inject energy into a specific point in the posecell network */
            bool inject(int act_x, int act_y, int act_g, double energy);

            int PC_DIM_XYZ, PC_DIM_ABG;
            // These are updated by find_best()
            double best_x, best_y, best_g;
            // These are updated by find_best() and used to compute how often we wrapped around
            double pc_x_wrap, pc_y_wrap = 0;

            double VT_ACTIVE_DECAY;
            double PC_VT_INJECT_ENERGY;
            double PC_VT_RESTORE;
            double EXP_DELTA_PC_THRESHOLD;
            double PC_C_SIZE_XYZ;                   // Size of a discretization step along PC_DIM_XYZ
            double PC_C_SIZE_ABG;                   // Size of a discretization step along PC_DIM_ABG
            unsigned int current_vt, prev_vt;
            unsigned int current_exp, prev_exp;
            bool odo_update;
            bool vt_update;
            bool tt_update;
            std::vector<PosecellVisualTemplate> visual_templates;
            std::vector<PosecellExperience> experiences;

            double vt_delta_pcvt_g; // Used in posecell_network.cpp:: on_view_template

            void create_view_template();
            void create_experience();

            /* Posecells related code */
            Posecells * posecells;
            Posecells * posecells_new;
            vector<double> * pc_rot;
            vector<double> * pc_rot2;
            vector<double> * pc_rot3;
            int PC_AVG_XYZ_WRAP;
            int PC_AVG_ABG_WRAP;
            // Excitation constants
            int PC_W_E_DIM;                         // Determines how far around a posecell the energy will be spread
            int PC_W_E_VAR;                         // Varilance of the pdf used to spread the energy
            // Inhibition constants
            int PC_W_I_DIM;
            int PC_W_I_VAR;
            double PC_GLOBAL_INHIB;                 // Constant value for the global inhibition. If a value in the posecells is lower than this value set it to zero
            std::vector<double> PC_W_EXCITE;
            std::vector<double> PC_W_INHIB;

            std::vector<double> PC_XYZ_SUM_SIN_LOOKUP;
            std::vector<double> PC_XYZ_SUM_COS_LOOKUP;
            std::vector<double> PC_ABG_SUM_SIN_LOOKUP;
            std::vector<double> PC_ABG_SUM_COS_LOOKUP;

            // The folowing functions are grouped in main function, and its helper functions. The main functions have a comment on what they do
            // void function();
            // void helper1();
            // ....

            /*! Locally excite and inhibit points. Exciting spreads energy and inhibition compresses. */
            bool excite();
            bool excite_helper(int x, int y, int g);
            
            /*! If a posecells is excited, spread the energy to prevent excessive spikes */
            bool inhibit();
            bool inhibit_helper(int x, int y, int g);

            /*! Shift the energy in the system by a translational and rotational velocity */
            void path_integration(double vtrans, double vrot, double angle_to_add);
            int get_flat_index(int x, int y, int xsize);

            /*! Find an approximation of the centre of the energy packet */
            double find_best();
            int wrap_index(int index, int size);
            double find_best_helper(double v1, double v2, int size);
            int PC_CELLS_TO_AVG;                    // Number of cells over which to average when computing find_best()
            double norm2d(double var, int x, int y, int dim_centre);
            double norm1d(double var, int g, int dim_centre);

            int stop_after;

            void circshift2d(vector<double>& pc_rot_in, int dim, int shiftx, int shifty);
            double get_min_delta(double d1, double d2, double max); 
            void printout_vector(vector<double> in);
            int count_nonzero(vector<double> in);

            //! This is used to set the initial best estimate
            // Vanilla ratslam used the center of the grid (STARTING_FACTOR = 0.5)
            double STARTING_FACTOR;
            int SENSOR_MODE;
            // This variable is used for storing the angle that is a result of the y translation
            double angle_to_add;
    };
}

#endif
