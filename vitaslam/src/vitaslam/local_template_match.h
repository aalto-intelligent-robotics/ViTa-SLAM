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

#ifndef _VISUAL_TEMPLATE_MATCH_H_
#define _VISUAL_TEMPLATE_MATCH_H_
   
#include <cstring>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <iostream>
#include <vector>
#include <fstream>
   
#define _USE_MATH_DEFINES
#include "math.h"
   
#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;
   
#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include "preprocess.h"
   
namespace vitaslam
{

struct CombinedTemplate
{
    unsigned int id;
    std::vector<double> visual_data;
    std::vector<double> tactile_data;
    double visual_mean;
    double tactile_mean;
};
class LocalTemplateMatch
{
    public:
        friend class LocalViewScene;
        /*! This class contains the functions for the local view matching.
        * It is initialized with a property tree that contains all the parameters.
        * */
        LocalTemplateMatch(ptree settings);
        ~LocalTemplateMatch();

        /*! This used to be the old on_image function. But now it is
         *  called if there has been a new image AND tactile data
         *  The function is called in main_lv.cpp
         *  First, the image is processed
         *  Then the current template is compared to all previous ones
         *  by separately comaparing the tactile and visual data
         */
        void on_whisk_completed(const unsigned char *view_rgb, bool grayscale, unsigned int image_width, unsigned int image_height, std::vector<double> contacts, std::vector<std::vector<double>> deflections);

        /*! In main_lv.cpp, when new odometry is received, update the xyz,abg 
         *  values and set them in this class.
         *  Updates the relative angles between two local view matches
         *  The function get_relative_rad() returns these angles and are used
         *  in main_lv.cpp to get the relative difference of the current
         *  template to create to the last template
         */
        void on_odo(double x, double y, double g);
        /*! Getter that returns the current template */
        int get_current_template() { return current_template; }
        /*! Getter that returns the relative difference of two templates in radians */
        double get_relative_rad() { return vt_relative_rad; }

    private:
        LocalTemplateMatch()
        {
           ;
        }
        /*! Clip the size of the template */
        void clip_view_x_y(int &x, int &y);

        /*! Create and add visual template to the collection */
        int create_template();

        /*! Store the current max visual template id so we dont accidentally use that*/
        int max_vt = 0;

        /*! Set the id of the current template to the id of the given ID. The old current template is 
         *  stored in prev_vt */
        void set_current_template(int id)
        {
            if (id > max_vt)
            {
                prev_vt = current_template;
                current_template = max_vt;
            }
            else
            {
                prev_vt = current_template;
                current_template = id;
            }
            if (id > max_vt)
            {
                max_vt = id;
            }
        }

        /*! Converts an image input into a picture */
        //void convert_view_to_view_template(bool grayscale);

        /*! Compare a visual template to all the stored templates, allowing for slen pixel shifts in each
        *   direction
        *   returns the matching template and the MSE */
        void compare(double &err, int &template_match_id);

        /*! This flag sets the mode of ViTa-SLAM: 0 - Visual only, 1 - Tactile only, 2 - Both */
        int SENSOR_MODE = 2;

        // Tactile Parameters
        /*! The size of a tactile tempalte (the combined PFH and SDA arrays) */
        int TT_PFH_SIZE;
        int TT_SDA_SIZE;
        int HISTOGRAM_DIMS;
        int WHISKER_COUNT;

        // Visual Parameters
        double TEMPLATE_MATCH_THRESHOLD;
        int VT_SHIFT_MATCH;
        int VT_STEP_MATCH;
        int VT_TEMPLATE_SIZE;
        int IMAGE_WIDTH;
        int IMAGE_HEIGHT;
        int IMAGE_VT_X_RANGE_MIN;
        int IMAGE_VT_X_RANGE_MAX;
        int IMAGE_VT_Y_RANGE_MIN;
        int IMAGE_VT_Y_RANGE_MAX;
        int TEMPLATE_X_SIZE;
        int TEMPLATE_Y_SIZE;
        int VT_PATCH_NORMALISATION;
        double VT_MIN_PATCH_NORMALISATION_STD;
        double VT_NORMALISATION;
        int VT_PANORAMIC;

        std::vector<CombinedTemplate> templates;
        std::vector<double> current_view;
        std::vector<double> current_touch;
        void pop_front(std::vector<int> &vec);
     
        int image_size;
        double current_mean;
        double error;
        int prev_vt;
        double vt_relative_rad;
        const unsigned char *view_rgb;
        bool grayscale;
        int current_template;
        double VISUAL_SCALAR;
        double PFH_SCALAR;
        double SDA_SCALAR;

        // Stores the last matched tempalte ID
        int template_match_id = 0;

        Preprocess *p;
};
}
#endif
