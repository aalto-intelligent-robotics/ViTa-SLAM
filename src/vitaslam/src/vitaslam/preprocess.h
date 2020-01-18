#ifndef _PREPROCESS_H_
#define _PREPROCESS_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <vector>

#include <boost/property_tree/ini_parser.hpp>
#include "../utils/utils.h"

using boost::property_tree::ptree;

namespace vitaslam
{
    /*! This class contains functions for tactile and visual preprocessing used
     * in local_view_match.cpp */
    class Preprocess
    {
        public:
            Preprocess(ptree settings);
            ~Preprocess();

            // Functions for visual preprocessing

            /*! Visual Preprocessing */
            void preprocess_vision(bool grayscale, const unsigned char *view_rgb, std::vector<double> *current_view, double *current_mean, int IMAGE_WIDTH, int IMAGE_VT_X_RANGE_MAX, int IMAGE_VT_Y_RANGE_MAX);
            /*! Clip the size of the template */
            void clip_view_x_y(int &x, int &y);

            // Functions for tactile preprocessing
            /*! Tactile Preprocessing */
            void preprocess_tactile(std::vector<double> *contacts, std::vector<std::vector<double>> *SDA, std::vector<double> *current_touch);
            void compute_PFH(std::vector<double> contacts, std::vector<double> *current_touch);
            void compute_SDA(std::vector<std::vector<double>> deflections, std::vector<double> *current_touch);

        private:
            // Parameters for visual preprocessing
            double VT_MIN_PATCH_NORMALISATION_STD;
            int VT_PATCH_NORMALISATION;
            double VT_NORMALISATION;
            int IMAGE_VT_X_RANGE_MIN;
            int IMAGE_VT_X_RANGE_MAX;
            int IMAGE_VT_Y_RANGE_MIN;
            int IMAGE_VT_Y_RANGE_MAX;
            int TEMPLATE_X_SIZE;
            int TEMPLATE_Y_SIZE;

            // Parameters vor tactile preprocessing
            int HISTOGRAM_DIMS;
            int WHISKER_COUNT;
            int SDA_SIZE;
            int PFH_SIZE;
    };
}

#endif
