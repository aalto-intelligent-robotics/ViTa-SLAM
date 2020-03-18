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
 */

#include "preprocess.h"
#include <assert.h>
// import utils to read from ptree
#include "../utils/utils.h"
#include <numeric>

using namespace std;

namespace vitaslam{
    Preprocess::Preprocess(ptree settings)
    {
        /* Get the settings from the property library tree */
        // Visual parameters
        get_setting_from_ptree(VT_MIN_PATCH_NORMALISATION_STD, settings, "vt_min_patch_normalisation_std", (double)0);
        get_setting_from_ptree(VT_PATCH_NORMALISATION, settings, "vt_patch_normalise", 0);
        get_setting_from_ptree(VT_NORMALISATION, settings, "vt_normalisation", (double) -1);
        get_setting_from_ptree(TEMPLATE_X_SIZE, settings, "template_x_size", 1);
        get_setting_from_ptree(TEMPLATE_Y_SIZE, settings, "template_y_size", 1);
        get_setting_from_ptree(IMAGE_VT_X_RANGE_MIN, settings, "image_crop_x_min", 0);
        get_setting_from_ptree(IMAGE_VT_Y_RANGE_MIN, settings, "image_crop_y_min", 0);

        // Tactile parameters
        get_setting_from_ptree(WHISKER_COUNT, settings, "whisker_number", 24);

        // Tactile parameters
        get_setting_from_ptree(HISTOGRAM_DIMS, settings, "histogram_dims", 3);
        PFH_SIZE = HISTOGRAM_DIMS*HISTOGRAM_DIMS*HISTOGRAM_DIMS;
        SDA_SIZE = WHISKER_COUNT;
    }
    Preprocess::~Preprocess()
    {

    }

    /*! Visual preprocessing functions as used in OpenRatSLAM
     * */
    void Preprocess::preprocess_vision(bool grayscale, const unsigned char *view_rgb, std::vector<double> *current_view, double *current_mean, int IMAGE_WIDTH, int IMAGE_VT_X_RANGE_MAX, int IMAGE_VT_Y_RANGE_MAX)
    {
        int data_next = 0;      
        int sub_range_x = IMAGE_VT_X_RANGE_MAX - IMAGE_VT_X_RANGE_MIN;
        int sub_range_y = IMAGE_VT_Y_RANGE_MAX - IMAGE_VT_Y_RANGE_MIN;
        int x_block_size = sub_range_x / TEMPLATE_X_SIZE;
        int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;
        int pos;                
                                
        for (unsigned int i; i < current_view->size(); i++)
        current_view->at(i) = 0;    
                                
        if (grayscale)          
        {                       
            for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block +=
            y_block_size, y_block_count++)
            {                   
                for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block +=
                x_block_size, x_block_count++)
                {               
                   for (int x = x_block; x < (x_block + x_block_size); x++)
                    {           
                        for (int y = y_block; y < (y_block + y_block_size); y++)
                        {       
                            pos = (x + y * IMAGE_WIDTH);
                            current_view->at(data_next) += (double)(view_rgb[pos]);
                        }       
                    }           
                    current_view->at(data_next) /= (255.0);
                    current_view->at(data_next) /= (x_block_size * y_block_size);
                    data_next++;
                }               
            }                   
        }                       
        else
        {                       
            for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)
            {                   
                for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)
                {               
                    for (int x = x_block; x < (x_block + x_block_size); x++)
                    {           
                        for (int y = y_block; y < (y_block + y_block_size); y++)
                            {   
                                pos = (x + y * IMAGE_WIDTH) * 3;
                                current_view->at(data_next) += ((double)(view_rgb[pos]) + (double)(view_rgb[pos + 1]) + (double)(view_rgb[pos + 2]));
                            }   
                    }           
                    current_view->at(data_next) /= (255.0 * 3.0);
                    current_view->at(data_next) /= (x_block_size * y_block_size);
                                
                    data_next++;
                }               
            }                   
        }
        if (VT_NORMALISATION > 0)
        {   
            double avg_value = 0;
            
            for (unsigned int i = 0; i < current_view->size(); i++)
            {
                avg_value += current_view->at(i);
            }
            
            avg_value /= current_view->size();
            
            for (unsigned int i = 0; i < current_view->size(); i++)
            {
                current_view->at(i) = std::max(0.0, std::min(current_view->at(i) * VT_NORMALISATION / avg_value, 1.0));
            }
        }   
            
        // now do patch normalisation
        // +- patch size on the pixel, ie 4 will give a 9x9
        if (VT_PATCH_NORMALISATION > 0)
        {   
            int patch_size = VT_PATCH_NORMALISATION;
            int patch_total = (patch_size * 2 + 1) * (patch_size * 2 + 1);
            double patch_sum;
            double patch_mean;
            double patch_std;
            int patch_x_clip;
            int patch_y_clip;
            
            // first make a copy of the view
            std::vector<double> current_view_copy;
            current_view_copy.resize(current_view->size());
            for (unsigned int i = 0; i < current_view->size(); i++)
                current_view_copy[i] = current_view->at(i);
            
            // this code could be significantly optimimised ....
            for (int x = 0; x < TEMPLATE_X_SIZE; x++)
            {
                for (int y = 0; y < TEMPLATE_Y_SIZE; y++)
                {
                    patch_sum = 0;
                    for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
                    {
                        for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
                        {
                            patch_x_clip = patch_x;
                            patch_y_clip = patch_y;
                            clip_view_x_y(patch_x_clip, patch_y_clip);
                            
                            patch_sum += current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE];
                        }
                    }
                    patch_mean = patch_sum / patch_total;
                    patch_sum = 0;
                    for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
                    {
                        for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
                        {
                            patch_x_clip = patch_x;
                            patch_y_clip = patch_y;
                            clip_view_x_y(patch_x_clip, patch_y_clip);
                            
                            patch_sum += ((current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean)
                            * (current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean));
                        }
                    }
                    
                    patch_std = sqrt(patch_sum / patch_total);
                    
                    if (patch_std < VT_MIN_PATCH_NORMALISATION_STD)
                        current_view->at(x + y * TEMPLATE_X_SIZE) = 0.5;
                    else {
                        current_view->at(x + y * TEMPLATE_X_SIZE) = max((double) 0, min(1.0, (((current_view_copy[x + y * TEMPLATE_X_SIZE] - patch_mean) / patch_std) + 3.0)/6.0 ));
                    }
                }
            }
        }    
             
        double sum = 0;
             
        // Find the mean of the data
        for (int i = 0; i < current_view->size(); i++)
        sum += current_view->at(i);
             
        *current_mean = (sum/current_view->size());
    }
    void Preprocess::clip_view_x_y(int &x, int &y)
    {
        if (x < 0)
            x = 0;
        else if (x > TEMPLATE_X_SIZE - 1)
            x = TEMPLATE_X_SIZE - 1;

        if (y < 0)
            y = 0;
        else if (y > TEMPLATE_Y_SIZE - 1)
            y = TEMPLATE_Y_SIZE - 1;
    }

    /* Tactile preprocessing functions
     * */
    void Preprocess::preprocess_tactile(vector<double> *contacts, vector<vector<double>> *deflections, vector<double> *current_touch)
    {
        // Compute pfh and fill it into current touch
        compute_PFH(*contacts, current_touch);

        // Deflections contains a 48 by t vector where t is a time series of x or y deflection angles during one whisk cycle
        // From this we compute the SDA now and fill it in the appropriate place in current_touch
        compute_SDA(*deflections, current_touch);
    }
    /*! Computes the SDA consisting of one slope for each whisker
     * deflections: a 48xN vector containing all N xy deflection angles for each whisker within 1 whisk cycle
     * current_touch: the vector containing the combined current PFH and SDA
     * */
    void Preprocess::compute_SDA(vector<vector<double>> deflections, vector<double> *current_touch)
    {
        vector<double> output (WHISKER_COUNT, 0);
        int whisker_index = 0;
        for (int i = 0; i < WHISKER_COUNT*2; i+=2)
        {
            double x_init = 0.0f;
            double y_init = 0.0f;
            double x_max = 0.0f;
            double y_max = 0.0f;
            // x-deflection
            for (int j = 0; j < deflections.at(i).size(); j++)
            {
               if (x_init <= 0 && deflections.at(i).at(j) > 0) { x_init = deflections.at(i).at(j); }
               if (deflections.at(i).at(j) > x_max) { x_max = deflections.at(i).at(j); }
            } 
            // y-deflection
            for (int j = 0; j < deflections.at(i+1).size(); j++)
            {
                if (y_init <= 0 && deflections.at(i+1).at(j) > 0) { y_init = deflections.at(i+1).at(j); }
                if (deflections.at(i+1).at(j) > y_max) { y_max = deflections.at(i+1).at(j); }
            } 
            double cur_slope = 0.0f;
            if (x_max-x_init != 0){
                cur_slope = (y_max - y_init)/(x_max - x_init);
            }
            output.at(whisker_index) = cur_slope;
            whisker_index++;
        }
        // Fill the SDA into current_touch
        for (int i = 0; i < output.size(); i++)
        {
            current_touch->at(i) = output.at(i);
        }
    }
    void Preprocess::compute_PFH(vector<double> contacts, vector<double> *current_touch)
    {
        // Make sure there is at least one contact consisting of a sextuple of pos xyz and norm xyz
        vector<double> output (HISTOGRAM_DIMS*HISTOGRAM_DIMS*HISTOGRAM_DIMS, 0);
        // If there is not contact, contacts contains only a 0
        if (contacts.size() >= 6)
        {
            vector<double> xvals;
            vector<double> yvals;
            vector<double> zvals;
            // Prepare the data, format: [pos x, pos y, pos z, norm x, norm y, norm z, ...]
            for (int i = 3; i < contacts.size(); i+=6)
                xvals.push_back(contacts.at(i));
            for (int i = 4; i < contacts.size(); i+=6)
                yvals.push_back(contacts.at(i));
            for (int i = 5; i < contacts.size(); i+=6)
                zvals.push_back(contacts.at(i));
            assert(xvals.size() == yvals.size());
            assert(yvals.size() == zvals.size());

            // Get min and max for x, y and z
            double max_x = *max_element(xvals.begin(), xvals.end());
            double min_x = *min_element(xvals.begin(), xvals.end());
            double x_total = abs(max_x) - abs(min_x);
            
            double max_y = *max_element(yvals.begin(), yvals.end());
            double min_y = *min_element(yvals.begin(), yvals.end());
            double y_total = abs(max_y) - abs(min_y);

            double max_z = *max_element(zvals.begin(), zvals.end());
            double min_z = *min_element(zvals.begin(), zvals.end());
            double z_total = abs(max_z) - abs(min_z);

            // Create bin limits
            vector<double> xlim ((int)(HISTOGRAM_DIMS+1), 0);
            vector<double> ylim ((int)(HISTOGRAM_DIMS+1), 0);
            vector<double> zlim ((int)(HISTOGRAM_DIMS+1), 0);

            xlim.at(0) = min_x;
            xlim.at(xlim.size()-1) = max_x;
            ylim.at(0) = min_y;
            ylim.at(ylim.size()-1) = max_y;
            zlim.at(0) = min_z;
            zlim.at(zlim.size()-1) = max_z;

            for (int i = 1; i < HISTOGRAM_DIMS; i++)
            {  
                xlim.at(i) = (min_x + ((float)i/HISTOGRAM_DIMS) * x_total);
                ylim.at(i) = (min_y + ((float)i/HISTOGRAM_DIMS) * y_total);
                zlim.at(i) = (min_z + ((float)i/HISTOGRAM_DIMS) * z_total);                                  
            }  
         
            // Compute output histogram
            for (int i = 0; i < xvals.size(); i++)
            {
                double cur_x = xvals.at(i);
                double cur_y = yvals.at(i);
                double cur_z = zvals.at(i);
                // Get the x index
                int x_index = 0;
                int y_index = 0;
                int z_index = 0;
                for (int limit = 0; limit < xlim.size(); limit++)
                {
                    if (xlim.at(limit) < cur_x)
                    {
                        x_index = limit;
                    } 
                }
                for (int limit = 0; limit < ylim.size(); limit++)
                {
                    if (ylim.at(limit) < cur_y)
                    {
                        y_index = limit;
                    } 
                }
                for (int limit = 0; limit < zlim.size(); limit++)
                {
                    if (zlim.at(limit) < cur_z)
                    {
                        z_index = limit;
                    }
                }
                output.at(x_index + HISTOGRAM_DIMS * (y_index + HISTOGRAM_DIMS * z_index)) += 1;
            }
        }
        for (int i = 0; i < output.size(); i++)
        {
            current_touch->at(i+SDA_SIZE-1) = output.at(i);
        }
        //cout << "PFH:" << endl;
        //printout_vector_double(output);
        //cout << "contacts size: " << contacts.size() << " by 6: " << contacts.size()/6.0f << endl;
        //cout << "sum: " << std::accumulate(output.begin(), output.end(), 0) << endl;
    }
}
