#include "local_template_match.h"
#include "../utils/utils.h"
#include <ros/ros.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <iomanip>
#include <vector>
using namespace std;
#include <boost/foreach.hpp>
#include <algorithm>
#include <numeric>
#include <stdio.h>
#include <fstream>

bool verbose = false; // This option prints debugs if a match has been found and if not what was the error

namespace vitaslam
{
LocalTemplateMatch::LocalTemplateMatch(ptree settings)
{
    /* Get the settings from the property library tree */
    // Visual parameters
    get_setting_from_ptree(VT_MIN_PATCH_NORMALISATION_STD, settings, "vt_min_patch_normalisation_std", (double)0);
    get_setting_from_ptree(VT_PATCH_NORMALISATION, settings, "vt_patch_normalise", 0);
    get_setting_from_ptree(VT_NORMALISATION, settings, "vt_normalisation", (double) 0);
    get_setting_from_ptree(VT_SHIFT_MATCH, settings, "vt_shift_match", 25);
    get_setting_from_ptree(VT_STEP_MATCH, settings, "vt_step_match", 5);
    get_setting_from_ptree(TEMPLATE_MATCH_THRESHOLD, settings, "template_match_threshold", 0.03);
    get_setting_from_ptree(TEMPLATE_X_SIZE, settings, "template_x_size", 1);
    get_setting_from_ptree(TEMPLATE_Y_SIZE, settings, "template_y_size", 1);
    get_setting_from_ptree(IMAGE_VT_X_RANGE_MIN, settings, "image_crop_x_min", 0);
    get_setting_from_ptree(IMAGE_VT_X_RANGE_MAX, settings, "image_crop_x_max", -1);
    get_setting_from_ptree(IMAGE_VT_Y_RANGE_MIN, settings, "image_crop_y_min", 0);
    get_setting_from_ptree(IMAGE_VT_Y_RANGE_MAX, settings, "image_crop_y_max", -1);

    // Parameters for template comparison
    get_setting_from_ptree(VISUAL_SCALAR, settings, "visual_scalar", 1.0);
    get_setting_from_ptree(PFH_SCALAR, settings, "pfh_scalar", 1.0);
    get_setting_from_ptree(SDA_SCALAR, settings, "sda_scalar", 1.0);
 
    // Tactile parameters
    get_setting_from_ptree(HISTOGRAM_DIMS, settings, "histogram_dims", 3);
    get_setting_from_ptree(WHISKER_COUNT, settings, "whisker_number", 24);
    TT_SDA_SIZE = WHISKER_COUNT;
    TT_PFH_SIZE = HISTOGRAM_DIMS*HISTOGRAM_DIMS*HISTOGRAM_DIMS;

    get_setting_from_ptree(SENSOR_MODE, settings, "sensor_mode", 2);


    VT_TEMPLATE_SIZE = TEMPLATE_X_SIZE * TEMPLATE_Y_SIZE;

    templates.reserve(10000);

    current_view.resize(VT_TEMPLATE_SIZE);
    current_touch.resize(TT_SDA_SIZE + TT_PFH_SIZE);

    current_template = 0;

    // Initialize the preprocessor
    p = new Preprocess(settings);
}
LocalTemplateMatch::~LocalTemplateMatch()
{

}

void LocalTemplateMatch::on_whisk_completed(const unsigned char *view_rgb, bool grayscale, unsigned int image_width, unsigned int image_height, std::vector<double> contacts, std::vector<std::vector<double>> deflections)
{
    if (view_rgb == NULL)
        return;

    IMAGE_WIDTH = image_width;
    IMAGE_HEIGHT = image_height;

    if (IMAGE_VT_X_RANGE_MAX == -1)
        IMAGE_VT_X_RANGE_MAX = IMAGE_WIDTH;
    if (IMAGE_VT_Y_RANGE_MAX == -1)
        IMAGE_VT_Y_RANGE_MAX = IMAGE_HEIGHT;


    this->view_rgb = view_rgb;
    this->grayscale = grayscale;
    // Preprocess the current visual data by 
    // converting the current grayscale image
    // data to a visual template stored in current_view
    p->preprocess_vision(grayscale, this->view_rgb, &current_view, &current_mean, IMAGE_WIDTH, IMAGE_VT_X_RANGE_MAX, IMAGE_VT_Y_RANGE_MAX);
    
    // Preprocess the tactile data stored in current_touch
    p->preprocess_tactile(&contacts, &deflections, &current_touch);
    assert (current_touch.size() == TT_SDA_SIZE + TT_PFH_SIZE);

    // Compare the new view template with all previously learnt templates
    compare(error, template_match_id);
    if (error <= TEMPLATE_MATCH_THRESHOLD)
    {
        if (verbose)
            if (template_match_id < get_current_template())
            {
                cout << "LOOP!" << " id: " << template_match_id << " error: " << error << " t: " << TEMPLATE_MATCH_THRESHOLD << endl;
            }
            else
            {
                cout << "MATCH!" << " id: " << template_match_id << " error: " << error << " t: " << TEMPLATE_MATCH_THRESHOLD << endl;
            }
        set_current_template((int)template_match_id);
    }
    else
    {
        if (verbose)
            cout << "NO_MATCH" << " id: " << template_match_id << " error: " << error << " t: " << TEMPLATE_MATCH_THRESHOLD << endl;
        set_current_template(create_template());
    }

}
void LocalTemplateMatch::pop_front(std::vector<int> &vec)
{
    assert(!vec.empty());
    vec.front() = std::move(vec.back());
    vec.pop_back();
}
/*! This function sets x and y size of the template */
void LocalTemplateMatch::clip_view_x_y(int &x, int &y)
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
/* This function creates a new view template and adds it to the templates array 
 * and sets the id, data and mean of the template
 * */
int LocalTemplateMatch::create_template()
{
    templates.resize(templates.size() + 1);
    CombinedTemplate * new_template = &(*(templates.end() - 1));

    new_template->id = templates.size() - 1;
    double * data_ptr = &current_view[0];
    new_template->visual_data.reserve(VT_TEMPLATE_SIZE);
    for (int i = 0; i < VT_TEMPLATE_SIZE; i++)
    {
        new_template->visual_data.push_back(*(data_ptr++));
    }
    new_template->tactile_data = current_touch;

    new_template->visual_mean = current_mean;

    return templates.size() - 1;
}

void LocalTemplateMatch::compare(double &err, int &template_match_id)
{
    // First compare the visual templates
    // This is pretty much like in vanilla ratslam
    if (templates.size() == 0)
    {
        err = DBL_MAX;
        error = err;
        return;
    }

    double *data = &current_view[0];
    double mindiff, cdiff;
    mindiff = DBL_MAX;

    double v_err = DBL_MAX;
    double sda_err = DBL_MAX;
    double pfh_err = DBL_MAX;

    int min_template = 0;

    double *template_ptr;
    double *column_ptr;
    double *template_row_ptr;
    double *column_row_ptr;
    double *template_start_ptr;
    double *column_start_ptr;
    int row_size;
    int sub_row_size;
    double *column_end_ptr;
    CombinedTemplate ct;

    int offset;
    double epsilon = 0.005;

    double cur_template_error = DBL_MAX;
    double combined_error = DBL_MAX;

    // Tactile variables
    double temp_diff;
    double mindiff_pfh = DBL_MAX;
    double mindiff_sda = DBL_MAX;
    double total_pfh_tt, total_pfh_cur;
    double total_sda_tt, total_sda_cur;
    double sum;
    vector<double> pfh_cur (current_touch.begin()+24, current_touch.end()); 
    vector<double> sda_cur (current_touch.begin(), current_touch.begin()+23); 
    vector<double> tmp = {};
    //cout << "========================" << endl;
    //for (vector<double>::const_iterator i = pfh_cur.begin(); i != pfh_cur.end(); ++i)
        //cout << *i << ' ';
    //cout << endl;

    int bla = 0;
    BOOST_FOREACH(ct, templates)
    {
        // ======================= Visual Error ===================================
        if (abs(current_mean - ct.visual_mean) > TEMPLATE_MATCH_THRESHOLD + epsilon)
            continue;

        // Handle the visual data
        for (offset = 0; offset < VT_SHIFT_MATCH*2+1; offset += VT_STEP_MATCH)
        {
            cdiff = 0;
            template_start_ptr = &ct.visual_data[0] + offset;
            column_start_ptr = &data[0] + VT_SHIFT_MATCH;
            row_size = TEMPLATE_X_SIZE;
            column_end_ptr = &data[0] + VT_TEMPLATE_SIZE - VT_SHIFT_MATCH;
            sub_row_size = TEMPLATE_X_SIZE - 2*VT_SHIFT_MATCH;

            for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
            {
                for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
                {
                    cdiff += abs(*column_ptr - *template_ptr);
                }
            }
            if (cdiff < mindiff)
            {
                mindiff = cdiff;
            }
        }
        v_err = mindiff / (double)(VT_TEMPLATE_SIZE - 2 * VT_SHIFT_MATCH * TEMPLATE_Y_SIZE);

        // ======================= PFH Error ===================================
        vector<double> pfh_tt (ct.tactile_data.begin()+24, ct.tactile_data.end()); 
        total_pfh_tt = abs(std::accumulate(pfh_tt.begin(), pfh_tt.end(), 0.0));
        total_pfh_cur = abs(std::accumulate(pfh_cur.begin(), pfh_cur.end(), 0.0));

        //cout << bla << " " ;
        //bla++;
        //cout << endl;
        //for (vector<double>::const_iterator i = pfh_tt.begin(); i != pfh_tt.end(); ++i)
            //cout << *i << ',';

        temp_diff = -1;
        tmp = {};
        for (int i = 0; i < pfh_cur.size(); i++)
        {
            tmp.push_back(abs(pfh_cur.at(i) - pfh_tt.at(i))); 
        }
        pfh_err = std::accumulate(tmp.begin(), tmp.end(), 0.0);
        //cout << ": " << pfh_err << endl;
        // ======================= SDA Error ===================================
        vector<double> sda_tt (ct.tactile_data.begin(), ct.tactile_data.begin()+23); 
        total_sda_tt = abs(std::accumulate(sda_tt.begin(), sda_tt.end(), 0.0));
        total_sda_cur = abs(std::accumulate(sda_cur.begin(), sda_cur.end(), 0.0));

        //for (vector<double>::const_iterator i = sda_tt.begin(); i != sda_tt.end(); ++i)
            //cout << *i << ',';


        temp_diff = -1;
        tmp = {};
        for (int i = 0; i < sda_cur.size(); i++)
        {
            tmp.push_back(abs(sda_cur.at(i) - sda_tt.at(i))); 
        }
        sda_err = std::accumulate(tmp.begin(), tmp.end(), 0.0);
        //cout << ": " << sda_err << endl;
        //cout << endl;
        // ======================= Combine the errors ==========================
        if (SENSOR_MODE == 0){
            // Turn off tactile data if in visual only mode
            combined_error = (1.0/VISUAL_SCALAR) * v_err;
        }
        else if (SENSOR_MODE == 1){
            // turn of visual mode when in tactile mode
            combined_error = (1.0/PFH_SCALAR) * pfh_err + (1.0/SDA_SCALAR )* sda_err;
        }
        else if (SENSOR_MODE == 2){
            combined_error = (1.0/VISUAL_SCALAR) * v_err + (1.0/PFH_SCALAR) * pfh_err + (1.0/SDA_SCALAR )* sda_err;
            //cout << combined_error << endl;
        }
        else{
            cout << "ERROR: unknown sensor mode!" << endl;
        }
       
        if (combined_error < cur_template_error)
        {
            cur_template_error = combined_error;
            template_match_id = ct.id;
            err = combined_error;
        }

    }
}
}
