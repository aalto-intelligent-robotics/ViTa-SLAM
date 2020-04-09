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

#include "posecell_network.h"
#include "../utils/utils.h"    
      
#include <stdlib.h>
#include <string.h>            
#include <assert.h>            
#include <float.h>

using namespace std;
      
namespace vitaslam
{     
    PosecellNetwork::PosecellNetwork(ptree settings)
    {
        // Posecell size constants
        get_setting_from_ptree(PC_DIM_XYZ, settings, "pc_dim_xyz", 21);
        get_setting_from_ptree(PC_DIM_ABG, settings, "pc_dim_abg", 36);
    
        // Inhibition and excitation configs
        get_setting_from_ptree(PC_W_E_DIM, settings, "pc_w_e_dim", 7);
        get_setting_from_ptree(PC_W_E_VAR, settings, "pc_w_e_var", 1);
        get_setting_from_ptree(PC_W_I_DIM, settings, "pc_w_i_dim", 7);
        get_setting_from_ptree(PC_W_I_VAR, settings, "pc_w_i_var", 1);
        get_setting_from_ptree(PC_GLOBAL_INHIB, settings, "pc_global_inhib", 0.00002);
        get_setting_from_ptree(PC_C_SIZE_XYZ, settings, "pc_cell_x_size", 1.0);

        // Sensing mode (0: vision, 1: tactile, 2: both)
        get_setting_from_ptree(SENSOR_MODE, settings, "sensor_mode", 2);

        PC_C_SIZE_ABG = (2.0 * M_PI) / PC_DIM_ABG;

        STARTING_FACTOR_X = 0.5;
        STARTING_FACTOR_Y = 0.5;
        STARTING_FACTOR_G = 0.5;

        // Set up the excite pdf function
        double total = 0;
        int dim_center = PC_W_E_DIM / 2;
        int i,j,k;
        for (k = 0; k < PC_W_E_DIM; k++)
        { 
            for (j = 0; j < PC_W_E_DIM; j++)
            {
                for (i = 0; i < PC_W_E_DIM; i++)
                {
                    PC_W_EXCITE.push_back(norm1d(PC_W_E_VAR, k, dim_center) * norm2d(PC_W_E_VAR, i, j, dim_center));
                    total += PC_W_EXCITE.at(PC_W_EXCITE.size() - 1);
                }
            }
        }
        for (i = 0; i < PC_W_EXCITE.size(); i++)
        {
            PC_W_EXCITE.at(i) = PC_W_EXCITE.at(i) / total;
        }

        // Set up the inhibit pdf function
        total = 0;
        dim_center = PC_W_I_DIM / 2;
        for (k = 0; k < PC_W_I_DIM; k++)
        {
            for (j = 0; j < PC_W_I_DIM; j++)
            {
                for (i = 0; i < PC_W_I_DIM; i++)
                {
                    PC_W_INHIB.push_back(norm1d(PC_W_I_VAR, k , dim_center) * norm2d(PC_W_I_VAR, i, j, dim_center));
                    total += PC_W_INHIB.at(PC_W_INHIB.size() - 1);
                }
            }
        }
        for (i = 0; i < PC_W_INHIB.size(); i++)
        {
            PC_W_INHIB.at(i) = PC_W_INHIB.at(i) / total;
        }
       
        get_setting_from_ptree(VT_ACTIVE_DECAY, settings, "vt_active_decay", 1.0);
        get_setting_from_ptree(PC_VT_INJECT_ENERGY, settings, "pc_vt_inject_energy", 0.15);
        get_setting_from_ptree(PC_VT_RESTORE, settings, "pc_vt_restore", 0.05);
        get_setting_from_ptree(EXP_DELTA_PC_THRESHOLD, settings, "exp_delta_pc_threshold", 2.0);

        /* Over how many posecells hould be averaged when finding the center of
         * the activation package */
        PC_CELLS_TO_AVG = 3;

        /* Generate the sine and cosine lookups for wrapping */
        for (i = 0; i < PC_DIM_XYZ; i++)
        {
            PC_XYZ_SUM_SIN_LOOKUP.push_back(sin((double)(i+1) * 2.0 * M_PI / (double)PC_DIM_XYZ)); 
            PC_XYZ_SUM_COS_LOOKUP.push_back(cos((double)(i+1) * 2.0 * M_PI / (double)PC_DIM_XYZ)); 
        }
        for (i = 0; i < PC_DIM_ABG; i++)
        {
            PC_ABG_SUM_SIN_LOOKUP.push_back(sin((double)(i+1) * 2.0 * M_PI / (double)PC_DIM_ABG)); 
            PC_ABG_SUM_COS_LOOKUP.push_back(cos((double)(i+1) * 2.0 * M_PI / (double)PC_DIM_ABG)); 
        }

        // Status variables for whether the odometry and visual template have been updated
        odo_update = false;
        vita_update = false;

        // Set the initial best estimates to the center of the posecell array
        best_x = floor((double)PC_DIM_XYZ*STARTING_FACTOR_X);
        best_y = floor((double)PC_DIM_XYZ*STARTING_FACTOR_Y);
        best_g = floor((double)PC_DIM_ABG*STARTING_FACTOR_G);
       
        posecells = new Posecells(PC_DIM_XYZ, PC_DIM_ABG, 2, 1);
        // Set the initial pose in the posecell array to the center of all dimensions
        posecells->set_posecell_value((int)best_x, (int)best_y, (int)best_g, 1.0);
        posecells_new = new Posecells(PC_DIM_XYZ, PC_DIM_ABG, 2, 1);
        pc_rot = new vector<double>(pow(PC_DIM_XYZ+2, 2), 0);
        pc_rot3 = new vector<double>(pow(PC_DIM_XYZ, 2), 0);
        pc_rot2 = new vector<double>(pow(PC_DIM_XYZ+2, 2), 0);

        stop_after = 0;

    }
     /* When new odometry is received the activation package is moved by exciting and inhibiting
     * the poseecell network */
    void PosecellNetwork::on_odo(double vtrans_x, double vtrans_y, double vrot, double time_diff_s)
    {
        if (stop_after >= 10000000000)
        {
           exit(0); 
        }
        else
        {
            stop_after++;
        }

        // Computing translation and rotation velocity based on the time difference between the odometry callbacks in main_pc.cpp
        vtrans_x = vtrans_x * time_diff_s;
        vtrans_y = vtrans_y * time_diff_s;
        vrot = vrot * time_diff_s;
        
        posecells_new->zero_content();
        excite();
        posecells->set_vector(posecells_new->get_vector());
        posecells_new->zero_content();
        inhibit();
        posecells->global_inhib(posecells_new->get_vector(), PC_GLOBAL_INHIB);
        posecells->normalize();
        
        float vtrans = sqrt(pow(vtrans_x,2)+pow(vtrans_y,2));
        path_integration(vtrans, vrot);


        double max = find_best();
              
        odo_update = true;
    } 
    
    void PosecellNetwork::create_experience()
    {
        PosecellVisualTemplate * pcvt = &visual_templates[current_vt];
        experiences.resize(experiences.size() + 1);
        current_exp = experiences.size() - 1;
        PosecellExperience * exp = &experiences[current_exp];
        exp->pcxp_x = x();
        exp->pcxp_y = y();
        exp->pcxp_g = g();
        pcvt->exps.push_back(current_exp);
    }

    void PosecellNetwork::create_view_template()
    {
        PosecellVisualTemplate * pcvt;
        visual_templates.resize(visual_templates.size() + 1);
        pcvt = &visual_templates[visual_templates.size() - 1];
        pcvt->pcvt_x = x();
        pcvt->pcvt_y = y();
        pcvt->pcvt_g = g();
        pcvt->decay = VT_ACTIVE_DECAY;
    }

    /*! This function is called whenever a new view template is received in main_pc.cpp.
     * If the visual template with id vt does not exist yet, create a new template with
     * the given id and add it to the visual_templateslist.
     * Set the visual templates x,y,g to the best values found by find_best()
     */
    void PosecellNetwork::on_vita_template(int vt, double vt_rad)
    {
        PosecellVisualTemplate * pcvt;
        if (vt >= (int)visual_templates.size())
        {
            create_view_template();
            assert (vt == (int)visual_templates.size()-1);    
        }
        else
        {
            // The template exists already and we will reuse it
            pcvt = &visual_templates[vt];

            // This prevents energy from being injected in recently created visual templates
            if (vt < ((int)visual_templates.size() - 10))
            {
                if (vt != current_vt)
                {
                } else {
                    pcvt->decay += VT_ACTIVE_DECAY;
                }

                // This line is magic. Taken from micheal balls implementation at
                // https://github.com/davidmball/ratslam/blob/ratslam_ros/src/ratslam/posecell_network.cpp
                // The inject energy is computed here
                double energy = PC_VT_INJECT_ENERGY * 1.0 / 30.0 * (30.0 - exp(1.2 * pcvt->decay));
                // If there is energy to inject
                if (energy > 0)
                {
                    // Transform the visual template relative angle to a posecell visual template angle
                    // Scale vt_rad_g (the relative angle around the z axis) to PC_DIM_ABG.
                    // For example if we have PC_DIM_ABG = 36 and a relative angle
                    // vt_rad_g = 1 rad -> vt_delta_pcvt_g = 5.76 (out of 36) 
                    // vt_rad_g is always set to zero in local_view_match.cpp if the mode is not panoramic
                    vt_delta_pcvt_g = vt_rad / (2.0 * M_PI) * PC_DIM_ABG;
                    vt_delta_pcvt_g = 0;
                    double pc_g_corrected = pcvt->pcvt_g + vt_delta_pcvt_g;
                    // Wrap around, if pcvt_a + vt_delta_pcvt_a is larger than PC_DIM_ABG or less than zero, wrap around
                    if (pc_g_corrected < 0) 
                        pc_g_corrected = PC_DIM_ABG + pc_g_corrected;
                    if (pc_g_corrected >= PC_DIM_ABG)
                        pc_g_corrected = pc_g_corrected - PC_DIM_ABG;
                    inject((int)pcvt->pcvt_x, (int)pcvt->pcvt_y, (int)pc_g_corrected, energy);
                  }
            }
        }
        for (int i = 0; i < (int)visual_templates.size(); i++){
            visual_templates[i].decay -= PC_VT_RESTORE;
            if (visual_templates[i].decay < VT_ACTIVE_DECAY);
                visual_templates[i].decay = VT_ACTIVE_DECAY;
        }
        prev_vt = current_vt;
        current_vt = vt;
        vita_update = true;
    }

    PosecellNetwork::PosecellAction PosecellNetwork::get_action()
    {
        PosecellExperience * experience;
        double delta_pc; 
        PosecellAction action = NO_ACTION;
        // If the odometry and visuo tactile template have been updated, reset the status variables
        if (odo_update && vita_update)
        {
            odo_update = false;
            vita_update = false;
        } else {
            return action;
        }

        if ((int)visual_templates.size() == 0)
        {
            return action;
        }

        if ((int)experiences.size() == 0)
        {
            create_experience();
            action = CREATE_NODE;
            //cout << "CREATE_NODE: experience size = 0" << endl;
        }
        else
        {
            experience = &experiences[current_exp];
            delta_pc = get_delta_pc(experience->pcxp_x, experience->pcxp_y, experience->pcxp_g);
            PosecellVisualTemplate * pcvt = &visual_templates[current_vt];
            if ((int)pcvt->exps.size() == 0)
            {
                create_experience();
                action = CREATE_NODE;
                //cout << "CREATE_NODE: posecell view template exp size = 0" << endl;
            }
            else if (delta_pc > EXP_DELTA_PC_THRESHOLD || current_vt != prev_vt)
            {
                // If the delta_pc, the difference between two posecells
                // Go through all experiences associatioed with the current view and find the one with the closest delta_pc
                int matched_exp_id = -1;
                unsigned int i;
                int min_delta_id = -1;
                double min_delta = DBL_MAX;
                double delta_pc;
                for (i = 0; i < (int)pcvt->exps.size(); i++)
                {
                    // make sure we arent' comparing to the current experience
                    if (current_exp == pcvt->exps[i]) { continue; } 

                    experience = &experiences[pcvt->exps[i]];
                    delta_pc = get_delta_pc(experience->pcxp_x, experience->pcxp_y, experience->pcxp_g);
                    if (delta_pc < min_delta)
                    {
                        min_delta = delta_pc;
                        min_delta_id = pcvt->exps[i];
                    }
                }
                //cout << "min_delta: " << min_delta << " delta_pc: " << delta_pc << endl;
                //cout << " min_delta_id " << min_delta_id << " EXP_THR: " << EXP_DELTA_PC_THRESHOLD << endl;
                if (min_delta < EXP_DELTA_PC_THRESHOLD)
                {
                    //cout << "CREATE_EDGE: min_delta < EXP_DELTA_PC_THRESHOLD" << endl;
                    matched_exp_id = min_delta_id;
                    action = CREATE_EDGE;
                }
                if (current_exp != (unsigned)matched_exp_id)
                {
                    if (matched_exp_id == -1)
                    {   
                        //cout << "CREATE_NODE matched_exp_id == -1" << endl;
                        create_experience();
                        action = CREATE_NODE;
                    }
                    else{
                        current_exp = matched_exp_id;
                        if (action == NO_ACTION)
                        {
                            //cout << "SET_NODE matched_exp_id >= 0" << endl;
                            action = SET_NODE;
                        }
                    }
                } 
                else if (current_vt == prev_vt)
                {
                    create_experience();
                    //cout << "CREATE NODE current_vt == prev_vt" << endl;
                    action = CREATE_NODE;
                }
            }
        }
        return action;
    }
    bool PosecellNetwork::inject(int act_x, int act_y, int act_g, double energy)
    {
        if (act_x < PC_DIM_XYZ && act_x >= 0 &&
                act_y < PC_DIM_XYZ && act_y >= 0 &&
                act_g < PC_DIM_ABG && act_g >= 0)
        {
            //cout << "#################  ###### injecting: " << "act_x " << act_x << " act_y " << act_y << " act_g " << act_g << endl;
            posecells->set_posecell_value(act_x, act_y, act_g, posecells->get_posecell_value(act_x, act_y, act_g) + energy);
        }
        return true;
    }
    double PosecellNetwork::get_delta_pc(double x, double y, double g){
        double pc_g_corrected = best_g - vt_delta_pcvt_g;
        // Wrap around, if best_a - vt_delta_pcvt_a is larger than PC_DIM_ABG or less than zero, wrap around
        if (pc_g_corrected < 0) 
            pc_g_corrected = PC_DIM_ABG + pc_g_corrected;
        if (pc_g_corrected >= PC_DIM_ABG)
            pc_g_corrected = pc_g_corrected - PC_DIM_ABG;
        return sqrt(pow(get_min_delta(best_x, x, PC_DIM_XYZ),2) + 
                    pow(get_min_delta(best_y, y, PC_DIM_XYZ),2) +
                    pow(get_min_delta(pc_g_corrected, g, PC_DIM_ABG),2));

    }
    double PosecellNetwork::get_min_delta(double d1, double d2, double max) { return min(abs(d1-d2), max-abs(d1-d2)); }

    /* Posecell related things */

    bool PosecellNetwork::excite()
    {
        int i, j, k;
        for (i = 0; i < PC_DIM_XYZ; i++)
        {
            for (j = 0; j < PC_DIM_XYZ; j++)
            {
                for (k = 0; k < PC_DIM_ABG; k++)
                {
                    if (posecells->get_posecell_value(i, j, k) != 0)
                    {
                        excite_helper(i, j, k);
                    }
                }
            }
        }
        return true;
    }
    bool PosecellNetwork::excite_helper(int x, int y, int g)
    {
        int xl, yl, gl = 0;
        int xw, yw, gw = 0;
        int excite_index = 0;
        for (gl = g; gl < g + PC_W_E_DIM; gl ++)
        {
            for (yl = y; yl < y + PC_W_E_DIM; yl++)
            {
                for (xl = x; xl < x + PC_W_E_DIM; xl++)
                {
                    xw = wrap_index(xl-(PC_W_E_DIM/2) ,PC_DIM_XYZ);
                    yw = wrap_index(yl-(PC_W_E_DIM/2) ,PC_DIM_XYZ);
                    gw = wrap_index(gl-(PC_W_E_DIM/2) ,PC_DIM_ABG);
                    posecells_new->set_posecell_value(xw, yw, gw, posecells_new->get_posecell_value(xw, yw, gw) + posecells->get_posecell_value(x, y, g) * PC_W_EXCITE[excite_index++]);
                }
            }
        }
        return true;
    }
    double PosecellNetwork::norm2d(double std, int x, int y, int dim_center)
    {
        return 1.0 / (std * sqrt(2.0 * M_PI)) * exp((-pow(x - dim_center,2) - pow(y - dim_center,2)) / (2.0 * std * std));
    }
    double PosecellNetwork::norm1d(double std, int g, int dim_center)
    {
        return 1 / (std * 2.0 * M_PI) * exp(-pow(g - dim_center, 2) / (2 * std * std));
    }

    bool PosecellNetwork::inhibit()
    {
        int i, j, k;
        for (i = 0; i < PC_DIM_XYZ; i++)
        {
            for (j = 0; j < PC_DIM_XYZ; j++)
            {
                for (k = 0; k < PC_DIM_ABG; k++)
                {
                    if (posecells->get_posecell_value(i, j, k) != 0)
                    {
                        inhibit_helper(i, j, k);
                    }
                }
            }
        }
        return true;
    }
    bool PosecellNetwork::inhibit_helper(int x, int y, int g)
    {
        int xl, yl, gl = 0;
        int xw, yw, gw = 0;
        int inhibit_index = 0;
        for (gl = g; gl < g + PC_W_I_DIM; gl ++)
        {
            for (yl = y; yl < y + PC_W_I_DIM; yl++)
            {
                for (xl = x; xl < x + PC_W_I_DIM; xl++)
                {
                    xw = wrap_index(xl-PC_W_I_DIM/2 ,PC_DIM_XYZ);
                    yw = wrap_index(yl-PC_W_I_DIM/2 ,PC_DIM_XYZ);
                    gw = wrap_index(gl-PC_W_I_DIM/2 ,PC_DIM_ABG);
                    posecells_new->set_posecell_value(xw, yw, gw, posecells_new->get_posecell_value(xw, yw, gw) + posecells->get_posecell_value(x, y, g) * PC_W_INHIB[inhibit_index++]);
                }
            }
        }
        return true;
    }
    double PosecellNetwork::find_best()
    {
        int i, j, k;
        double x = -1, y = -1, z = -1;
        double a = -1, b = -1, g = -1;
        // Find the max activated cell
        double max = -1;
        double cur_posecell = 0;
        for (k = 0; k < PC_DIM_ABG; k++)
        {
            for (j = 0; j < PC_DIM_XYZ; j++)
            {
                for (i = 0; i < PC_DIM_XYZ; i++)
                {
                    cur_posecell = posecells->get_posecell_value(i, j, k);
                    if (cur_posecell > max)
                    {
                        max = cur_posecell;
                        x = double(i);
                        y = double(j);
                        z = 0;
                        a = 0;
                        b = 0;
                        g = double(k);
                    }
                }
            }
        }
        // Take the maximum activated cell +- PC_CELLS_TO_AVG in 3D space and get the sums for each axis
        std::vector<double> x_sums (PC_DIM_XYZ);
        std::vector<double> y_sums (PC_DIM_XYZ);
        std::vector<double> g_sums (PC_DIM_ABG);
        // Iterate through the values in the posecell 6D hypercube
        for (k = (int)g; k < g + PC_CELLS_TO_AVG * 2 + 1; k++)
        {
            for (j = (int)y; j < y + PC_CELLS_TO_AVG * 2 + 1; j++)
            {
                for (i = (int)x; i < x + PC_CELLS_TO_AVG * 2 + 1; i++)
                {
                    // For each dimensions we now have to subtract PC_CELLS_TO_AVG to make sure the actual value is in the center
                    // However if for example x < PC_CELLS_TO_AVG we have to make sure to wrap to prevent from acessing outside of the sums arrays
                    int xw = wrap_index(i - PC_CELLS_TO_AVG, PC_DIM_XYZ);
                    int yw = wrap_index(j - PC_CELLS_TO_AVG, PC_DIM_XYZ);
                    int gw = wrap_index(k - PC_CELLS_TO_AVG, PC_DIM_ABG);
                    cur_posecell = posecells->get_posecell_value(xw,yw,gw);
                    x_sums.at(xw) = x_sums.at(xw) + cur_posecell;
                    y_sums.at(yw) = y_sums.at(yw) + cur_posecell;
                    g_sums.at(gw) = g_sums.at(gw) + cur_posecell;
                }
            }
        }
        // Now find x,y,z,a,b,g using population vector decoding to handle the wrap around
        double sum_x1 = 0;
        double sum_x2 = 0;
        double sum_y1 = 0;
        double sum_y2 = 0;
        for (i = 0; i < PC_DIM_XYZ; i++)
        {
            sum_x1 += PC_XYZ_SUM_SIN_LOOKUP[i] * x_sums[i];
            sum_x2 += PC_XYZ_SUM_COS_LOOKUP[i] * x_sums[i];
            sum_y1 += PC_XYZ_SUM_SIN_LOOKUP[i] * y_sums[i];
            sum_y2 += PC_XYZ_SUM_COS_LOOKUP[i] * y_sums[i];
        }
        double sum_g1 = 0;
        double sum_g2 = 0;
        for (i = 0; i < PC_DIM_ABG; i++)
        {
            sum_g1 += PC_ABG_SUM_SIN_LOOKUP[i] * g_sums[i];
            sum_g2 += PC_ABG_SUM_COS_LOOKUP[i] * g_sums[i];
        }
        x = find_best_helper(sum_x1, sum_x2, PC_DIM_XYZ);
        y = find_best_helper(sum_y1, sum_y2, PC_DIM_XYZ);
        g = find_best_helper(sum_g1, sum_g2, PC_DIM_ABG);

        // Save the new found best x,y,z,a,b,g
        best_x = x;
        best_y = y;
        best_g = g;

        return max;
    }
    int PosecellNetwork::wrap_index(int index, int size)
    {
        while (index < 0)
        {
            index += size;
        }
        while (index >= size)
        {
            index -= size;
        }
        return index;
    }
    double PosecellNetwork::find_best_helper(double v1, double v2 , int size)
    {
        double v = atan2(v1, v2) * (size) / (2.0 * M_PI) - 1;
        while (v < 0)
        {
            v += size;
        } 
        while (v > size)
        {
            v -= size;
        } 
        return v;
    }
    int PosecellNetwork::count_nonzero(vector<double> in)
    {
        int nonzero = 0;
        for (int j = 0; j < PC_DIM_XYZ; j++)
        {
            for (int i = 0; i<PC_DIM_XYZ; i++)
            {
                if (in.at(get_flat_index(i,j,PC_DIM_XYZ)) > 0 )
                {
                    nonzero++;
                }
            }
        }
        return nonzero;
    }

    void PosecellNetwork::printout_vector(vector<double> in)
    {
        for (int j = PC_DIM_XYZ-1; j>=0; j--)
        {
            for (int i = 0; i<PC_DIM_XYZ; i++)
            {
                if (in.at(get_flat_index(i,j,PC_DIM_XYZ)) > 0 )
                    printf(" %.10f", in.at(get_flat_index(i,j,PC_DIM_XYZ)));
            }
            //cout << endl;
        }
        //cout << endl;
    }
    //void PosecellNetwork::path_integration(double vtrans, double vrot, double angle_to_add)
    void PosecellNetwork::path_integration(double vtrans, double vrot)
    {
        // We have to scale the translational velocity to the pose cell x size
        vtrans /= PC_C_SIZE_XYZ;
        // This angle is used to flip the vtrans sign. Invert the sign and add 180 degrees.
        angle_to_add = 0;
          
        // Take the absolute value of the translational velocity
        if (vtrans < 0)
        { 
            vtrans *= -1;
            angle_to_add += M_PI;
        } 
        //cout << "path int vtrans: " << vtrans << " vrot: " << vrot << " angle_to_add " << angle_to_add << endl;
        // Iterate through all xy planes for all angles gamma
        int g, i, j;
        double dir, dir90, weight_sw, weight_se, weight_nw, weight_ne;
        // Iterate through all xy layers of posecells for a given yaw angle g
        for (g = 0; g < PC_DIM_ABG; g++)
        { 
            // Scale the angle properly and add the angle to add in case vtrans was negative
            dir = g * PC_C_SIZE_ABG + angle_to_add;
          
            // Rotate the posecell by 90 degrees
            posecells->rot90((int)floor(dir * 2.0 / M_PI), g);        
          
            dir90 = dir - floor(dir * 2 / M_PI) * M_PI / 2;
            //cout << "dir90: " << dir90 << " dir: " << dir << " g: " << g << " PC_C_SIZE_ABG: " << PC_C_SIZE_ABG << endl;
          
            // Extend the posecells one unit in each direction
            // Work out the weight contribution to the NE cell from SW, NW, SE cells given vtrans and the direction
            // pc_rot contains 1 "slice" of the space in which the robot can be located. In our case the robot is only active in
            // the xy place, thus pc_rot contains the slice of the xy plane for an angle gamma
            for (i = 0; i < pc_rot->size(); i++)
            {
                pc_rot->at(i) = 0;
                pc_rot2->at(i) = 0;
            }
            for (i = 0; i < PC_DIM_XYZ; i++)
            {
                for (j = 0; j < PC_DIM_XYZ; j++)
                {
                   pc_rot->at(get_flat_index(i+1,j+1,PC_DIM_XYZ+2)) = posecells->get_posecell_value(i,j,g);
                }
            }
            weight_sw = pow(vtrans, 2) * cos(dir90) * sin(dir90);
            weight_se = vtrans * sin(dir90) * (1.0 - vtrans * cos(dir90));
            weight_nw = vtrans * cos(dir90) * (1.0 - vtrans * sin(dir90));
            weight_ne = 1.0 - weight_sw - weight_se - weight_nw;
            //cout << "dir: " << dir << " dir90: " << dir90 << endl;
            //cout << "g: " << g << " weight_sw: " << weight_sw << " weight_se: " << weight_se << " weight_nw: " << weight_nw << " weight_ne: " << weight_ne << endl;
            if (weight_sw < 0 || weight_se < 0 || weight_nw < 0 || weight_ne < 0)
            {
                printf("WARNING: weights are negative, vtrans(%f) is either negative or too big\n", vtrans);
                printf("WARNING: continuing, but expect possible failures soon! Update POSECELL_VTRANS_SCALING to fix this.\n");
            }


            // Circular shift and multiply by the contributing weights
            // Then copy the shifted elements for the wrap around
            // First element
            pc_rot2->at(0) = pc_rot->at(0) * weight_ne + pc_rot->at(get_flat_index(PC_DIM_XYZ+1,0,PC_DIM_XYZ+2)) * weight_se + pc_rot->at(get_flat_index(0, PC_DIM_XYZ + 1, PC_DIM_XYZ + 2)) * weight_nw;
            
            // First row
            for (i = 1; i < PC_DIM_XYZ + 2; i++)
            {
                pc_rot2->at(get_flat_index(i,0,PC_DIM_XYZ+2)) = pc_rot->at(get_flat_index(i,0,PC_DIM_XYZ+2)) * weight_ne + pc_rot->at(get_flat_index(i-1, 0, PC_DIM_XYZ + 2)) * weight_se + pc_rot->at(get_flat_index(i, PC_DIM_XYZ+1, PC_DIM_XYZ+2)) * weight_nw;
            }
            for (j = 1; j < PC_DIM_XYZ + 2; j++)
            {
                // First column
                pc_rot2->at(get_flat_index(0,j,PC_DIM_XYZ+2)) = pc_rot->at(get_flat_index(0,j,PC_DIM_XYZ+2)) * weight_ne + pc_rot->at(get_flat_index(PC_DIM_XYZ+1,j,PC_DIM_XYZ+2)) * weight_se + pc_rot->at(get_flat_index(0,j-1,PC_DIM_XYZ+2)) * weight_nw;
                // All other
                for (i = 1; i < PC_DIM_XYZ + 2; i++)
                {
                    pc_rot2->at(get_flat_index(i,j,PC_DIM_XYZ+2)) = pc_rot->at(get_flat_index(i,j,PC_DIM_XYZ+2)) * weight_ne + pc_rot->at(get_flat_index(i-1,j,PC_DIM_XYZ+2)) * weight_se + pc_rot->at(get_flat_index(i,j-1,PC_DIM_XYZ+2)) * weight_nw;
                }
            }
            
            circshift2d(*pc_rot, PC_DIM_XYZ + 2, 1, 1);
            for (i = 0; i < pc_rot->size(); i++)
            {
                pc_rot2->at(i) = pc_rot2->at(i) + pc_rot->at(i) * weight_sw;
            }

            // Copy the temporary posecell layer back to the posecells array
            for (j = 0; j < PC_DIM_XYZ; j++)
            {
                for (i = 0; i < PC_DIM_XYZ; i++)
                {
                    posecells->set_posecell_value(i,j,g,pc_rot2->at(i+1 + (PC_DIM_XYZ+2)*(j+1)));
                }
            }
         
            for (i = 1; i < PC_DIM_XYZ; i++)
            {
                posecells->set_posecell_value(i,0,g,posecells->get_posecell_value(i,0,g)+pc_rot2->at(get_flat_index(i+1,PC_DIM_XYZ+1,PC_DIM_XYZ + 2)));
            }
            for (j = 1; j < PC_DIM_XYZ; j++)
            {
                posecells->set_posecell_value(0,j,g,posecells->get_posecell_value(0,j,g)+pc_rot2->at(get_flat_index(PC_DIM_XYZ+1,j+1,PC_DIM_XYZ + 2)));
            }
            posecells->set_posecell_value(0,0,g, posecells->get_posecell_value(0,0,g) + pc_rot2->at(get_flat_index(PC_DIM_XYZ+1, PC_DIM_XYZ+1, PC_DIM_XYZ + 2))); 
         
            // Rotate the posecells array back
            posecells->rot90(4 - (int)floor(dir * 2.0 / M_PI), g);
        }
        // Path integration Gamma
        // Shift the posecells +/- gamma given by vrot
        if (vrot != 0)
        { 
            int i, j, k;
            double weight;
            double sign_vrot;
            int shifty1, shifty2;
            int newk1, newk2;
            // Determine the partial shift amount
            weight = abs(vrot) / PC_C_SIZE_ABG;
            while (weight > 1) { weight -= 1; }
            if (weight == 0) { weight = 1.0; }
            // Work on a temporary copy of the posecells
            posecells_new->set_vector(posecells->get_vector());
            sign_vrot = vrot < 0 ? -1 : 1;
            shifty1 = (int)(sign_vrot * floor(abs(vrot) / PC_C_SIZE_ABG));
            shifty2 = (int)(sign_vrot * ceil(abs(vrot) / PC_C_SIZE_ABG));
            while (shifty1 > 0) { shifty1 -= PC_DIM_ABG; }
            while (shifty2 > 0) { shifty2 -= PC_DIM_ABG; }
          
            for (i = 0; i < PC_DIM_XYZ; i++)
            {
                for (j = 0; j < PC_DIM_XYZ; j ++)
                {
                    for (k = 0; k < PC_DIM_ABG; k++)
                    {
                        newk1 = (k - shifty1) % PC_DIM_ABG;
                        newk2 = (k - shifty2) % PC_DIM_ABG;
                        assert(newk1 < PC_DIM_ABG);
                        assert(newk2 < PC_DIM_ABG);
                        assert(newk1 >= 0);
                        assert(newk2 >= 0);
                        posecells->set_posecell_value(i,j,k,posecells_new->get_posecell_value(i,j,newk1) * (1.0 - weight) + posecells_new->get_posecell_value(i, j, newk2) * weight);
                        assert(posecells->get_posecell_value(i,j,k) >= 0);
                    }   
                }   
            }   
        }
    }
    void PosecellNetwork::circshift2d(vector<double>& pc_rot_in, int dim, int shiftx, int shifty)
    {
        int splity;
        if (shifty == 0)
        {
            if (shiftx == 0)
            {
                return;
            }
        } 
        else if (shifty > 0)
        { 
            splity = (dim-shifty) * dim + 1;
            rotate(pc_rot_in.begin(), pc_rot_in.begin() + splity, pc_rot_in.end());
        } 
        else if (shifty < 0)
        { 
            // This code is never used
            // Untested
            splity = -shifty*dim;
            rotate(pc_rot_in.begin(), pc_rot_in.begin() + splity, pc_rot_in.end());
        }

        if (shiftx > 0)
        {
            int x,y;
            double temp, temp2;
            double first, last;
            for (y = 0; y < dim; y++)
            {
                temp = pc_rot_in.at(0 + dim * y);
                last = pc_rot_in.at(dim-1 + dim * y);
                for (x = 1; x < dim; x++)
                {
                    temp2 = pc_rot_in.at(x + dim * y);
                    pc_rot_in.at(x + dim * y) = temp;
                    temp = temp2;
                }
                pc_rot_in.at(0 + dim * y) = last;
            }
        }
        else if (shiftx < 0)
        {
            int x,y;
            double temp, temp2;
            double first, last;
            for (y = 0; y < dim; y++)
            {
                temp = pc_rot_in.at(dim-1 + dim * y);
                first = pc_rot_in.at(0 + dim * y);
                for (x = dim-2; x >= 0; x--)
                {
                    temp2 = pc_rot_in.at(x + dim * y);
                    pc_rot_in.at(x + dim * y) = temp;
                    temp = temp2;
                }
                pc_rot_in.at(dim-1 + dim * y) = first;
            }
        }
    }
    int PosecellNetwork::get_flat_index(int x, int y, int xsize)
    {
        return (unsigned int) (x + xsize * y);
    }
}
