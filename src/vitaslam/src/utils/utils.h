/*
 * openRatSLAM
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _RATSLAM_UTILS_H
#define _RATSLAM_UTILS_H

#include <boost/property_tree/ptree.hpp>
using boost::property_tree::ptree;

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <iostream>
#include <float.h>

//! utils - General purpose utility helper functions mainly for angles and readings settings from property trees
namespace vitaslam {

template<typename T>
//! This function returns a setting from the given property tree
/*! The Property Tree library provides a data structure that stores an arbitrarily deeply nested tree of values, indexed at each level by some key. Each node of the tree stores its own value, plus an ordered list of its subnodes and their keys. The tree allows easy access to any of its nodes by means of a path, which is a concatenation of multiple keys.*/
/*!
  \param &var Pointer to the setting tree object with the given name
  \param settings The settings/property_tree from which the function reads
  \param name The name of the setting to be read form the property tree
  \param default_value Return a default value if the property tree path is bad
  \return void, (&var)
*/
inline void get_setting_from_ptree(T & var, boost::property_tree::ptree & settings, std::string name, T default_value)
{
	try
	{
		var = settings.get<T>(name);
	}
	catch(boost::property_tree::ptree_bad_path pbp)
	{
		var = default_value;
		std::cout << "SETTINGS(warning): " << name << " not found so default (" << default_value << ") used." << std::endl;
	}
}
//! This function returns a child of a setting within a property tree
/*! The Property Tree library provides a data structure that stores an arbitrarily deeply nested tree of values, indexed at each level by some key. Each node of the tree stores its own value, plus an ordered list of its subnodes and their keys. The tree allows easy access to any of its nodes by means of a path, which is a concatenation of multiple keys.*/
/*!
  \param child Pointer to the child tree object that the function retrieves form the property tree
  \param settings A pointer to the settings tree object whose child the function shall find
  \param name The name of the child tree object
  \return void, (&child)
*/
inline bool get_setting_child(boost::property_tree::ptree & child, boost::property_tree::ptree & settings, std::string name, bool pause_on_error = true)
{
	try
	{
		child = settings.get_child(name);
	}
	catch(boost::property_tree::ptree_bad_path pbp)
	{
		std::cout << "SETTINGS(error): " << name << " child not found." << std::endl;
		return false;
	}
	return true;
}
//! Clip the input angle to between 0 and 2 pi radians
/*!
  \param angle : double, the input angle to clip
  \return angle : double,  the clipped angle in the range from 0 to 2 pi radians 
*/
inline double clip_rad_360(double angle)
{
    while (angle < 0)
        angle += 2.0 * M_PI;

    while (angle >= 2.0 * M_PI)
        angle -= 2.0 * M_PI;
 
    return angle;
}
//! Clip the input angle to between -pi and pi radians
/*!
  \param angle : double, the input angle to clip
  \return angle : double,  the clipped angle in the range from -pi to  pi radians 
*/
inline double clip_rad_180(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;

    while (angle <= -M_PI)
        angle += 2.0 * M_PI;
    
    return angle;
}
//! Get the signed delta angle form two angles handling the wrap from 2 pi to 0
/*!
  \param angle1 : double, first input angle
  \param angle2 : double, second input angle
  \return angle : double, angle in the range of 0 to 2pi depending on the direction from angle2-angle1
*/
inline double get_signed_delta_rad(double angle1, double angle2)
{
    // Get the direction of the angle
    double dir = clip_rad_180(angle2 - angle1);

    // Get absolute difference in angles
    double delta_angle = clip_rad_360(angle1) - clip_rad_360(angle2);
	delta_angle = fabs(delta_angle);

    // Return angle in the range of 0 to 2pi, also depending on the direction of the angle
    if (delta_angle < 2.0 * M_PI - delta_angle)
    {
        if (dir > 0)
            return delta_angle;
        else
            return -delta_angle;
    }
    else
    {
        if (dir > 0)
            return 2.0 * M_PI - delta_angle;
        else
            return -(2.0 * M_PI - delta_angle);
    }
}
inline void printout_vector_double(std::vector<double> toprint)
{
    for (std::vector<double>::const_iterator i = toprint.begin(); i != toprint.end(); i++)
    {
        std::cout << *i << ",";
    }
    std::cout << std::endl;
}

}; // namspace ratslam

#endif // _RATSLAM_UTILS_H
