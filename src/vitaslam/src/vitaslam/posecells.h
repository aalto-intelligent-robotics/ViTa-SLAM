#ifndef _POSECELLS_HPP
#define _POSECELLS_HPP

#include <iostream>
#include <vector>
#include "math.h"

namespace vitaslam
{
    /*! This class provides an abstraction layer to the posecells array with the following functions:
     * - It contains a 1D vector that represents the posecell array with dimensionality dims_xyz X dims_abg
     * - It allows setting and reading values from the 1D vector based on individual indices or a flattened index
     * - It can zero the contents of the posecell array
     * - Global inhibition by subtracting and thresholding a temporary vector (genereated by the inhibit function in posecell_network
     * - Normalizing the posecells array by dividing all energies in the vector through the sum of the energy
     * - In the 3D posecells array case: Rotating a xy slice of the posecells array by multiples of 90 degrees
     * */
    class Posecells{
        public:
            Posecells(int xyz, int abg, int dims_xyz, int dims_abg);
            /*! Returns the number of posecells */
            int get_size();

            /*! Returns the energy of the posecell at the specified position
             * with each dimension being indexed individually */
            double get_posecell_value(int x, int y, int g);

            /*! Returns the energy of the posecell at the specified position
             * with an index referring to the flattened posecells */
            double get_posecell_value(int flat_index);

            /*! Set the energy of the posecell at the specified position
             * with each dimension being indexed individually */
            void set_posecell_value(int x, int y, int g, double value);

            /*! Set the energy of the posecell at the specified position
             * with an index referring to the flattened posecells */
            void set_posecell_value(int flat_index, double value);

            /*! Set the contents of the internal vector to an input vector */
            void set_vector(std::vector<double> in);
            
            /*! Returns the vector that contains all posecells */
            std::vector<double> get_vector();

            /*! Sets all elements of the posecells to zero */
            void zero_content();

            /*! Globally inhibits the posecells by dividing a temporary copy of the posecells
             * array from the posecells array. 
             * This temporary array is generated in inhibit()*/
            void global_inhib(std::vector<double> in, double PC_GLOBAL_INHIB);

            /*! Normalizes the posecells by division by the total energy in all posecells
             * as determined in global_inhib() */
            void normalize();

            /*! Adds the input energy to the posecell value at the given position*/
            void add_value(int x, int y, int g, double value); 

            /*! Rotates a 2D slice of the posecells array at angle gamme by multiples of 90 degrees */
            int rot90(int rot, int g);

            /*! Prints out the contents of the posecells vector */
            void printout();

            int get_nonzeros();
            double get_total();

        private:

            /*! Vector containing the energies of each individual posecell */
            std::vector<double> p;

            /*! Total energy of all posecells */
            double total;

            /*! Size of the translational dimensions */
            unsigned int DIM_ABG;

            /*! Size of the rotiational dimensions */
            unsigned int DIM_XYZ;

            /*! Helper funcitons that convertsl indices to flattened values to access the 1D vector */
            unsigned int get_flattened_index(int x, int y, int g);
    };
}
#endif
