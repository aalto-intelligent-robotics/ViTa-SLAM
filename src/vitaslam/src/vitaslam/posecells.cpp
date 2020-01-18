#include "posecells.h"
#include <assert.h>

using namespace std;

namespace vitaslam
{
    Posecells::Posecells(int xyz, int abg, int dims_xyz, int dims_abg)
    {
        assert(xyz > 0);
        assert(abg > 0);
        assert(dims_xyz > 0);
        assert(dims_abg > 0);
        DIM_XYZ = xyz;
        DIM_ABG = abg;
        total = 0;
        p = vector<double>(pow(DIM_XYZ,dims_xyz)*pow(DIM_ABG,dims_abg));
    }
    int Posecells::get_size()
    {
        return p.size();
    }

    double Posecells::get_posecell_value(int x, int y, int g)
    {
        assert(x >= 0);
        assert(y >= 0);
        assert(g >= 0);
        unsigned int flat_index = get_flattened_index(x, y, g); 
        return p.at(flat_index);
    }

    double Posecells::get_posecell_value(int flat_index) 
    {
        assert(flat_index >= 0);
        assert(flat_index < get_size());
        return p.at(flat_index);
    }

    void Posecells::set_posecell_value(int x, int y, int g, double value)
    {
        assert(x >= 0);
        assert(y >= 0);
        assert(g >= 0);
        unsigned int flat_index = get_flattened_index(x, y, g); 
        p.at(flat_index) = value;
    }

    void Posecells::set_posecell_value(int flat_index, double value)
    {
        assert(flat_index >= 0);
        assert(flat_index < get_size());
        p.at(flat_index) = value;
    }

    void Posecells::set_vector(vector<double> in)
    {
        assert(in.size() == get_size());
        for (int i = 0; i < in.size(); i++)
        {
            p.at(i) = in.at(i);
        }
        //p = vector<double>(in);
    }

    vector<double> Posecells::get_vector()
    {
        return p;
    }

    void Posecells::zero_content()
    {
        int i;
        for (i = 0; i < p.size(); i++)
        {
            p.at(i) = 0;
        }
    }

    void Posecells::global_inhib(vector<double> in, double PC_GLOBAL_INHIB)
    {
        assert(in.size() == get_size());
        total = 0;
        int i;
        for (i = 0; i < p.size(); i++)
        {
            p.at(i) = p.at(i) - in.at(i); 
            if (p.at(i) > PC_GLOBAL_INHIB)
            {
                set_posecell_value(i, p.at(i) - PC_GLOBAL_INHIB);
            }
            else
            {
                set_posecell_value(i, 0);
            }
            total += p.at(i);
        }
    }

    void Posecells::normalize()
    {
        int i;
        assert(total > 0);
        for (i = 0; i < get_size(); i++)
        {
           p.at(i) /= total; 
        }
    }

    void Posecells::add_value(int x, int y, int g, double value)
    {
        assert(x >= 0 && y >= 0 && g >= 0);
        unsigned int flat_index = get_flattened_index(x, y, g);
        p.at(flat_index) = p.at(flat_index) + value;
    }

    int Posecells::rot90(int rot, int g)
    {
        assert(g >= 0 && g < DIM_ABG);
        double center = (double)(DIM_XYZ-1)/2.0f;
        double a, b, c, d;
        double tmp_new, tmp_old;
        int i, j, quad, id, jd, is1, js1;
             
        if (rot < 0)
        {   
            rot += 4;
        }   
        switch (rot % 4)
        {   
            case 0:
                return 1;
            case 1:
                a = 0;
                b = -1; 
                c = 1;
                d = 0;
                break;
            case 2:
                a = -1; 
                b = 0;
                c = 0;
                d = -1; 
                break;
            case 3:
                a = 0;
                b = 1;
                c = -1;
                d = 0;
                break;
            default:
                return 1;
        }  
        if (rot % 2 == 1)
        {  
            for (j = 0; j < (int)center + (1 - DIM_XYZ % 2); j++)
            {
                for (i = 0; i < (int)center + 1; i++)
                {
                    id = i;
                    jd = j;
                    tmp_old = get_posecell_value(i,j,g);
                    for (quad = 0;  quad < 4; quad++)
                    {
                        is1 = id;
                        js1 = jd;
                        id = (int)(a * ((float)(is1) - center) + b * ((float)(js1) - center) + center);
                        jd = (int)(c * ((float)(is1) - center) + d * ((float)(js1) - center) + center);
                        tmp_new = get_posecell_value(id, jd, g);
                        set_posecell_value(id, jd, g, tmp_old);
                        tmp_old = tmp_new;
                    }
                }
            }
        } 
        else
        {
            rot90(1, g);
            rot90(1, g);
        }
        return true;
    }

    void Posecells::printout()
    {
        cout << "Printing out posecells array: " << endl;
        int x,y,g;
        for (g = DIM_ABG-1; g >= 0; g--)
        { 
            for (y = DIM_XYZ-1; y>=0; y--)
            {
                for (x = 0; x < DIM_XYZ; x++)
                {
                    printf("%.3f ", get_posecell_value(x,y,g));
                }
                cout << endl;
            }
            cout << endl;
        } 
        cout << "nonzero elements in posecell: " << get_nonzeros() << endl;
        cout << endl;
        cout << endl;

    }
    double Posecells::get_total() 
    {
        double total = 0;
        int i,j,k;
        for (k = 0; k < DIM_ABG; k++)
        { 
            for (i = 0; i < DIM_XYZ; i++)
            {
                for (j = 0; j < DIM_XYZ; j++)
                {
                    total+= get_posecell_value(i,j,k);
                }
            }
        } 
        return total;
    }

    int Posecells::get_nonzeros() 
    {
        int nonzero = 0;
        int i,j,k;
        for (k = 0; k < DIM_ABG; k++)
        { 
            for (i = 0; i < DIM_XYZ; i++)
            {
                for (j = 0; j < DIM_XYZ; j++)
                {
                    if (get_posecell_value(i,j,k) > 0){
                        nonzero++;
                    }
                }
            }
        } 
        return nonzero;
    }

    unsigned int Posecells::get_flattened_index(int x, int y, int g)
    {
        unsigned int flattened_index = (unsigned int) x + DIM_XYZ * (y + DIM_XYZ * g);
        assert(flattened_index < get_size() && flattened_index >= 0);
        return flattened_index;
    }
}
