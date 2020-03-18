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

#ifndef PATHGRAPHNODE_H_
#define PATHGRAPHNODE_H_

#include <irrlicht/irrlicht.h>

/*! This class is a */

class PathNode : public irr::scene::ISceneNode
{
public:
    //! Constructor
    /*! Calls the constructor derived from, an irrlicht ISceneNode 
     * with the parameter parent and its scene manager
     * \param *parent : Pointer to an irrlicht ISceneNode
     * */
	PathNode(irr::scene::ISceneNode * parent)
	: irr::scene::ISceneNode(parent, parent->getSceneManager())
	{

	}
    //! Destructor
	~PathNode()
	{

	}

    /*! This function registers the PathNode for rendering if it is 
     * visible
     * */
	virtual void OnRegisterSceneNode()
	{
		// If the node is visible then register it for rendering
		if (IsVisible)
		{
			SceneManager->registerNodeForRendering(this);
		}

		// register children
		ISceneNode::OnRegisterSceneNode();
	}
    /*! This function renders the PathNode */
	virtual void render()
	{
		// get the device
		irr::video::IVideoDriver * driver = SceneManager->getVideoDriver();
        /* Here we set the material of the path node */
        mat.EmissiveColor = irr::video::SColor(255,255,255,255);
		driver->setMaterial(mat);
		driver->setTransform(irr::video::ETS_WORLD, AbsoluteTransformation);


		//std::vector<irr::video::S3DVertex>::iterator it1 = vertices.begin();
		//std::vector<irr::video::S3DVertex>::iterator it2;

		if (vertices.size() == 0)
		{
			return;
		}

		irr::u32 primitive_count;

		switch(primitive_type)
		{
		case irr::scene::EPT_LINES:
			primitive_count = indices.size() / 2;
			break;
		case irr::scene::EPT_POINTS:
			primitive_count = indices.size();
			break;
		case irr::scene::EPT_LINE_STRIP:
			primitive_count = indices.size() - 1;
			break;
		default:
			primitive_count = indices.size() / 3;
			break;
		}
        if (vertices.size() > 1 && vertices.size() != 24)
        {
            for (int i = 0; i <= vertices.size()-2; i++)
            {
                // Here we change the color according to the experience_type
                // TODO, Currently all white for visibility
                if (t[i] == 0){
                    mat.EmissiveColor = irr::video::SColor(255,255,255,255);
                }
                else if (t[i] == 2)
                {
                    mat.EmissiveColor = irr::video::SColor(255,0,255,0);
                }
                else
                {
                    mat.EmissiveColor = irr::video::SColor(255,255,255,255);
                }
                driver->setMaterial(mat);
                driver->setTransform(irr::video::ETS_WORLD, irr::core::IdentityMatrix);
                driver->draw3DLine(vertices[i].Pos, vertices[i+1].Pos,irr::video::SColor(255,0,0,255));
            }
        }
        else
        {
            driver->drawVertexPrimitiveList((void*)&vertices[0], vertices.size(),
				&indices[0], primitive_count, irr::video::EVT_STANDARD, this->primitive_type,
				irr::video::EIT_32BIT);
        }
		/*switch (primitive_type)
		{
			case irr::scene::EPT_LINES:
				for (it2 = it1 + 1; it2 != points.end(); it1++, it2++)
				{
					if ((*it1).X == FLT_MAX || (*it2).X == FLT_MAX)
					{
						continue;
					}
					driver->draw3DLine(*it1, *it2, irr::video::SColor(255, 255, 0, 0));
				}
				break;

			default:

		}*/

	}

    /*! This functions returns the PathNodes bounding box */
	virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const
	{
		// not actually using this, but return the box
		// anyway
		return box;
	}

    /*! This function returns the PathNodes material count (always 1) */
	virtual irr::u32 getMaterialCount()
	{
		// return the material count
		return 1;
	}

    /*! This function returns the PathNodes material */
	virtual irr::video::SMaterial& getMaterial(irr::u32 i)
	{
		// return the material
		return mat;
	}

    /*! This funciton creates a new irr::video::S3DVertex from the point and pushes it onto the vertices vector
     * \param &point : An irrlicht vector3df point
     * */
	void addPoint(const irr::core::vector3df point, unsigned int point_type)
	{
		// points.push_back(point);
		irr::video::S3DVertex vertex;
		vertex.Pos = point;
		indices.push_back(vertices.size());
		vertices.push_back(vertex);
        t.push_back(point_type);
	}

    /*! This function clears the vertices and indices vectors */
	void clearPoints()
	{
		indices.clear();
		vertices.clear();
        t.clear();
	}

	/*void addGap()
	{
		irr::core::vector3df gap;
		gap.X = FLT_MAX;
		points.push_back(gap);
	}*/

    /*! This function sets the primitive datatype of this PathNode*/
	void setPrimitiveType(irr::scene::E_PRIMITIVE_TYPE primitive_type)
	{
		this->primitive_type = primitive_type;
	}

private:
	// scene node variables
	irr::core::aabbox3d<irr::f32> box;
	irr::video::SMaterial mat;
	// std::vector<irr::core::vector3df> points;
	std::vector<irr::video::S3DVertex> vertices; // Vector that contains the vertices of this PathNode
	std::vector<unsigned int> indices; // Vector that contains the indices of this Pathnode
    std::vector<unsigned int> t; // Type of the vector that depening on if it is information rich or not
	irr::scene::E_PRIMITIVE_TYPE primitive_type;
};


#endif /* PATHGRAPHNODE_H_ */
