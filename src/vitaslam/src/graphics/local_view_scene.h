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
#ifndef VIEW_TEMPLATE_SCENE_HPP_
#define VIEW_TEMPLATE_SCENE_HPP_

#include <irrlicht/irrlicht.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <iostream>
#include "../utils/utils.h"
#include "../vitaslam/local_template_match.h"


namespace vitaslam
{
class LocalViewScene
{
public:
  LocalViewScene(ptree & settings, LocalTemplateMatch *in_t)
  {

    get_setting_from_ptree(vt_window_width, settings, "vt_window_width", 640);
    get_setting_from_ptree(vt_window_height, settings, "vt_window_height", 480);

    update_ptr(in_t);

    // the camera image is in the top half and the two template windows in the bottom half
  //  vt_window_height = tm->IMAGE_HEIGHT * ((double)vt_window_width/tm->IMAGE_WIDTH) * 2;

    device = irr::createDevice(irr::video::EDT_OPENGL, irr::core::dimension2d<irr::u32>(vt_window_width, vt_window_height), 32, false,
                               false, false);
    device->setWindowCaption(L"ViTa-SLAM Local View");

    driver = device->getVideoDriver();
    scene = device->getSceneManager();


    view_template_scene = scene->createNewSceneManager(false);

  }

  ~LocalViewScene()
  {

  }

  void draw_all()
  {
    device->run(); // TODO return the bool for quiting
    driver->beginScene(true, true, irr::video::SColor(255, 0, 0, 0));
    // TODO not always true for grayscale
    //cout << "wdith: " << (double)vt_window_width/tm->IMAGE_WIDTH << " height: " << (double)vt_window_height/tm->IMAGE_HEIGHT << endl;
    draw_image(tm->view_rgb, tm->grayscale, -1.0f, 1.0f, tm->IMAGE_WIDTH, tm->IMAGE_HEIGHT, (double)vt_window_width/tm->IMAGE_WIDTH, -(double)vt_window_width/tm->IMAGE_WIDTH);
    //cout << "wdith: " << (double)vt_window_width/tm->TEMPLATE_X_SIZE << " height: " << (double)vt_window_height/tm->TEMPLATE_Y_SIZE<< endl;

    draw_image((const double*)&tm->templates[tm->current_template].visual_data[0], true, -1, 0.0,
                               tm->TEMPLATE_X_SIZE, tm->TEMPLATE_Y_SIZE,
                               (double)vt_window_width/tm->TEMPLATE_X_SIZE,
                               -(double)vt_window_height/tm->TEMPLATE_Y_SIZE/4);
    draw_image((const double*)&tm->current_view[0],true, -1.0, -0.5,
               tm->TEMPLATE_X_SIZE, tm->TEMPLATE_Y_SIZE,
               (double)vt_window_width/tm->TEMPLATE_X_SIZE,
               -(double)vt_window_height/tm->TEMPLATE_Y_SIZE/4);
    view_template_scene->drawAll();
    driver->endScene();
  }

  void update_ptr(LocalTemplateMatch *t_in){
    tm = t_in;
  }

private:

  void draw_image(const double * image, bool grayscale, float x, float y, int width, int height, double scale_x, double scale_y)
   {
    unsigned char* texture_ptr_start = (unsigned char *) malloc(width*height * 3);//(grayscale ? 1 : 3));

    const double * image_ptr = image;
    const double * image_end = image_ptr + width * height * (grayscale ? 1 : 3);
    unsigned char *texture_ptr = texture_ptr_start;
    for (; image_ptr < image_end;)
    {
        *(texture_ptr++) = (unsigned char)(*(image_ptr++) * 255.0);
    }

    draw_image(texture_ptr_start, grayscale, x, y, width, height, scale_x, scale_y);
    free(texture_ptr_start);
   }

   void draw_image(const unsigned char * image, bool grayscale, float x, float y, int width, int height, double scale_x, double scale_y)
   {
     glRasterPos2f(x,y);
     glPixelZoom(scale_x, scale_y);
     if (grayscale)
       glDrawPixels(width, height, GL_LUMINANCE, GL_UNSIGNED_BYTE, image);
     else
       glDrawPixels(width, height, GL_BGR, GL_UNSIGNED_BYTE, image);
   }


  irr::IrrlichtDevice *device;
  irr::video::IVideoDriver * driver;
  irr::scene::ISceneManager * scene;
  LocalTemplateMatch *tm;
  irr::scene::ISceneManager * view_template_scene;

  int vt_window_width, vt_window_height;
};

};
#endif /* VIEW_TEMPLATE_SCENE_HPP_ */
