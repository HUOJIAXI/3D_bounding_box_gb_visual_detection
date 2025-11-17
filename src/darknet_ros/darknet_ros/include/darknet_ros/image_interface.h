/*
 * image_interface.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef IMAGE_INTERFACE_H
#define IMAGE_INTERFACE_H

#include "image.h"

#ifdef __cplusplus
extern "C" {
#endif

image **load_alphabet_with_file(char *datafile);
void generate_image(image p, IplImage *disp);

#ifdef __cplusplus
}
#endif

#endif
