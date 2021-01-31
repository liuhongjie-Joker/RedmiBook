#ifndef IMAGECONSPROD_H
#define IMAGECONSPROD_H

#endif // IMAGECONSPROD_H
#pragma once
#include <iostream>
#include "armordetection.h"

#define EXPOSURE  2000
#define GAIN 1
#define WHITEBALANCE 1
#define GAIN_SET                //增益
#define WHITEBALANCE_SET

#define VIDEO_WIDTH  1280       //相机分辨率
#define VIDEO_HEIGHT 1024
#define V_OFFSET 304

using namespace std;

static armor_detector *detector;
void set_detector(armor_detector *d);

class ImageConsProd {
public:
    ImageConsProd(){
    }


    void ImageProducer();
    void ImageConsumer();

public:



};
