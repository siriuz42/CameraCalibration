//
//  main.cpp
//  zPainter
//
//  Created by 周益辰 on 13-9-3.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//  g++ Painter.cpp -o painter `pkg-config --cflags --libs opencv`

#include <iostream>
#include <fstream>
#include <cv.h>
#include <highgui.h>

#define newInt(w) (floor((w)+0.5))

using namespace std;
using namespace cv;

int main(int argc, const char * argv[])
{
    //0. read the image and point coordinates
    CvFont *font = new CvFont;
    cvInitFont(font, CV_FONT_HERSHEY_SIMPLEX, .5, .5, 0, 1, CV_AA);
    char *wchar = new char;
    
    IplImage *img = cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR);
    fstream inf(argv[2]);
    double x, y;
    inf >> x;
    while (inf >> x >> y)
    {
        cvCircle(img, cvPoint(newInt(x), newInt(y)), 2, CV_RGB(0,255,0), -1);
        sprintf(wchar, "(%.2f,%.2f)",x, y);
        if(strcmp(argv[4], "on")==0) cvPutText(img, wchar, cvPoint(newInt(x)+3, newInt(y)+3), font, CV_RGB(0,255,0));
    }
    cvSaveImage(argv[3], img);
    cvReleaseImage(&img);
    inf.close();
    return 0;
}