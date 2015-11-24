//
//  main.cpp
//  Camera Pose
//
//  Created by 周益辰 on 13-8-31.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//

#include <iostream>
#include <fstream>
#include "highgui.h"
#include "cv.h"
#include <cmath>
#include <string.h>
#include "zOpenCVLab.h"

using namespace std;

int npoint, nview;
Mat point;
Mat intrinsicMatrix(3, 3, CV_64FC1);
Mat intrinsicMatrixInv(3, 3, CV_64FC1);

int init()
{
    string filepath;
    cout << "Input filename: "<< endl;
    cin >> filepath;
    ifstream inf(filepath);
    inf >> nviews >> npoint;
    point.create(nview*npoint, 3, CV_64FC1);
    int i, j;
    double x,y;
    for(i = 0; i<nview; i++)
        for(j = 0; j<npoint; j++)
        {
            inf >> x >> y;
            point.at<double>(i*npoint+j, 0) = x;
            point.at<double>(i*npoint+j, 1) = y;
            point.at<double>(i*npoint+j, 2) = 1;
        }
    inf.close();
}

int main(int argc, const char * argv[])
{

}

