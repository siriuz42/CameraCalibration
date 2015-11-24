//
//  escheme.cpp
//  CameraPose
//
//  Created by 周益辰 on 13-12-6.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//  g++ escheme.cpp -o escheme `pkg-config --cflags --libs opencv`

#include <stdio.h>
#include "zOpenCVLab.h"

int main(int agrc, char** argv)
{
    Mat R, T, intrinsicMatrix;
    
    zExternalParameter(argv[1], R, T, intrinsicMatrix);
    cout << R << T << endl;
    
    Mat origin(3,2,CV_64FC1);
    Mat direction(3,2,CV_64FC1);
    Mat tempVec(3,1,CV_64FC1);
    Mat dst(3,1,CV_64FC1);
    double x0,y0,x1,y1;
    cin >> x0;
    while(x0>0)
    {
        cin >> y0 >> x1 >> y1;
        origin.col(0) = origin.col(0) * 0;
        tempVec.at<double>(0,0) = x0;
        tempVec.at<double>(1,0) = y0;
        tempVec.at<double>(2,0) = 1;
        direction.col(0) = intrinsicMatrix.inv()*tempVec;
        origin.col(1) = -R*T;
        tempVec.at<double>(0,0) = x1;
        tempVec.at<double>(1,0) = y1;
        tempVec.at<double>(2,0) = 1;
        direction.col(1) = R*intrinsicMatrix.inv()*tempVec;
        zMeanDistancePoint(2, origin, direction, dst);
        cout << dst.at<double>(0,0) << " " << dst.at<double>(1,0) << " " << dst.at<double>(2,0) << endl;
        
        cin >> x0;
    }
    return 0;
}