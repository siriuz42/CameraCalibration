//
//  fscheme..c
//  CameraPose
//
//  Created by 周益辰 on 13-10-10.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//  g++ fscheme2.cpp -o fscheme2 `pkg-config --cflags --libs opencv`

#include <stdio.h>
#include "zOpenCVLab.h"

int main(int argc, char** argv)
{
    srand((unsigned)time(NULL));
    ifstream inf(argv[1]);
    int n1, n2;
    double cx, cy, fx;
    inf >> n1 >> n2 >>fx >> cx >> cy;
    Mat intrinsicMatrix=Mat::zeros(3,3,CV_64FC1);
    intrinsicMatrix.at<double>(0,0) = fx;
    intrinsicMatrix.at<double>(1,1) = fx;
    intrinsicMatrix.at<double>(0,2) = cx/2;
    intrinsicMatrix.at<double>(1,2) = cy/2;
    intrinsicMatrix.at<double>(2,2) = 1;
    Mat R(3,3,CV_64FC1);
    Mat T(3,1,CV_64FC1);
    CvPoint2D32f line11[200];
    CvPoint2D32f line12[200];
    CvPoint2D32f line21[200];
    CvPoint2D32f line22[200];
    int i;
    for(i = 0; i<n1; i++) inf >> line11[i].x >> line11[i].y;
    for(i = 0; i<n2; i++) inf >> line12[i].x >> line12[i].y;
    for(i = 0; i<n1; i++) inf >> line21[i].x >> line21[i].y;
    for(i = 0; i<n2; i++) inf >> line22[i].x >> line22[i].y;
    inf.close();
    //if(abs(fx)>1) zTwoLineCameraPose(n1, line11, line21, n2, line12, line22, 10, R, T, intrinsicMatrix, 1); else zTwoLineCameraPose(n1, line11, line21, n2, line12, line22, 10, R, T, intrinsicMatrix, 0);
    if(abs(fx)>1)
    {
        cout << zTwoLineCameraPose2(70, n1, line11, line21, n2, line12, line22, 10, R, T, intrinsicMatrix, 1);
    }
    else
    {
        cout << zTwoLineCameraPose2(70, n2, line11, line21, n2, line12, line22, 10, R, T, intrinsicMatrix, 0);
    }
    
   // cout << "RT" << R << endl << T << endl;
    Mat origin(3,2,CV_64FC1);
    Mat direction(3,2,CV_64FC1);
    Mat tempVec(3,1,CV_64FC1);
    Mat dst(3,1,CV_64FC1);
/*    double x0,y0,x1,y1;
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
    }*/
    return 0;
    
}

