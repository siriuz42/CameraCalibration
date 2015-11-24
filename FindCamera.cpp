//
//  FindCamera.cpp
//  CameraPose
//
//  Created by 周益辰 on 13-9-15.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//  g++ FindCamera.cpp -o findcamera `pkg-config --cflags --libs opencv`


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <cstring>
#include "zOpenCVLab.h"

using namespace std;

double gaussrand()
{
    static double V1, V2, S;
    static int phase = 0;
    double X;
    if ( phase == 0 ) {
        do {
            double U1 = (double)rand() / RAND_MAX;
            double U2 = (double)rand() / RAND_MAX;
            
            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);
        
        X = V1 * sqrt(-2 * log(S) / S);
    } else X = V2 * sqrt(-2 * log(S) / S);
    phase = 1 - phase;
    return X*0;
}

int main(int argc, char** argv)
{
    srand((unsigned)time(NULL));
    ifstream inf(argv[1]);
    int n1, n2;
    double cx, cy, fx;
    inf >> n1 >> n2 >>fx >> cx >> cy;
    Mat intrinsicMatrix(3,3,CV_64FC1);
    intrinsicMatrix.at<double>(0,0) = fx;
    intrinsicMatrix.at<double>(1,1) = fx;
    intrinsicMatrix.at<double>(0,2) = cx/2;
    intrinsicMatrix.at<double>(1,2) = cy/2;
    Mat R(3,3,CV_64FC1);
    Mat T(3,1,CV_64FC1);
    
    CvPoint2D32f line11[200];
    CvPoint2D32f line12[200];
    CvPoint2D32f line21[200];
    CvPoint2D32f line22[200];
    int i;
    for(i = 0; i<n1; i++)
    {
        inf >> line11[i].x >> line11[i].y;
//        cout << line11[i].x << " " << line11[i].y << endl;
        line11[i].x += gaussrand();
        line11[i].y += gaussrand();
    }
//    cout << "---" << endl;
    for(i = 0; i<n2; i++)
    {
        inf >> line12[i].x >> line12[i].y;
//        cout << line12[i].x << " " << line12[i].y << endl;
        line12[i].x += gaussrand();
        line12[i].y += gaussrand();
    }
//    cout << "---" << endl;
    for(i = 0; i<n1; i++)
    {
        inf >> line21[i].x >> line21[i].y;
//        cout << line21[i].x << " " << line21[i].y << endl;
        line21[i].x += gaussrand();
        line21[i].y += gaussrand();
    }
//    cout << "---" << endl;
    for(i = 0; i<n2; i++)
    {
        inf >> line22[i].x >> line22[i].y;
//        cout << line22[i].x << " " << line22[i].y << endl;
        line22[i].x += gaussrand();
        line22[i].y += gaussrand();

    }
//    cout << "---" << endl;
    inf.close();
    cout << "Begin process."<<endl;
    zTwoLineCameraPose(n1, line11, line21, n2, line12, line22, 10, R, T, intrinsicMatrix, 1);
    ofstream ouf("out.txt");
    ouf << "f: " << intrinsicMatrix.at<double>(0,0) << endl;
    ouf << "Rotation: " << endl << R << endl;
    ouf << "Translation: " << endl << T << endl << norm(T) << endl;
    ouf.close();
        
    return 0;
}
