 //
//  Center.cpp
//  CameraPose
//
//  Created by 周益辰 on 13-10-5.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//  g++ Center.cpp -o center `pkg-config --cflags --libs opencv`


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <cstring>
#include "zOpenCVLab.h"

using namespace std;

int main(int argc, char** argv)
{
    ofstream ouf(argv[1]);
    ifstream inf;
    bool flag = false;
    int i, j;
    int n, m;
    cout << "Number of cameras:" << endl;
    cin >> n;
    
    Mat rotation[n+1];
    Mat translation[n+1];
    Mat intrinsicMatrix[n+1];

    for(i = 0; i<n; i++)
    {
        cout << "(f, cx, cy) for camera No." << i << ":" << endl;
        intrinsicMatrix[i] = Mat(3,3,CV_64FC1);
        cin >> intrinsicMatrix[i].at<double>(0,0) >> intrinsicMatrix[i].at<double>(0,2) >> intrinsicMatrix[i].at<double>(1,2);
        intrinsicMatrix[i].at<double>(1,1) = intrinsicMatrix[i].at<double>(0,0);
        intrinsicMatrix[i].at<double>(2,2) = 1;
        
        cout << "Rotation for camera No." << i << ":" << endl;
        rotation[i] = Mat(3,3,CV_64FC1);
        cin >> rotation[i].at<double>(0,0) >> rotation[i].at<double>(0,1) >> rotation[i].at<double>(0,2) >> rotation[i].at<double>(1,0) >> rotation[i].at<double>(1,1) >> rotation[i].at<double>(1,2) >> rotation[i].at<double>(2,0) >> rotation[i].at<double>(2,1) >> rotation[i].at<double>(2,2);
        
        cout << "Translation for camera No." << i << ":" << endl;
        translation[i] = Mat(3,1,CV_64FC1);
        cin >> translation[i].at<double>(0,0) >> translation[i].at<double>(1,0) >> translation[i].at<double>(2,0);
        cout << translation[i] << endl;
    }
    
    
    cout << "Camera setting ends." << endl;
    cout << "The number of points:" << endl;
    cin >> m;

    Mat origin(3,n,CV_64FC1);
    Mat direction(3,n,CV_64FC1);
    Mat tempVec(3,1,CV_64FC1);
    Mat dst(3,1,CV_64FC1);
    int numLine;
    
    double x,y;
    for(i = 0; i<m; i++)
    {
        numLine = 0;
        for(j = 0; j<n; j++)
        {
            cout << "Coordinates of point No." << i << " in camera No." << j << ", (0,0) for not showing:" << endl;
            cin >> x >> y;
            if(x>1e-6 || y>1e-6)
            {
                origin.col(numLine) = -rotation[j]*translation[j];
                tempVec.at<double>(0,0) = x;
                tempVec.at<double>(1,0) = y;
                tempVec.at<double>(2,0) = 1;
                direction.col(numLine) = rotation[j]*intrinsicMatrix[j].inv()*tempVec;
                numLine ++;
            }
        }
        cout << origin << direction << endl;
        zMeanDistancePoint(numLine, origin, direction, dst);
        ouf << "Point " << i << ": " << dst << endl;
        cout << "Point " << i << ": " << dst << endl;
    }
    ouf.close();
    
    return 0;
}