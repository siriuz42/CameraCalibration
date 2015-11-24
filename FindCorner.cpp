//
//  FindCorner.cpp
//  CameraPose
//
//  Created by 周益辰 on 13-9-15.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//
//  g++ FindCorner.cpp -o findcorner `pkg-config --cflags --libs opencv`


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <cstring>
#include "zOpenCVLab.h"

using namespace std;

int main(int argc, const char * argv[])
{
    CvPoint2D32f points[10000];
    int count;
    
    int j;
    ofstream ouf;
    
    zGetPointFromFile(argv[1], points, count);
    
    cout << "Point Gained" << endl;
    ouf.open(argv[2]);
    ouf << count << endl;
    for(j = 0; j<count; j++)
        ouf << points[j].x << " " << points[j].y << endl;
    ouf.close();
    cout <<"Finished" << endl;

    return 0;
}
