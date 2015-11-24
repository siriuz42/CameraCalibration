//
//  FindLine.cpp
//  CameraPose
//
//  Created by 周益辰 on 13-9-15.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//  g++ FindLineAgain.cpp -o findlineagain `pkg-config --cflags --libs opencv`

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <cstring>
#include "zOpenCVLab.h"

using namespace std;

int main(int argc, char * argv[])
{
    CvPoint2D32f source, target;
    source.x = atof(argv[2]);
    source.y = atof(argv[3]);
    target.x = atof(argv[4]);
    target.y = atof(argv[5]);
    CvPoint2D32f points[10000];
    cout << "Begin" << endl;
    int count;
    count = zLineSegment(argv[1], source, target, 100, points);
    int j;
    ofstream ouf;
    ouf.open(argv[6]);
    ouf << count << endl;
    for(j = 0; j<count; j++)
        ouf << points[j].x << " " << points[j].y << endl;
    ouf.close();
    cout <<"Finished" << endl;
    return 0;
}
