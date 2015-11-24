
//
//  PrcImgPtCount.c
//  CameraPose
//
//  Created by 周益辰 on 13-9-4.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//
//  g++ PrcImgPtCount.cpp -o run `pkg-config --cflags --libs opencv`


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
    
    int i, j;
    char a,b,c;
    string sa, sb, sc;
    string filename;
    ofstream ouf;
    for(i = 1; i<=331; i++)
    {
        a = i/100 + 48;
        b = (i/10) % 10 + 48;
        c = i % 10 + 48;
        sa = a;
        sb = b;
        sc = c;
        filename = "PrcImg" + sa+sb+sc+ ".jpg";
        cout << i << " File Created" << endl;
        count = 0;
        zGetPointFromFile(filename.c_str(), points, count);
        cout << "Point Gained" << endl;
        filename = "PrcImg" + sa+sb+sc+ ".txt";
        ouf.open(filename.c_str());
        ouf << count << endl;
        for(j = 0; j<count; j++)
            ouf << points[j].x << " " << points[j].y << endl;
        ouf.close();
        cout << i << " Finished" << endl;
    }
    return 0;
}
