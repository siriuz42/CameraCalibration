//
//  FindLine.cpp
//  CameraPose
//
//  Created by 周益辰 on 13-9-15.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//  g++ FindLine.cpp -o findline `pkg-config --cflags --libs opencv`

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <iomanip>
#include <cstring>
#include "zOpenCVLab.h"

using namespace std;

int main(int argc, char * argv[])
{
    CvPoint2D32f source, target, wsource, wtarget;
    
    ifstream inf(argv[1]);
    inf >> source.x >> source.y >> target.x >> target.y;
    wsource.x = source.x*0.9+target.x*0.1;
    wtarget.x = source.x*0.1+target.x*0.9;
    
    string jpg = ".jpg";
    int j;
    double wx[1000], wy[1000];
    CvPoint2D32f points[10000];
    CvPoint2D32f ansPoints[10000];
    int i, k = 0;
    double a, b;
    int count, nfile = 0;
    
    string filename;
    while(inf>>filename)
    {
        nfile ++;
        cout << filename <<" Begin" << endl;
        count = zLineSegment2((filename+jpg).c_str(), source, target, 100, points);
        cout << "# of Points (init): " << count << endl;
        k = 0;
        for(i = 10; i<count-10; i++)
        {
            wx[k] = points[i].x;
            wy[k] = points[i].y;
            k++;
        }

        yaxb(k, wx, wy, a, b);
    
        wsource.y = a*wsource.x+b;
        wtarget.y = a*wtarget.x+b;
        count = zLineSegment((filename+jpg).c_str(), wsource, wtarget, 100, points);
        cout << "# of Points: " << count << endl;
        cout << "Start: " << points[0].x << " " << points[0].y << endl;
        for(i = 0; i<count; i++)
        {
            ansPoints[i].x += points[i].x;
            ansPoints[i].y += points[i].y;
        }
    }
    
    ofstream ouf(argv[2]);
    ouf << count << endl;
    for(i = 0; i<count; i++) ouf << fixed << setprecision(4) << ansPoints[i].x/nfile << " " << ansPoints[i].y/nfile << endl;
    ouf.close();
    cout <<"Finished" << endl;
    return 0;
}
