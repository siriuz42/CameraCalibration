//
//  Mask.cpp
//  CameraPose
//
//  Created by 周益辰 on 13-11-23.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//  g++ Mask.cpp -o mask `pkg-config --cflags --libs opencv`

#include <iostream>
#include <fstream>
#include <string.h>
#include <cstring>
#include <iomanip>
#include "zOpenCVLab.h"

using namespace std;

int main(int argc, const char * argv[])
{
    CvPoint2D32f points[100000];
    int count;

    Vec3b anc = Vec3b(0,0,255); //BGR
    cout << "ANC:" << anc << endl;
    Mat mask = imread(argv[1], IMREAD_COLOR);
    ifstream inf(argv[2]);
    ofstream ouf;
    string filename;
    int i,j,k,l;
    double ans[10000][2];

    string jpg = ".jpg";
    string txt = ".txt";
    l = 0;
    while(inf >> filename)
    {
        l++;
        count = 0;
        zGetPointFromFile2((filename+jpg).c_str(), points, count);
        ouf.open((filename+txt).c_str());
        cout << filename << " Point Gained" << endl;
        ouf << count << endl;
        k = 0;
        for(j = 0; j<count; j++)
        {
            anc = mask.at<Vec3b>(newInt(points[j].y), newInt(points[j].x));
            if(anc.val[0]<5 && anc.val[1]<5 && anc.val[2]>250)
            {
                ouf << points[j].x << " " << points[j].y << endl;
                ans[k][0] += points[j].x;
                ans[k][1] += points[j].y;
                k++;
            }
        }
        ouf.close();
        cout <<"Finished" << endl;
    }
    ouf.open(argv[3]);
    for(i = 0; i<k; i++) ouf << fixed << setprecision(4) << ans[i][0]/l << " " << ans[i][1]/l << endl;
    ouf.close();
    inf.close();
    return 0;
}
