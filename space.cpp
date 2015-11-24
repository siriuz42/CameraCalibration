//
//  space.cpp
//  CameraPose
//
//  Created by 周益辰 on 13-11-26.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include <cmath>

using namespace std;

inline double dist(double x1, double y1, double z1, double x2, double y2, double z2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2));
}


int main(int argc, char ** argv)
{
    int n;
    cin  >> n;
    double x[n], y[n], z[n];
    double nx, ny, nz;
    int i, j;
    cout << "Normal: ";
    cin >> nx >> ny >> nz;
    double w = sqrt(nx*nx+ny*ny+nz*nz);
    nx /= w;
    ny /= w;
    nz /= w;
    for(i = 0; i<n; i++)
    {
        cin >> x[i] >> y[i] >> z[i];
        w = x[i]*nx+y[i]*ny+z[i]*nz;
        cout << (x[i] = x[i]-nx*w) << " " << (y[i] = y[i]-ny*w) << " " << (z[i] = z[i]-nz*w) << endl;
    }
    cout << "Chart: " << endl;
    for(i = 0; i<n; i++)
    {
        for(j = 0; j<n; j++)
            cout << dist(x[i],y[i],z[i],x[j],y[j],z[j])/dist(x[0],y[0],z[0],x[1],y[1],z[1]) << " ";
        cout << endl;
    }
    return 0;
}

