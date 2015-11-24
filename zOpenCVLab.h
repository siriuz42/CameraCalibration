//
//  zOpenCVLab.h
//  Camera Pose
//
//  Created by 周益辰 on 13-8-31.
//  Copyright (c) 2013年 周益辰. All rights reserved.
//


#include <highgui.h>
#include <cv.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <cmath>


#define newInt(w) (floor((w)+0.5))
#define PI 3.141592654 

using namespace std;
using namespace cv;

inline double zDist(double x1, double y1, double z1, double x2, double y2, double z2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2));
}

int yaxb(int n, double *x, double *y, double & a, double & b)
{
    int i;
    double xbar = 0, ybar = 0;
    for(i = 0; i<n; i++)
    {
        xbar += x[i];
        ybar += y[i];
    }
    xbar /= n;
    ybar /= n;
    double w1 = 0, w2 = 0;
    for(i = 0; i<n; i++)
    {
        w1 += (x[i]-xbar)*(y[i]-ybar);
        w2 += (x[i]-xbar)*(x[i]-xbar);
    }
    a = w1 / w2;
    b = ybar - xbar*a;
    return 0;
}

double power(double w, double e)
{
    return exp(log(w)*e);
}

// Return the value of a polynomial
double zPolyValue(int n, double c[], double x)
{
    double ans = 0, w = 1;
    int i;
    for(i = 0; i<=n; i++)
    {
        ans += w*c[i];
        w *= x;
    }
    return ans;
}

// Return the derivative of a polynomial
double zPolyDerive(int n, double c[], double x)
{
    double ans = 0, w = 1;
    int i;
    for(i = 1; i<=n; i++)
    {
        ans += i*c[i]*w;
        w *= x;
    }
    return ans;
}

// Use Newton method to solve the polynomial
double zPolyNewton(int n, double c[], double initGuess)
{
    double w;
    double ans = initGuess;
    while(abs(w = zPolyValue(n, c, ans))>1e-6)  cout << (ans -= w/zPolyDerive(n, c, ans)) << " " << w << endl;
    return ans;
}

// Given 8 cooresponding pairs, solve for the affine transformation between two camera frames.
int z8PointCameraPose(const Mat& c1, const Mat& c2, Mat t, Mat R)
{
    int i, j;
    
// 0. Preprocess the points with the intrinsic matrix
    
// 1. Calculate the coefficients of the linear system. WLOG, we set E(2,2) = 1
    
    Mat A(8, 8, CV_64FC1);
    Mat b(8, 1, CV_64FC1);
    Mat c(8, 1, CV_64FC1);
    Mat E(3, 3, CV_64FC1);
    for(i = 0; i<8; i++)
    {
        A.at<double>(i,0) = c1.at<double>(i,0)*c2.at<double>(i,0);
        A.at<double>(i,1) = c1.at<double>(i,0)*c2.at<double>(i,1);
        A.at<double>(i,2) = c1.at<double>(i,0)*c2.at<double>(i,2);
        A.at<double>(i,3) = c1.at<double>(i,1)*c2.at<double>(i,0);
        A.at<double>(i,4) = c1.at<double>(i,1)*c2.at<double>(i,1);
        A.at<double>(i,5) = c1.at<double>(i,1)*c2.at<double>(i,2);
        A.at<double>(i,6) = c1.at<double>(i,2)*c2.at<double>(i,0);
        A.at<double>(i,7) = c1.at<double>(i,2)*c2.at<double>(i,1);
        b.at<double>(i,0) = -c1.at<double>(i,2)*c2.at<double>(i,2);
    }
    //c = A.inv()*b;
    cv::solve(A,b,c);
    cout << "The conditional number of the linear system: " << endl << norm(A.inv())*norm(A) << endl;
    E.at<double>(0,0) = c.at<double>(0,0);
    E.at<double>(0,1) = c.at<double>(1,0);
    E.at<double>(0,2) = c.at<double>(2,0);
    E.at<double>(1,0) = c.at<double>(3,0);
    E.at<double>(1,1) = c.at<double>(4,0);
    E.at<double>(1,2) = c.at<double>(5,0);
    E.at<double>(2,0) = c.at<double>(6,0);
    E.at<double>(2,1) = c.at<double>(7,0);
    E.at<double>(2,2) = 1;
    
// 2. Decomposite E to get T and R
   
    SVD ESVD(E, SVD::FULL_UV);
    Mat M1(3, 3, CV_64FC1);
    M1.at<double>(0,1) = 1;
    M1.at<double>(1,0) = -1;
    M1.at<double>(2,2) = 1;
    Mat R1 = ESVD.u*M1*ESVD.vt;
    Mat R2 = ESVD.u*M1.t()*ESVD.vt;
    Mat t1 = ESVD.vt.t().col(2);
    Mat t2 = 0.0 - t1;
    
// 3. Determine the combination
    
    return 0;
}


// Given several spatial lines, solve for their (pseudo) intersection by LSE
/*
int zMeanDistancePoint(int numberLines, const Mat& origin, const Mat& direction, Mat dst)
{
    Mat A(3, 3, CV_64FC1);
    Mat b(3, 1, CV_64FC1);
    Mat tempVec(3, 1, CV_64FC1);
    Mat tempVec1(3, 1, CV_64FC1);
    int i, j;
    for(i = 0; i<3; i++)
    {
        b.at<double>(i,0) = 0;
        for(j = 0; j<3; j++)
            A.at<double>(i,j) = 0;
    }
    double wproduct;
    for(i = 0; i<numberLines; i++)
    {
        tempVec = direction.col(i) / norm(direction.col(i));
        tempVec1 = origin.col(i);
        wproduct = tempVec1.dot(tempVec);
        
        A.at<double>(0,0) += 1-tempVec.at<double>(0,0)*tempVec.at<double>(0, 0);
        A.at<double>(0,1) += -tempVec.at<double>(1,0)*tempVec.at<double>(0,0);
        A.at<double>(0,2) += -tempVec.at<double>(2,0)*tempVec.at<double>(0,0);
        A.at<double>(1,0) += -tempVec.at<double>(0,0)*tempVec.at<double>(1,0);
        A.at<double>(1,1) += 1-tempVec.at<double>(1,0)*tempVec.at<double>(1,0);
        A.at<double>(1,2) += -tempVec.at<double>(2,0)*tempVec.at<double>(1,0);
        A.at<double>(2,0) += -tempVec.at<double>(0,0)*tempVec.at<double>(2,0);
        A.at<double>(2,1) += -tempVec.at<double>(1,0)*tempVec.at<double>(2,0);
        A.at<double>(2,2) += 1-tempVec.at<double>(2,0)*tempVec.at<double>(2,0);
        b += origin.col(i)-wproduct*tempVec;
    }
    //    cout << "initialzed" << endl;
    solve(A, b, dst);
    
    return 0;
}
*/

double zMeanDistancePoint(int numberLines, const Mat& origin, const Mat& direction, Mat& dst)
{
    Mat A = Mat::zeros(3, 3 ,CV_64FC1);
    Mat b = Mat::zeros(3, 1, CV_64FC1);
    int i;
    for(i = 0; i<numberLines; i++)
    {
        direction.col(i) = direction.col(i)/norm(direction.col(i));
        A = A + Mat::eye(3, 3, CV_64FC1) - direction.col(i)*direction.col(i).t();
        b = b + (Mat::eye(3, 3, CV_64FC1) - direction.col(i)*direction.col(i).t())*origin.col(i);
    }
    solve(A,b,dst);
    double w = 0;
    for(i = 0; i<numberLines; i++)
        w += power(norm(dst-origin.col(i)),2) - power(direction.col(i).dot(dst-origin.col(i)),2);
    return w;
}

// Withdraw subpixel corner points from an image
int zGetPointFromFile(const char* filename, CvPoint2D32f* points, int& count, int distFilt = 10, int ignRegion = -1)
{
 
// 0. Predetermine the corners by given functions
    
    CvPoint2D32f ptTemp[100000];
    IplImage *img = cvLoadImage(filename, CV_LOAD_IMAGE_GRAYSCALE);
    IplImage *imgEig = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);
    IplImage *imgTemp = cvCloneImage(imgEig);
    cvGoodFeaturesToTrack(img, imgEig, imgTemp, ptTemp, &count, 0.01, 3);
    cout << "Got features. Count: " << count << endl;
    cvFindCornerSubPix(img, ptTemp, count, cvSize(distFilt,distFilt), cvSize(ignRegion,ignRegion), cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 80, 0.01));
    cout << "Found subpixels. Count: " << count << endl;
    
// 1. Filter the waste corners
    
    Mat graph(img);
    int i, j;
    int x, y;
    int k = 0;
    bool flag0, flag1, flag2;
    int cTran1, cBlack1;
    CvPoint2D32f drc[32];
    for(i = 0; i<32; i++)
    {
        drc[i].x = 10*cos(PI/16*i);
        drc[i].y = 10*sin(PI/16*i);
    }
    for(i = 0; i<count; i++)
        if(ptTemp[i].x>50 && ptTemp[i].y>50 && ptTemp[i].x<img->width-50 && ptTemp[i].y<img->height-50)
        {
            cTran1 = 0;
            cBlack1 = 0;
            for(j = 0; j<32; j++)
            {
                x = newInt(ptTemp[i].x+drc[j].x);
                y = newInt(ptTemp[i].y+drc[j].y);
                flag2 = (graph.at<uchar>(y,x)<100);
                if(flag2) cBlack1++;
                if(j==0)
                {
                    flag0 = flag2;
                    flag1 = flag2;
                }
                else if (flag2!=flag1)
                {
                    flag1 = flag2;
                    cTran1++;
                }
            }
            if(flag1!=flag0) cTran1++;

            if(cTran1==4 && cBlack1>11 && cBlack1 <21)
            {
                flag0 = true;
                for(j = 0; j<k; j++)
                    if(abs(points[j].x-ptTemp[i].x)<2 && abs(points[j].y-ptTemp[i].y)<2)
                    {
                        flag0 = false;
                        break;
                    }
                if(flag0)
                {
                    points[k].x = ptTemp[i].x;
                    points[k].y = ptTemp[i].y;
                    k++;
                }
            }
        }
    count = k;
    cout << "Subpixels filtered. Count: " << count << endl;

// 2. Print the corners
/*
    cvNamedWindow("Board", CV_WINDOW_AUTOSIZE);
    for(i = 0; i<count; i++)
        cvCircle(img, cvPoint(newInt(points[i].x),newInt(points[i].y)), 2, CV_RGB(255,255,255), -1);
    
    IplImage *img2 = cvCreateImage(cvSize(img->width*0.2, img->height*0.2), img->depth, img->nChannels);
    cvResize(img, img2, CV_INTER_LINEAR);
    cvShowImage("Board", img2);
    cvSaveImage("img.jpg", img);
    cvWaitKey(0);
    cvDestroyWindow("Board");
*/ 

// 3. Sort the points
    double w;
    for(i = 0; i<count-1; i++)
        for(j = i+1; j<count; j++)
            if(points[i].y>points[j].y || (points[i].y==points[j].y && points[i].x>points[j].x))
            {
                w = points[i].x;
                points[i].x = points[j].x;
                points[j].x = w;
                w = points[i].y;
                points[i].y = points[j].y;
                points[j].y = w;
            }
       
    cvReleaseImage(&img);
    cvReleaseImage(&imgEig);
    cvReleaseImage(&imgTemp);
    return 0;
}

int zGetPointFromFile2(const char* filename, CvPoint2D32f* points, int& count, int distFilt = 10, int ignRegion = -1)
{
    
    // 0. Predetermine the corners by given functions
    
    CvPoint2D32f ptTemp[100000];
    IplImage *img = cvLoadImage(filename, CV_LOAD_IMAGE_GRAYSCALE);
    IplImage *imgEig = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);
    IplImage *imgTemp = cvCloneImage(imgEig);
    cvGoodFeaturesToTrack(img, imgEig, imgTemp, ptTemp, &count, 0.01, 3);
    cout << "Got features. Count: " << count << endl;
    cvFindCornerSubPix(img, ptTemp, count, cvSize(distFilt,distFilt), cvSize(ignRegion,ignRegion), cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 80, 0.01));
    cout << "Found subpixels. Count: " << count << endl;
    
    // 1. Filter the waste corners
    
    Mat graph(img);
    int i, j;
    int x, y;
    int k = 0;
    bool flag0;
    for(i = 0; i<count; i++)
        if(ptTemp[i].x>50 && ptTemp[i].y>50 && ptTemp[i].x<img->width-50 && ptTemp[i].y<img->height-50)
        {
            flag0 = true;
            for(j = 0; j<k; j++)
                if(abs(points[j].x-ptTemp[i].x)<2 && abs(points[j].y-ptTemp[i].y)<2)
                {
                    flag0 = false;
                    break;
                }
            if(flag0)
            {
                points[k].x = ptTemp[i].x;
                points[k].y = ptTemp[i].y;
                k++;
            }
        }
    count = k;
    cout << "Subpixels filtered. Count: " << count << endl;
    
    // 2. Print the corners
    /*
     cvNamedWindow("Board", CV_WINDOW_AUTOSIZE);
     for(i = 0; i<count; i++)
     cvCircle(img, cvPoint(newInt(points[i].x),newInt(points[i].y)), 2, CV_RGB(255,255,255), -1);
     
     IplImage *img2 = cvCreateImage(cvSize(img->width*0.2, img->height*0.2), img->depth, img->nChannels);
     cvResize(img, img2, CV_INTER_LINEAR);
     cvShowImage("Board", img2);
     cvSaveImage("img.jpg", img);
     cvWaitKey(0);
     cvDestroyWindow("Board");
     */
    
    // 3. Sort the points
    double w;
    for(i = 0; i<count-1; i++)
        for(j = i+1; j<count; j++)
            if(points[i].y>points[j].y || (points[i].y==points[j].y && points[i].x>points[j].x))
            {
                w = points[i].x;
                points[i].x = points[j].x;
                points[j].x = w;
                w = points[i].y;
                points[i].y = points[j].y;
                points[j].y = w;
            }
    
    cvReleaseImage(&img);
    cvReleaseImage(&imgEig);
    cvReleaseImage(&imgTemp);
    return 0;
}


// Find all intersections between the ruler and the line
int zLineSegment(const char* filename, const CvPoint2D32f source, const CvPoint2D32f target, int guess, CvPoint2D32f* ans)
{
    const int STEP = 20000;
    const int WIDTH = STEP / guess / 8;
    const int HEIGHT = 4;
    
    Mat img = imread(filename, 0);
    double dx = (target.x - source.x)/STEP;
    double dy = (target.y - source.y)/STEP;
    double nx = -dy;
    double ny = dx;
    int wmin;
    int w, i, j;
    double wx, wy;
    int nans = 0, lab;
    int weight;
    
    CvPoint2D32f center;
    for(lab = 0; lab<=STEP; lab++)
    {
        wmin = 256;
        while((w = img.at<uchar>(newInt(source.y+dy*lab),newInt(source.x+dx*lab)))<=wmin || w>100)
        {
            wmin = w;
            lab++;
        }
        lab--;
        if(lab>STEP) break;
        center.x = source.x+dx*lab;
        center.y = source.y+dy*lab;
        weight = 0;
        wx = 0;
        wy = 0;
        for(i = -WIDTH; i<=WIDTH; i++)
            for(j = -HEIGHT; j<=HEIGHT; j++)
            {
                w = (255 - img.at<uchar>(newInt(center.y+i*dy+j*ny),newInt(center.x+i*dx+j*nx)))*(HEIGHT+1-abs(j));
                if(w < 100) w = 0; else if (w>155) w = 255;
                weight += w;
                wx += w*(center.x+i*dx+j*nx);
                wy += w*(center.y+i*dy+j*ny);
            }
        nans++;
        wx /= weight;
        wy /= weight;
        ans[nans-1].x = source.x+((wx-source.x)*dx+(wy-source.y)*dy)/(dx+dy/dx*dy);
        ans[nans-1].y = source.y+((wx-source.x)*dx+(wy-source.y)*dy)/(dx/dy*dx+dy);
        lab += WIDTH;
    }
    return nans;
}

// Find the midpoints of all the labels
int zLineSegment2(const char* filename, const CvPoint2D32f source, const CvPoint2D32f target, int guess, CvPoint2D32f* ans)
{
    const int STEP = 10000;
    
    Mat img = imread(filename, 0);
    double dx = (target.x - source.x)/STEP;
    double dy = (target.y - source.y)/STEP;
    double nx = -dy;
    double ny = dx;
    int wmin;
    int w, i, j;
    double wx, wy;
    int nans = 0, lab = 0;
    int weight;
    int top, bottom, left, right;
    
    CvPoint2D32f center;
    while(lab<STEP)
    {
        while(img.at<uchar>(newInt(source.y+dy*lab),newInt(source.x+dx*lab))>100) lab++;
        if(lab>STEP) break;
        center.x = source.x+dx*lab;
        center.y = source.y+dy*lab;
        weight = 0;
        wx = 0;
        wy = 0;
        bottom = top = left = right = 5;
        while((top<1000) && (img.at<uchar>(newInt(center.y+top*dy), newInt(center.x+top*dx))<180)) top ++;
        while((bottom<1000) && (img.at<uchar>(newInt(center.y-bottom*dy), newInt(center.x-bottom*dx))<180)) bottom ++;
        while((left<1000) && (img.at<uchar>(newInt(center.y-left*ny), newInt(center.x-left*nx))<180)) left ++;
        while((right<1000) && (img.at<uchar>(newInt(center.y+right*ny), newInt(center.x+right*nx))<180)) right ++;
        top += 10;
        bottom += 10;
        right += 10;
        left += 10;
        weight = 0;
        wx = 0;
        wy = 0;
        for(i = -bottom; i<=top; i++)
            for(j = -left; j<=right; j++)
            {
                w = 255 - img.at<uchar>(newInt(center.y+i*dy+j*ny),newInt(center.x+i*dx+j*nx));
                if(w < 100) w = 0; else if (w>155) w = 255;
                weight += w;
                wx += w*(center.x+i*dx+j*nx);
                wy += w*(center.y+i*dy+j*ny);
            }
        nans++;
        ans[nans-1].x = wx / weight;
        ans[nans-1].y = wy / weight;
        lab += top;
    }

    return nans;
}

bool zFront(double x1, double y1, double x2, double y2, Mat R, Mat T, Mat intrinsic)
{
    Mat origin(3, 2, CV_64FC1);
    Mat direction(3, 2, CV_64FC1);
    Mat temp(3, 1, CV_64FC1);
    Mat wdst;
    cout << "RT" << R << T << endl;
    origin.col(0) = 0.0;
    origin.col(1) = T;
    temp.at<double>(0,0) = x1;
    temp.at<double>(1,0) = y1;
    temp.at<double>(2,0) = 1;
    direction.col(0) = intrinsic.inv()*temp;
    temp.at<double>(0,0) = x2;
    temp.at<double>(1,0) = y2;
    temp.at<double>(2,0) = 1;
    direction.col(1) = R.t()*intrinsic.inv()*temp;
    zMeanDistancePoint(2, origin, direction, wdst);
    Mat G = R*(wdst-T);
    cout << wdst << G << endl;
    if((wdst.at<double>(0,2)>0) && (G.at<double>(0,2)>0)) return true; else return false;
}

Mat zCross(Mat T)
{
    Mat ans(3, 1, CV_64FC1);
    ans.at<double>(0,0) = T.at<double>(2,1);
    ans.at<double>(1,0) = T.at<double>(0,2);
    ans.at<double>(2,0) = T.at<double>(1,0);
    return ans;
}

int zExternalParameter(char* filename, Mat& R, Mat& T, Mat& intrinsic)
{
    ifstream inf(filename);
    int n;
    intrinsic = Mat::zeros(3, 3, CV_64FC1);
    intrinsic.at<double>(2,2) = 1;
    inf >> n >> intrinsic.at<double>(0,2) >> intrinsic.at<double>(1,2);
    intrinsic.at<double>(0,2) /= 2;
    intrinsic.at<double>(1,2) /= 2;
    int i;
    Mat point1(n, 2, CV_64FC1);
    Mat point2(n, 2, CV_64FC1);
    for(i = 0; i<n; i++) inf >> point1.at<double>(i,0) >> point1.at<double>(i,1);
    for(i = 0; i<n; i++) inf >> point2.at<double>(i,0) >> point2.at<double>(i,1);
    cout << point1 << point2 << endl;
    Mat status;
    Mat fund = findFundamentalMat(point2, point1, FM_RANSAC, 0.5, 0.999, status);
    cout << "Fundamental Matrix: " << endl << fund << endl;
    cout << "Status: " << status << endl;
    
    Mat vec1(3, 1, CV_64FC1);
    Mat vec2(3, 1, CV_64FC1);
    for(i = 0; i<n; i++)
    {
        vec1.at<double>(0,0) = point1.at<double>(i,0);
        vec1.at<double>(1,0) = point1.at<double>(i,1);
        vec1.at<double>(2,0) = 1;
        vec2.at<double>(0,0) = point2.at<double>(i,0);
        vec2.at<double>(1,0) = point2.at<double>(i,1);
        vec2.at<double>(2,0) = 1;
        cout << vec1.t()*fund*vec2 << " " << vec2.t()*fund*vec1 << endl;
    }
    
    cout << "Give two matched pairs and their shared distance: " << endl;
    int p11, p12, p21, p22;
    double d;
    cin >> p11 >> p12 >> p21 >> p22 >> d;
    
    double fmin = 9000;
    double fmax = 11000;
    double f;
    Mat essence(3, 3, CV_64FC1);
    SVD essSVD;
    
    Mat W = Mat::zeros(3, 3, CV_64FC1);
    W.at<double>(0,1) = -1;
    W.at<double>(1,0) = 1;
    W.at<double>(2,2) = 1;
    Mat Z = Mat::zeros(3, 3, CV_64FC1);
    Z.at<double>(0,1) = 1000;
    Z.at<double>(1,0) = -1000;

    double x1 = point1.at<double>(0,0);
    double y1 = point1.at<double>(0,1);
    double x2 = point2.at<double>(0,0);
    double y2 = point2.at<double>(0,1);
    Mat origin(3, 2, CV_64FC1);
    Mat direction(3, 2, CV_64FC1);
    Mat temp(3, 1, CV_64FC1);
    Mat dst(3, 4, CV_64FC1);
    Mat wdst;
    double d1, d2, sc;
    while(fmax-fmin>1)
    {
        f = (fmax+fmin)/2;
        cout << "Current f: " << f << endl;
        intrinsic.at<double>(0,0) = f;
        intrinsic.at<double>(1,1) = f;
        essence = intrinsic.t()*fund*intrinsic;
        essSVD = SVD::SVD(essence, SVD::FULL_UV);
        sc = (essSVD.w.at<double>(0,0) + essSVD.w.at<double>(0,1))/2;
        cout << sc << endl;
        cout << essSVD.w << endl;
        cout << determinant(essSVD.u*W.t()*essSVD.vt) << endl;
        if(zFront(x1, y1, x2, y2, essSVD.u*W*essSVD.vt, zCross(essSVD.vt.t()*Z*essSVD.vt), intrinsic))
        {
            R = essSVD.u*W*essSVD.vt;
            T = zCross(essSVD.vt.t()*Z*essSVD.vt);
        }
        else if(zFront(x1, y1, x2, y2, essSVD.u*W*essSVD.vt, zCross(-essSVD.vt.t()*Z*essSVD.vt), intrinsic))
        {
            R = essSVD.u*W*essSVD.vt;
            T = zCross(-essSVD.vt.t()*Z*essSVD.vt);
        }
        else if(zFront(x1, y1, x2, y2, essSVD.u*W.t()*essSVD.vt, zCross(essSVD.vt.t()*Z*essSVD.vt), intrinsic))
        {
            R = essSVD.u*W.t()*essSVD.vt;
            T = zCross(essSVD.vt.t()*Z*essSVD.vt);
        }
        else if(zFront(x1, y1, x2, y2, essSVD.u*W.t()*essSVD.vt, zCross(-essSVD.vt.t()*Z*essSVD.vt), intrinsic))
        {
            R = essSVD.u*W.t()*essSVD.vt;
            T = zCross(-essSVD.vt.t()*Z*essSVD.vt);
        }
        else cout << "Fatal Error! " << endl;
        origin.col(0) = 0.0;
        origin.col(1) = T;
        
        temp.at<double>(0,0) = point1.at<double>(p11,0);
        temp.at<double>(1,0) = point1.at<double>(p11,1);
        temp.at<double>(2,0) = 1;
        direction.col(0) = intrinsic.inv()*temp;
        temp.at<double>(0,0) = point2.at<double>(p11,0);
        temp.at<double>(1,0) = point2.at<double>(p11,1);
        temp.at<double>(2,0) = 1;
        direction.col(1) = R.t()*intrinsic.inv()*temp;
        zMeanDistancePoint(2, origin, direction, wdst);
        dst.col(0) = wdst + 0.0;
        
        temp.at<double>(0,0) = point1.at<double>(p12,0);
        temp.at<double>(1,0) = point1.at<double>(p12,1);
        temp.at<double>(2,0) = 1;
        direction.col(0) = intrinsic.inv()*temp;
        temp.at<double>(0,0) = point2.at<double>(p12,0);
        temp.at<double>(1,0) = point2.at<double>(p12,1);
        temp.at<double>(2,0) = 1;
        direction.col(1) = R.t()*intrinsic.inv()*temp;
        zMeanDistancePoint(2, origin, direction, wdst);
        dst.col(1) = wdst + 0.0;
        
        temp.at<double>(0,0) = point1.at<double>(p21,0);
        temp.at<double>(1,0) = point1.at<double>(p21,1);
        temp.at<double>(2,0) = 1;
        direction.col(0) = intrinsic.inv()*temp;
        temp.at<double>(0,0) = point2.at<double>(p21,0);
        temp.at<double>(1,0) = point2.at<double>(p21,1);
        temp.at<double>(2,0) = 1;
        direction.col(1) = R.t()*intrinsic.inv()*temp;
        zMeanDistancePoint(2, origin, direction, wdst);
        dst.col(2) = wdst + 0.0;
        
        temp.at<double>(0,0) = point1.at<double>(p22,0);
        temp.at<double>(1,0) = point1.at<double>(p22,1);
        temp.at<double>(2,0) = 1;
        direction.col(0) = intrinsic.inv()*temp;
        temp.at<double>(0,0) = point2.at<double>(p22,0);
        temp.at<double>(1,0) = point2.at<double>(p22,1);
        temp.at<double>(2,0) = 1;
        direction.col(1) = R.t()*intrinsic.inv()*temp;
        zMeanDistancePoint(2, origin, direction, wdst);
        dst.col(3) = wdst + 0.0;
        
        d1 = zDist(dst.at<double>(0,0), dst.at<double>(0,1), dst.at<double>(0,2), dst.at<double>(1,0), dst.at<double>(1,1), dst.at<double>(1,2));
        d2 = zDist(dst.at<double>(2,0), dst.at<double>(2,1), dst.at<double>(2,2), dst.at<double>(3,0), dst.at<double>(3,1), dst.at<double>(3,2));
        cout << "Pair 1: " << d1 << " " << "Pair 2: " << d2 << endl;
        if(dst.at<double>(0,2)+dst.at<double>(1,2) > dst.at<double>(2,2)+dst.at<double>(3,2))
        {
            if(d1>d2) fmax = f; else fmin = f;
        }
        else if(d1>d2) fmin = f; else fmax = f;
    }
    cout << "Qualified f = " << f << endl;
    cout << "Absolute discrepancy: " << d1 - d2 << endl;
    T = T*(d/d1);
    return 0;
}

// Given two cooresponding pairs of lines, solve for the affine transformation of two camera frames
// Camera 2 should be considered the canonical one
int zTwoLineCameraPose(const int n1, const CvPoint2D32f* line11, const CvPoint2D32f* line21, const int n2, const CvPoint2D32f* line12, const CvPoint2D32f* line22, const double delta, Mat& R, Mat& T, Mat& intrinsicMatrix, int knownf = 0)
{
    Mat A1(n1*2, 5, CV_64FC1);
    Mat b1(n1*2, 1, CV_64FC1);
    Mat A2(n2*2, 5, CV_64FC1);
    Mat b2(n2*2, 1, CV_64FC1);
    
    Mat c11(5, 1, CV_64FC1);
    Mat c12(5, 1, CV_64FC1);
    Mat c21(5, 1, CV_64FC1);
    Mat c22(5, 1, CV_64FC1);
    Mat c(5, 1, CV_64FC1);
    double cx = intrinsicMatrix.at<double>(0,2);
    double cy = intrinsicMatrix.at<double>(1,2);
    double fx = intrinsicMatrix.at<double>(0,0);
    double z0;
    int i, j, k;
    CvPoint3D32f pLine11[n1], pLine21[n1], pLine12[n2], pLine22[n2];
    
    Mat wM(5, 5, CV_64FC1);
    ofstream ouf("LSEResult.txt");
    // 1.1 Line 1-1
    for(i = 0; i<n1; i++)
    {
        A1.at<double>(i+i,0) = i;
        A1.at<double>(i+i,1) = 0;
        A1.at<double>(i+i,2) = (cx-line11[i].x)*i;
        A1.at<double>(i+i,3) = 1;
        A1.at<double>(i+i,4) = 0;
        A1.at<double>(i+i+1,0) = 0;
        A1.at<double>(i+i+1,1) = i;
        A1.at<double>(i+i+1,2) = (cy-line11[i].y)*i;
        A1.at<double>(i+i+1,3) = 0;
        A1.at<double>(i+i+1,4) = 1;
        b1.at<double>(i+i,0) = line11[i].x-cx;
        b1.at<double>(i+i+1,0) = line11[i].y-cy;
    };
    solve(A1, b1, c11, DECOMP_SVD);
    wM = A1.t()*A1;
    //  ouf << "Line1-1:" << endl << A1*c11 << endl << endl;
    //  cout << "CN: " << norm(wM)*norm(wM.inv()) << endl;
    //  cout << "Line11: " << endl << A1*c11-b1 << endl;
    
    // 1.2 Line 2-1
    for(i = 0; i<n1; i++)
    {
        A1.at<double>(i+i,0) = i;
        A1.at<double>(i+i,1) = 0;
        A1.at<double>(i+i,2) = (cx-line21[i].x)*i;
        A1.at<double>(i+i,3) = 1;
        A1.at<double>(i+i,4) = 0;
        A1.at<double>(i+i+1,0) = 0;
        A1.at<double>(i+i+1,1) = i;
        A1.at<double>(i+i+1,2) = (cy-line21[i].y)*i;
        A1.at<double>(i+i+1,3) = 0;
        A1.at<double>(i+i+1,4) = 1;
        b1.at<double>(i+i,0) = line21[i].x-cx;
        b1.at<double>(i+i+1,0) = line21[i].y-cy;
    };
    solve(A1, b1, c21, DECOMP_SVD);
    wM = A1.t()*A1;
    //  ouf << "Line2-1:" << endl << A1*c21 << endl << endl;
    //  cout << "CN: " << norm(wM)*norm(wM.inv()) << endl;
    //  cout << "Line21: " << endl << A1*c21-b1 << endl;
    
    
    // 1.3 Line 1-2
    for(i = 0; i<n2; i++)
    {
        A2.at<double>(i+i,0) = i;
        A2.at<double>(i+i,1) = 0;
        A2.at<double>(i+i,2) = (cx-line12[i].x)*i;
        A2.at<double>(i+i,3) = 1;
        A2.at<double>(i+i,4) = 0;
        A2.at<double>(i+i+1,0) = 0;
        A2.at<double>(i+i+1,1) = i;
        A2.at<double>(i+i+1,2) = (cy-line12[i].y)*i;
        A2.at<double>(i+i+1,3) = 0;
        A2.at<double>(i+i+1,4) = 1;
        b2.at<double>(i+i,0) = line12[i].x-cx;
        b2.at<double>(i+i+1,0) = line12[i].y-cy;
    };
    //    cout << A2 << endl << b1 << endl;
    solve(A2, b2, c12, DECOMP_SVD);
    wM = A2.t()*A2;
    //  ouf << "Line1-2:" << endl << A2*c12 << endl << endl;
    //  cout << "CN: " << norm(wM)*norm(wM.inv()) << endl;
    //  cout << "Line12: " << endl << A2*c12-b2 << endl;
    
    // 1.4 Line 2-2
    for(i = 0; i<n2; i++)
    {
        A2.at<double>(i+i,0) = i;
        A2.at<double>(i+i,1) = 0;
        A2.at<double>(i+i,2) = (cx-line22[i].x)*i;
        A2.at<double>(i+i,3) = 1;
        A2.at<double>(i+i,4) = 0;
        A2.at<double>(i+i+1,0) = 0;
        A2.at<double>(i+i+1,1) = i;
        A2.at<double>(i+i+1,2) = (cy-line22[i].y)*i;
        A2.at<double>(i+i+1,3) = 0;
        A2.at<double>(i+i+1,4) = 1;
        b2.at<double>(i+i,0) = line22[i].x-cx;
        b2.at<double>(i+i+1,0) = line22[i].y-cy;
    };
    solve(A2, b2, c22, DECOMP_SVD);
    wM = A2.t()*A2;
    //  ouf << "Line2-2:" << endl << A2*c22 << endl << endl;
    //  cout << "CN: " << norm(wM)*norm(wM.inv()) << endl;
    //  cout << "Line22: " << endl << A2*c22-b2 << endl;
    
    ouf.close();
    //  cout << "Line section finished." << endl;
    cout << "11: " << c11 << endl;
    cout << "12: " << c12 << endl;
    cout << "21: " << c21 << endl;
    cout << "22: " << c22 << endl;
    
    
    // 2.1 Solve for f
    if(knownf == 0)
    {
        // Compromise 1M
        const double S = 100;
        double initGuess = 10000;
        double pA = c11.at<double>(0,0)*c12.at<double>(0,0)+c11.at<double>(1,0)*c12.at<double>(1,0);
        double pC = c11.at<double>(2,0)*S*c12.at<double>(2,0)*S;
        double pE = c11.at<double>(0,0)*c11.at<double>(0,0)+c11.at<double>(1,0)*c11.at<double>(1,0);
        double pF = c11.at<double>(2,0)*S*c11.at<double>(2,0)*S;
        double pG = c12.at<double>(0,0)*c12.at<double>(0,0)+c12.at<double>(1,0)*c12.at<double>(1,0);
        double pH = c12.at<double>(2,0)*S*c12.at<double>(2,0)*S;
        
        double qA = c21.at<double>(0,0)*c22.at<double>(0,0)+c21.at<double>(1,0)*c22.at<double>(1,0);
        double qC = c21.at<double>(2,0)*S*c22.at<double>(2,0)*S;
        double qE = c21.at<double>(0,0)*c21.at<double>(0,0)+c21.at<double>(1,0)*c21.at<double>(1,0);
        double qF = c21.at<double>(2,0)*S*c21.at<double>(2,0)*S;
        double qG = c22.at<double>(0,0)*c22.at<double>(0,0)+c22.at<double>(1,0)*c22.at<double>(1,0);
        double qH = c22.at<double>(2,0)*S*c22.at<double>(2,0)*S;
        
        double coef[5];
        coef[0] = qA*qA*pE*pG - pA*pA*qE*qG;
        coef[1] = 2*qA*qC*pE*pG + qA*qA*(pF*pG+pE*pH) - 2*pA*pC*qE*qG - pA*pA*(qF*qG+qE*qH);
        coef[2] = qA*qA*pF*pH + 2*qA*qC*(pF*pG+pH*pE) + qC*qC*pE*pG - pA*pA*qF*qH - 2*pA*pC*(qF*qG+qH*qE) - pC*pC*qE*qG;
        coef[3] = 2*qA*qC*pF*pH + qC*qC*(pF*pG+pE*pH) - 2*pA*pC*qF*qH - pC*pC*(qF*qG+qE*qH);
        //coef[4] = qC*qC*pF*pH - pC*pC*qF*qH;
        
        //        cout << coef[3] << " " << coef[2] << " " << coef[1] << " " << coef[0] << endl;
        fx = zPolyNewton(3, coef, initGuess);
        fx = sqrt(fx)*S;
        intrinsicMatrix.at<double>(0,0) = fx;
        intrinsicMatrix.at<double>(1,1) = fx;
        cout << "f gained. f = : " << fx << endl;
    }
    
    // 3.1
    c11 = c11 / fx;
    c11.at<double>(2,0) = c11.at<double>(2,0)*fx;
    z0 = delta/sqrt(c11.at<double>(0,0)*c11.at<double>(0,0)+c11.at<double>(1,0)*c11.at<double>(1,0)+c11.at<double>(2,0)*c11.at<double>(2,0));
    c = c11 * z0;
    //  cout << z0 << " -- " << c << endl;
    pLine11[0].x = c.at<double>(0,3);
    pLine11[0].y = c.at<double>(0,4);
    pLine11[0].z = z0;
    for(i = 1; i<n1; i++)
    {
        pLine11[i].x = pLine11[0].x+c.at<double>(0,0)*i;
        pLine11[i].y = pLine11[0].y+c.at<double>(1,0)*i;
        pLine11[i].z = z0 + c.at<double>(2,0)*i;
    }
    
    c21 = c21 / fx;
    c21.at<double>(2,0) = c21.at<double>(2,0)*fx;
    z0 = delta/sqrt(c21.at<double>(0,0)*c21.at<double>(0,0)+c21.at<double>(1,0)*c21.at<double>(1,0)+c21.at<double>(2,0)*c21.at<double>(2,0));
    c = c21 * z0;
    //  cout << z0 << " -- " << c << endl;
    pLine21[0].x = c.at<double>(0,3);
    pLine21[0].y = c.at<double>(0,4);
    pLine21[0].z = z0;
    for(i = 1; i<n1; i++)
    {
        pLine21[i].x = pLine21[0].x+c.at<double>(0,0)*i;
        pLine21[i].y = pLine21[0].y+c.at<double>(1,0)*i;
        pLine21[i].z = z0 + c.at<double>(2,0)*i;
    }
    
    c12 = c12 / fx;
    c12.at<double>(2,0) = c12.at<double>(2,0)*fx;
    z0 = delta/sqrt(c12.at<double>(0,0)*c12.at<double>(0,0)+c12.at<double>(1,0)*c12.at<double>(1,0)+c12.at<double>(2,0)*c12.at<double>(2,0));
    c = c12 * z0;
    //  cout << z0 << " -- " << c << endl;
    pLine12[0].x = c.at<double>(0,3);
    pLine12[0].y = c.at<double>(0,4);
    pLine12[0].z = z0;
    for(i = 1; i<n2; i++)
    {
        pLine12[i].x = pLine12[0].x+c.at<double>(0,0)*i;
        pLine12[i].y = pLine12[0].y+c.at<double>(1,0)*i;
        pLine12[i].z = z0 + c.at<double>(2,0)*i;
    }
    
    c22 = c22 / fx;
    c22.at<double>(2,0) = c22.at<double>(2,0)*fx;
    z0 = delta/sqrt(c22.at<double>(0,0)*c22.at<double>(0,0)+c22.at<double>(1,0)*c22.at<double>(1,0)+c22.at<double>(2,0)*c22.at<double>(2,0));
    c = c22 * z0;
    //  cout << z0 << " -- " << c << endl;
    pLine22[0].x = c.at<double>(0,3);
    pLine22[0].y = c.at<double>(0,4);
    pLine22[0].z = z0;
    for(i = 1; i<n2; i++)
    {
        pLine22[i].x = pLine22[0].x+c.at<double>(0,0)*i;
        pLine22[i].y = pLine22[0].y+c.at<double>(1,0)*i;
        pLine22[i].z = z0 + c.at<double>(2,0)*i;
    }
    //  cout << "Lines retrieved." << endl;
    
    //    cout << "Line11:" << endl;
    //    for(i = 0; i<n1; i++) cout << pLine11[i].x << " " << pLine11[i].y << " " << pLine11[i].z << endl;
    //    cout << "Line12:" << endl;
    //    for(i = 0; i<n2; i++) cout << pLine12[i].x << " " << pLine12[i].y << " " << pLine12[i].z << endl;
    //    cout << "Line21:" << endl;
    //    for(i = 0; i<n1; i++) cout << pLine21[i].x << " " << pLine21[i].y << " " << pLine21[i].z << endl;
    //    cout << "Line22:" << endl;
    //    for(i = 0; i<n2; i++) cout << pLine22[i].x << " " << pLine22[i].y << " " << pLine22[i].z << endl;
    
    // 4.1 LSE the rotation. Use 16 pairs of corresponding points
    /*
     const int nPair = 10;
     int label1[nPair], label2[nPair];
     for(i = 0; i<nPair; i++)
     {
     label1[i] = (n1-1)*i/(nPair-1);
     label2[i] = (n2-1)*i/(nPair-1);
     }
     
     Mat A3(nPair*nPair*3, 9, CV_64FC1);
     Mat b3(nPair*nPair*3, 1, CV_64FC1);
     Mat RCol(9, 1, CV_64FC1);
     for(i = 0; i<nPair; i++)
     for(j = 0; j<nPair; j++)
     {
     k = i*nPair+j;
     cout << label2[j] << " " << label1[i] << " " << pLine22[label2[j]].z << " " << pLine21[label1[i]].z << endl;
     A3.at<double>(k+k+k, 0) = pLine22[label2[j]].x-pLine21[label1[i]].x;
     A3.at<double>(k+k+k, 1) = pLine22[label2[j]].y-pLine21[label1[i]].y;
     A3.at<double>(k+k+k, 2) = pLine22[label2[j]].z-pLine21[label1[i]].z;
     A3.at<double>(k+k+k+1, 3) = pLine22[label2[j]].x-pLine21[label1[i]].x;
     A3.at<double>(k+k+k+1, 4) = pLine22[label2[j]].y-pLine21[label1[i]].y;
     A3.at<double>(k+k+k+1, 5) = pLine22[label2[j]].z-pLine21[label1[i]].z;
     A3.at<double>(k+k+k+2, 6) = pLine22[label2[j]].x-pLine21[label1[i]].x;
     A3.at<double>(k+k+k+2, 7) = pLine22[label2[j]].y-pLine21[label1[i]].y;
     A3.at<double>(k+k+k+2, 8) = pLine22[label2[j]].z-pLine21[label1[i]].z;
     b3.at<double>(k+k+k, 0) = pLine12[label2[j]].x-pLine11[label1[i]].x;
     b3.at<double>(k+k+k+1, 0) = pLine12[label2[j]].y-pLine11[label1[i]].y;
     b3.at<double>(k+k+k+2, 0) = pLine12[label2[j]].z-pLine11[label1[i]].z;
     }
     solve(A3, b3, RCol, DECOMP_SVD);
     cout << A3 << endl << b3 << endl << RCol << endl << b3-A3*RCol << endl;;
     cout << "LSE the rotation." << endl;
     
     */
    
    // 4.1 LSE is ill-posed. Use another algorithm.
    Mat M1(3, 3, CV_64FC1);
    Mat M2(3, 3, CV_64FC1);
    M1.at<double>(0,0) = pLine12[0].x-pLine11[n1/2].x;
    M1.at<double>(1,0) = pLine12[0].y-pLine11[n1/2].y;
    M1.at<double>(2,0) = pLine12[0].z-pLine11[n1/2].z;
    M1.at<double>(0,1) = pLine12[n2-1].x-pLine11[n1/2].x;
    M1.at<double>(1,1) = pLine12[n2-1].y-pLine11[n1/2].y;
    M1.at<double>(2,1) = pLine12[n2-1].z-pLine11[n1/2].z;
    M1.col(0) = M1.col(0) / norm(M1.col(0));
    M1.col(1) = M1.col(1) / norm(M1.col(1));
    M1.at<double>(0,2) = M1.at<double>(1,0)*M1.at<double>(2,1)-M1.at<double>(2,0)*M1.at<double>(1,1);
    M1.at<double>(1,2) = M1.at<double>(2,0)*M1.at<double>(0,1)-M1.at<double>(0,0)*M1.at<double>(2,1);
    M1.at<double>(2,2) = M1.at<double>(0,0)*M1.at<double>(1,1)-M1.at<double>(1,0)*M1.at<double>(0,1);
    M1.col(2) = M1.col(2) / norm(M1.col(2));
    
    M2.at<double>(0,0) = pLine22[0].x-pLine21[n1/2].x;
    M2.at<double>(1,0) = pLine22[0].y-pLine21[n1/2].y;
    M2.at<double>(2,0) = pLine22[0].z-pLine21[n1/2].z;
    M2.at<double>(0,1) = pLine22[n2-1].x-pLine21[n1/2].x;
    M2.at<double>(1,1) = pLine22[n2-1].y-pLine21[n1/2].y;
    M2.at<double>(2,1) = pLine22[n2-1].z-pLine21[n1/2].z;
    M2.col(0) = M2.col(0) / norm(M2.col(0));
    M2.col(1) = M2.col(1) / norm(M2.col(1));
    M2.at<double>(0,2) = M2.at<double>(1,0)*M2.at<double>(2,1)-M2.at<double>(2,0)*M2.at<double>(1,1);
    M2.at<double>(1,2) = M2.at<double>(2,0)*M2.at<double>(0,1)-M2.at<double>(0,0)*M2.at<double>(2,1);
    M2.at<double>(2,2) = M2.at<double>(0,0)*M2.at<double>(1,1)-M2.at<double>(1,0)*M2.at<double>(0,1);
    M2.col(2) = M2.col(2) / norm(M2.col(2));
    cout << "Normal: " << fixed << setprecision(6) << M1.col(2) << endl;
    //  cout << M1 << endl <<M2 << endl;
    R = M1*M2.inv();
    
    
    // 4.2 Adjust R to be unitary
    /*
     R.at<double>(0,0) = RCol.at<double>(0,0);
     R.at<double>(0,1) = RCol.at<double>(1,0);
     R.at<double>(0,2) = RCol.at<double>(2,0);
     R.at<double>(1,0) = RCol.at<double>(3,0);
     R.at<double>(1,1) = RCol.at<double>(4,0);
     R.at<double>(1,2) = RCol.at<double>(5,0);
     R.at<double>(2,0) = RCol.at<double>(6,0);
     R.at<double>(2,1) = RCol.at<double>(7,0);
     R.at<double>(2,2) = RCol.at<double>(8,0);
     
     cout << "Raw R:" << endl << R << endl;
     SVD RSVD(R, SVD::FULL_UV);
     cout << "SVD: " << RSVD.w << endl;
     R = RSVD.u*RSVD.vt;
     */
    //    cout << "R:" << endl << R << endl;
    
    //5.1 Calculate T
    
    Mat tempT(3, 1, CV_64FC1);
    Mat tempV1(3, 1, CV_64FC1);
    Mat tempV2(3, 1, CV_64FC1);
    Mat Rt = R.t();
    tempT.at<double>(0,0) = 0;
    tempT.at<double>(1,0) = 0;
    tempT.at<double>(2,0) = 0;
    for(i = 0; i<n1; i++)
    {
        tempV1.at<double>(0,0) = pLine11[i].x;
        tempV1.at<double>(1,0) = pLine11[i].y;
        tempV1.at<double>(2,0) = pLine11[i].z;
        tempV2.at<double>(0,0) = pLine21[i].x;
        tempV2.at<double>(1,0) = pLine21[i].y;
        tempV2.at<double>(2,0) = pLine21[i].z;
        tempT += tempV2 - Rt*tempV1;
    }
    for(i = 0; i<n2; i++)
    {
        tempV1.at<double>(0,0) = pLine12[i].x;
        tempV1.at<double>(1,0) = pLine12[i].y;
        tempV1.at<double>(2,0) = pLine12[i].z;
        tempV2.at<double>(0,0) = pLine22[i].x;
        tempV2.at<double>(1,0) = pLine22[i].y;
        tempV2.at<double>(2,0) = pLine22[i].z;
        tempT += tempV2 - Rt*tempV1;
    }
    tempT = tempT * (1.0 / (n1+n2));
    T = tempT;
    //  cout << "T: " << T << endl;
    cout << "IR1: " << endl;
    for(i = 0; i<n1; i++) cout << pLine11[i].x << " " << pLine11[i].y << " " << pLine11[i].z << " " << pLine21[i].x << " " << pLine21[i].y << " " << pLine21[i].z << endl;
    cout << "IR2: " << endl;
    for(i = 0; i<n2; i++) cout << pLine12[i].x << " " << pLine12[i].y << " " << pLine12[i].z << " " << pLine22[i].x << " " << pLine22[i].y << " " << pLine22[i].z << endl;
    
    return 0;
}

double zTwoLineCameraPose2(const int st, const int n1, const CvPoint2D32f* line11, const CvPoint2D32f* line21, const int n2, const CvPoint2D32f* line12, const CvPoint2D32f* line22, const double delta, Mat& R, Mat& T, Mat& intrinsicMatrix, int knownf = 0)
{
    cout << "zTwoLineCameraPose2" << endl;
    Mat A1(st*2, 5, CV_64FC1);
    Mat b1(st*2, 1, CV_64FC1);
    Mat A2(st*2, 5, CV_64FC1);
    Mat b2(st*2, 1, CV_64FC1);
    
    Mat wc11(5, 1, CV_64FC1);
    Mat wc12(5, 1, CV_64FC1);
    Mat wc21(5, 1, CV_64FC1);
    Mat wc22(5, 1, CV_64FC1);
    Mat c11(5, 1, CV_64FC1);
    Mat c12(5, 1, CV_64FC1);
    Mat c21(5, 1, CV_64FC1);
    Mat c22(5, 1, CV_64FC1);
    Mat c(5, 1, CV_64FC1);
    double cx = intrinsicMatrix.at<double>(0,2);
    double cy = intrinsicMatrix.at<double>(1,2);
    double fx = intrinsicMatrix.at<double>(0,0);
    double z0;
    int i, j, k;
    CvPoint3D32f pLine11[n1], pLine21[n1], pLine12[n2], pLine22[n2];

    Mat wM(5, 5, CV_64FC1);
    //cout << "CHECKPOINT" << endl;
    ofstream ouf("LSEResult.txt", ios::app);
    int l;
    
    // 1.1 Line 1-1
    c11 = 0.0*c11;
    for(l = 0; l<n1-st; l++)
    {
        for(i = 0; i<st; i++)
        {
            A1.at<double>(i+i,0) = i+l;
            A1.at<double>(i+i,1) = 0;
            A1.at<double>(i+i,2) = (cx-line11[i+l].x)*(i+l);
            A1.at<double>(i+i,3) = 1;
            A1.at<double>(i+i,4) = 0;
            A1.at<double>(i+i+1,0) = 0;
            A1.at<double>(i+i+1,1) = i+l;
            A1.at<double>(i+i+1,2) = (cy-line11[i+l].y)*(i+l);
            A1.at<double>(i+i+1,3) = 0;
            A1.at<double>(i+i+1,4) = 1;
            b1.at<double>(i+i,0) = line11[i+l].x-cx;
            b1.at<double>(i+i+1,0) = line11[i+l].y-cy;
        }
        solve(A1, b1, wc11, DECOMP_SVD);
        c11 = c11 + wc11;
    }
    c11 = c11 / (n1-st);
    wM = A1.t()*A1;
    //  ouf << "Line1-1:" << endl << A1*c11 << endl << endl;
    //  cout << "CN: " << norm(wM)*norm(wM.inv()) << endl;
    //  cout << "Line11: " << endl << A1*c11-b1 << endl;
    //cout << "CHECKPOINT" << endl;
    // 1.2 Line 2-1
    c21 = 0.0*c21;
    for(l = 0; l<n1-st; l++)
    {
        for(i = 0; i<st; i++)
        {
            A1.at<double>(i+i,0) = i+l;
            A1.at<double>(i+i,1) = 0;
            A1.at<double>(i+i,2) = (cx-line21[i+l].x)*(i+l);
            A1.at<double>(i+i,3) = 1;
            A1.at<double>(i+i,4) = 0;
            A1.at<double>(i+i+1,0) = 0;
            A1.at<double>(i+i+1,1) = i+l;
            A1.at<double>(i+i+1,2) = (cy-line21[i+l].y)*(i+l);
            A1.at<double>(i+i+1,3) = 0;
            A1.at<double>(i+i+1,4) = 1;
            b1.at<double>(i+i,0) = line21[i+l].x-cx;
            b1.at<double>(i+i+1,0) = line21[i+l].y-cy;
        }
        solve(A1, b1, c21, DECOMP_SVD);
        c21 = c21 + wc21;
    }
    c21 = c21 / (n1-st);
    wM = A1.t()*A1;
    //  ouf << "Line2-1:" << endl << A1*c21 << endl << endl;
    //  cout << "CN: " << norm(wM)*norm(wM.inv()) << endl;
    //  cout << "Line21: " << endl << A1*c21-b1 << endl;
    
    //cout << "CHECKPOINT" << endl;
    // 1.3 Line 1-2
    c12 = 0.0*c12;
    for(l = 0; l<n2-st; l++)
    {
        for(i = 0; i<st; i++)
        {
            A2.at<double>(i+i,0) = i+l;
            A2.at<double>(i+i,1) = 0;
            A2.at<double>(i+i,2) = (cx-line12[i+l].x)*(i+l);
            A2.at<double>(i+i,3) = 1;
            A2.at<double>(i+i,4) = 0;
            A2.at<double>(i+i+1,0) = 0;
            A2.at<double>(i+i+1,1) = i+l;
            A2.at<double>(i+i+1,2) = (cy-line12[i+l].y)*(i+l);
            A2.at<double>(i+i+1,3) = 0;
            A2.at<double>(i+i+1,4) = 1;
            b2.at<double>(i+i,0) = line12[i+l].x-cx;
            b2.at<double>(i+i+1,0) = line12[i+l].y-cy;
        }
    //    cout << A2 << endl << b1 << endl;
        solve(A2, b2, wc12, DECOMP_SVD);
        c12 = c12 + wc12;
    }
    c12 = c12 / (n2-st);
    wM = A2.t()*A2;
    //  ouf << "Line1-2:" << endl << A2*c12 << endl << endl;
    //  cout << "CN: " << norm(wM)*norm(wM.inv()) << endl;
    //  cout << "Line12: " << endl << A2*c12-b2 << endl;
    //cout << "CHECKPOINT" << endl;
    // 1.4 Line 2-2
    c22 = 0.0*c22;
    for(l = 0; l<n2-st; l++)
    {
        for(i = 0; i<st; i++)
        {
            A2.at<double>(i+i,0) = i+l;
            A2.at<double>(i+i,1) = 0;
            A2.at<double>(i+i,2) = (cx-line22[i+l].x)*(i+l);
            A2.at<double>(i+i,3) = 1;
            A2.at<double>(i+i,4) = 0;
            A2.at<double>(i+i+1,0) = 0;
            A2.at<double>(i+i+1,1) = i+l;
            A2.at<double>(i+i+1,2) = (cy-line22[i+l].y)*(i+l);
            A2.at<double>(i+i+1,3) = 0;
            A2.at<double>(i+i+1,4) = 1;
            b2.at<double>(i+i,0) = line22[i+l].x-cx;
            b2.at<double>(i+i+1,0) = line22[i+l].y-cy;
        }
        solve(A2, b2, wc22, DECOMP_SVD);
        c22 = c22 + wc22;
    }
    c22 = c22 / (n2-st);
    wM = A2.t()*A2;
    //  ouf << "Line2-2:" << endl << A2*c22 << endl << endl;
    //  cout << "CN: " << norm(wM)*norm(wM.inv()) << endl;
    //  cout << "Line22: " << endl << A2*c22-b2 << endl;
    //cout << "CHECKPOINT" << endl;
    
    //  cout << "Line section finished." << endl;
    ouf << "11: " << c11 << endl;
    ouf << "12: " << c12 << endl;
    ouf << "21: " << c21 << endl;
    ouf << "22: " << c22 << endl;

    
    // 2.1 Solve for f
    if(knownf == 0)
    {
        // Compromise 1M
        const double S = 100;
        double initGuess = 10000;
        double pA = c11.at<double>(0,0)*c12.at<double>(0,0)+c11.at<double>(1,0)*c12.at<double>(1,0);
        double pC = c11.at<double>(2,0)*S*c12.at<double>(2,0)*S;
        double pE = c11.at<double>(0,0)*c11.at<double>(0,0)+c11.at<double>(1,0)*c11.at<double>(1,0);
        double pF = c11.at<double>(2,0)*S*c11.at<double>(2,0)*S;
        double pG = c12.at<double>(0,0)*c12.at<double>(0,0)+c12.at<double>(1,0)*c12.at<double>(1,0);
        double pH = c12.at<double>(2,0)*S*c12.at<double>(2,0)*S;
        
        double qA = c21.at<double>(0,0)*c22.at<double>(0,0)+c21.at<double>(1,0)*c22.at<double>(1,0);
        double qC = c21.at<double>(2,0)*S*c22.at<double>(2,0)*S;
        double qE = c21.at<double>(0,0)*c21.at<double>(0,0)+c21.at<double>(1,0)*c21.at<double>(1,0);
        double qF = c21.at<double>(2,0)*S*c21.at<double>(2,0)*S;
        double qG = c22.at<double>(0,0)*c22.at<double>(0,0)+c22.at<double>(1,0)*c22.at<double>(1,0);
        double qH = c22.at<double>(2,0)*S*c22.at<double>(2,0)*S;
        
        double coef[5];
        coef[0] = qA*qA*pE*pG - pA*pA*qE*qG;
        coef[1] = 2*qA*qC*pE*pG + qA*qA*(pF*pG+pE*pH) - 2*pA*pC*qE*qG - pA*pA*(qF*qG+qE*qH);
        coef[2] = qA*qA*pF*pH + 2*qA*qC*(pF*pG+pH*pE) + qC*qC*pE*pG - pA*pA*qF*qH - 2*pA*pC*(qF*qG+qH*qE) - pC*pC*qE*qG;
        coef[3] = 2*qA*qC*pF*pH + qC*qC*(pF*pG+pE*pH) - 2*pA*pC*qF*qH - pC*pC*(qF*qG+qE*qH);
        //coef[4] = qC*qC*pF*pH - pC*pC*qF*qH;
        
        //        cout << coef[3] << " " << coef[2] << " " << coef[1] << " " << coef[0] << endl;
        fx = zPolyNewton(3, coef, initGuess);
        fx = sqrt(fx)*S;
        intrinsicMatrix.at<double>(0,0) = fx;
        intrinsicMatrix.at<double>(1,1) = fx;
        cout << "f gained. f = : " << fx << endl;
    }
    
    // 3.1
    c11 = c11 / fx;
    c11.at<double>(2,0) = c11.at<double>(2,0)*fx;
    z0 = delta/sqrt(c11.at<double>(0,0)*c11.at<double>(0,0)+c11.at<double>(1,0)*c11.at<double>(1,0)+c11.at<double>(2,0)*c11.at<double>(2,0));
    c = c11 * z0;
    ouf << "Modified 11: " << c << endl;
    //  cout << z0 << " -- " << c << endl;
    pLine11[0].x = c.at<double>(0,3);
    pLine11[0].y = c.at<double>(0,4);
    pLine11[0].z = z0;
    for(i = 1; i<n1; i++)
    {
        pLine11[i].x = pLine11[0].x+c.at<double>(0,0)*i;
        pLine11[i].y = pLine11[0].y+c.at<double>(1,0)*i;
        pLine11[i].z = z0 + c.at<double>(2,0)*i;
    }
    
    c21 = c21 / fx;
    c21.at<double>(2,0) = c21.at<double>(2,0)*fx;
    z0 = delta/sqrt(c21.at<double>(0,0)*c21.at<double>(0,0)+c21.at<double>(1,0)*c21.at<double>(1,0)+c21.at<double>(2,0)*c21.at<double>(2,0));
    c = c21 * z0;
    ouf << "Modified 21: " << c << endl;
    //  cout << z0 << " -- " << c << endl;
    pLine21[0].x = c.at<double>(0,3);
    pLine21[0].y = c.at<double>(0,4);
    pLine21[0].z = z0;
    for(i = 1; i<n1; i++)
    {
        pLine21[i].x = pLine21[0].x+c.at<double>(0,0)*i;
        pLine21[i].y = pLine21[0].y+c.at<double>(1,0)*i;
        pLine21[i].z = z0 + c.at<double>(2,0)*i;
    }
    
    c12 = c12 / fx;
    c12.at<double>(2,0) = c12.at<double>(2,0)*fx;
    z0 = delta/sqrt(c12.at<double>(0,0)*c12.at<double>(0,0)+c12.at<double>(1,0)*c12.at<double>(1,0)+c12.at<double>(2,0)*c12.at<double>(2,0));
    c = c12 * z0;
    //  cout << z0 << " -- " << c << endl;
    ouf << "Modified 12: " << c << endl;
    pLine12[0].x = c.at<double>(0,3);
    pLine12[0].y = c.at<double>(0,4);
    pLine12[0].z = z0;
    for(i = 1; i<n2; i++)
    {
        pLine12[i].x = pLine12[0].x+c.at<double>(0,0)*i;
        pLine12[i].y = pLine12[0].y+c.at<double>(1,0)*i;
        pLine12[i].z = z0 + c.at<double>(2,0)*i;
    }
    
    c22 = c22 / fx;
    c22.at<double>(2,0) = c22.at<double>(2,0)*fx;
    z0 = delta/sqrt(c22.at<double>(0,0)*c22.at<double>(0,0)+c22.at<double>(1,0)*c22.at<double>(1,0)+c22.at<double>(2,0)*c22.at<double>(2,0));
    c = c22 * z0;
    ouf << "Modified 22: " << c << endl;
    //  cout << z0 << " -- " << c << endl;
    pLine22[0].x = c.at<double>(0,3);
    pLine22[0].y = c.at<double>(0,4);
    pLine22[0].z = z0;
    for(i = 1; i<n2; i++)
    {
        pLine22[i].x = pLine22[0].x+c.at<double>(0,0)*i;
        pLine22[i].y = pLine22[0].y+c.at<double>(1,0)*i;
        pLine22[i].z = z0 + c.at<double>(2,0)*i;
    }
    //  cout << "Lines retrieved." << endl;
    
    //    cout << "Line11:" << endl;
    //    for(i = 0; i<n1; i++) cout << pLine11[i].x << " " << pLine11[i].y << " " << pLine11[i].z << endl;
    //    cout << "Line12:" << endl;
    //    for(i = 0; i<n2; i++) cout << pLine12[i].x << " " << pLine12[i].y << " " << pLine12[i].z << endl;
    //    cout << "Line21:" << endl;
    //    for(i = 0; i<n1; i++) cout << pLine21[i].x << " " << pLine21[i].y << " " << pLine21[i].z << endl;
    //    cout << "Line22:" << endl;
    //    for(i = 0; i<n2; i++) cout << pLine22[i].x << " " << pLine22[i].y << " " << pLine22[i].z << endl;
    
    // 4.1 LSE the rotation. Use 16 pairs of corresponding points
    /*
     const int nPair = 10;
     int label1[nPair], label2[nPair];
     for(i = 0; i<nPair; i++)
     {
     label1[i] = (n1-1)*i/(nPair-1);
     label2[i] = (n2-1)*i/(nPair-1);
     }
     
     Mat A3(nPair*nPair*3, 9, CV_64FC1);
     Mat b3(nPair*nPair*3, 1, CV_64FC1);
     Mat RCol(9, 1, CV_64FC1);
     for(i = 0; i<nPair; i++)
     for(j = 0; j<nPair; j++)
     {
     k = i*nPair+j;
     cout << label2[j] << " " << label1[i] << " " << pLine22[label2[j]].z << " " << pLine21[label1[i]].z << endl;
     A3.at<double>(k+k+k, 0) = pLine22[label2[j]].x-pLine21[label1[i]].x;
     A3.at<double>(k+k+k, 1) = pLine22[label2[j]].y-pLine21[label1[i]].y;
     A3.at<double>(k+k+k, 2) = pLine22[label2[j]].z-pLine21[label1[i]].z;
     A3.at<double>(k+k+k+1, 3) = pLine22[label2[j]].x-pLine21[label1[i]].x;
     A3.at<double>(k+k+k+1, 4) = pLine22[label2[j]].y-pLine21[label1[i]].y;
     A3.at<double>(k+k+k+1, 5) = pLine22[label2[j]].z-pLine21[label1[i]].z;
     A3.at<double>(k+k+k+2, 6) = pLine22[label2[j]].x-pLine21[label1[i]].x;
     A3.at<double>(k+k+k+2, 7) = pLine22[label2[j]].y-pLine21[label1[i]].y;
     A3.at<double>(k+k+k+2, 8) = pLine22[label2[j]].z-pLine21[label1[i]].z;
     b3.at<double>(k+k+k, 0) = pLine12[label2[j]].x-pLine11[label1[i]].x;
     b3.at<double>(k+k+k+1, 0) = pLine12[label2[j]].y-pLine11[label1[i]].y;
     b3.at<double>(k+k+k+2, 0) = pLine12[label2[j]].z-pLine11[label1[i]].z;
     }
     solve(A3, b3, RCol, DECOMP_SVD);
     cout << A3 << endl << b3 << endl << RCol << endl << b3-A3*RCol << endl;;
     cout << "LSE the rotation." << endl;
     
     */
    
    // 4.1 LSE is ill-posed. Use another algorithm.
    Mat M1(3, 3, CV_64FC1);
    Mat M2(3, 3, CV_64FC1);
    M1.at<double>(0,0) = pLine12[0].x-pLine11[n1/2].x;
    M1.at<double>(1,0) = pLine12[0].y-pLine11[n1/2].y;
    M1.at<double>(2,0) = pLine12[0].z-pLine11[n1/2].z;
    M1.at<double>(0,1) = pLine12[n2-1].x-pLine11[n1/2].x;
    M1.at<double>(1,1) = pLine12[n2-1].y-pLine11[n1/2].y;
    M1.at<double>(2,1) = pLine12[n2-1].z-pLine11[n1/2].z;
    M1.col(0) = M1.col(0) / norm(M1.col(0));
    M1.col(1) = M1.col(1) / norm(M1.col(1));
    M1.at<double>(0,2) = M1.at<double>(1,0)*M1.at<double>(2,1)-M1.at<double>(2,0)*M1.at<double>(1,1);
    M1.at<double>(1,2) = M1.at<double>(2,0)*M1.at<double>(0,1)-M1.at<double>(0,0)*M1.at<double>(2,1);
    M1.at<double>(2,2) = M1.at<double>(0,0)*M1.at<double>(1,1)-M1.at<double>(1,0)*M1.at<double>(0,1);
    M1.col(2) = M1.col(2) / norm(M1.col(2));
    
    M2.at<double>(0,0) = pLine22[0].x-pLine21[n1/2].x;
    M2.at<double>(1,0) = pLine22[0].y-pLine21[n1/2].y;
    M2.at<double>(2,0) = pLine22[0].z-pLine21[n1/2].z;
    M2.at<double>(0,1) = pLine22[n2-1].x-pLine21[n1/2].x;
    M2.at<double>(1,1) = pLine22[n2-1].y-pLine21[n1/2].y;
    M2.at<double>(2,1) = pLine22[n2-1].z-pLine21[n1/2].z;
    M2.col(0) = M2.col(0) / norm(M2.col(0));
    M2.col(1) = M2.col(1) / norm(M2.col(1));
    M2.at<double>(0,2) = M2.at<double>(1,0)*M2.at<double>(2,1)-M2.at<double>(2,0)*M2.at<double>(1,1);
    M2.at<double>(1,2) = M2.at<double>(2,0)*M2.at<double>(0,1)-M2.at<double>(0,0)*M2.at<double>(2,1);
    M2.at<double>(2,2) = M2.at<double>(0,0)*M2.at<double>(1,1)-M2.at<double>(1,0)*M2.at<double>(0,1);
    M2.col(2) = M2.col(2) / norm(M2.col(2));
    cout << "Normal: " << fixed << setprecision(6) << M1.col(2) << endl;
    //  cout << M1 << endl <<M2 << endl;
    R = M1*M2.inv();
    
    
    // 4.2 Adjust R to be unitary
    /*
     R.at<double>(0,0) = RCol.at<double>(0,0);
     R.at<double>(0,1) = RCol.at<double>(1,0);
     R.at<double>(0,2) = RCol.at<double>(2,0);
     R.at<double>(1,0) = RCol.at<double>(3,0);
     R.at<double>(1,1) = RCol.at<double>(4,0);
     R.at<double>(1,2) = RCol.at<double>(5,0);
     R.at<double>(2,0) = RCol.at<double>(6,0);
     R.at<double>(2,1) = RCol.at<double>(7,0);
     R.at<double>(2,2) = RCol.at<double>(8,0);
     
     cout << "Raw R:" << endl << R << endl;
     SVD RSVD(R, SVD::FULL_UV);
     cout << "SVD: " << RSVD.w << endl;
     R = RSVD.u*RSVD.vt;
     */
    //    cout << "R:" << endl << R << endl;
    
    //5.1 Calculate T
    
    Mat tempT(3, 1, CV_64FC1);
    Mat tempV1(3, 1, CV_64FC1);
    Mat tempV2(3, 1, CV_64FC1);
    Mat Rt = R.t();
    tempT.at<double>(0,0) = 0;
    tempT.at<double>(1,0) = 0;
    tempT.at<double>(2,0) = 0;
    for(i = 0; i<n1; i++)
    {
        tempV1.at<double>(0,0) = pLine11[i].x;
        tempV1.at<double>(1,0) = pLine11[i].y;
        tempV1.at<double>(2,0) = pLine11[i].z;
        tempV2.at<double>(0,0) = pLine21[i].x;
        tempV2.at<double>(1,0) = pLine21[i].y;
        tempV2.at<double>(2,0) = pLine21[i].z;
        tempT += tempV2 - Rt*tempV1;
    }
    for(i = 0; i<n2; i++)
    {
        tempV1.at<double>(0,0) = pLine12[i].x;
        tempV1.at<double>(1,0) = pLine12[i].y;
        tempV1.at<double>(2,0) = pLine12[i].z;
        tempV2.at<double>(0,0) = pLine22[i].x;
        tempV2.at<double>(1,0) = pLine22[i].y;
        tempV2.at<double>(2,0) = pLine22[i].z;
        tempT += tempV2 - Rt*tempV1;
    }
    tempT = tempT * (1.0 / (n1+n2));
    T = tempT;
    ouf.close();
    //  cout << "T: " << T << endl;
    return fx;
}
