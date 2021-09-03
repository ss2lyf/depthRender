//
// Created by simon on 9/1/21.
//

#ifndef PCLTEST_UTIL_H
#define PCLTEST_UTIL_H


#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

struct Camera {
    cv::Matx33d R;
    cv::Vec3d t;
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;
    int width, height;
};


double minDouble(double a, double b);

double maxDouble(double a, double b) ;

//this function is borrowed from:
//https://www.cnblogs.com/graphics/archive/2010/08/05/1793393.html
bool inTriangle(const int x, const int y, cv::Vec2d &p0, cv::Vec2d &p1, cv::Vec2d &p2) ;


void depth2color(cv::Mat &color, const cv::Mat &depth, const double max, const double min) ;


void readCameraParam(Camera &camera, std::string cameraFile) ;

void genPly(Camera &camera, cv::Mat &depth, std::string savePath);



//codes below are borrowed from ACMH
//https://github.com/GhiXu/ACMH
int writeDepthDmb(const std::string file_path, cv::Mat &depth, bool saveShowImg) ;


int readDepthDmb(const std::string file_path, cv::Mat &depth) ;


void StoreColorPlyFileBinaryPointCloud(const std::string &plyFilePath, std::vector<cv::Vec3f> &pc) ;



#endif //PCLTEST_UTIL_H
