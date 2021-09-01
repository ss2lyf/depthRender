//
// Created by simon on 9/1/21.
//

#ifndef PCLTEST_UTIL_H
#define PCLTEST_UTIL_H


#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
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


double minDouble(double a, double b) {
    return a > b ? b : a;
}

double maxDouble(double a, double b) {
    return a > b ? a : b;
}

//this function is borrowed from:
//https://www.cnblogs.com/graphics/archive/2010/08/05/1793393.html
bool inTriangle(const int x, const int y, cv::Vec2d &p0, cv::Vec2d &p1, cv::Vec2d &p2) {

    //v0 = C – A, v1 = B – A, v2 = P – A, v2 = u * v0 + v * v1
    //u = ((v1•v1)(v2•v0)-(v1•v0)(v2•v1)) / ((v0•v0)(v1•v1) - (v0•v1)(v1•v0))
    //v = ((v0•v0)(v2•v1)-(v0•v1)(v2•v0)) / ((v0•v0)(v1•v1) - (v0•v1)(v1•v0))

    cv::Vec2d v0(p2[0] - p0[0], p2[1] - p0[1]);
    cv::Vec2d v1(p1[0] - p0[0], p1[1] - p0[1]);
    cv::Vec2d v2(double(x) - p0[0], double(y) - p0[1]);

    double dot00 = v0.dot(v0);
    double dot01 = v0.dot(v1);
    double dot02 = v0.dot(v2);
    double dot11 = v1.dot(v1);
    double dot12 = v1.dot(v2);

    double temp = 1 / (dot00 * dot11 - dot01 * dot01);

    double u = (dot11 * dot02 - dot01 * dot12) * temp;
    if (u < 0 || u > 1) // if u out of range, return directly
    {
        return false;
    }

    double v = (dot00 * dot12 - dot01 * dot02) * temp;
    if (v < 0 || v > 1) // if v out of range, return directly
    {
        return false;
    }

    return u + v <= 1;

}


void depth2color(cv::Mat &color, const cv::Mat &depth, const double max, const double min) {
    cv::Mat grayImage;
    double alpha = 255.0 / (max - min);
    depth.convertTo(grayImage, CV_8UC1, alpha, -alpha * min);// expand your range to 0..255. Similar to histEq();
    cv::applyColorMap(grayImage, color,
                      cv::COLORMAP_JET);// this is great. It converts your grayscale image into a tone-mapped one, much more pleasing for the eye function is found in contrib module, so include contrib.hpp  and link accordingly
}


void readCameraParam(Camera &camera, std::string cameraFile) {

    //read cam
    std::ifstream camFile(cameraFile);
    if (!camFile) {
        std::cout << "could not open cam file " << cameraFile << std::endl;
        exit(1);
    }

    std::string temp = "";
    while (temp != "extrinsic") {
        std::getline(camFile, temp);
    }

    for (int row = 0; row < 3; row++) {

        std::getline(camFile, temp);
        std::istringstream istr(temp);
        std::string sTmp;

        for (int col = 0; col < 3; col++) {
            istr >> sTmp;
            camera.R(row, col) = std::atof(sTmp.c_str());
        }

        istr >> sTmp;
        camera.t(row) = std::atof(sTmp.c_str());

    }

    while (temp != "intrinsic") {
        std::getline(camFile, temp);
    }

    cv::Matx33d K;

    for (int row = 0; row < 3; row++) {

        std::getline(camFile, temp);
        std::istringstream istr(temp);
        std::string sTmp;

        for (int col = 0; col < 3; col++) {

            istr >> sTmp;
            K(row, col) = std::atof(sTmp.c_str());
        }
    }

    camera.fx = K(0, 0);
    camera.fy = K(1, 1);
    camera.cx = K(0, 2);
    camera.cy = K(1, 2);

    while (temp != "width") {
        std::getline(camFile, temp);
    }

    std::getline(camFile, temp);
    camera.width = std::atof(temp.c_str());


    while (temp != "height") {
        std::getline(camFile, temp);
    }

    std::getline(camFile, temp);
    camera.height = std::atof(temp.c_str());

    camFile.close();

}

//codes below are borrowed from ACMH
//https://github.com/GhiXu/ACMH
int writeDepthDmb(const std::string file_path, cv::Mat &depth, bool saveShowImg) {
    FILE *outimage;
    outimage = fopen(file_path.c_str(), "wb");
    if (!outimage) {
        std::cout << "Error opening file " << file_path << std::endl;
    }

    int32_t type = 1;
    int32_t h = depth.rows;
    int32_t w = depth.cols;
    int32_t nb = 1;

    fwrite(&type, sizeof(int32_t), 1, outimage);
    fwrite(&h, sizeof(int32_t), 1, outimage);
    fwrite(&w, sizeof(int32_t), 1, outimage);
    fwrite(&nb, sizeof(int32_t), 1, outimage);

    float *data = (float *) depth.data;

    int32_t datasize = w * h * nb;
    fwrite(data, sizeof(float), datasize, outimage);

    fclose(outimage);

    if (saveShowImg) {
        cv::Mat depthColor;
        const int size = depth.cols * depth.rows;
        float *p = (float *) depth.data;

        double min = 1000;
        double max = 0;

        for (int i = 0; i < size; i++) {
            if (*p > 0) {
                if (*p > max) {
                    max = *p;
                }
                if (*p < min) {
                    min = *p;
                }
            }
            p++;
        }

        depth2color(depthColor, depth, max, min);
        cv::imwrite(file_path + ".jpg", depthColor);
    }


    return 0;
}




void StoreColorPlyFileBinaryPointCloud(const std::string &plyFilePath, std::vector<cv::Vec3f> &pc) {
    std::cout << "store 3D points to ply file" << std::endl;

    FILE *outputPly;
    outputPly = fopen(plyFilePath.c_str(), "wb");

    /*write header*/
    fprintf(outputPly, "ply\n");
    fprintf(outputPly, "format binary_little_endian 1.0\n");
    fprintf(outputPly, "element vertex %d\n", int(pc.size()));
    fprintf(outputPly, "property float x\n");
    fprintf(outputPly, "property float y\n");
    fprintf(outputPly, "property float z\n");
    fprintf(outputPly, "property uchar red\n");
    fprintf(outputPly, "property uchar green\n");
    fprintf(outputPly, "property uchar blue\n");
    fprintf(outputPly, "end_header\n");

    for (size_t i = 0; i < pc.size(); i++) {

        const char b_color = 128;
        const char g_color = 128;
        const char r_color = 128;

        if (!(pc[i][0] < FLT_MAX && pc[i][0] > -FLT_MAX) || !(pc[i][1] < FLT_MAX && pc[i][1] > -FLT_MAX) ||
            !(pc[i][2] < FLT_MAX && pc[i][2] >= -FLT_MAX)) {
            pc[i][0] = 0.0f;
            pc[i][1] = 0.0f;
            pc[i][2] = 0.0f;
        }

        fwrite(&pc[i][0], sizeof(pc[i][0]), 1, outputPly);
        fwrite(&pc[i][1], sizeof(pc[i][1]), 1, outputPly);
        fwrite(&pc[i][2], sizeof(pc[i][2]), 1, outputPly);
        fwrite(&r_color, sizeof(char), 1, outputPly);
        fwrite(&g_color, sizeof(char), 1, outputPly);
        fwrite(&b_color, sizeof(char), 1, outputPly);

    }
    fclose(outputPly);
}

int readDepthDmb(const std::string file_path, cv::Mat &depth) {
    FILE *inimage;
    inimage = fopen(file_path.c_str(), "rb");
    if (!inimage) {
        std::cout << "Error opening file " << file_path << std::endl;
        return -1;
    }

    int32_t type, h, w, nb;

    type = -1;

    int returnValue = 0;
    returnValue = fread(&type, sizeof(int32_t), 1, inimage);
    returnValue = fread(&h, sizeof(int32_t), 1, inimage);
    returnValue = fread(&w, sizeof(int32_t), 1, inimage);
    returnValue = fread(&nb, sizeof(int32_t), 1, inimage);

    if (type != 1) {
        fclose(inimage);
        return -1;
    }

    int32_t dataSize = h * w * nb;

    float *data;
    data = (float *) malloc(sizeof(float) * dataSize);
    returnValue = fread(data, sizeof(float), dataSize, inimage);

    depth = cv::Mat(h, w, CV_32F, data);

    fclose(inimage);
    return 0;
}


#endif //PCLTEST_UTIL_H
