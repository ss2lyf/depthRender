//
// Created by simon on 9/1/21.
//

#include "test.h"
#include "util.h"
#include <iostream>


void projectCloud(Camera &camera, cv::Mat &depth,
                  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> &points) {
    for (int i = 0; i < points.size(); i++) {

        cv::Vec3d P(points[i].x, points[i].y, points[i].z);

        P = camera.R * P;
        P += camera.t;
        cv::Vec2d p(P[0] / P[2] * camera.fx + camera.cx, P[1] / P[2] * camera.fy + camera.cy);

        if (p[0] >= 0 && p[1] >= 0 && p[0] < camera.width && p[1] < camera.height) {
            float depthVPre = depth.at<float>(cv::Point(p[0], p[1]));

            if (P[2] > 0) {
                if (depthVPre <= 0 || P[2] < depthVPre) {
                    depth.at<float>(cv::Point(p[0], p[1])) = P[2];
                }
            }
        }

    }
}





void genPly(Camera &camera, cv::Mat &depth, std::string savePath) {

    std::vector<cv::Vec3f> points;
    points.clear();
    cv::Matx33d Rinv;
    cv::transpose(camera.R, Rinv);
    cv::Vec3d C = -Rinv * camera.t;

    for (int i = 0; i < camera.width; i++) {
        for (int j = 0; j < camera.height; j++) {

            double depthTemp = (depth.at<float>(cv::Point(i, j)));
            if (depthTemp > 0) {
                cv::Vec3f pointTemp((double(i) - camera.cx) / camera.fx * depthTemp,
                                    (double(j) - camera.cy) / camera.fy * depthTemp,
                                    depthTemp
                );
                pointTemp = Rinv * pointTemp + C;

                points.push_back(pointTemp);
            }


        }
    }
    StoreColorPlyFileBinaryPointCloud(savePath, points);
}
