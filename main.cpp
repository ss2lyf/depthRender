
#include "util.h"


void project(cv::Vec2d &p0, cv::Vec2d &p1, cv::Vec2d &p2, cv::Mat &depth, Camera &camera, cv::Vec4d &plane) {

    double minx = minDouble(p0[0], p1[0]);
    minx = minDouble(minx, p2[0]);

    double miny = minDouble(p0[1], p1[1]);
    miny = minDouble(miny, p2[1]);

    double maxx = maxDouble(p0[0], p1[0]);
    maxx = maxDouble(maxx, p2[0]);

    double maxy = maxDouble(p0[1], p1[1]);
    maxy = maxDouble(maxy, p2[1]);

    minx = maxDouble(0.0f, minx);
    miny = maxDouble(0.0f, miny);
    maxx = minDouble(maxx, double(camera.width - 1));
    maxy = minDouble(maxy, double(camera.height - 1));

    if (maxx <= minx || maxy <= miny) {
        return;
    }

    int xBegin = int(minx + 0.5f);
    int yBegin = int(miny + 0.5f);
    int xEnd = int(maxx + 0.5f);
    int yEnd = int(maxy + 0.5f);

    for (int x = xBegin; x <= xEnd; x++) {
        for (int y = yBegin; y <= yEnd; y++) {

            if (inTriangle(x, y, p0, p1, p2)) {

                double depthV = plane[3] / (plane[0] * (double(x) - camera.cx) / camera.fx +
                                            plane[1] * (double(y) - camera.cy) / camera.fy + plane[2]);

                double depthVPre = depth.at<double>(cv::Point(x, y));

                if (depthV > 0) {
                    if (depthVPre <= 0 || depthV < depthVPre) {
                        depth.at<double>(cv::Point(x, y)) = depthV;
                    }
                }
            }
        }
    }
}


void triangleToDepth(Camera &camera, cv::Mat &depth, std::vector<::pcl::Vertices> &polygons,
                     std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > &points) {

    const int polygonsNum = polygons.size();
    std::cout << "polygons number: "<< polygonsNum << std::endl;

    for (int i = 0; i < polygonsNum; i++) {
        if (polygons[i].vertices.size() != 3) {

            std::cout << "here " << i << " " << polygons[i].vertices.size() << std::endl;

        } else {

            cv::Vec3d  points3D[3];
            cv::Vec2d  points2D[3];

            for(int j=0;j<3;j++){
                points3D[j][0]=points[polygons[i].vertices[j]].x;
                points3D[j][1]=points[polygons[i].vertices[j]].y;
                points3D[j][2]=points[polygons[i].vertices[j]].z;
                points3D[j] = camera.R * points3D[j];
                points3D[j] += camera.t;

            }

            cv::Vec3d n=(points3D[2] - points3D[0]);
            n=n.cross((points3D[1] - points3D[0]));
            n=cv::normalize(n);

            //plane function : Ax+By+Cz=D
            cv::Vec4d plane(n[0], n[1], n[2], n[0] * points3D[0][0] + n[1] * points3D[0][1] + n[2] * points3D[0][2]);

            for(int j=0;j<3;j++) {
                points2D[j][0]=points3D[j][0] / points3D[j][2] * camera.fx + camera.cx;
                points2D[j][1]=points3D[j][1] / points3D[j][2] * camera.fy + camera.cy;
            }
            project(points2D[0], points2D[1], points2D[2], depth, camera, plane);
        }
    }
}


int main(int argc, char* argv[]) {


    if(argc<4){
        std::cout<<"usage ./depthRender meshFile cameraFile savePath"<<std::endl;
        exit(-1);
    }

    std::string meshFile=argv[1];
    std::string cameraFile=argv[2];
    std::string savePath=argv[3];

    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    Camera camera;
    readCameraParam(camera,cameraFile);

    if(camera.width<=0||camera.height<=0){
        std::cout<<"wrong image size, width: "<<camera.width<<", height: "<<camera.height<<std::endl;
        exit(-1);
    }

    cv::Mat depth = cv::Mat::zeros(cv::Size(camera.width, camera.height), CV_64FC1);

    if (pcl::io::loadPLYFile(meshFile, *mesh) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read mesh file  \n");
        std::cout<<meshFile<<std::endl;
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

    triangleToDepth(camera, depth, mesh->polygons, cloud->points);

    cv::Mat saveDepth;

    depth.convertTo(saveDepth, CV_32FC1);
    writeDepthDmb(savePath+"/depth.dmb", saveDepth,true);


    return 0;
}
