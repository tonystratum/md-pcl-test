#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "pcl/common/centroid.h"
#include "pcl/common/transforms.h"
#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"

#define FLOAT_SIZE 4


pcl::PointCloud<pcl::PointXYZ> load_from_bin(const char *path) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::ifstream pcl_file;
    pcl_file.open(path,
                    std::ifstream::in | std::ifstream::binary | std::ifstream::ate);

    if (pcl_file.is_open()) {
        const size_t flength = pcl_file.tellg();
        pcl_file.seekg(std::ifstream::beg);

        const size_t n_floats = flength / FLOAT_SIZE;
        const size_t n_points = n_floats / 3; // 3 coords in Point
        cloud.reserve(n_points);

        char *buf = new char[FLOAT_SIZE];
        for (size_t i = 0; i < n_points; ++i) {
            std::array<float, 3> point = {0., 0., 0.};
            for (auto &item: point) {
                pcl_file.read(buf, FLOAT_SIZE);
                item = *(reinterpret_cast<float *>(buf));
            }

            cloud.emplace_back(
                    point[0],
                    point[1],
                    point[2]
            );
        }
        delete[] buf;
    }
    pcl_file.close();
    return cloud;
}

int main() {
    const auto cloud = load_from_bin(
            "/home/stratum/CLionProjects/md_task_pcl/car.bin");

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud, centroid);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -centroid[0], -centroid[1], -centroid[2];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_ptr = cloud;

    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, transform);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(cloud_ptr);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spin();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
