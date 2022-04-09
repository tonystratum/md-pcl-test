#include <array>
#include <iostream>
#include <fstream>
#include <vector>

#include <pcl/point_cloud.h>
#include "pcl/impl/point_types.hpp"

#define FLOAT_SIZE 4


pcl::PointCloud<pcl::PointXYZ> load_from_bin(const char *path) {
    pcl::PointCloud<pcl::PointXYZ> points;
    std::ifstream pcl_stream;
    pcl_stream.open(path,
                    std::ifstream::in | std::ifstream::binary | std::ifstream::ate);

    if (pcl_stream.is_open()) {
        const size_t flength = pcl_stream.tellg();
        pcl_stream.seekg(std::ifstream::beg);

        const size_t n_floats = flength / FLOAT_SIZE;
        const size_t n_points = n_floats / 3; // 3 coords in Point
        points.reserve(n_points);

        char *buf = new char[FLOAT_SIZE];
        for (size_t i = 0; i < n_points; ++i) {
            std::array<float, 3> point = {0., 0., 0.};
            for (auto &item: point) {
                pcl_stream.read(buf, FLOAT_SIZE);
                item = *(reinterpret_cast<float *>(buf));
            }

            points.emplace_back(
                    point[0],
                    point[1],
                    point[2]
            );
        }
        delete[] buf;
    }
    pcl_stream.close();
    return points;
}

int main() {
    const pcl::PointCloud<pcl::PointXYZ> points = load_from_bin(
            "/home/stratum/CLionProjects/md_task_pcl/car.bin");
    for (const auto &p: points) {
        std::cout << p.x << ' '
                  << p.y << ' '
                  << p.z << '\n';
    }
    return 0;
}
