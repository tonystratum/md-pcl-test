#include <array>
#include <iostream>
#include <fstream>
#include <vector>

#define FLOAT_SIZE 4

struct Point {
    float x;
    float y;
    float z;
};

int main() {
    std::ifstream pcl_stream;
    pcl_stream.open("/home/stratum/CLionProjects/md_task_pcl/car.bin",
                    std::ifstream::in | std::ifstream::binary | std::ifstream::ate);

    std::vector<Point> points;
    if (pcl_stream.is_open()) {
        const size_t flength = pcl_stream.tellg();
        pcl_stream.seekg(pcl_stream.beg);

        const size_t n_floats = flength / sizeof(char*);
        const size_t n_points = n_floats / 3; // 3 coords in Point
        std::cout << flength << ' '
                  << n_floats << ' '
                  << n_points << '\n';

        points.reserve(n_points);

        char* buf = new char [FLOAT_SIZE];

        for (size_t i = 0; i < n_points; ++i) {
            std::array<float, 3> point = {0., 0., 0.};
            for (auto& item : point) {
                pcl_stream.read(buf, FLOAT_SIZE);
                item = *(reinterpret_cast<float*>(buf));
            }

            points.push_back({
                                     point[0],
                                     point[1],
                                     point[2],
                             });
        }
        delete[] buf;
    }
    pcl_stream.close();

    std::cout << points.empty() << '\n';
    for (const Point& p : points) {
        std::cout << p.x << ' '
                  << p.y << ' '
                  << p.z << '\n';
    }

    return 0;
}
