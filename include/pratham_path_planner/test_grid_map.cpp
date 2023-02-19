// #include "grid_conversions.hpp"
#include "test.hpp"
#include <iostream>
#include <vector>


int main() {

    grid_conversions::indices_t *points = ros_api::init_points();
    std::cout << "got points " << points[0].x << " " << points[50].y << std::endl;
    std::vector<grid_conversions::indices_t> indices_vector(points, points+200);
    std::cout << "got points " << indices_vector[0].x << " " << indices_vector[50].y << std::endl;
    std::vector<grid_conversions::point_t> converted_points(200, grid_conversions::point_t(0, 0));
    bool succesful_conversion = grid_conversions::grid_indices_to_map_points_threaded(indices_vector, converted_points);

    if (succesful_conversion) {
        for(int i=0; i < converted_points.size(); i++) {
            std::cout << "got the converted path " << converted_points[i].x << ", " << converted_points[i].y << std::endl;
        }
    }
    auto [successful_conversion, XY] = grid_conversions::grid_to_map(320, 320);
    std::cout << XY.x << ", " << XY.y << std::endl;
    auto [conversion_done, XY1] = grid_conversions::map_to_grid(4.5, 0);
    std::cout << XY1.x << ", " << XY1.y << std::endl;
    return 0;
}
