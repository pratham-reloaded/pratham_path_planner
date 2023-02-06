#include "grid_conversions.hpp"
#include <iostream>


int main() {

    auto [successful_conversion, XY] = grid_conversions::grid_to_map(320, 320);
    std::cout << XY.x << ", " << XY.y << std::endl;
    auto [conversion_done, XY1] = grid_conversions::map_to_grid(4.5, 0);
    std::cout << XY1.x << ", " << XY1.y << std::endl;
    return 0;
}
