#include <stdio.h>
#include <tuple>

#define GRID_SIZE 320
#define GRID_LEN 6 // meters


// the grid forms an angle of 180 degrees with the map link
#define GRID_ORIGIN_X 240
#define GRID_ORIGIN_Y 160

namespace grid_conversions {
    struct point_t {
        float x, y;
        point_t(float a, float b) { this->x = a; this->y = b; }
    };

    std::tuple<bool, point_t> map_to_grid(float x, float y) {
        int delta_x = x/(1.875*0.01);
        int delta_y = y/(1.875*0.01);

        return {true, point_t(GRID_ORIGIN_X - delta_x,
                              GRID_ORIGIN_Y - delta_y)};
    }

    std::tuple<bool, point_t> grid_to_map(unsigned int x, unsigned int y) {
        int delta_x = GRID_ORIGIN_X - x;
        int delta_y = GRID_ORIGIN_Y - y;

        float map_frame_x = (delta_x * 1.875)*0.01;
        float map_frame_y = (delta_y * 1.875)*0.01;
        return {true, point_t(map_frame_x, map_frame_y)};
    }
}