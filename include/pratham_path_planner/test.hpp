#include <cstdio>
#include <cstdlib>
#include "grid_conversions.hpp"
#include <vector>

namespace ros_api {
    grid_conversions::indices_t *init_points() {
        grid_conversions::indices_t *points;

        points = (grid_conversions::indices_t*) malloc(200*sizeof(grid_conversions::indices_t));
        for (int i=0; i<200; i++) {
            points[i] = grid_conversions::indices_t(i, i);
        }
        return points;
    }
    void hello_world() {
        printf("hello world");

    }
}
