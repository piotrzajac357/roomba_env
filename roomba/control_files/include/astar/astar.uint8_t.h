#ifndef ASTAR_H
#define ASTAR_H

#include "grid.uint8_t.h"
#include "array.coordinate_t.h=path.h"

path_t astar_uint8_get_path(
    grid_uint8_t* grid,
    coordinate_t begin_coordinate,
    coordinate_t end_coordinate,
    int(*is_traversable)(const uint8_t *));

#endif
