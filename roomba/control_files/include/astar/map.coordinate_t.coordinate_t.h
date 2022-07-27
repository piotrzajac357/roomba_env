#ifndef MAP_COORDINATE_COORDINATE_PB_H
#define MAP_COORDINATE_COORDINATE_PB_H
#include "coordinate.h"

typedef coordinate_t _coordinate_t;
typedef coordinate_t _coordinate_t;

#include "pair.coordinate_t.coordinate_t.h"
#include "array.pair_coordinate_coordinate_t.h"

typedef struct {
    array_pair_coordinate_coordinate_t pairs;
    size_t(*hash)(coordinate_t);
    int(*match)(coordinate_t, coordinate_t);
} map_coordinate_coordinate_t;

void map_coordinate_coordinate_init(map_coordinate_coordinate_t* map_coordinate_V, 
                size_t num_pairs,
                size_t(*hash)(coordinate_t),
                int(*match)(coordinate_t, coordinate_t));

void map_coordinate_coordinate_destroy(map_coordinate_coordinate_t* map_coordinate_V);

coordinate_t * map_coordinate_coordinate_find(const map_coordinate_coordinate_t* map_coordinate_V, coordinate_t key);

coordinate_t * map_coordinate_coordinate_insert(map_coordinate_coordinate_t* map_coordinate_V, coordinate_t key, coordinate_t  value);

coordinate_t * map_coordinate_coordinate_insert_r(map_coordinate_coordinate_t* map_coordinate_V, coordinate_t key, coordinate_t * value);

coordinate_t * map_coordinate_coordinate_find_insert(map_coordinate_coordinate_t* map_coordinate_V, coordinate_t key, coordinate_t  value);

coordinate_t * map_coordinate_coordinate_find_insert_r(map_coordinate_coordinate_t* map_coordinate_V, coordinate_t key, coordinate_t * value);

void map_coordinate_coordinate_erase(map_coordinate_coordinate_t* map_coordinate_V, coordinate_t key);

size_t map_coordinate_coordinate_size(const map_coordinate_coordinate_t* map_coordinate_V);

#endif