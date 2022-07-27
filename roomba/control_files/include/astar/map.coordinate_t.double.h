#ifndef MAP_COORDINATE_DOUBLE_PB_H
#define MAP_COORDINATE_DOUBLE_PB_H
#include "coordinate.h"

typedef coordinate_t _coordinate_t;
typedef double _double_t;

#include "pair.coordinate_t.double.h"
#include "array.pair_coordinate_double_t.h"

typedef struct {
    array_pair_coordinate_double_t pairs;
    size_t(*hash)(coordinate_t);
    int(*match)(coordinate_t, coordinate_t);
} map_coordinate_double_t;

void map_coordinate_double_init(map_coordinate_double_t* map_coordinate_V, 
                size_t num_pairs,
                size_t(*hash)(coordinate_t),
                int(*match)(coordinate_t, coordinate_t));

void map_coordinate_double_destroy(map_coordinate_double_t* map_coordinate_V);

double * map_coordinate_double_find(const map_coordinate_double_t* map_coordinate_V, coordinate_t key);

double * map_coordinate_double_insert(map_coordinate_double_t* map_coordinate_V, coordinate_t key, double  value);

double * map_coordinate_double_insert_r(map_coordinate_double_t* map_coordinate_V, coordinate_t key, double * value);

double * map_coordinate_double_find_insert(map_coordinate_double_t* map_coordinate_V, coordinate_t key, double  value);

double * map_coordinate_double_find_insert_r(map_coordinate_double_t* map_coordinate_V, coordinate_t key, double * value);

void map_coordinate_double_erase(map_coordinate_double_t* map_coordinate_V, coordinate_t key);

size_t map_coordinate_double_size(const map_coordinate_double_t* map_coordinate_V);

#endif