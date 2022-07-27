#ifndef ARRAY_PAIR_COORDINATE_DOUBLE_H
#define ARRAY_PAIR_COORDINATE_DOUBLE_H
#include "pair.coordinate_t.double.h"

typedef pair_coordinate_double_t _pair_coordinate_double_t;

#include <unistd.h>

typedef struct {
    size_t size;
    size_t capacity;
    pair_coordinate_double_t* values;
} array_pair_coordinate_double_t;

void array_pair_coordinate_double_init(array_pair_coordinate_double_t* array_T, size_t capacity);

void array_pair_coordinate_double_destroy(array_pair_coordinate_double_t* array_T);

void array_pair_coordinate_double_clear(array_pair_coordinate_double_t* array_T);

size_t array_pair_coordinate_double_capacity(const array_pair_coordinate_double_t* array_T);

size_t array_pair_coordinate_double_size(const array_pair_coordinate_double_t* array_T);

int array_pair_coordinate_double_empty(const array_pair_coordinate_double_t* array_T);

int array_pair_coordinate_double_full(const array_pair_coordinate_double_t* array_T);

int array_pair_coordinate_double_inbounds(const array_pair_coordinate_double_t* array_T, size_t index);

pair_coordinate_double_t* array_pair_coordinate_double_begin(const array_pair_coordinate_double_t* array_T);

pair_coordinate_double_t* array_pair_coordinate_double_end(const array_pair_coordinate_double_t* array_T);

pair_coordinate_double_t array_pair_coordinate_double_get(const array_pair_coordinate_double_t* array_T, size_t index);

pair_coordinate_double_t array_pair_coordinate_double_front(const array_pair_coordinate_double_t* array_T);

pair_coordinate_double_t array_pair_coordinate_double_back(const array_pair_coordinate_double_t* array_T);

void array_pair_coordinate_double_set(array_pair_coordinate_double_t* array_T, size_t, pair_coordinate_double_t  value);

void array_pair_coordinate_double_set_r(array_pair_coordinate_double_t* array_T, size_t, pair_coordinate_double_t * value);

void array_pair_coordinate_double_push_back(array_pair_coordinate_double_t* array_T, pair_coordinate_double_t  value);

void array_pair_coordinate_double_push_back_r(array_pair_coordinate_double_t* array_T, pair_coordinate_double_t * value);

void array_pair_coordinate_double_pop_back(array_pair_coordinate_double_t* array_T);

void array_pair_coordinate_double_for_each(array_pair_coordinate_double_t * array, void(*func)(const pair_coordinate_double_t ));

void array_pair_coordinate_double_for_each_r(array_pair_coordinate_double_t * array, void(*func)(const pair_coordinate_double_t *));

void array_pair_coordinate_double_grow(array_pair_coordinate_double_t* array_T);

#endif