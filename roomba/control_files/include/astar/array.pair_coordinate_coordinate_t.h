#ifndef ARRAY_PAIR_COORDINATE_COORDINATE_H
#define ARRAY_PAIR_COORDINATE_COORDINATE_H
#include "pair.coordinate_t.coordinate_t.h"

typedef pair_coordinate_coordinate_t _pair_coordinate_coordinate_t;

#include <unistd.h>

typedef struct {
    size_t size;
    size_t capacity;
    pair_coordinate_coordinate_t* values;
} array_pair_coordinate_coordinate_t;

void array_pair_coordinate_coordinate_init(array_pair_coordinate_coordinate_t* array_T, size_t capacity);

void array_pair_coordinate_coordinate_destroy(array_pair_coordinate_coordinate_t* array_T);

void array_pair_coordinate_coordinate_clear(array_pair_coordinate_coordinate_t* array_T);

size_t array_pair_coordinate_coordinate_capacity(const array_pair_coordinate_coordinate_t* array_T);

size_t array_pair_coordinate_coordinate_size(const array_pair_coordinate_coordinate_t* array_T);

int array_pair_coordinate_coordinate_empty(const array_pair_coordinate_coordinate_t* array_T);

int array_pair_coordinate_coordinate_full(const array_pair_coordinate_coordinate_t* array_T);

int array_pair_coordinate_coordinate_inbounds(const array_pair_coordinate_coordinate_t* array_T, size_t index);

pair_coordinate_coordinate_t* array_pair_coordinate_coordinate_begin(const array_pair_coordinate_coordinate_t* array_T);

pair_coordinate_coordinate_t* array_pair_coordinate_coordinate_end(const array_pair_coordinate_coordinate_t* array_T);

pair_coordinate_coordinate_t array_pair_coordinate_coordinate_get(const array_pair_coordinate_coordinate_t* array_T, size_t index);

pair_coordinate_coordinate_t array_pair_coordinate_coordinate_front(const array_pair_coordinate_coordinate_t* array_T);

pair_coordinate_coordinate_t array_pair_coordinate_coordinate_back(const array_pair_coordinate_coordinate_t* array_T);

void array_pair_coordinate_coordinate_set(array_pair_coordinate_coordinate_t* array_T, size_t, pair_coordinate_coordinate_t  value);

void array_pair_coordinate_coordinate_set_r(array_pair_coordinate_coordinate_t* array_T, size_t, pair_coordinate_coordinate_t * value);

void array_pair_coordinate_coordinate_push_back(array_pair_coordinate_coordinate_t* array_T, pair_coordinate_coordinate_t  value);

void array_pair_coordinate_coordinate_push_back_r(array_pair_coordinate_coordinate_t* array_T, pair_coordinate_coordinate_t * value);

void array_pair_coordinate_coordinate_pop_back(array_pair_coordinate_coordinate_t* array_T);

void array_pair_coordinate_coordinate_for_each(array_pair_coordinate_coordinate_t * array, void(*func)(const pair_coordinate_coordinate_t ));

void array_pair_coordinate_coordinate_for_each_r(array_pair_coordinate_coordinate_t * array, void(*func)(const pair_coordinate_coordinate_t *));

void array_pair_coordinate_coordinate_grow(array_pair_coordinate_coordinate_t* array_T);

#endif