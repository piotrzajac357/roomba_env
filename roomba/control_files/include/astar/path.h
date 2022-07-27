#ifndef PATH_H
#define PATH_H
#include "coordinate.h"

typedef coordinate_t _coordinate_t;

#include <unistd.h>

typedef struct {
    size_t size;
    size_t capacity;
    coordinate_t* values;
} path_t;

void path_init(path_t* path, size_t capacity);

void path_destroy(path_t* path);

void path_clear(path_t* path);

size_t path_capacity(const path_t* path);

size_t path_size(const path_t* path);

int path_empty(const path_t* path);

int path_full(const path_t* path);

int path_inbounds(const path_t* path, size_t index);

coordinate_t* path_begin(const path_t* path);

coordinate_t* path_end(const path_t* path);

coordinate_t path_get(const path_t* path, size_t index);

coordinate_t path_front(const path_t* path);

coordinate_t path_back(const path_t* path);

void path_set(path_t* path, size_t, coordinate_t  value);

void path_set_r(path_t* path, size_t, coordinate_t * value);

void path_push_back(path_t* path, coordinate_t  value);

void path_push_back_r(path_t* path, coordinate_t * value);

void path_pop_back(path_t* path);

void path_for_each(path_t * array, void(*func)(const coordinate_t ));

void path_for_each_r(path_t * array, void(*func)(const coordinate_t *));

void path_grow(path_t* path);

#endif