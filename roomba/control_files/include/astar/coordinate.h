
#ifndef COORDINATE_H
#define COORDINATE_H

#include <unistd.h>
#include <stdio.h>
#include <math.h>

typedef struct {
    size_t row;
    size_t col;
} coordinate_t;

static inline coordinate_t make_coordinate(
	size_t row,
	size_t col)
{
    return (coordinate_t){row, col};
}

static inline int coordinate_equal(const coordinate_t a, const coordinate_t b) {
    return (a.row == b.row) && (a.col == b.col);
}

static inline size_t coordinate_hash(coordinate_t cell) {
    return (cell.col*cell.col + cell.row*cell.row);
}

static inline void coordinate_print(const coordinate_t* coordinate) {
    char delim = '\n';
    printf("{%lu, %lu}%c", coordinate->row, coordinate->col, delim);
}

static inline size_t abs_diff_size_t(size_t a, size_t b) {
    return a > b ? a - b : b - a;
}

static inline double coordinate_euclidean_distance(const coordinate_t* a, const coordinate_t* b) {
    size_t dx = abs_diff_size_t(a->col, b->col);
    size_t dy = abs_diff_size_t(a->row, b->row);
	return sqrt((double)(dx*dx + dy*dy));
}

#endif