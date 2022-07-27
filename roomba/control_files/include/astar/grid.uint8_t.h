#ifndef GRID_UINT8_H
#define GRID_UINT8_H
#include "inttypes.h"

#include <unistd.h>
#include "coordinate.h"

typedef uint8_t _uint8_t;

typedef struct {
	size_t num_rows;
	size_t num_cols;
    size_t num_cells;
	uint8_t* values;
} grid_uint8_t;

coordinate_t index_to_coordinate(
	const grid_uint8_t* grid_T,
	size_t grid_index);

size_t coordinate_to_index(
	const grid_uint8_t* grid_T,
	coordinate_t coordinate);

void grid_uint8_init(
	grid_uint8_t* grid_T,
	size_t num_rows,
	size_t num_cols,
	uint8_t* values);

void grid_uint8_copy(
	grid_uint8_t* grid_dest,
	const grid_uint8_t* grid_src);

void grid_uint8_destroy(grid_uint8_t* grid_T);

// void grid_uint8_resize(grid_uint8_t* grid_T, size_t num_rows, size_t num_cols);

size_t grid_uint8_count_value(
	const grid_uint8_t* grid_T,
	uint8_t  value,
	int(*comp)(const uint8_t , const uint8_t ));

size_t grid_uint8_count_value_r(
	const grid_uint8_t* grid_T,
	uint8_t * value,
	int(*comp)(const uint8_t *, const uint8_t *));

size_t grid_uint8_find_index(
	const grid_uint8_t* grid_T,
	uint8_t  value,
	size_t start,
	int(*comp)(const uint8_t , const uint8_t ));

size_t grid_uint8_find_index_r(
	const grid_uint8_t* grid_T,
	uint8_t * value,
	size_t start,
	int(*comp)(const uint8_t *, const uint8_t *));

coordinate_t grid_uint8_find_coordinate(
	const grid_uint8_t* grid_T,
	uint8_t  value,
	coordinate_t start,
	int(*comp)(const uint8_t , const uint8_t ));

coordinate_t grid_uint8_find_coordinate_r(
	const grid_uint8_t* grid_T,
	uint8_t * value,
	coordinate_t start,
	int(*comp)(const uint8_t *, const uint8_t *));

uint8_t  grid_uint8_get_value_coordinate(
	const grid_uint8_t* grid_T,
	coordinate_t coordinate);

uint8_t * grid_uint8_get_value_coordinate_r(
	const grid_uint8_t* grid_T,
	coordinate_t coordinate);

void grid_uint8_set_value_index(
	grid_uint8_t* grid_T,
	size_t index,
	uint8_t  value);

void grid_uint8_set_value_index_r(
	grid_uint8_t* grid_T,
	size_t index,
	uint8_t * value);

void grid_uint8_set_values_index(
	grid_uint8_t* grid_T,
	size_t begin_index,
	size_t end_index,
	uint8_t  value);

void grid_uint8_set_values_index_r(
	grid_uint8_t* grid_T,
	size_t begin_index,
	size_t end_index,
	uint8_t * value);

void grid_uint8_set_value_coordinate(
	grid_uint8_t* grid_T,
	coordinate_t coordinate,
	uint8_t  value);

void grid_uint8_set_value_coordinate_r(
	grid_uint8_t* grid_T,
	coordinate_t coordinate,
	uint8_t * value);

void grid_uint8_set_values_coordinate(
	grid_uint8_t* grid_T,
	coordinate_t begin_coordinate,
	coordinate_t end_coordinate,
	uint8_t  value);

void grid_uint8_set_values_coordinate_r(
	grid_uint8_t* grid_T,
	coordinate_t begin_coordinate,
	coordinate_t end_coordinate,
	uint8_t * value);

void grid_uint8_fill(
	grid_uint8_t* grid_T,
	uint8_t  value);

void grid_uint8_fill_r(
	grid_uint8_t* grid_T,
	uint8_t * value);

void grid_uint8_swap_value_index(
	grid_uint8_t* grid_T,
	size_t index_a,
	size_t index_b);

void grid_uint8_swap_value_coordinate(
	grid_uint8_t* grid_T,
	coordinate_t coordinate_a,
	coordinate_t coordinate_b);

int grid_uint8_coordinate_inbounds(
	const grid_uint8_t* grid_T,
	coordinate_t coordinate);

int grid_uint8_index_inbounds(
	const grid_uint8_t* grid_T,
	size_t index);

void grid_uint8_for_each(
	const grid_uint8_t* grid_T,
	void(*func)(const grid_uint8_t*, size_t, const uint8_t ));

void grid_uint8_for_each_r(
	const grid_uint8_t* grid_T,
	void(*func)(const grid_uint8_t*, size_t, const uint8_t *));

#endif