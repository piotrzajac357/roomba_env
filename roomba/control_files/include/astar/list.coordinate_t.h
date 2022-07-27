#ifndef LIST_COORDINATE_H
#define LIST_COORDINATE_H
#include "coordinate.h"

typedef coordinate_t _coordinate_t;

#include <unistd.h>

typedef struct node_t node_t;

typedef struct {
	node_t * head;
	node_t * tail;
} list_coordinate_t;

void list_coordinate_init(list_coordinate_t * list_T);

void list_coordinate_destroy(list_coordinate_t * list_T);

void list_coordinate_clear(list_coordinate_t * list_T);

size_t list_coordinate_size(const list_coordinate_t * list_T);

int list_coordinate_empty(const list_coordinate_t * list_T);

coordinate_t  list_coordinate_get(const list_coordinate_t * list_T, size_t index);

coordinate_t * list_coordinate_get_r(const list_coordinate_t * list_T, size_t index);

coordinate_t  list_coordinate_front(const list_coordinate_t * list_T);

coordinate_t * list_coordinate_front_r(const list_coordinate_t * list_T);

coordinate_t  list_coordinate_back(const list_coordinate_t * list_T);

coordinate_t * list_coordinate_back_r(const list_coordinate_t * list_T);

coordinate_t * list_coordinate_insert_after(list_coordinate_t * list_T, size_t index, const coordinate_t  value);

coordinate_t * list_coordinate_insert_after_r(list_coordinate_t * list_T, size_t index, const coordinate_t * value);

coordinate_t * list_coordinate_insert_before(list_coordinate_t * list_T, size_t index, const coordinate_t  value);

coordinate_t * list_coordinate_insert_before_r(list_coordinate_t * list_T, size_t index, const coordinate_t * value);

coordinate_t * list_coordinate_push_back(list_coordinate_t * list_T, const coordinate_t  value);

coordinate_t * list_coordinate_push_back_r(list_coordinate_t * list_T, const coordinate_t * value);

coordinate_t * list_coordinate_push_front(list_coordinate_t * list_T, const coordinate_t  value);

coordinate_t * list_coordinate_push_front_r(list_coordinate_t * list_T, const coordinate_t * value);

void list_coordinate_swap(list_coordinate_t * list_T, size_t index_a, size_t index_b);

void list_coordinate_for_each(list_coordinate_t * list_T, void(*func)(const coordinate_t ));

void list_coordinate_for_each_r(list_coordinate_t * list_T, void(*func)(const coordinate_t *));

void list_coordinate_pop_back(list_coordinate_t * list_T);

void list_coordinate_pop_front(list_coordinate_t * list_T);

void list_coordinate_remove(list_coordinate_t * list_T, size_t index);

void list_coordinate_bubble_sort(list_coordinate_t * list_T, int(*comp)(const coordinate_t , const coordinate_t ));

void list_coordinate_bubble_sort_r(list_coordinate_t * list_T, int(*comp)(const coordinate_t *, const coordinate_t *));

#endif