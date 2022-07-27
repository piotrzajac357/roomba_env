#ifndef PAIR_COORDINATE_DOUBLE_H
#define PAIR_COORDINATE_DOUBLE_H
#include "coordinate.h"

typedef coordinate_t _coordinate_t;
typedef double _double_t;

typedef struct {
    unsigned short valid : 1;
    coordinate_t key;
    double value;
} pair_coordinate_double_t;

#endif