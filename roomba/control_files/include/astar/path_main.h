#ifndef PATH_MAIN
#define PATH_MAIN

void grid_uint8_print(const grid_uint8_t* grid, size_t index, const uint8_t value);

void grid_uint8_init_file(grid_uint8_t* grid, char * file_path);

void grid_uint8_fill_random(grid_uint8_t* grid);

int is_traversable(const uint8_t* val);

void print_begin_end(coordinate_t* begin, coordinate_t* end);

int run_test_1();

int run_test_2();

int run_test_3();

int we_co_napisz();

#endif /* PATH_MAIN */