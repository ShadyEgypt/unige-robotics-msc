#ifndef MAP_UTILS_H
#define MAP_UTILS_H

#include <ncurses.h>
#include <stdbool.h>
#include "globals.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/wait.h>

// Global variable declarations (extern)
extern FILE *log_file;
extern sem_t *sem_grid;
extern sem_t *sem_g;
extern sem_t *sem_drone;
extern sem_t *sem_map;

extern bool resources_exist;
extern Grid *grid;
extern Globals *globals;
extern Drone *drone;
extern MapServerClient *map;

extern pid_t child1_pid;
extern pid_t child2_pid;

extern int fd;
extern WINDOW *map_window;
extern WINDOW *drone_window;

// Function prototypes for map window creation and destruction
WINDOW *setup_win(int height, int width, int starty, int startx);
void destroy_win(WINDOW *local_win);
// map functions
void setup_resources();
void child1_task();
void child2_task();
void handle_sigint_map(int sig);
void reset_targets(Grid *grid);
void reset_obstacles(Grid *grid);
bool is_adjacent_occupied(Grid *grid, int x, int y);
void reset_map(Grid *grid, WINDOW *map_window, WINDOW *drone_window);
void move_drone(WINDOW *map_window, WINDOW *drone_window, int old_x, int old_y, int new_x, int new_y);
void drone_hit_target(WINDOW *map_window, WINDOW *drone_window, int old_x, int old_y, int new_x, int new_y, int score);
void drone_hit_obstacle(WINDOW *map_window, WINDOW *drone_window, int old_x, int old_y, int new_x, int new_y, int score);

void set_targets_randomly(Grid *grid, FILE *log_file);
void set_obstacles_randomly(Grid *grid, FILE *log_file);

#endif // MAP_UTILS_H
