#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>
#include "globals.h"
#include <semaphore.h>

// Initialize the log file in the logs folder
FILE *initialize_log_file(const char *log_filename);

// Function to log messages to a log file with a timestamp
void log_message(FILE *log_file, const char *message);

// Function prototypes
int create_shared_memory(const char *shm_name, size_t size); // Creates shared memory and returns its ID
int attach_shared_memory(const char *shm_name, size_t size); // Attaches to the shared memory and returns its ID
void detach_shared_memory(void *addr, size_t size);          // Detaches shared memory
void destroy_shared_memory(const char *shm_name);
void *map_shared_memory(int shm_fd, size_t size); // Destroys the shared memory
sem_t *create_semaphore(const char *sem_name);
sem_t *open_semaphore(const char *sem_name);
void acquire_semaphore(sem_t *sem);
void release_semaphore(sem_t *sem);
void destroy_semaphore(const char *sem_name, sem_t *sem);
bool is_point_occupied(Grid *grid, int x, int y);                                             // Check if a point is occupied
void set_grid_point(Grid *grid, int x, int y, int value, GridPointType type, FILE *log_file); // Set a point in the grid
void set_map_point(Grid *grid, Drone *drone, MapServerClient *map, FILE *log_file);
#endif // UTILS_
