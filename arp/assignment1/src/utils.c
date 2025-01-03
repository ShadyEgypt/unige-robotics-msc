#include "utils.h"
#include "globals.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <fcntl.h>

FILE *initialize_log_file(const char *log_filename)
{
    // Ensure the logs folder exists
    const char *log_folder = "logs";
    if (mkdir(log_folder, 0777) == -1 && errno != EEXIST)
    {
        perror("Failed to create logs folder");
        exit(EXIT_FAILURE);
    }

    // Construct the full log file path
    char log_path[256];
    snprintf(log_path, sizeof(log_path), "%s/%s", log_folder, log_filename);

    // Open the log file for appending
    FILE *log_file = fopen(log_path, "a");
    if (!log_file)
    {
        perror("Failed to open log file");
        exit(EXIT_FAILURE);
    }

    return log_file;
}

void log_message(FILE *log_file, const char *message)
{
    if (!log_file)
    {
        fprintf(stderr, "Log file is not initialized.\n");
        return;
    }

    // Write a timestamped message to the log file
    time_t now = time(NULL);
    char *timestamp = ctime(&now);
    timestamp[strlen(timestamp) - 1] = '\0'; // Remove newline
    fprintf(log_file, "[%s] %s\n", timestamp, message);
    fflush(log_file); // Ensure message is written immediately
}

// Create a shared memory object with a given name and size and return its file descriptor
int create_shared_memory(const char *shm_name, size_t size)
{
    int shm_fd = shm_open(shm_name, O_RDWR | O_CREAT, 0666);
    if (shm_fd == -1)
    {
        perror("shm_open (create) failed");
        exit(EXIT_FAILURE);
    }
    // Set the size of the shared memory
    if (ftruncate(shm_fd, size) == -1)
    {
        perror("ftruncate failed");
        exit(EXIT_FAILURE);
    }
    return shm_fd;
}

// Map the shared memory into the address space
void *map_shared_memory(int shm_fd, size_t size)
{
    void *addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (addr == MAP_FAILED)
    {
        perror("mmap failed");
        exit(EXIT_FAILURE);
    }
    return addr;
}

// Attach to an existing shared memory object with a given name and size
int attach_shared_memory(const char *shm_name, size_t size)
{
    int shm_fd = shm_open(shm_name, O_RDWR, 0666);
    if (shm_fd == -1)
    {
        perror("shm_open (attach) failed");
        exit(EXIT_FAILURE);
    }
    return shm_fd;
}

// Detach the shared memory region
void detach_shared_memory(void *addr, size_t size)
{
    if (munmap(addr, size) == -1)
    {
        perror("munmap failed");
        exit(EXIT_FAILURE);
    }
}

// Destroy the shared memory object with a given name
void destroy_shared_memory(const char *shm_name)
{
    if (shm_unlink(shm_name) == -1)
    {
        perror("shm_unlink failed");
        exit(EXIT_FAILURE);
    }
}

// Create and initialize a POSIX named semaphore with a given name
sem_t *create_semaphore(const char *sem_name)
{
    sem_t *sem = sem_open(sem_name, O_CREAT, 0666, 1);
    if (sem == SEM_FAILED)
    {
        perror("sem_open (create) failed");
        exit(EXIT_FAILURE);
    }
    return sem;
}

// Open an already-created POSIX named semaphore
sem_t *open_semaphore(const char *sem_name)
{
    sem_t *sem = sem_open(sem_name, 0);
    if (sem == SEM_FAILED)
    {
        perror("sem_open (open) failed");
        exit(EXIT_FAILURE);
    }
    return sem;
}

// Acquire the semaphore (lock)
void acquire_semaphore(sem_t *sem)
{
    if (sem_wait(sem) < 0)
    {
        perror("sem_wait failed");
        exit(EXIT_FAILURE);
    }
}

// Release the semaphore (unlock)
void release_semaphore(sem_t *sem)
{
    if (sem_post(sem) < 0)
    {
        perror("sem_post failed");
        exit(EXIT_FAILURE);
    }
}

// Destroy the semaphore with a given name
void destroy_semaphore(const char *sem_name, sem_t *sem)
{
    if (sem_close(sem) < 0)
    {
        perror("sem_close failed");
        exit(EXIT_FAILURE);
    }
    if (sem_unlink(sem_name) < 0)
    {
        perror("sem_unlink failed");
        exit(EXIT_FAILURE);
    }
}

bool is_point_occupied(Grid *grid, int x, int y, int grid_h, int grid_w)
{
    // a function that returns a value greater than 0 if the point is not free
    if (x < 0 || x >= grid_w || y < 0 || y >= grid_h)
    {
        return true;
    }
    int value = grid->grid[x][y];
    return value;
}

// Set a grid point with additional handling for TARGET, OBSTACLE, and DRONE
void set_grid_point(Grid *grid, GridPointType type, FILE *log_file, Config *config, int x, int y, int value)
{
    char log_buffer[256];

    if (x < 0 || x >= config->Map.Size.Width || y < 0 || y >= config->Map.Size.Height)
    {
        sprintf(log_buffer, "Error: Grid coordinates out of bounds.");
        log_message(log_file, log_buffer);
        return;
    }

    switch (type)
    {
    case TARGET:
        grid->targets[grid->target_count].x = (float)x;
        grid->targets[grid->target_count].y = (float)y;
        grid->targets[grid->target_count].id = value; // Use value as ID
        grid->target_count++;

        sprintf(log_buffer, "Target set at (%d, %d) with ID %d.", x, y, value);
        log_message(log_file, log_buffer);
        break;

    case OBSTACLE:
        grid->obstacles[grid->obstacle_count].x = x;
        grid->obstacles[grid->obstacle_count].y = y;
        grid->obstacle_count++;

        sprintf(log_buffer, "Obstacle set at (%d, %d).", x, y);
        log_message(log_file, log_buffer);
        break;

    case DRONE:
        if (grid->grid[y][x] == 255) // Drone hits an obstacle
        {
            grid->score -= 2;
            sprintf(log_buffer, "Drone hit an obstacle at (%d, %d).", x, y);
            log_message(log_file, log_buffer);
            grid->drone_pos.x = x + 1;
            grid->drone_pos.y = y;
            // it will be scaled by the map process
        }
        else if (grid->grid[y][x] == 0) // Empty space
        {
            sprintf(log_buffer, "Drone moved to empty space at (%d, %d).", x, y);
            log_message(log_file, log_buffer);
            grid->drone_pos.x = x;
            grid->drone_pos.y = y;
        }
        else if (grid->grid[y][x] > 0 & grid->grid[y][x] < 255) // Any other value (assuming it's a target)
        {
            char log_buffer[256];
            int final_number = grid->grid[y][x]; // Start with the current digit

            // Check the left adjacent cell
            if (x > 0 && grid->grid[y][x - 1] >= 0 && grid->grid[y][x - 1] <= 9)
            {
                // Left cell is a digit; concatenate
                final_number = grid->grid[y][x - 1] * 10 + grid->grid[y][x];
                sprintf(log_buffer, "Drone hit a multi-digit target %d formed at (%d, %d) and (%d, %d).",
                        final_number, x - 1, y, x, y);
                log_message(log_file, log_buffer);

                // Clear the left and current grid spots
                grid->grid[y][x - 1] = 0;
                grid->grid[y][x] = 0;
            }
            // Check the right adjacent cell
            else if (x < config->Map.Size.Width - 1 && grid->grid[y][x + 1] >= 0 && grid->grid[y][x + 1] <= 9)
            {
                // Right cell is a digit; concatenate
                final_number = grid->grid[y][x] * 10 + grid->grid[y][x + 1];
                sprintf(log_buffer, "Drone hit a multi-digit target %d formed at (%d, %d) and (%d, %d).",
                        final_number, x, y, x + 1, y);
                log_message(log_file, log_buffer);

                // Clear the current and right grid spots
                grid->grid[y][x] = 0;
                grid->grid[y][x + 1] = 0;
            }
            else
            {
                // Single-digit target
                sprintf(log_buffer, "Drone hit a single-digit target %d at (%d, %d).", final_number, x, y);
                log_message(log_file, log_buffer);
                grid->grid[y][x] = 0;
            }

            // Remove the target from the list
            for (int i = 0; i < grid->target_count; i++)
            {
                if ((int)grid->targets[i].id == final_number)
                {
                    for (int j = i; j < grid->target_count - 1; j++)
                    {
                        grid->targets[j] = grid->targets[j + 1];
                    }
                    grid->target_count--;
                    sprintf(log_buffer, "Target %d removed from target list.", final_number);
                    log_message(log_file, log_buffer);
                    break;
                }
            }

            // Update score
            grid->score += 5;
            sprintf(log_buffer, "Score increased by 5. Current score: %d.", grid->score);
            log_message(log_file, log_buffer);

            // Set the drone's new position
            grid->drone_pos.x = x;
            grid->drone_pos.y = y;
            sprintf(log_buffer, "Drone set at (%d, %d) with value %d.", x, y, 254);
            log_message(log_file, log_buffer);
        }
        break;

    case FREE:
        grid->grid[y][x] = 0;
        sprintf(log_buffer, "Free spot set at (%d, %d).", x, y);
        log_message(log_file, log_buffer);
        break;

    default:
        sprintf(log_buffer, "Error: Unknown grid point type.");
        log_message(log_file, log_buffer);
    }
}
