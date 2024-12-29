#ifndef GLOBALS_H
#define GLOBALS_H

#include <ncurses.h>
#include <stdio.h>
#include <sys/types.h>

extern WINDOW *win;
extern FILE *log_file;

// some macros for the dynamics display keyboard grid
#define cell_height 5
#define cell_width 7
#define grid_height 15
#define grid_width 21

// Size of the grid
#define GRID_HEIGHT 100
#define GRID_WIDTH 100
#define GRID_SIZE (GRID_HEIGHT * GRID_WIDTH)

#define MAX_TARGETS 15
#define MAX_OBSTACLES 15

#define MASS 1.0
#define VISCOUS_COEFFICIENT 0.1f
#define INTEGRATION_INTERVAL 0.1f

// Force Constants
#define FORCE_STEP 0.5              // Step increment for user-applied forces
#define MAX_FORCE 5.0               // Maximum allowed force
#define OBSTACLE_FORCE_CONSTANT 1.0 // Scaling for obstacle repulsion
#define WALL_FORCE_CONSTANT 1.0     // Scaling for wall repulsion
#define TARGET_FORCE_CONSTANT 1.0   // Scaling for target attraction

// Effect Radius Constants
#define OBSTACLE_EFFECT_RADIUS 5.0 // Radius within which obstacles exert force
#define TARGET_EFFECT_RADIUS 5.0   // Radius within which targets exert force
#define WALL_EFFECT_RADIUS 5.0     // Distance from walls to exert repulsion
#define MIN_WALL_DISTANCE 0.1f
#define MAX_VELOCITY 3.0f
#define MAX_WALL_FORCE 3.0f
// Thresholds
#define MIN_DISTANCE 1.0    // Minimum distance for obstacle repulsion
#define ZERO_THRESHOLD 0.01 // Threshold to stop movement due to low velocity

// Display Constants
#define MAX_OBST_FORCES 5.0 // Cap for obstacle force in either direction
#define MAX_TARG_FORCES 5.0 // Cap for target force in either direction

#define KEYBOARD_PIPE "keyboard_pipe"
#define SERVER_PIPE "server_pipe"
#define DRONE_PIPE "drone_pipe"

#define MAX_PAYLOAD_SIZE 256
#define MAX_RESPONSE_SIZE 256
#define REQUEST_PENDING 0
#define REQUEST_PROCESSED 1
#define TIMEOUT_SECONDS 5

#define CEIL_TO_INT(x) ((int)ceil(x))

struct force
{
    float x;
    float y;
};

struct pos
{
    float x;
    float y;
};

struct vel
{
    float x;
    float y;
};

typedef struct
{
    struct force user_force;
    struct force wall_force;
    struct force obs_force;
    struct force tar_force;
    struct force drone_force;
    struct pos drone_pos;
    struct pos drone_pos_1;
    struct pos drone_pos_2;
    struct vel drone_vel;
    char cmd[3];
    int cmd_front;
    int cmd_rear;
} Drone;

typedef struct
{
    WINDOW *left_split;
    WINDOW *right_split;
    WINDOW *tl_win;
    WINDOW *tc_win;
    WINDOW *tr_win;
    WINDOW *cl_win;
    WINDOW *cc_win;
    WINDOW *cr_win;
    WINDOW *bl_win;
    WINDOW *bc_win;
    WINDOW *br_win;
} WindowLayout;

// Enumeration for grid point types
typedef enum
{
    DRONE,
    TARGET,
    OBSTACLE,
    FREE
} GridPointType;

typedef struct
{
    float x;
    float y;
    unsigned char id; // Unique identifier for the target
    bool is_two_digit;
} Target;

typedef struct
{
    float x;
    float y;
} Obstacle;

// Structure to hold grid data in shared memory
typedef struct
{
    int grid[200][200]; // max resolution that the map can reach
    int grid_actual_height;
    int grid_actual_width;
    struct pos drone_pos;
    int score;

    Target targets[MAX_TARGETS];
    int target_count;

    Obstacle obstacles[MAX_OBSTACLES];
    int obstacle_count;
} Grid;

typedef struct
{
    pid_t display_pid;
    pid_t server_pid;
    pid_t drone_pid;
    pid_t targets_pid;
    pid_t obstacles_pid;
    pid_t map_pid;
    int input;
} Globals;

// Define a shared memory communication object
typedef struct
{
    // Request Section
    int request_id;                 // Unique identifier for the request
    char action[MAX_PAYLOAD_SIZE];  // The action or command (e.g., "get_data", "set_data")
    char payload[MAX_PAYLOAD_SIZE]; // Additional data needed for the request
    bool is_valid;                  // Flag to indicate if the request is valid

    // Response Section
    int status;                       // Status of the request (e.g., REQUEST_PENDING, REQUEST_PROCESSED)
    char response[MAX_RESPONSE_SIZE]; // Response data
    bool has_response;                // Flag to indicate if response is ready

    // Synchronization
    bool is_request_active; // Indicates if a request is currently being processed
    bool is_response_ready; // Indicates if the response is ready
} MapServerClient;

// Shared memory object name (used with shm_open)
#define SHM_GRID_NAME "/shared_memory_grid"
#define SEM_GRID_NAME "/shared_semaphore_grid"
#define SHM_GRID_SIZE sizeof(Grid)

#define SHM_G_NAME "/shared_memory_general"
#define SEM_G_NAME "/shared_semaphore_general"
#define SHM_G_SIZE sizeof(Globals)

#define SHM_DRONE_NAME "/shared_memory_drone"
#define SEM_DRONE_NAME "/shared_semaphore_drone"
#define SHM_DRONE_SIZE sizeof(Drone)

#define SHM_MAP_NAME "/shared_memory_map"
#define SEM_MAP_NAME "/shared_semaphore_map"
#define SHM_MAP_SIZE sizeof(MapServerClient)

#endif // GLOBALS_H