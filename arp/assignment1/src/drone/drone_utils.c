#include "drone_utils.h"

// Define global variables
FILE *log_file = NULL;
sem_t *sem_grid = NULL;
sem_t *sem_g = NULL;
sem_t *sem_drone = NULL;

bool resources_exist = false;
Grid *grid = NULL;
Globals *globals = NULL;
Drone *drone = NULL;
MapServerClient *map = NULL;

pid_t child1_pid = -1;
pid_t child2_pid = -1;

int fd = -1;
char input = '\0';
bool force_updated = false;
bool vel_updated = false;

// Resource Setup
void setup_resources()
{
    // Attach and map shared memory
    int shm_grid_fd = attach_shared_memory(SHM_GRID_NAME, SHM_GRID_SIZE);
    void *grid_addr = map_shared_memory(shm_grid_fd, SHM_GRID_SIZE);
    grid = (Grid *)grid_addr;

    int shm_g_fd = attach_shared_memory(SHM_G_NAME, SHM_G_SIZE);
    void *globals_addr = map_shared_memory(shm_g_fd, SHM_G_SIZE);
    globals = (Globals *)globals_addr;

    int shm_drone_fd = attach_shared_memory(SHM_DRONE_NAME, SHM_DRONE_SIZE);
    void *drone_addr = map_shared_memory(shm_drone_fd, SHM_DRONE_SIZE);
    drone = (Drone *)drone_addr;

    int shm_map_fd = attach_shared_memory(SHM_MAP_NAME, SHM_MAP_SIZE);
    void *map_addr = map_shared_memory(shm_map_fd, SHM_MAP_SIZE);
    map = (MapServerClient *)map_addr;

    srand(time(NULL));

    // Open semaphore
    sem_grid = open_semaphore(SEM_GRID_NAME);
    sem_g = open_semaphore(SEM_G_NAME);
    sem_drone = open_semaphore(SEM_DRONE_NAME);

    log_message(log_file, "shared memories got attached!");

    resources_exist = true;

    pid_t drone_pid = getpid();
    globals->drone_pid = drone_pid;

    // open named pipe in read mode (non-blocking)
    fd = open(DRONE_PIPE, O_RDONLY);
    if (fd == -1)
    {
        perror("Failed to open drone pipe");
    }
    log_message(log_file, "opened fd in read mode!");

    // Set the drone in the shared memory to be (1,1)
    drone->drone_pos.x = 1;
    drone->drone_pos.y = 1;
    drone->drone_pos_1.x = 1;
    drone->drone_pos_1.y = 1;
    set_grid_point(grid, drone->drone_pos.x, drone->drone_pos.y, 254, DRONE, log_file);
    printf("Parent: All resources are initialized successfully.\n");
}

// Signal handler function for SIGINT
void handle_sigint(int sig)
{
    printf("\nReceived SIGINT. Cleaning up resources...\n");

    // Kill child processes if they exist
    if (child1_pid > 0)
    {
        printf("Terminating child process 1 (PID: %d)\n", child1_pid);
        kill(child1_pid, SIGTERM); // Gracefully terminate child 1
    }
    if (child2_pid > 0)
    {
        printf("Terminating child process 2 (PID: %d)\n", child2_pid);
        kill(child2_pid, SIGTERM); // Gracefully terminate child 2
    }

    // Optionally wait for child processes to ensure they exit
    if (child1_pid > 0)
    {
        waitpid(child1_pid, NULL, 0);
    }
    if (child2_pid > 0)
    {
        waitpid(child2_pid, NULL, 0);
    }

    if (resources_exist)
    {
        detach_shared_memory(grid, SHM_GRID_SIZE);
        detach_shared_memory(globals, SHM_G_SIZE);
        detach_shared_memory(drone, SHM_DRONE_SIZE);
        detach_shared_memory(map, SHM_MAP_SIZE);
        printf("Shared memory detached.\n");
    }
    close(fd);
    exit(0); // Exit the program
}

// Function to calculate the diagonal force
float diag(float side)
{
    float sqrt2_half = 0.7071;
    return side * sqrt2_half;
}

// Function to implement the decreasing of the force
float slow_down(void)
{
    return 0.f;
}

// Update wall repulsive force with clamping
void update_wall_force(Drone *drone)
{
    drone->wall_force.x = 0.0f;
    drone->wall_force.y = 0.0f;

    // Handle X-axis Wall Repulsion
    if (drone->drone_pos.x < WALL_EFFECT_RADIUS)
    {
        float safe_distance = fmaxf(drone->drone_pos.x, MIN_WALL_DISTANCE);
        drone->wall_force.x += WALL_FORCE_CONSTANT / (safe_distance * safe_distance);
    }
    if (drone->drone_pos.x > GRID_WIDTH - WALL_EFFECT_RADIUS)
    {
        float safe_distance = fmaxf(GRID_WIDTH - drone->drone_pos.x, MIN_WALL_DISTANCE);
        drone->wall_force.x -= WALL_FORCE_CONSTANT / (safe_distance * safe_distance);
    }

    // Handle Y-axis Wall Repulsion
    if (drone->drone_pos.y < WALL_EFFECT_RADIUS)
    {
        float safe_distance = fmaxf(drone->drone_pos.y, MIN_WALL_DISTANCE);
        drone->wall_force.y += WALL_FORCE_CONSTANT / (safe_distance * safe_distance);
    }
    if (drone->drone_pos.y > GRID_HEIGHT - WALL_EFFECT_RADIUS)
    {
        float safe_distance = fmaxf(GRID_HEIGHT - drone->drone_pos.y, MIN_WALL_DISTANCE);
        drone->wall_force.y -= WALL_FORCE_CONSTANT / (safe_distance * safe_distance);
    }

    // Clamp Wall Forces
    drone->wall_force.x = fminf(fmaxf(drone->wall_force.x, -MAX_WALL_FORCE), MAX_WALL_FORCE);
    drone->wall_force.y = fminf(fmaxf(drone->wall_force.y, -MAX_WALL_FORCE), MAX_WALL_FORCE);

    // Logging
    char log_msg[256];
    snprintf(log_msg, sizeof(log_msg), "Wall Force (Clamped): X = %.2f, Y = %.2f", drone->wall_force.x, drone->wall_force.y);
    log_message(log_file, log_msg);
}

// Update obstacle repulsive force
void update_obstacle_force(Drone *drone, Obstacle obstacles[], int obstacles_num)
{
    drone->obs_force.x = 0.0f;
    drone->obs_force.y = 0.0f;

    for (int i = 0; i < obstacles_num; i++)
    {
        float distance = sqrt(pow(obstacles[i].x - drone->drone_pos.x, 2) + pow(obstacles[i].y - drone->drone_pos.y, 2));

        if (distance < OBSTACLE_EFFECT_RADIUS && distance > MIN_DISTANCE)
        {
            float force = OBSTACLE_FORCE_CONSTANT / (distance * distance);
            float angle = atan2(obstacles[i].y - drone->drone_pos.y, obstacles[i].x - drone->drone_pos.x);

            drone->obs_force.x -= cos(angle) * force;
            drone->obs_force.y -= sin(angle) * force;
        }
    }

    char log_msg[256];
    snprintf(log_msg, sizeof(log_msg), "Obstacle Force: X = %.2f, Y = %.2f", drone->obs_force.x, drone->obs_force.y);
    log_message(log_file, log_msg);
}

// Update target attractive force
void update_target_force(Drone *drone, Target targets[], int targets_num)
{
    drone->tar_force.x = 0.0f;
    drone->tar_force.y = 0.0f;

    for (int i = 0; i < targets_num; i++)
    {
        float distance = sqrt(pow(targets[i].x - drone->drone_pos.x, 2) + pow(targets[i].y - drone->drone_pos.y, 2));

        if (distance < TARGET_EFFECT_RADIUS)
        {
            float force = TARGET_FORCE_CONSTANT / distance;
            float angle = atan2(targets[i].y - drone->drone_pos.y, targets[i].x - drone->drone_pos.x);

            drone->tar_force.x += cos(angle) * force;
            drone->tar_force.y += sin(angle) * force;
        }
    }

    char log_msg[256];
    snprintf(log_msg, sizeof(log_msg), "Target Force: X = %.2f, Y = %.2f", drone->tar_force.x, drone->tar_force.y);
    log_message(log_file, log_msg);
}

// Update user force based on keyboard input
void update_user_force(Drone *drone, char cmd)
{

    char log_msg[256];
    snprintf(log_msg, sizeof(log_msg), "Updating user force with input '%c'", cmd);
    log_message(log_file, log_msg);

    bool ret = true;
    float X_FORCE = 0.0f, Y_FORCE = 0.0f;

    switch (cmd)
    {
    case 'q':
        X_FORCE -= diag(FORCE_STEP);
        Y_FORCE -= diag(FORCE_STEP);
        break;
    case 'w':
        Y_FORCE -= FORCE_STEP;
        break;
    case 'e':
        X_FORCE += diag(FORCE_STEP);
        Y_FORCE -= diag(FORCE_STEP);
        break;
    case 'a':
        X_FORCE -= FORCE_STEP;
        break;
    case 's':
        X_FORCE = slow_down();
        Y_FORCE = slow_down();
        break;
    case 'd':
        X_FORCE += FORCE_STEP;
        break;
    case 'z':
        X_FORCE -= diag(FORCE_STEP);
        Y_FORCE += diag(FORCE_STEP);
        break;
    case 'x':
        Y_FORCE += FORCE_STEP;
        break;
    case 'c':
        X_FORCE += diag(FORCE_STEP);
        Y_FORCE += diag(FORCE_STEP);
        break;
    case ' ':
        X_FORCE = slow_down();
        Y_FORCE = slow_down();
        break;
    default:
        ret = false;
    }

    // Clamp forces
    X_FORCE = fminf(fmaxf(X_FORCE, -MAX_FORCE), MAX_FORCE);
    Y_FORCE = fminf(fmaxf(Y_FORCE, -MAX_FORCE), MAX_FORCE);

    drone->user_force.x = X_FORCE;
    drone->user_force.y = Y_FORCE;

    snprintf(log_msg, sizeof(log_msg), "User Force: X = %.2f, Y = %.2f", X_FORCE, Y_FORCE);
    log_message(log_file, log_msg);
}

// Calculate total force from all individual forces
void calculate_total_force(Drone *drone)
{
    drone->drone_force.x = drone->user_force.x +
                           drone->wall_force.x +
                           drone->obs_force.x +
                           drone->tar_force.x;

    drone->drone_force.y = drone->user_force.y +
                           drone->wall_force.y +
                           drone->obs_force.y +
                           drone->tar_force.y;

    char log_msg[256];
    snprintf(log_msg, sizeof(log_msg), "Total Force: X = %.2f, Y = %.2f", drone->drone_force.x, drone->drone_force.y);
    log_message(log_file, log_msg);
}

void update_position(Drone *drone, Grid *grid)
{
    char log_msg[255];
    log_message(log_file, "updating position");
    // acquire_semaphore(sem_drone);

    float pos_x = (drone->drone_force.x * INTEGRATION_INTERVAL * INTEGRATION_INTERVAL +
                   (2 * MASS + VISCOUS_COEFFICIENT * INTEGRATION_INTERVAL) * drone->drone_pos_1.x -
                   MASS * drone->drone_pos_2.x) /
                  (MASS + VISCOUS_COEFFICIENT * INTEGRATION_INTERVAL);

    float pos_y = (drone->drone_force.y * INTEGRATION_INTERVAL * INTEGRATION_INTERVAL +
                   (2 * MASS + VISCOUS_COEFFICIENT * INTEGRATION_INTERVAL) * drone->drone_pos_1.y -
                   MASS * drone->drone_pos_2.y) /
                  (MASS + VISCOUS_COEFFICIENT * INTEGRATION_INTERVAL);

    // Update position with clamping
    if (pos_x > GRID_WIDTH)
    {
        pos_x = GRID_WIDTH - 1;
        drone->drone_vel.x = 0.0f; // Reset velocity along X-axis
    }
    if (pos_x < 0)
    {
        pos_x = 1;
        drone->drone_vel.x = 0.0f;
    }
    if (pos_y > GRID_HEIGHT)
    {
        pos_y = GRID_HEIGHT - 1;
        drone->drone_vel.y = 0.0f; // Reset velocity along Y-axis
    }
    if (pos_y < 0)
    {
        pos_y = 1;
        drone->drone_vel.y = 0.0f;
    }

    drone->drone_pos_2 = drone->drone_pos_1;
    drone->drone_pos_1 = drone->drone_pos;
    drone->drone_pos.x = CEIL_TO_INT(pos_x);
    drone->drone_pos.y = CEIL_TO_INT(pos_y);
    snprintf(log_msg, sizeof(log_msg), "new positions: X_POS = %.2f, Y_POS = %.2f", drone->drone_pos.x, drone->drone_pos.y);
    log_message(log_file, log_msg);

    // release_semaphore(sem_drone);

    // acquire_semaphore(sem_grid);
    set_grid_point(grid, drone->drone_pos_1.x, drone->drone_pos_1.y, 0, FREE, log_file);
    set_map_point(grid, drone, map, log_file);
    // release_semaphore(sem_grid);

    log_message(log_file, "updated position of the drone in the grid!");
}

void update_velocity(Drone *drone)
{
    char log_msg[255];
    // acquire_semaphore(sem_drone);
    log_message(log_file, "updating velocities");

    float dx_dt = (drone->drone_pos.x - drone->drone_pos_1.x) / INTEGRATION_INTERVAL;
    float dy_dt = (drone->drone_pos.y - drone->drone_pos_1.y) / INTEGRATION_INTERVAL;

    snprintf(log_msg, sizeof(log_msg), "dx/dt = %.2f, dy/dt = %.2f", dx_dt, dy_dt);
    log_message(log_file, log_msg);
    // Cap the velocity
    drone->drone_vel.x = fminf(fmaxf(dx_dt, -MAX_VELOCITY), MAX_VELOCITY);
    drone->drone_vel.y = fminf(fmaxf(dy_dt, -MAX_VELOCITY), MAX_VELOCITY);

    drone->drone_pos_1.x = drone->drone_pos.x;
    drone->drone_pos_1.y = drone->drone_pos.y;
    drone->cmd_front = 0;
    drone->cmd_rear = 0;
    // release_semaphore(sem_drone);
    log_message(log_file, "updated velocities");
}

int enqueue_cmd(Drone *drone, char command)
{
    // Check if the queue is full
    if ((drone->cmd_rear + 1) % 3 == drone->cmd_front)
    {
        log_message(log_file, "Command queue is full!\n");
    }
    char log_msg[256];
    drone->cmd[drone->cmd_rear] = command;       // Add command to rear
    drone->cmd_rear = (drone->cmd_rear + 1) % 3; // Move rear forward (circular buffer)
    snprintf(log_msg, sizeof(log_msg), "Enqueued Command: %c", command);
    log_message(log_file, log_msg);
}

char dequeue_cmd(Drone *drone)
{
    // Check if the queue is empty
    if (drone->cmd_front == drone->cmd_rear)
    {
        log_message(log_file, "Command queue is empty!\n");
    }
    char log_msg[256];
    char command = drone->cmd[drone->cmd_front];   // Get the front command
    drone->cmd_front = (drone->cmd_front + 1) % 3; // Move front forward (circular buffer)
    snprintf(log_msg, sizeof(log_msg), "Dequeued Command: %c", command);
    log_message(log_file, log_msg);
    return command;
}

int is_cmd_empty(Drone *drone)
{
    return (drone->cmd_front == drone->cmd_rear);
}

void child1_task()
{
    while (1)
    {
        // Read the key from the named pipe (non-blocking)
        if (read(fd, &input, sizeof(char)) <= 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // No input available, continue looping
                usleep(100000); // Optional sleep before retrying
                continue;
            }
            else
            {
                perror("Failed to read the input key");
                exit(EXIT_FAILURE);
            }
        }

        // Check if the input is invalid (e.g., unwanted character such as apostrophe)
        if (input == '\0' || input == '\n' || input == '\r')
        {
            usleep(100000);
            continue;
        }
        if (input == 'q' || input == 'w' || input == 'e' || input == 'a' || input == 's' || input == 'd' || input == 'z' || input == 'x' || input == 'c')
        {
            acquire_semaphore(sem_drone);
            enqueue_cmd(drone, input);
            release_semaphore(sem_drone);
            printf("[child1_task] cmd set to: '%c'\n", input);

            char log_msg[256];
            snprintf(log_msg, sizeof(log_msg), "Input '%c' received from the pipe", input);
            log_message(log_file, log_msg);
            input = '\0';
        }

        // Optional sleep to prevent overloading
        usleep(100000);
    }
}

void child2_task()
{
    while (1)
    {
        while (!is_cmd_empty(drone))
        {
            log_message(log_file, "Equations are processing!");
            char cmd = dequeue_cmd(drone);
            // Stop command
            if (cmd == 's')
            {
                // Reset forces
                drone->obs_force.x = 0.0;
                drone->obs_force.y = 0.0;

                drone->tar_force.x = 0.0;
                drone->tar_force.y = 0.0;

                drone->user_force.x = 0.0;
                drone->user_force.y = 0.0;

                drone->wall_force.x = 0.0;
                drone->wall_force.y = 0.0;

                drone->drone_force.x = 0.0;
                drone->drone_force.y = 0.0;

                // Reset position
                drone->drone_pos_1 = drone->drone_pos;
                drone->drone_pos_2 = drone->drone_pos;

                // Reset velocity
                drone->drone_vel.x = 0.0;
                drone->drone_vel.y = 0.0;

                printf("All drone forces and velocity have been reset.\n");
            }
            else
            {
                // Update forces based on the grid's targets and obstacles
                update_user_force(drone, cmd);
                update_wall_force(drone);
                update_obstacle_force(drone, grid->obstacles, grid->obstacle_count);
                update_target_force(drone, grid->targets, grid->target_count);
                calculate_total_force(drone);
                // Update drone's state
                update_position(drone, grid);
                update_velocity(drone);
            }

            // Log state after processing
            char log_msg[256];
            snprintf(log_msg, sizeof(log_msg), "Processed input: '%c'", cmd);
            log_message(log_file, log_msg);

            acquire_semaphore(sem_drone);
            release_semaphore(sem_drone);
        }

        // Optional sleep to prevent high CPU usage
        usleep(100000);
    }
}
