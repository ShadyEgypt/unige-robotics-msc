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
Config *config = NULL;

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

    int shm_config_fd = attach_shared_memory(SHM_CONFIG_NAME, SHM_CONFIG_SIZE);
    void *config_addr = map_shared_memory(shm_config_fd, SHM_CONFIG_SIZE);
    config = (Config *)config_addr;

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
    fd = open(config->Pipes.DronePipe, O_RDONLY);
    if (fd == -1)
    {
        perror("Failed to open drone pipe");
    }
    log_message(log_file, "opened fd in read mode!");

    // Set the drone in the shared memory to be (1,1)
    drone->drone_pos.x = 1;
    drone->drone_pos.y = 1;
    drone->drone_pos_1.x = 2;
    drone->drone_pos_1.y = 2;
    grid->drone_pos.x = 1;
    grid->drone_pos.y = 1;
    // log_message(log_file, "Drone position set to (1, 1)");
    // set_grid_point(grid, DRONE, log_file, config, drone->drone_pos.x, drone->drone_pos.y, 254);
    log_message(log_file, "Parent: All resources are initialized successfully.");
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
        detach_shared_memory(config, SHM_CONFIG_SIZE);
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

float repulsive_force(float distance, float function_scale,
                      float area_of_effect, float vel_x, float vel_y)
{
    return function_scale * ((1 / distance) - (1 / area_of_effect)) *
           (1 / (distance * distance)) * sqrt(pow(vel_x, 2) + pow(vel_y, 2));
}

// Update wall repulsive force with clamping
void update_wall_force(Drone *drone)
{
    drone->wall_force.x = 0.0f;
    drone->wall_force.y = 0.0f;
    // if the left edge is detected
    if (drone->drone_pos.x < config->EffectRadius.Wall)
    {
        drone->wall_force.x = repulsive_force(
            drone->drone_pos.x, config->Forces.Wall, config->EffectRadius.Wall,
            drone->drone_vel.x, drone->drone_vel.y);
        // if the right edge is detected
    }
    else if (drone->drone_pos.x > config->Map.Size.Width - config->EffectRadius.Wall)
    {
        drone->wall_force.y = -repulsive_force(
            config->Map.Size.Width - drone->drone_pos.x, config->Forces.Wall,
            config->EffectRadius.Wall, drone->drone_vel.x, drone->drone_vel.y);
    }
    // Otherwise set it to 0
    else
    {
        drone->wall_force.x = 0;
    }

    // if the upper edge is detected
    if (drone->drone_pos.y < config->EffectRadius.Wall)
    {
        drone->wall_force.y = repulsive_force(
            drone->drone_pos.y, config->Forces.Wall, config->EffectRadius.Wall,
            drone->drone_vel.x, drone->drone_vel.y);
        // if the lower edge is detected
    }
    else if (drone->drone_pos.y > config->Map.Size.Height - config->EffectRadius.Wall)
    {
        drone->wall_force.y = -repulsive_force(
            config->Map.Size.Height - drone->drone_pos.y, config->Forces.Wall,
            config->EffectRadius.Wall, drone->drone_vel.y, drone->drone_vel.y);
    }
    // Otherwise set it to 0
    else
    {
        drone->wall_force.y = 0;
    }

    // Clamp Wall Forces
    // drone->wall_force.x = fminf(fmaxf(drone->wall_force.x, -MAX_WALL_FORCE), MAX_WALL_FORCE);
    // drone->wall_force.y = fminf(fmaxf(drone->wall_force.y, -MAX_WALL_FORCE), MAX_WALL_FORCE);

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
        float distance = sqrt(pow(obstacles[i].x - drone->drone_pos.x, 2) +
                              pow(obstacles[i].y - drone->drone_pos.y, 2));
        // If it's quite close but not too much then apply the force.
        if (distance < config->EffectRadius.Obstacle && distance > 1)
        {
            double x_distance = obstacles[i].x - drone->drone_pos.x;
            double y_distance = obstacles[i].y - drone->drone_pos.y;

            // Compute the magnitude of the repulsive force
            double force =
                -repulsive_force(distance, 10000, config->EffectRadius.Obstacle,
                                 drone->drone_vel.x, drone->drone_vel.y);

            // Compute the direction of the repulsive force
            double angle = atan2(y_distance, x_distance);

            // Add the force to the accumulation variable
            drone->obs_force.x += cos(angle) * force;
            drone->obs_force.y += sin(angle) * force;

            // Cap the force at a certain threshold.
            if (drone->obs_force.x > config->Thresholds.MaxObstacleForces)
                drone->obs_force.x = config->Thresholds.MaxObstacleForces;
            if (drone->obs_force.x < -config->Thresholds.MaxObstacleForces)
                drone->obs_force.x = -config->Thresholds.MaxObstacleForces;
            if (drone->obs_force.y > config->Thresholds.MaxObstacleForces)
                drone->obs_force.y = config->Thresholds.MaxObstacleForces;
            if (drone->obs_force.y < -config->Thresholds.MaxObstacleForces)
                drone->obs_force.y = -config->Thresholds.MaxObstacleForces;
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
        float distance = sqrt(pow(targets[i].x - drone->drone_pos.x, 2) +
                              pow(targets[i].y - drone->drone_pos.y, 2));
        // If it's quite close but not too much then apply the force.
        if (distance < config->EffectRadius.Target && distance > 1)
        {
            double x_distance = targets[i].x - drone->drone_pos.x;
            double y_distance = targets[i].y - drone->drone_pos.y;

            // Compute the magnitude of the repulsive force
            double force =
                -repulsive_force(distance, 10000, config->EffectRadius.Target,
                                 drone->drone_vel.x, drone->drone_vel.y);

            // Compute the direction of the repulsive force
            double angle = atan2(y_distance, x_distance);

            // Add the force to the accumulation variable
            drone->tar_force.x += cos(angle) * force;
            drone->tar_force.y += sin(angle) * force;

            // Cap the force at a certain threshold.
            if (drone->tar_force.x > config->Thresholds.MaxTargetForces)
                drone->tar_force.x = config->Thresholds.MaxTargetForces;
            if (drone->tar_force.x < -config->Thresholds.MaxTargetForces)
                drone->tar_force.x = -config->Thresholds.MaxTargetForces;
            if (drone->tar_force.y > config->Thresholds.MaxTargetForces)
                drone->tar_force.y = config->Thresholds.MaxTargetForces;
            if (drone->tar_force.y < -config->Thresholds.MaxTargetForces)
                drone->tar_force.y = -config->Thresholds.MaxTargetForces;
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
        X_FORCE -= diag(config->Forces.Step);
        Y_FORCE -= diag(config->Forces.Step);
        break;
    case 'w':
        Y_FORCE -= config->Forces.Step;
        break;
    case 'e':
        X_FORCE += diag(config->Forces.Step);
        Y_FORCE -= diag(config->Forces.Step);
        break;
    case 'a':
        X_FORCE -= config->Forces.Step;
        break;
    case 's':
        X_FORCE = slow_down();
        Y_FORCE = slow_down();
        break;
    case 'd':
        X_FORCE += config->Forces.Step;
        break;
    case 'z':
        X_FORCE -= diag(config->Forces.Step);
        Y_FORCE += diag(config->Forces.Step);
        break;
    case 'x':
        Y_FORCE += config->Forces.Step;
        break;
    case 'c':
        X_FORCE += diag(config->Forces.Step);
        Y_FORCE += diag(config->Forces.Step);
        break;
    case ' ':
        X_FORCE = slow_down();
        Y_FORCE = slow_down();
        break;
    default:
        ret = false;
    }

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
    float pos_x;
    float pos_y;
    log_message(log_file, "updating position");
    // acquire_semaphore(sem_drone);

    if (drone->drone_vel.x < config->Thresholds.ZeroThreshold &&
        drone->drone_vel.x > -config->Thresholds.ZeroThreshold &&
        drone->drone_force.x == 0.0)
    {
        drone->drone_pos.x = drone->drone_pos_1.x;
    }
    else
    {
        // Precompute constants to improve readability
        float mass_term = config->Physics.Mass / (config->Physics.IntegrationInterval * config->Physics.IntegrationInterval);
        float viscous_term = config->Physics.ViscousCoefficient / config->Physics.IntegrationInterval;
        float denominator = mass_term + viscous_term;
        float numerator = drone->drone_force.x - mass_term * (drone->drone_pos_2.x - 2 * drone->drone_pos_1.x) + viscous_term * drone->drone_pos_1.x;

        pos_x = numerator / denominator;
    }

    if (drone->drone_vel.y < config->Thresholds.ZeroThreshold &&
        drone->drone_vel.y > -config->Thresholds.ZeroThreshold &&
        drone->drone_force.y == 0.0)
    {
        drone->drone_pos.y = drone->drone_pos_1.y;
    }
    else
    {
        // Precompute constants to improve readability
        float mass_term = config->Physics.Mass / (config->Physics.IntegrationInterval * config->Physics.IntegrationInterval);
        float viscous_term = config->Physics.ViscousCoefficient / config->Physics.IntegrationInterval;
        float denominator = mass_term + viscous_term;
        float numerator = drone->drone_force.y - mass_term * (drone->drone_pos_2.y - 2 * drone->drone_pos_1.y) + viscous_term * drone->drone_pos_1.y;

        pos_y = numerator / denominator;
    }

    if (pos_x > config->Map.Size.Width)
        pos_x = config->Map.Size.Width - 1;
    else if (pos_x < 0)
        pos_x = 1;
    if (pos_y > config->Map.Size.Height)
        pos_x = config->Map.Size.Height - 1;
    else if (pos_y < 0)
        pos_y = 1;

    drone->drone_pos_2 = drone->drone_pos_1;
    drone->drone_pos_1 = drone->drone_pos;
    drone->drone_pos.x = CEIL_TO_INT(pos_x);
    drone->drone_pos.y = CEIL_TO_INT(pos_y);
    snprintf(log_msg, sizeof(log_msg), "new positions: X_POS = %.2f, Y_POS = %.2f", drone->drone_pos.x, drone->drone_pos.y);
    log_message(log_file, log_msg);

    // release_semaphore(sem_drone);
    // acquire_semaphore(sem_grid);
    set_grid_point(grid, FREE, log_file, config, drone->drone_pos_1.x, drone->drone_pos_1.y, 0);
    set_grid_point(grid, DRONE, log_file, config, drone->drone_pos.x, drone->drone_pos.y, 254);
    // release_semaphore(sem_grid);

    log_message(log_file, "updated position of the drone in the grid!");
}

void update_velocity(Drone *drone)
{
    char log_msg[255];
    // acquire_semaphore(sem_drone);
    log_message(log_file, "updating velocities");

    float dx_dt = (drone->drone_pos.x - drone->drone_pos_1.x) / config->Physics.IntegrationInterval;
    float dy_dt = (drone->drone_pos.y - drone->drone_pos_1.y) / config->Physics.IntegrationInterval;

    snprintf(log_msg, sizeof(log_msg), "dx/dt = %.2f, dy/dt = %.2f", dx_dt, dy_dt);
    log_message(log_file, log_msg);
    // // Cap the velocity
    // drone->drone_vel.x = fminf(fmaxf(dx_dt, -config->Thresholds.MaxVelocity), config->Thresholds.MaxVelocity);
    // drone->drone_vel.y = fminf(fmaxf(dy_dt, -config->Thresholds.MaxVelocity), config->Thresholds.MaxVelocity);

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

        // sleep to prevent overloading
        usleep(1000000 * config->Physics.IntegrationInterval);
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
            // Update forces based on the grid's targets and obstacles
            update_user_force(drone, cmd);
            update_wall_force(drone);
            update_obstacle_force(drone, grid->obstacles, grid->obstacle_count);
            update_target_force(drone, grid->targets, grid->target_count);
            calculate_total_force(drone);
            // Update drone's state
            update_position(drone, grid);
            update_velocity(drone);

            // Log state after processing
            char log_msg[256];
            snprintf(log_msg, sizeof(log_msg), "Processed input: '%c'", cmd);
            log_message(log_file, log_msg);
        }

        // sleep to prevent high CPU usage
        usleep(1000000 * config->Physics.IntegrationInterval);
    }
}
