#include "server_utils.h"

FILE *log_file = NULL;
bool resources_exist = false;
sem_t *g1 = NULL, *g2 = NULL, *g3 = NULL, *g4 = NULL;
;
int server_fd = -1, drone_fd = -1;

Grid *grid = NULL;
Globals *globals = NULL;
Drone *drone = NULL;
MapServerClient *map = NULL;

pid_t child1_pid = -1;
pid_t child2_pid = -1;
pid_t child3_pid = -1;
// Resource Setup
void setup_resources()
{
    int shm_grid_fd = create_shared_memory(SHM_GRID_NAME, SHM_GRID_SIZE);
    void *grid_addr = map_shared_memory(shm_grid_fd, SHM_GRID_SIZE);
    grid = (Grid *)grid_addr;

    int shm_g_fd = create_shared_memory(SHM_G_NAME, SHM_G_SIZE);
    void *globals_addr = map_shared_memory(shm_g_fd, SHM_G_SIZE);
    globals = (Globals *)globals_addr;

    int shm_drone_fd = create_shared_memory(SHM_DRONE_NAME, SHM_DRONE_SIZE);
    void *drone_addr = map_shared_memory(shm_drone_fd, SHM_DRONE_SIZE);
    drone = (Drone *)drone_addr;

    int shm_map_fd = create_shared_memory(SHM_MAP_NAME, SHM_MAP_SIZE);
    void *map_addr = map_shared_memory(shm_map_fd, SHM_MAP_SIZE);
    map = (MapServerClient *)map_addr;

    // Semaphore creation
    g1 = create_semaphore(SEM_GRID_NAME);
    g2 = create_semaphore(SEM_G_NAME);
    g3 = create_semaphore(SEM_DRONE_NAME);
    g4 = create_semaphore(SEM_MAP_NAME);

    create_fifo(SERVER_PIPE);
    create_fifo(DRONE_PIPE);
    grid->score = 0;
    server_fd = open(SERVER_PIPE, O_RDONLY);
    drone_fd = open(DRONE_PIPE, O_WRONLY);

    if (server_fd == -1 || drone_fd == -1)
    {
        perror("Failed to open pipes");
        exit(1);
    }
    printf("Parent: All resources are created successfully.\n");
    resources_exist = true;
}

// Resource Cleanup
void cleanup_resources()
{
    if (resources_exist)
    {
        destroy_shared_memory(SHM_GRID_NAME);
        destroy_shared_memory(SHM_G_NAME);
        destroy_shared_memory(SHM_DRONE_NAME);
        printf("Shared memory detached and destroyed.\n");
    }
    destroy_semaphore(SEM_GRID_NAME, g1);
    destroy_semaphore(SEM_G_NAME, g2);
    destroy_semaphore(SEM_DRONE_NAME, g3);
    close(server_fd);
    close(drone_fd);
    unlink(SERVER_PIPE);
    unlink(DRONE_PIPE);
    printf("Resources cleaned up successfully.\n");
}

// Signal handler for graceful cleanup
void handle_sigint(int sig)
{
    printf("\nReceived SIGINT. Cleaning up resources...\n");
    cleanup_resources();
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
    if (child3_pid > 0)
    {
        printf("Terminating child process 3 (PID: %d)\n", child3_pid);
        kill(child3_pid, SIGTERM); // Gracefully terminate child 3
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
    if (child3_pid > 0)
    {
        waitpid(child3_pid, NULL, 0);
    }

    kill(globals->display_pid, SIGINT);
    kill(globals->map_pid, SIGINT);
    kill(globals->obstacles_pid, SIGINT);
    kill(globals->targets_pid, SIGINT);
    kill(globals->drone_pid, SIGINT);

    printf("All child processes terminated, resources cleaned up. Exiting.\n");
    exit(0);
}

void child1_task()
{
    printf("Child 1: Initializing Grid and Globals\n");

    memset(grid->grid, 0, sizeof(grid->grid));
    set_grid_point(grid, 1, 1, 254, DRONE, log_file);

    printf("Child 1: set Grid\n");

    globals->server_pid = getpid();

    printf("Child 1: set Globals\n");

    drone->drone_force = (struct force){0, 0};
    drone->user_force = (struct force){0, 0};
    drone->obs_force = (struct force){0, 0};
    drone->tar_force = (struct force){0, 0};
    drone->wall_force = (struct force){0, 0};
    drone->drone_vel = (struct vel){0, 0};
    drone->drone_pos = (struct pos){0, 0};

    printf("Child 1: set Drone\n");

    printf("Child 1: Initialization complete\n");
}

void child2_task()
{
    printf("Child 2: Reading from FIFO\n");
    char input;

    while (1)
    {
        if (read(server_fd, &input, sizeof(input)) > 0)
        {
            printf("Child 2: Server received %c\n", input);
            if (input == 'p')
            {
                kill(getppid(), SIGINT);
                break;
            }
            write(drone_fd, &input, sizeof(input));
        }
        usleep(10000);
    }
}

void child3_task()
{
    printf("Child 3: Resetting grid periodically\n");
    while (!globals->display_pid || !globals->drone_pid)
    {
        printf("Child 3: Waiting for PIDs...\n");
        printf("Child 3: display_pid: %d, drone_pid: %d\n",
               globals->display_pid, globals->drone_pid);
        sleep(10000);
    }

    while (!globals->targets_pid || !globals->obstacles_pid || !globals->map_pid)
    {
        printf("Waiting for PIDs...\n");
        printf("targets_pid: %d, obstacles_pid: %d, map_pid: %d\n",
               globals->targets_pid, globals->obstacles_pid, globals->map_pid);
        sleep(2); // Pause for 2 seconds
    }
    printf("Child 3: display_pid: %d, drone_pid: %d, targets_pid: %d, obstacles_pid: %d, map_pid: %d\n",
           globals->display_pid, globals->drone_pid, globals->targets_pid, globals->obstacles_pid, globals->map_pid);
    while (1)
    {
        kill(globals->targets_pid, SIGUSR1);
        kill(globals->obstacles_pid, SIGUSR1);

        printf("Child 3: Grid reset, sleeping...\n");
        sleep(120);
    }
}

int create_fifo(const char *fifo_path)
{
    if (mkfifo(fifo_path, 0666) == -1 && errno != EEXIST)
    {
        perror("Error creating FIFO");
        exit(1);
    }
    return 0;
}

void clear_grid_except_254(Grid *grid)
{
    for (int i = 0; i < GRID_WIDTH; i++)
    {
        for (int j = 0; j < GRID_HEIGHT; j++)
        {
            // If the point is not 254, set it to 0
            if (grid->grid[i][j] != 254)
            {
                grid->grid[i][j] = 0;
            }
        }
    }
}