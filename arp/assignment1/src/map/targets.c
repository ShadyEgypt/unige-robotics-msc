#include "map_utils.h"

// Global variables for cleanup
bool targets_resources_exist = false; // To store shared memory address
Grid *g1;
Globals *g2;
MapServerClient *g3;
sem_t *s1;
sem_t *s2;

// Signal handler function for SIGINT
void handle_sigint(int sig)
{
    printf("\nReceived SIGINT. Cleaning up resources...\n");

    if (targets_resources_exist)
    {
        detach_shared_memory(g1, SHM_GRID_SIZE);
        detach_shared_memory(g2, SHM_G_SIZE);
        printf("Shared memory detached and destroyed.\n");
    }

    exit(0); // Exit the program
}

void reset_targets_handler(int sig)
{
    // Lock semaphore before accessing shared memory
    acquire_semaphore(s1);
    printf("Semaphore locked!\n");
    reset_targets(g1);
    set_targets_randomly(g1, log_file);
    release_semaphore(s1);
    printf("Semaphore unlocked!\n");
    // // reset the map
    g3->request_id = 1;
    sprintf(g3->action, "reset_map");
    sprintf(g3->payload, ""); // No specific payload needed
    g3->is_valid = true;
    g3->has_response = false;
    int wait_time = 0;
    printf("[DEBUG] Waiting for reset_map response...\n");
}

int main()
{
    log_file = initialize_log_file("targets.txt");
    log_message(log_file, "Targets process started.");
    // Register signal handlers
    signal(SIGINT, handle_sigint);
    signal(SIGUSR1, reset_targets_handler);

    // Attach and map shared memory
    int shm_grid_fd = attach_shared_memory(SHM_GRID_NAME, SHM_GRID_SIZE);
    void *grid_addr = map_shared_memory(shm_grid_fd, SHM_GRID_SIZE);
    Grid *grid = (Grid *)grid_addr;

    int shm_g_fd = attach_shared_memory(SHM_G_NAME, SHM_G_SIZE);
    void *globals_addr = map_shared_memory(shm_g_fd, SHM_G_SIZE);
    Globals *globals = (Globals *)globals_addr;

    int shm_map_fd = attach_shared_memory(SHM_MAP_NAME, SHM_MAP_SIZE);
    void *map_addr = map_shared_memory(shm_map_fd, SHM_MAP_SIZE);
    MapServerClient *map_targets = (MapServerClient *)map_addr;

    srand(time(NULL));

    // Open semaphore
    sem_t *sem_grid = open_semaphore(SEM_GRID_NAME);
    sem_t *sem_g = open_semaphore(SEM_G_NAME);

    g1 = grid;
    g2 = globals;
    g3 = map_targets;
    s1 = sem_grid;
    s2 = sem_g;

    targets_resources_exist = true;

    acquire_semaphore(s2);
    pid_t targets_pid = getpid();
    g2->targets_pid = targets_pid;
    release_semaphore(s2);

    while (1)
    {
        sleep(20);
    }
    return 0;
}
