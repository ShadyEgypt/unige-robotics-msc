#include "map_utils.h"

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
WINDOW *target_window;
WINDOW *obstacle_window;
WINDOW *drone_window;

WINDOW *setup_win(int height, int width, int startx, int starty)
{
    WINDOW *local_win;

    local_win = newwin(height, width, starty, startx);
    box(local_win, 0, 0);
    return local_win;
}

void destroy_win(WINDOW *local_win)
{
    wborder(local_win, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
    wrefresh(local_win);
    delwin(local_win);
}

// map functions
void setup_resources()
{
    // Attach and map shared memory
    int shm_grid_fd = attach_shared_memory(SHM_GRID_NAME, SHM_GRID_SIZE);
    void *grid_addr = map_shared_memory(shm_grid_fd, SHM_GRID_SIZE);
    grid = (Grid *)grid_addr;

    int shm_g_fd = attach_shared_memory(SHM_G_NAME, SHM_G_SIZE);
    void *globals_addr = map_shared_memory(shm_g_fd, SHM_G_SIZE);
    globals = (Globals *)globals_addr;

    int shm_config_fd = attach_shared_memory(SHM_CONFIG_NAME, SHM_CONFIG_SIZE);
    void *config_addr = map_shared_memory(shm_config_fd, SHM_CONFIG_SIZE);
    config = (Config *)config_addr;
    srand(time(NULL));

    // Open semaphore
    sem_grid = open_semaphore(SEM_GRID_NAME);
    sem_g = open_semaphore(SEM_G_NAME);
    // g1 = sem_grid;
    // g2 = sem_g;
    // g_grid = grid;

    resources_exist = true;

    acquire_semaphore(sem_g);
    pid_t map_pid = getpid();
    globals->map_pid = map_pid;
    release_semaphore(sem_g);

    // Setting up ncurses
    initscr();
    cbreak();
    curs_set(0);
    start_color();
    init_color(10, 1000, 647, 0);
    init_pair(1, COLOR_GREEN, COLOR_BLACK);
    init_pair(2, 10, COLOR_BLACK);
    init_pair(3, COLOR_BLUE, COLOR_BLACK);
    init_pair(4, COLOR_RED, COLOR_BLACK);

    target_window = setup_win(getmaxy(stdscr) - 1, getmaxx(stdscr) - 1, 1, 0);
    obstacle_window = setup_win(getmaxy(stdscr) - 1, getmaxx(stdscr) - 1, 1, 0);
    drone_window = setup_win(getmaxy(stdscr) - 1, getmaxx(stdscr) - 1, 1, 0);
    box(drone_window, 0, 0);
    wrefresh(drone_window);
    wrefresh(target_window);
    wrefresh(obstacle_window);
}

void child1_task()
{
    char buffer_msg[256]; // Buffer for log messages
    // printf("[DEBUG] Child1: Shared memory address: %p\n", (void *)map);
    int x, y;
    while (1)
    {
        target_window = setup_win(LINES - 1, COLS - 1, 1, 0);
        obstacle_window = setup_win(LINES - 1, COLS - 1, 1, 0);
        drone_window = setup_win(LINES - 1, COLS - 1, 1, 0);

        mvprintw(0, 3, "MAP DISPLAY");
        mvprintw(0, 25, "Score: %d", grid->score);
        mvprintw(0, 50, "Press Ctrl+C to exit.");
        box(drone_window, 0, 0);
        wrefresh(target_window);
        wrefresh(obstacle_window);
        wrefresh(drone_window);
        // Draw Targets
        for (int i = 0; i < grid->target_count; i++)
        {
            Target target = grid->targets[i];
            x = target.x;
            y = target.y;

            int screen_x = 1 + x * (getmaxx(target_window) - 3) / config->Map.Size.Width;
            int screen_y = 1 + y * (getmaxy(target_window) - 3) / config->Map.Size.Height;
            wattron(target_window, COLOR_PAIR(1));
            mvwprintw(target_window, screen_y, screen_x, "%d", target.id);
            wattroff(target_window, COLOR_PAIR(1));
            if (target.is_two_digit)
            {
                // Split two-digit target into two digits
                int first_digit = target.id / 10;
                int second_digit = target.id % 10;
                grid->grid[y][x] = first_digit;
                grid->grid[y][x + 1] = second_digit;
            }
            else
            {
                // Single-digit target drawing
                grid->grid[y][x] = target.id;
            }
        }
        wrefresh(target_window);

        // Draw Obstacles
        for (int i = 0; i < grid->obstacle_count; i++)
        {
            Obstacle obstacle = grid->obstacles[i];
            // x = obstacle.x;
            // y = obstacle.y;
            int screen_x = 1 + obstacle.x * (getmaxx(obstacle_window) - 3) / config->Map.Size.Width;
            int screen_y = 1 + obstacle.y * (getmaxy(obstacle_window) - 3) / config->Map.Size.Height;

            grid->grid[obstacle.y][obstacle.x] = 255;
            wattron(obstacle_window, COLOR_PAIR(2));
            mvwprintw(obstacle_window, screen_y, screen_x, "O");
            wattroff(obstacle_window, COLOR_PAIR(2));
            sprintf(buffer_msg, "Obstacle %d at grid (%d, %d) mapped to screen (%d, %d)",
                    i, obstacle.x, obstacle.y, screen_x, screen_y);
            log_message(log_file, buffer_msg);
        }
        wrefresh(obstacle_window);

        int drone_x = grid->drone_pos.x;
        int drone_y = grid->drone_pos.y;
        // Draw Drone (Assuming single drone at specific coordinates)
        int drone_screen_x = 1 + grid->drone_pos.x * (getmaxx(drone_window) - 3) / config->Map.Size.Width;
        int drone_screen_y = 1 + grid->drone_pos.y * (getmaxy(drone_window) - 3) / config->Map.Size.Height;
        grid->grid[drone_x][drone_y] = 254;
        wattron(drone_window, COLOR_PAIR(3));
        mvwprintw(drone_window, drone_screen_y, drone_screen_x, "+");
        wattroff(drone_window, COLOR_PAIR(3));
        // log_grid(grid, log_file);
        // switch case
        // Refresh the window
        wrefresh(drone_window);
        refresh();
        sleep(1);
    }
}

void child2_task()
{
    // Local variables for current screen size
    int current_height, current_width;
    int prev_height = -1, prev_width = -1; // Initialize with invalid dimensions
    char log_buffer[256];
    // printf("[DEBUG] Child2: Shared memory address: %p\n", (void *)map);
    while (1)
    {
        // Get the current window size
        getmaxyx(stdscr, current_height, current_width);
        grid->grid_actual_height = current_height;
        grid->grid_actual_width = current_width;

        usleep(5000000); // 5000ms delay
    }
}

void handle_sigint_map(int sig)
{
    printf("\nReceived SIGINT. Cleaning up resources...\n");
    if (child1_pid > 0)
    {
        printf("Terminating child process 1 (PID: %d)\n", child1_pid);
        kill(child1_pid, SIGTERM); // Gracefully terminate child 1
    }
    if (child2_pid > 0)
    {
        printf("Terminating child process 1 (PID: %d)\n", child2_pid);
        kill(child2_pid, SIGTERM); // Gracefully terminate child 1
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
        printf("Shared memory detached and destroyed.\n");
    }
    // Cleanup ncurses
    delwin(target_window);
    delwin(obstacle_window);
    delwin(drone_window);
    endwin();

    exit(0); // Exit the program
}
// targets and obstacles
void reset_targets(Grid *grid)
{
    grid->target_count = 0;                                                     // Reset target count
    memset(grid->targets, 0, sizeof(Target) * config->Map.MaxEntities.Targets); // Clear targets array
    printf("All targets have been reset.\n");
}

void reset_obstacles(Grid *grid)
{
    grid->obstacle_count = 0;                                                         // Reset obstacle count
    memset(grid->obstacles, 0, sizeof(Obstacle) * config->Map.MaxEntities.Obstacles); // Clear obstacles array
    printf("All obstacles have been reset.\n");
}

// Helper function to check if adjacent cells are occupied
bool is_adjacent_occupied(Grid *grid, int x, int y)
{
    int dx[] = {-1, 1, 0, 0}; // Left, Right, Up, Down
    int dy[] = {0, 0, -1, 1};

    for (int i = 0; i < 4; i++)
    {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (nx >= 0 && nx < config->Map.Size.Width && ny >= 0 && ny < config->Map.Size.Height)
        {
            if (is_point_occupied(grid, nx, ny, config->Map.Size.Height, config->Map.Size.Width))
            {
                return true; // Adjacent point is occupied
            }
        }
    }
    return false; // No adjacent points occupied
}

void set_obstacles_randomly(Grid *grid, FILE *log_file)
{
    char log_buffer[256];
    srand(time(NULL) + 1); // Slightly different seed for randomness

    int placed_obstacles = 0;
    while (placed_obstacles < config->Map.MaxEntities.Obstacles)
    {
        int x = rand() % config->Map.Size.Width;
        int y = rand() % config->Map.Size.Height;

        if (is_point_occupied(grid, x, y, config->Map.Size.Height, config->Map.Size.Width) || is_adjacent_occupied(grid, x, y))
        {
            continue; // Skip occupied or adjacent points
        }

        grid->obstacles[placed_obstacles].x = x;
        grid->obstacles[placed_obstacles].y = y;
        grid->obstacle_count++;

        sprintf(log_buffer, "Placed obstacle %d at (%d, %d)", placed_obstacles, x, y);
        log_message(log_file, log_buffer);
        placed_obstacles++;
    }

    sprintf(log_buffer, "Obstacle Count: %d", grid->obstacle_count);
    log_message(log_file, log_buffer);

    sprintf(log_buffer, "%d obstacles have been randomly placed on the grid.", placed_obstacles);
    log_message(log_file, log_buffer);
}

void set_targets_randomly(Grid *grid, FILE *log_file)
{
    int placed_targets = 0;
    int target_id = 1;
    char log_buffer[512];
    srand(time(NULL)); // Seed the random number generator

    log_message(log_file, "Starting target placement...");

    while (placed_targets < config->Map.MaxEntities.Targets)
    {
        int x = rand() % config->Map.Size.Width;
        int y = rand() % config->Map.Size.Height;

        snprintf(log_buffer, sizeof(log_buffer), "Checking position (%d, %d)", x, y);
        log_message(log_file, log_buffer);

        // Skip invalid or occupied positions
        if ((x == 1 && y == 1) || is_adjacent_occupied(grid, x, y) || is_point_occupied(grid, x, y, config->Map.Size.Height, config->Map.Size.Width))
        {
            snprintf(log_buffer, sizeof(log_buffer), "Skipping invalid position (%d, %d)", x, y);
            log_message(log_file, log_buffer);
            continue;
        }

        bool is_two_digit = (target_id >= 10); // Two-digit starts from 10

        if (is_two_digit && x + 1 < config->Map.Size.Width &&
            !is_point_occupied(grid, x + 1, y, config->Map.Size.Height, config->Map.Size.Width) &&
            !is_adjacent_occupied(grid, x + 1, y))
        {
            // Two-digit target placement
            int first_digit = target_id / 10;
            int second_digit = target_id % 10;

            // Store only one entry in the targets array
            grid->targets[placed_targets].x = x;
            grid->targets[placed_targets].y = y;
            grid->targets[placed_targets].id = target_id;
            grid->targets[placed_targets].is_two_digit = true;

            snprintf(log_buffer, sizeof(log_buffer),
                     "Two-digit target %d placed at (%d, %d) and (%d, %d).",
                     target_id, x, y, x + 1, y);
            log_message(log_file, log_buffer);

            target_id++;
            placed_targets++;
        }
        else if (!is_two_digit && !is_point_occupied(grid, x, y, config->Map.Size.Height, config->Map.Size.Width))
        {

            // Store one entry in the targets array
            grid->targets[placed_targets].x = x;
            grid->targets[placed_targets].y = y;
            grid->targets[placed_targets].id = target_id;
            grid->targets[placed_targets].is_two_digit = false;

            snprintf(log_buffer, sizeof(log_buffer),
                     "Single-digit target %d placed at (%d, %d).",
                     target_id, x, y);
            log_message(log_file, log_buffer);

            target_id++;
            placed_targets++;
        }

        snprintf(log_buffer, sizeof(log_buffer),
                 "%d targets (single and two-digit) have been sequentially placed on the grid.",
                 placed_targets);
        log_message(log_file, log_buffer);

        log_message(log_file, "Target placement complete.");
        grid->target_count += 1;
    }
}

void log_grid(Grid *grid, FILE *log_file)
{
    if (!grid || !log_file)
    {
        fprintf(stderr, "Invalid grid or log file pointer.\n");
        return;
    }

    // Log grid dimensions and metadata
    fprintf(log_file, "Logging Grid State:\n");
    fprintf(log_file, "Grid Dimensions: %d x %d\n", config->Map.Size.Height, config->Map.Size.Width);
    fprintf(log_file, "Obstacle Count: %d, Target Count: %d\n", grid->obstacle_count, grid->target_count);
    fprintf(log_file, "Drone Position: (%.2f, %.2f)\n", grid->drone_pos.x, grid->drone_pos.y);
    fprintf(log_file, "+"); // Top-left corner of the grid border
    for (int i = 0; i < config->Map.Size.Width; i++)
    {
        fprintf(log_file, "-");
    }
    fprintf(log_file, "+\n"); // Top-right corner of the grid border

    // Loop through the grid and log each row
    for (int y = 0; y < config->Map.Size.Height; y++)
    {
        fprintf(log_file, "|"); // Left border of the row
        for (int x = 0; x < config->Map.Size.Width; x++)
        {
            switch (grid->grid[y][x])
            {
            case 0: // Empty cell
                fprintf(log_file, " ");
                break;
            case 254: // Drone
                fprintf(log_file, "D");
                break;
            case 255: // Obstacle
                fprintf(log_file, "O");
                break;
            default: // Target or other values
                if (grid->grid[y][x] >= 1 && grid->grid[y][x] <= 9)
                    fprintf(log_file, "%d", grid->grid[y][x]);
                else
                    fprintf(log_file, "?"); // Unknown value
                break;
            }
        }
        fprintf(log_file, "|\n"); // Right border of the row
    }

    // Bottom border
    fprintf(log_file, "+");
    for (int i = 0; i < config->Map.Size.Width; i++)
    {
        fprintf(log_file, "-");
    }
    fprintf(log_file, "+\n"); // Bottom-right corner of the grid border

    // Log completion
    fprintf(log_file, "Grid log completed.\n\n");
    fflush(log_file); // Ensure the log is written immediately
}
