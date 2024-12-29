#include "map_utils.h"

FILE *log_file = NULL;
sem_t *sem_grid = NULL;
sem_t *sem_g = NULL;
sem_t *sem_drone = NULL;
sem_t *sem_map = NULL;

bool resources_exist = false;
Grid *grid = NULL;
Globals *globals = NULL;
Drone *drone = NULL;
MapServerClient *map = NULL;

pid_t child1_pid = -1;
pid_t child2_pid = -1;
WINDOW *map_window;
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

    int shm_map_fd = attach_shared_memory(SHM_MAP_NAME, SHM_MAP_SIZE);
    void *map_addr = map_shared_memory(shm_map_fd, SHM_MAP_SIZE);
    map = (MapServerClient *)map_addr;

    srand(time(NULL));

    // Open semaphore
    sem_grid = open_semaphore(SEM_GRID_NAME);
    sem_g = open_semaphore(SEM_G_NAME);
    sem_map = open_semaphore(SEM_MAP_NAME);
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

    map_window = setup_win(getmaxy(stdscr) - 1, getmaxx(stdscr) - 1, 1, 0);
    drone_window = setup_win(getmaxy(stdscr) - 1, getmaxx(stdscr) - 1, 1, 0);
    box(drone_window, 0, 0);
    wrefresh(drone_window);
    wrefresh(map_window);
}

void child1_task()
{
    char buffer_msg[256]; // Buffer for log messages
    printf("[DEBUG] Child1: Shared memory address: %p\n", (void *)map);

    while (1)
    {
        sprintf(buffer_msg, "[DEBUG] Child1: map->is_valid=%d, map->is_request_active=%d, action=%s",
                map->is_valid, map->is_request_active, map->action);
        log_message(log_file, buffer_msg);
        if (map->is_valid)
        {
            if (strcmp(map->action, "reset_map") == 0)
            {
                map->is_request_active = true;
                sprintf(buffer_msg, "child1_task: Received action 'reset_map'.");
                log_message(log_file, buffer_msg);

                reset_map(grid, map_window, drone_window);
                sprintf(map->response, "Map reset and redrawn.");

                sprintf(buffer_msg, "child1_task: Map reset and redrawn successfully.");
                log_message(log_file, buffer_msg);
                map->is_valid = false;
                map->is_request_active = false;
                map->is_response_ready = true;
                strncpy(map->action, "", sizeof(map->action) - 1);
            }
            else if (strcmp(map->action, "move_drone") == 0)
            {
                map->is_request_active = true;
                sprintf(buffer_msg, "child1_task: Received action 'move_drone'. Parsing payload.");
                log_message(log_file, buffer_msg);

                int old_x, old_y, new_x, new_y;
                sscanf(map->payload, "{\"old_x\":%d,\"old_y\":%d,\"new_x\":%d,\"new_y\":%d}", &old_x, &old_y, &new_x, &new_y);
                move_drone(map_window, drone_window, old_x, old_y, new_x, new_y);
                sprintf(map->response, "Drone moved successfully.");

                sprintf(buffer_msg, "child1_task: Drone moved successfully from (%d, %d) to (%d, %d).", old_x, old_y, new_x, new_y);
                log_message(log_file, buffer_msg);
                map->is_valid = false;
                map->is_request_active = false;
                map->is_response_ready = true;
            }
            else if (strcmp(map->action, "drone_hit_target") == 0)
            {
                map->is_request_active = true;
                sprintf(buffer_msg, "child1_task: Received action 'drone_hit_target'. Parsing payload.");
                log_message(log_file, buffer_msg);

                int old_x, old_y, new_x, new_y, new_score;
                sscanf(map->payload, "{\"old_x\":%d,\"old_y\":%d,\"new_x\":%d,\"new_y\":%d,\"score\":%d}", &old_x, &old_y, &new_x, &new_y, &new_score);
                drone_hit_target(map_window, drone_window, old_x, old_y, new_x, new_y, new_score);
                sprintf(map->response, "Target hit, score updated.");

                sprintf(buffer_msg, "child1_task: Target hit at (%d, %d). New score: %d", new_x, new_y, new_score);
                log_message(log_file, buffer_msg);
                map->is_valid = false;
                map->is_request_active = false;
                map->is_response_ready = true;
            }
            else if (strcmp(map->action, "drone_hit_obstacle") == 0)
            {
                map->is_request_active = true;
                sprintf(buffer_msg, "child1_task: Received action 'drone_hit_obstacle'. Parsing payload.");
                log_message(log_file, buffer_msg);

                int old_x, old_y, new_x, new_y, new_score;
                sscanf(map->payload, "{\"old_x\":%d,\"old_y\":%d,\"new_x\":%d,\"new_y\":%d,\"score\":%d}", &old_x, &old_y, &new_x, &new_y, &new_score);
                drone_hit_obstacle(map_window, drone_window, old_x, old_y, new_x, new_y, new_score);
                sprintf(map->response, "Obstacle hit, drone marked in red.");

                sprintf(buffer_msg, "child1_task: Obstacle hit at (%d, %d). Drone marked in red.", new_x, new_y);
                log_message(log_file, buffer_msg);
                map->is_valid = false;
                map->is_request_active = false;
                map->is_response_ready = true;
            }
            else
            {
                sprintf(map->response, "Unknown action '%s' received.", map->action);
                sprintf(buffer_msg, "child1_task: Unknown action '%s' received.", map->action);
                log_message(log_file, buffer_msg);
                map->is_valid = false;
                map->is_request_active = false;
                map->is_response_ready = true;
            }
            map->status = REQUEST_PROCESSED;
            map->is_valid = false;
            sprintf(buffer_msg, "child1_task: Request processed. Response ready.");
            log_message(log_file, buffer_msg);
        }
        else
        {
            sprintf(buffer_msg, "child1_task: Map is either invalid or no active request. Skipping iteration.");
            log_message(log_file, buffer_msg);
            sleep(1);
        }

        sleep(1);
    }
}

void child2_task()
{
    // Local variables for current screen size
    int current_height, current_width;
    int prev_height = -1, prev_width = -1; // Initialize with invalid dimensions
    char log_buffer[256];
    printf("[DEBUG] Child2: Shared memory address: %p\n", (void *)map);
    while (1)
    {
        // Get the current window size
        getmaxyx(stdscr, current_height, current_width);
        grid->grid_actual_height = current_height;
        grid->grid_actual_width = current_width;
        // Check if the screen size has changed
        if (current_height != prev_height || current_width != prev_width)
        {
            snprintf(log_buffer, sizeof(log_buffer),
                     "Child2: Screen size changed (Previous: %dx%d, Current: %dx%d). Triggering reset_map.",
                     prev_width, prev_height, current_width, current_height);
            log_message(log_file, log_buffer);

            // Prepare and send reset_map request
            if (!map->is_request_active)
            {
                // Acquire semaphore to ensure exclusive access to shared memory
                // acquire_semaphore(sem_map);
                map->request_id = 1;
                strncpy(map->action, "reset_map", sizeof(map->action) - 1);
                snprintf(map->payload, sizeof(map->payload), "{\"width\":%d,\"height\":%d}", current_width, current_height);
                map->is_valid = true;
                map->is_request_active = true;
                map->has_response = false;

                snprintf(log_buffer, sizeof(log_buffer),
                         "Child2: reset_map request prepared. map->is_valid=%d, map->is_request_active=%d, action=%s, payload=%s",
                         map->is_valid, map->is_request_active, map->action, map->payload);
                log_message(log_file, log_buffer);

                // release_semaphore(sem_map);

                // Wait for response
                int wait_time = 0;
                while (!map->is_response_ready && wait_time < TIMEOUT_SECONDS)
                {
                    sleep(1);
                    wait_time++;
                }

                if (!map->is_response_ready)
                {
                    snprintf(log_buffer, sizeof(log_buffer),
                             "Child2: Timeout after %d seconds. No response received.", TIMEOUT_SECONDS);
                    log_message(log_file, log_buffer);
                }
                else
                {
                    snprintf(log_buffer, sizeof(log_buffer), "Child2: Response received: %s", map->response);
                    log_message(log_file, log_buffer);

                    // acquire_semaphore(sem_map);
                    map->has_response = false;
                    map->is_response_ready = false;
                    strncpy(map->response, "", sizeof(map->response) - 1);
                    // release_semaphore(sem_map);
                    // Update previous dimensions
                    prev_height = current_height;
                    prev_width = current_width;
                }
            }
        }
        else
        {
            // No change detected, log it at a lower frequency to prevent log spamming
            static int log_counter = 0;
            if (log_counter++ % 5 == 0) // Log every 5 iterations
            {
                snprintf(log_buffer, sizeof(log_buffer),
                         "Child2: Screen size unchanged (Current: %dx%d). No reset_map triggered.",
                         current_width, current_height);
                log_message(log_file, log_buffer);
            }
        }

        // Sleep briefly to prevent excessive polling
        usleep(500000); // 500ms delay
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
        detach_shared_memory(map, SHM_MAP_SIZE);
        printf("Shared memory detached and destroyed.\n");
    }
    // Cleanup ncurses
    delwin(map_window);
    endwin();

    exit(0); // Exit the program
}
// targets and obstacles
void reset_targets(Grid *grid)
{
    grid->target_count = 0;                                 // Reset target count
    memset(grid->targets, 0, sizeof(Target) * MAX_TARGETS); // Clear targets array
    printf("All targets have been reset.\n");
}

void reset_obstacles(Grid *grid)
{
    grid->obstacle_count = 0;                                     // Reset obstacle count
    memset(grid->obstacles, 0, sizeof(Obstacle) * MAX_OBSTACLES); // Clear obstacles array
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

        if (nx >= 0 && nx < GRID_WIDTH && ny >= 0 && ny < GRID_HEIGHT)
        {
            if (is_point_occupied(grid, nx, ny))
            {
                return true; // Adjacent point is occupied
            }
        }
    }
    return false; // No adjacent points occupied
}

void reset_map(Grid *grid, WINDOW *map_window, WINDOW *drone_window)
{
    char log_buffer[256];
    // log_message(log_file, "[DEBUG] reset_map: Starting map reset...");

    // End ncurses session
    // log_message(log_file, "[DEBUG] reset_map: Calling endwin()...");
    endwin();

    // Refresh terminal
    // log_message(log_file, "[DEBUG] reset_map: Calling refresh()...");
    refresh();

    // Clear terminal screen
    // log_message(log_file, "[DEBUG] reset_map: Calling clear()...");
    clear();

    // Recreate the map window
    // log_message(log_file, "[DEBUG] reset_map: Recreating map window...");
    map_window = setup_win(LINES - 1, COLS - 1, 1, 0);

    // Clear and refresh map window
    // log_message(log_file, "[DEBUG] reset_map: Clearing and refreshing map window...");
    wclear(map_window);
    wrefresh(map_window);

    // Print UI elements
    // log_message(log_file, "[DEBUG] reset_map: Printing UI elements...");
    mvprintw(0, 3, "MAP DISPLAY");
    mvprintw(0, 25, "Score: %d", grid->score);
    mvprintw(0, 50, "Press Ctrl+C to exit.");
    refresh();

    // Draw Targets
    for (int i = 0; i < grid->target_count; i++)
    {
        Target target = grid->targets[i];
        if (target.id)
        {
            int screen_x = 1 + target.x * (getmaxx(map_window) - 3) / GRID_WIDTH;
            int screen_y = 1 + target.y * (getmaxy(map_window) - 3) / GRID_HEIGHT;
            wattron(map_window, COLOR_PAIR(1));
            mvwprintw(map_window, screen_y, screen_x, "%d", target.id);
            wattroff(map_window, COLOR_PAIR(1));
            if (target.is_two_digit)
            {
                // Split two-digit target into two digits
                int first_digit = target.id / 10;
                int second_digit = target.id % 10;
                grid->grid[screen_y][screen_x] = first_digit;
                grid->grid[screen_y][screen_x + 1] = second_digit;
            }
            else
            {
                // Single-digit target drawing
                grid->grid[screen_y][screen_x] = target.id;
            }
        }
        else
        {
            log_message(log_file, "[DEBUG] reset_map: Target ID is 0. Skipping drawing.");
        }
    }

    // Draw Obstacles
    for (int i = 0; i < grid->obstacle_count; i++)
    {
        Obstacle obstacle = grid->obstacles[i];
        int screen_x = 1 + obstacle.x * (getmaxx(map_window) - 3) / GRID_WIDTH;
        int screen_y = 1 + obstacle.y * (getmaxy(map_window) - 3) / GRID_HEIGHT;
        grid->grid[screen_y][screen_x] = 255;
        wattron(map_window, COLOR_PAIR(2));
        mvwprintw(map_window, screen_y, screen_x, "O");
        wattroff(map_window, COLOR_PAIR(2));
    }

    // Draw Drone (Assuming single drone at specific coordinates)
    int drone_x = 1 + grid->drone_pos.x * (getmaxx(map_window) - 3) / GRID_WIDTH;
    int drone_y = 1 + grid->drone_pos.y * (getmaxy(map_window) - 3) / GRID_HEIGHT;
    grid->grid[drone_y][drone_x] = 254;
    wattron(drone_window, COLOR_PAIR(3));
    mvwprintw(drone_window, drone_y, drone_x, "+");
    wattroff(drone_window, COLOR_PAIR(3));

    // Refresh the window
    wrefresh(map_window);
    wrefresh(drone_window);
    refresh();
    log_message(log_file, "[DEBUG] reset_map: Map window recreated and refreshed successfully.");
}

void move_drone(WINDOW *map_window, WINDOW *drone_window, int old_x, int old_y, int new_x, int new_y)
{
    // Clear old drone position
    int screen_old_x = 1 + old_x * (getmaxx(drone_window) - 3) / GRID_WIDTH;
    int screen_old_y = 1 + old_y * (getmaxy(drone_window) - 3) / GRID_HEIGHT;
    mvwprintw(drone_window, screen_old_y, screen_old_x, " ");

    // Print the drone in the new position
    int screen_new_x = 1 + new_x * (getmaxx(drone_window) - 3) / GRID_WIDTH;
    int screen_new_y = 1 + new_y * (getmaxy(drone_window) - 3) / GRID_HEIGHT;
    wattron(drone_window, COLOR_PAIR(3)); // Drone color
    mvwprintw(drone_window, screen_new_y, screen_new_x, "+");
    wattroff(drone_window, COLOR_PAIR(3));

    // Refresh the window
    wrefresh(drone_window);
}

void drone_hit_target(WINDOW *map_window, WINDOW *drone_window, int old_x, int old_y, int new_x, int new_y, int score)
{
    // Clear old drone position
    int screen_old_x = 1 + old_x * (getmaxx(drone_window) - 3) / GRID_WIDTH;
    int screen_old_y = 1 + old_y * (getmaxy(drone_window) - 3) / GRID_HEIGHT;
    mvwprintw(drone_window, screen_old_y, screen_old_x, " ");

    // Print the drone in the new position
    int screen_new_x = 1 + new_x * (getmaxx(drone_window) - 3) / GRID_WIDTH;
    int screen_new_y = 1 + new_y * (getmaxy(drone_window) - 3) / GRID_HEIGHT;
    // Clear the target
    mvwprintw(map_window, screen_new_y, screen_new_x, " ");
    // Print the drone
    wattron(drone_window, COLOR_PAIR(3)); // Drone color
    mvwprintw(drone_window, screen_new_y, screen_new_x, "+");
    wattroff(drone_window, COLOR_PAIR(3));

    // Update the score on the screen
    mvprintw(map_window, 0, 25, "Score: %d", grid->score);

    // Refresh both the map and the main screen
    wrefresh(map_window);
    wrefresh(drone_window);
}

void drone_hit_obstacle(WINDOW *map_window, WINDOW *drone_window, int old_x, int old_y, int new_x, int new_y, int score)
{
    // Clear old drone position
    int screen_old_x = 1 + old_x * (getmaxx(drone_window) - 3) / GRID_WIDTH;
    int screen_old_y = 1 + old_y * (getmaxy(drone_window) - 3) / GRID_HEIGHT;
    mvwprintw(drone_window, screen_old_y, screen_old_x, " ");

    // Print the drone in RED at the new position
    int screen_new_x = 1 + new_x * (getmaxx(map_window) - 3) / GRID_WIDTH;
    int screen_new_y = 1 + new_y * (getmaxy(map_window) - 3) / GRID_HEIGHT;
    wattron(map_window, COLOR_PAIR(4)); // Red color for drone collision
    mvwprintw(map_window, screen_new_y, screen_new_x, "+");
    wattroff(map_window, COLOR_PAIR(4));

    // Update the score on the screen
    mvprintw(map_window, 0, 25, "Score: %d", grid->score);

    // Refresh both the map and the main screen
    wrefresh(map_window);
    wrefresh(drone_window);
    refresh();
}

void set_obstacles_randomly(Grid *grid, FILE *log_file)
{
    srand(time(NULL) + 1); // Slightly different seed for randomness

    int placed_obstacles = 0;
    while (placed_obstacles < MAX_OBSTACLES)
    {
        int x = rand() % GRID_WIDTH;
        int y = rand() % GRID_HEIGHT;

        // Skip (1,1) and ensure no adjacent points are occupied
        if ((x == 1 && y == 1) || is_adjacent_occupied(grid, x, y))
        {
            continue; // Retry with a new random position
        }

        // Check if the cell is free
        if (!is_point_occupied(grid, x, y))
        {
            set_grid_point(grid, x, y, 255, OBSTACLE, log_file); // 255 indicates an obstacle
            placed_obstacles++;
        }
    }

    char log_buffer[256];
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

    while (placed_targets < MAX_TARGETS)
    {
        int x = rand() % GRID_WIDTH;
        int y = rand() % GRID_HEIGHT;

        snprintf(log_buffer, sizeof(log_buffer), "Checking position (%d, %d)", x, y);
        log_message(log_file, log_buffer);

        // Skip invalid or occupied positions
        if ((x == 1 && y == 1) || is_adjacent_occupied(grid, x, y) || is_point_occupied(grid, x, y))
        {
            snprintf(log_buffer, sizeof(log_buffer), "Skipping invalid position (%d, %d)", x, y);
            log_message(log_file, log_buffer);
            continue;
        }

        bool is_two_digit = (target_id >= 10); // Two-digit starts from 10

        if (is_two_digit && x + 1 < GRID_WIDTH &&
            !is_point_occupied(grid, x + 1, y) &&
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
        else if (!is_two_digit && !is_point_occupied(grid, x, y))
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