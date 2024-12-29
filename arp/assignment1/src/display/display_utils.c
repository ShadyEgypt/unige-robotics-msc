#include "display_utils.h"

FILE *log_file = NULL;
sem_t *sem_g = NULL;
sem_t *sem_drone = NULL;

bool resources_exist = false;
int fd = 0;
int input = 0;
int parent_height = 0, parent_width = 0;
int start_y_l = 0, start_x_l = 0, start_y_r = 0, start_x_r = 0;

Globals *globals = NULL;
Drone *drone = NULL;
WindowLayout layout;

pid_t child1_pid = -1;
pid_t child2_pid = -1;

// Create the outer border of the window
WINDOW *setup_win(int height, int width, int startx, int starty)
{
    WINDOW *local_win;

    local_win = newwin(height, width, starty, startx);
    box(local_win, 0, 0);
    return local_win;
}

// Destroy the map window
void destroy_win(WINDOW *local_win)
{
    wborder(local_win, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
    wrefresh(local_win);
    delwin(local_win);
}

void setup_resources()
{
    initscr();
    cbreak();
    noecho();
    curs_set(0);
    start_color();
    init_pair(1, COLOR_GREEN, COLOR_BLACK);
    log_message(log_file, "ncurses initialized!");

    int shm_drone_fd = attach_shared_memory(SHM_DRONE_NAME, SHM_DRONE_SIZE);
    void *drone_addr = map_shared_memory(shm_drone_fd, SHM_DRONE_SIZE);
    drone = (Drone *)drone_addr;

    int shm_g_fd = attach_shared_memory(SHM_G_NAME, SHM_G_SIZE);
    void *globals_addr = map_shared_memory(shm_g_fd, SHM_G_SIZE);
    globals = (Globals *)globals_addr;

    sem_drone = open_semaphore(SEM_DRONE_NAME);
    sem_g = open_semaphore(SEM_G_NAME);

    log_message(log_file, "shared memories got attached!");

    acquire_semaphore(sem_g);
    pid_t display_pid = getpid();
    globals->display_pid = display_pid;
    release_semaphore(sem_g);
    log_message(log_file, "wrote my pid in the shared memory globals!");

    layout.left_split = setup_win(LINES - 1, COLS / 2 - 1, 0, 0);

    layout.right_split = setup_win(LINES - 1, COLS / 2 - 1, COLS / 2, 0);

    getmaxyx(layout.left_split, parent_height, parent_width);

    start_y_l = (parent_height - grid_height) / 2;
    start_x_l = (parent_width - grid_width) / 2;
    start_y_r = (parent_height - 14) / 2;
    start_x_r = (parent_width - 15) / 2;

    layout.tl_win = setup_win(5, 7, start_y_l, start_x_l);
    layout.tc_win = setup_win(5, 7, start_y_l + 6, start_x_l);
    layout.tr_win = setup_win(5, 7, start_y_l + 12, start_x_l);
    layout.cl_win = setup_win(5, 7, start_y_l, start_x_l + 4);
    layout.cc_win = setup_win(5, 7, start_y_l + 6, start_x_l + 4);
    layout.cr_win = setup_win(5, 7, start_y_l + 12, start_x_l + 4);
    layout.bl_win = setup_win(5, 7, start_y_l, start_x_l + 8);
    layout.bc_win = setup_win(5, 7, start_y_l + 6, start_x_l + 8);
    layout.br_win = setup_win(5, 7, start_y_l + 12, start_x_l + 8);
    fd = open(SERVER_PIPE, O_WRONLY);
    if (fd == -1)
    {
        perror("Failed to open server pipe");
        exit(EXIT_FAILURE);
    }
    log_message(log_file, "opened fd in write mode!");
};
void child1_task()
{
    char log_msg[256];
    while (1)
    {
        getmaxyx(layout.left_split, parent_height, parent_width);

        start_x_l = (parent_height - grid_height) / 2;
        start_y_l = (parent_width - grid_width) / 2 + 1;
        start_y_r = (parent_height - 14) / 2;
        start_x_r = (parent_width - 15) / 2;
        destroy_win(layout.left_split);
        destroy_win(layout.right_split);
        layout.left_split = setup_win(LINES, COLS / 2 - 1, 0, 0);
        layout.right_split = setup_win(LINES, COLS / 2 - 1, COLS / 2, 0);
        layout.tl_win = setup_win(5, 7, start_y_l, start_x_l);
        layout.tc_win = setup_win(5, 7, start_y_l + 6, start_x_l);
        layout.tr_win = setup_win(5, 7, start_y_l + 12, start_x_l);
        layout.cl_win = setup_win(5, 7, start_y_l, start_x_l + 4);
        layout.cc_win = setup_win(5, 7, start_y_l + 6, start_x_l + 4);
        layout.cr_win = setup_win(5, 7, start_y_l + 12, start_x_l + 4);
        layout.bl_win = setup_win(5, 7, start_y_l, start_x_l + 8);
        layout.bc_win = setup_win(5, 7, start_y_l + 6, start_x_l + 8);
        layout.br_win = setup_win(5, 7, start_y_l + 12, start_x_l + 8);
        // Setting the "titles" of the splits
        mvwprintw(layout.left_split, 0, 1, "INPUT DISPLAY");
        mvwprintw(layout.right_split, 0, 1, "DYNAMICS DISPLAY");
        // snprintf(log_msg, sizeof(log_msg), "Input '%c' received", globals->input);
        // log_message(log_file, log_msg);
        format_left_win(globals->input, layout);
        format_right_win(drone, layout, start_y_r, start_x_r);
        // Refreshing all the windows
        wrefresh(layout.right_split);
        refresh_left_win(layout);
        sleep(2);
    }
};
void child2_task()
{
    while (1)
    {

        // Getting user input if present
        input = getch();
        // Check if the input is valid (not ERR and not a control character like ESC)
        if (input == 'p' || input == 'q' || input == 'w' || input == 'e' || input == 'a' || input == 's' || input == 'd' || input == 'z' || input == 'x' || input == 'c')
        {
            globals->input = input;
            // Valid input, write it to the pipe
            if (write(fd, &input, sizeof(input)) == -1)
            {
                perror("Failed to write to server pipe");
            }
        }
        else
        {
            // No valid input or ESC pressed, skip writing to pipe
            continue;
        }
        // Log the action of writing input
        char log_msg[256];
        snprintf(log_msg, sizeof(log_msg), "Input '%c' written to the pipe", input);
        log_message(log_file, log_msg);
        usleep(10000);
    }
};

void handle_sigint(int sig)
{
    printf("\nReceived SIGINT. Cleaning up resources...\n");
    if (child1_pid > 0)
    {
        printf("Terminating child process 1 (PID: %d)\n", child1_pid);
        kill(child1_pid, SIGTERM); // Gracefully terminate child 1
    }
    // Optionally wait for child processes to ensure they exit
    if (child1_pid > 0)
    {
        waitpid(child1_pid, NULL, 0);
    }

    if (child2_pid > 0)
    {
        printf("Terminating child process 1 (PID: %d)\n", child1_pid);
        kill(child1_pid, SIGTERM); // Gracefully terminate child 1
    }
    // Optionally wait for child processes to ensure they exit
    if (child2_pid > 0)
    {
        waitpid(child1_pid, NULL, 0);
    }
    if (resources_exist)
    {
        detach_shared_memory(drone, SHM_DRONE_SIZE);
        detach_shared_memory(globals, SHM_G_SIZE);
        printf("Shared memory detached.\n");
    }
    close(fd);
    destroy_win(layout.left_split);
    destroy_win(layout.right_split);
    endwin();
    exit(0); // Exit the program
}