#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include "display_utils.h"
#include "utils.h"
#include "globals.h"
// Paths for each FIFO
const char *display_fifo = "display_pipe";
const char *input_fifo = "input_pipe";
const char *dynamics_fifo = "dynamics_pipe";
int display_fd;
int input_fd;
int dynamics_fd;
WINDOW *win = NULL;
FILE *log_file = NULL;
// Process IDs for the monitored processes
pid_t input_pid = 0;
pid_t dynamics_pid = 0;
// Function to create a FIFO and handle errors
int create_fifo(const char *fifo_path)
{
    if (mkfifo(fifo_path, 0666) == -1)
    {
        if (errno != EEXIST)
        {
            perror("Error creating FIFO");
            exit(1);
        }
    }
    int fifo_fd = open(fifo_path, O_RDWR | O_NONBLOCK); // Open FIFO in non-blocking mode
    if (fifo_fd == -1)
    {
        perror("Error opening FIFO");
        exit(1);
    }
    return fifo_fd;
}
// Function to terminate all tasks and exit
void terminate_tasks()
{
    char log_msg[100];
    snprintf(log_msg, sizeof(log_msg), "Terminating tasks.");
    log_message(log_file, log_msg);
    kill(input_pid, SIGINT);
    kill(dynamics_pid, SIGINT);
}
// Function to terminate all tasks and exit
void terminate_tasks_and_clean(int sig)
{
    terminate_tasks();
    close(input_fd);
    close(dynamics_fd);
    close(display_fd);
    unlink(input_fifo);
    unlink(dynamics_fifo);
    unlink(display_fifo);
    // Close the log file
    fclose(log_file);
    exit(1);
}
// Signal handler for SIGUSR1
void handle_sigusr1(int sig)
{
    pid_t received_pid = 0;
    // Check and assign PIDs from each FIFO if they match the format
    if (input_pid == 0 && read(input_fd, &received_pid, sizeof(pid_t)) > 0)
    {
        if (received_pid != getpid())
        {
            input_pid = received_pid;
            log_message(log_file, "input PID received and set.");
            kill(input_pid, SIGUSR2);
        }
    }
    received_pid = 0;
    if (dynamics_pid == 0 && read(dynamics_fd, &received_pid, sizeof(pid_t)) > 0)
    {
        if (received_pid != getpid())
        {
            dynamics_pid = received_pid;
            log_message(log_file, "dynamics PID received and set.");
            kill(dynamics_pid, SIGUSR2);
        }
    }
}
int main()
{
    // Set up SIGUSR1 and SIGINT signal handlers
    signal(SIGUSR1, handle_sigusr1);
    // signal(SIGUSR2, SIG_IGN);
    signal(SIGINT, terminate_tasks_and_clean);
    // Open the log file
    FILE *log_file = initialize_log_file("parent_display_logs.txt");
    if (!log_file)
    {
        perror("Error opening log file");
        return 1;
    }
    log_message(log_file, "display.c started and waiting for PIDs...");
    // Initialize ncurses
    initscr();
    start_color();
    cbreak();
    noecho();
    curs_set(0); // Hide cursor
    // Define colors
    if (has_colors())
    {
        init_pair(1, COLOR_GREEN, COLOR_BLACK); // Green text, black background
        init_pair(2, COLOR_WHITE, COLOR_BLUE);  // Alternate pair
    }
    else
    {
        endwin();
        fprintf(stderr, "Your terminal does not support color.\n");
        exit(1);
    }
    // Set up the BIOS-like background
    setup_background();
    // // Open FIFOs for each process
    // input_fd = create_fifo(input_fifo);
    // dynamics_fd = create_fifo(dynamics_fifo);
    // display_fd = create_fifo(display_fifo);
    // // Send this program's PID to each monitored process
    // pid_t pid = getpid();
    // // Initial waiting loop until all PIDs are set
    // while (input_pid == 0 || dynamics_pid == 0)
    // {
    //     if (write(display_fd, &pid, sizeof(pid_t)) == -1)
    //     {
    //         perror("write to display_fifo failed");
    //         return 1;
    //     }
    //     sleep(1); // Wait for signals
    // }
    // log_message(log_file, "All PIDs received. Starting while loop...");
    create_input_window(&win);
    // Main loop
    while (1)
    {
        usleep(500000);
    }
    // End ncurses mode
    endwin();
    return 0;
}