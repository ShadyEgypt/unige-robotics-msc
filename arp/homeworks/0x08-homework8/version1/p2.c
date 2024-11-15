#include <ncurses.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

bool running = true; // Flag to keep the program running
bool confirmed = false;

WINDOW *win = NULL;

void handle_confirmed(int sig)
{
    confirmed = true;
    printf("received ack from watchdog!\n");
}

// Signal handler for SIGTERM
void handle_sigint(int sig)
{
    running = false; // Set the flag to false to exit the loop
    unlink("./italy_pipe");
    // Clean up ncurses
    delwin(win);
    endwin();
}

int main()
{
    // Set up SIGTERM signal handler
    signal(SIGINT, handle_sigint);
    signal(SIGUSR2, handle_confirmed);

    // Create a named pipe (FIFO)
    const char *fifo_path = "./egypt_pipe";
    const char *watchdog_fifo = "watchdog_pipe";
    printf("reading wd pid!\n");
    int wd_fd = open(watchdog_fifo, O_RDONLY | O_NONBLOCK);
    if (wd_fd == -1)
    {
        endwin(); // End ncurses mode before printing to stderr
        perror("Error opening FIFO");
        unlink(fifo_path); // Remove the FIFO if it couldn't be opened
        return 1;
    }
    // Get the PID of the watchdog
    pid_t watch_dog_pid;
    ssize_t read_bytes = read(wd_fd, &watch_dog_pid, sizeof(pid_t));
    if (read_bytes <= 0)
    {
        perror("Failed to read watchdog PID");
        close(wd_fd);
        return 1;
    }
    printf("Read watchdog PID: %d\n", watch_dog_pid);
    close(wd_fd);

    int fifo_fd = open(fifo_path, O_WRONLY);
    if (fifo_fd == -1)
    {
        endwin(); // End ncurses mode before printing to stderr
        perror("Error opening FIFO");
        unlink(fifo_path); // Remove the FIFO if it couldn't be opened
        return 1;
    }

    // Send my PID to the watchdog
    pid_t pid = getpid();
    ssize_t written_bytes = write(fifo_fd, &pid, sizeof(pid_t));
    if (written_bytes == -1)
    {
        perror("Failed to write PID to FIFO");
        close(fifo_fd);
        return 1;
    }
    kill(watch_dog_pid, SIGUSR1);
    printf("Sent my PID: %d to the watchdog\n", pid);

    while (!confirmed)
    {
        usleep(5);
    }
    // Initialize ncurses
    initscr();
    cbreak();
    noecho();
    curs_set(0);

    // Get screen size
    int screen_height, screen_width;
    getmaxyx(stdscr, screen_height, screen_width);

    // Define window size
    int win_height = 7;
    int win_width = 20;
    int start_y = (screen_height - win_height) / 2;
    int start_x = (screen_width - win_width) / 2;

    // Create a new window
    win = newwin(win_height, win_width, start_y, start_x);

    // Loop to update time every second, running until SIGTERM is received
    while (running)
    {

        // Clear the window and draw border
        werase(win);
        box(win, 0, 0);

        // Get current time in UTC
        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);

        // Adjust time to Rome time zone (UTC+1 or UTC+2 if daylight saving)
        timeinfo = gmtime(&rawtime);
        timeinfo->tm_hour += 2; // Rome time is UTC+1
        mktime(timeinfo);       // Adjust for possible overflow in hours

        // Display formatted time in the ncurses window
        mvwprintw(win, 1, (win_width - 11) / 2, "Egypt  Cairo");
        mvwprintw(win, 2, (win_width - 11) / 2, "############");
        mvwprintw(win, 4, (win_width - 9) / 2, "%02d/%02d/%04d", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
        mvwprintw(win, 5, (win_width - 8) / 2, "%02d:%02d:%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

        // Refresh the window to display the contents
        wrefresh(win); // Refresh only the window content
        refresh();     // Ensure the screen is refreshed

        // Prepare the time string for writing to the pipe
        char time_str[20];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

        // Write the time string to the named pipe
        // Write the time string to the named pipe (non-blocking)
        // No reader is connected, skip this write

        if (write(fifo_fd, time_str, strlen(time_str)) == -1)
        {
            if (errno == EAGAIN)
            {
                // No reader is connected, skip this write
                mvwprintw(win, 6, 1, "No reader connected to FIFO.");
                wrefresh(win);
                refresh();
            }
            else
            {
                perror("Error writing to FIFO");
                break;
            }
        }

        // Wait for 1 second
        sleep(1);
    }

    // Clean up: close the FIFO and delete it
    close(fifo_fd);
    unlink(fifo_path);

    // Clean up ncurses
    delwin(win);
    endwin();

    return 0;
}
