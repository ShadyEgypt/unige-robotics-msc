#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <string.h>

#define TIMEOUT 5 // Timeout in seconds

// Paths for each FIFO
const char *watchdog_fifo = "watchdog_pipe";
const char *egypt_fifo = "egypt_pipe";
const char *italy_fifo = "italy_pipe";
const char *japan_fifo = "japan_pipe";

int wd_fd;
int egypt_fd;
int italy_fd;
int japan_fd;

// Process IDs for the monitored processes
pid_t egypt_pid = 0;
pid_t italy_pid = 0;
pid_t japan_pid = 0;

FILE *log_file; // Log file pointer

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

// Function to log messages to the log file with a timestamp
void log_message(const char *message)
{
    time_t now = time(NULL);
    char *timestamp = ctime(&now);
    timestamp[strlen(timestamp) - 1] = '\0'; // Remove newline
    fprintf(log_file, "[%s] %s\n", timestamp, message);
    fflush(log_file); // Ensure message is written immediately
}

// Function to terminate all tasks and exit
void terminate_tasks_and_exit(const char *task_name)
{
    char log_msg[100];
    snprintf(log_msg, sizeof(log_msg), "Timeout detected in %s. Terminating tasks.", task_name);
    log_message(log_msg);

    if (egypt_pid != 0)
        kill(egypt_pid, SIGINT);
    if (italy_pid != 0)
        kill(italy_pid, SIGINT);
    if (japan_pid != 0)
        kill(japan_pid, SIGINT);

    fclose(log_file);
    exit(1);
}

// Function to terminate all tasks and exit
void terminate_task(int sig)
{
    terminate_tasks_and_exit("Egypt");
    terminate_tasks_and_exit("Italy");
    terminate_tasks_and_exit("Japan");
    close(egypt_fd);
    close(italy_fd);
    close(japan_fd);
    close(wd_fd);
    unlink(egypt_fifo);
    unlink(italy_fifo);
    unlink(japan_fifo);
    unlink(watchdog_fifo);

    // Close the log file
    fclose(log_file);
    exit(1);
}

// Signal handler for SIGUSR1
void handle_sigusr1(int sig)
{
    pid_t received_pid = 0;

    // Check and assign PIDs from each FIFO if they match the format
    if (egypt_pid == 0 && read(egypt_fd, &received_pid, sizeof(pid_t)) > 0)
    {
        if (received_pid != getpid())
        {
            egypt_pid = received_pid;
            log_message("Egypt PID received and set.");
            kill(egypt_pid, SIGUSR2);
        }
    }
    received_pid = 0;
    if (italy_pid == 0 && read(italy_fd, &received_pid, sizeof(pid_t)) > 0)
    {
        if (received_pid != getpid())
        {
            italy_pid = received_pid;
            log_message("Italy PID received and set.");
            kill(italy_pid, SIGUSR2);
        }
    }
    received_pid = 0;
    if (japan_pid == 0 && read(japan_fd, &received_pid, sizeof(pid_t)) > 0)
    {
        if (received_pid != getpid())
        {
            japan_pid = received_pid;
            log_message("Japan PID received and set.");
            kill(japan_pid, SIGUSR2);
        }
    }
}

int main()
{
    // Set up SIGUSR1 and SIGINT signal handlers
    signal(SIGUSR1, handle_sigusr1);
    signal(SIGUSR2, SIG_IGN);
    signal(SIGINT, terminate_task);

    // Open the log file
    log_file = fopen("watchdog_log.txt", "a");
    if (!log_file)
    {
        perror("Error opening log file");
        return 1;
    }

    log_message("Watchdog started and waiting for PIDs...");

    // Open FIFOs for each process
    egypt_fd = create_fifo(egypt_fifo);
    italy_fd = create_fifo(italy_fifo);
    japan_fd = create_fifo(japan_fifo);
    wd_fd = create_fifo(watchdog_fifo);

    // Send this program's PID to each monitored process
    pid_t pid = getpid();
    if (write(wd_fd, &pid, sizeof(pid_t)) == -1)
    {
        perror("write to watchdog_fifo failed");
        return 1;
    }

    // Initial waiting loop until all PIDs are set
    while (egypt_pid == 0 || italy_pid == 0 || japan_pid == 0)
    {
        if (write(wd_fd, &pid, sizeof(pid_t)) == -1)
        {
            perror("write to watchdog_fifo failed");
            return 1;
        }
        sleep(1); // Wait for signals
    }

    log_message("All PIDs received. Starting monitoring loop...");

    // Timestamps for last data received from each FIFO
    time_t last_time_egypt = time(NULL);
    time_t last_time_italy = time(NULL);
    time_t last_time_japan = time(NULL);

    // Buffer to store the read data
    char buffer[20];
    ssize_t bytesRead;

    // Main loop to read from each FIFO every second
    while (1)
    {
        time_t current_time = time(NULL);

        // Read from Egypt pipe
        bytesRead = read(egypt_fd, buffer, sizeof(buffer) - 1);
        if (bytesRead > 0)
        {
            buffer[bytesRead] = '\0';
            log_message("Received from Egypt: ");
            log_message(buffer);
            last_time_egypt = current_time; // Reset the timestamp
        }

        // Read from Italy pipe
        bytesRead = read(italy_fd, buffer, sizeof(buffer) - 1);
        if (bytesRead > 0)
        {
            buffer[bytesRead] = '\0';
            log_message("Received from Italy: ");
            log_message(buffer);
            last_time_italy = current_time; // Reset the timestamp
        }

        // Read from Japan pipe
        bytesRead = read(japan_fd, buffer, sizeof(buffer) - 1);
        if (bytesRead > 0)
        {
            buffer[bytesRead] = '\0';
            log_message("Received from Japan: ");
            log_message(buffer);
            last_time_japan = current_time; // Reset the timestamp
        }

        // Check if any process has timed out (no data for more than TIMEOUT seconds)
        if (difftime(current_time, last_time_egypt) > TIMEOUT)
        {
            terminate_tasks_and_exit("Egypt");
        }
        if (difftime(current_time, last_time_italy) > TIMEOUT)
        {
            terminate_tasks_and_exit("Italy");
        }
        if (difftime(current_time, last_time_japan) > TIMEOUT)
        {
            terminate_tasks_and_exit("Japan");
        }

        // Wait for 1 second before next iteration
        sleep(1);
    }

    // Clean up and close FIFOs (not reached if terminated early)
    close(egypt_fd);
    close(italy_fd);
    close(japan_fd);
    close(wd_fd);
    unlink(egypt_fifo);
    unlink(italy_fifo);
    unlink(japan_fifo);
    unlink(watchdog_fifo);

    // Close the log file
    fclose(log_file);

    return 0;
}
