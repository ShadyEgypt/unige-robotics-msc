#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <string.h>

#define TIMEOUT 10 // Timeout in seconds

const char *watchdog_fifo = "watchdog_pipe";
const char *egypt_fifo = "egypt_pipe";
const char *italy_fifo = "italy_pipe";
const char *japan_fifo = "japan_pipe";

int wd_fd;
int egypt_fd;
int italy_fd;
int japan_fd;

time_t last_time_egypt;
time_t last_time_italy;
time_t last_time_japan;

pid_t egypt_pid = 0;
pid_t italy_pid = 0;
pid_t japan_pid = 0;

FILE *log_file;

// Function to log messages
void log_message(const char *message)
{
    time_t now = time(NULL);
    char *timestamp = ctime(&now);
    timestamp[strlen(timestamp) - 1] = '\0'; // Remove newline
    fprintf(log_file, "[%s] %s\n", timestamp, message);
    fflush(log_file);
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

// Function to create a FIFO
int create_fifo(const char *fifo_path)
{
    if (mkfifo(fifo_path, 0666) == -1 && errno != EEXIST)
    {
        perror("Error creating FIFO");
        exit(1);
    }
    int fifo_fd = open(fifo_path, O_RDWR | O_NONBLOCK);
    if (fifo_fd == -1)
    {
        perror("Error opening FIFO");
        exit(1);
    }
    return fifo_fd;
}

// Signal handler for SIGUSR1
void handle_sigusr1(int sig)
{
    pid_t received_pid = 0;
    char log_msg[100];

    if (egypt_pid == 0 && read(egypt_fd, &received_pid, sizeof(pid_t)) > 0)
    {
        egypt_pid = received_pid;
        snprintf(log_msg, sizeof(log_msg), "Egypt PID received: %d", egypt_pid);
        log_message(log_msg);
        kill(egypt_pid, SIGUSR2);
    }

    if (italy_pid == 0 && read(italy_fd, &received_pid, sizeof(pid_t)) > 0)
    {
        italy_pid = received_pid;
        snprintf(log_msg, sizeof(log_msg), "Italy PID received: %d", italy_pid);
        log_message(log_msg);
        kill(italy_pid, SIGUSR2);
    }

    if (japan_pid == 0 && read(japan_fd, &received_pid, sizeof(pid_t)) > 0)
    {
        japan_pid = received_pid;
        snprintf(log_msg, sizeof(log_msg), "Japan PID received: %d", japan_pid);
        log_message(log_msg);
        kill(japan_pid, SIGUSR2);
    }
}

void handle_sigusr2(int sig, siginfo_t *info, void *context)
{
    pid_t sender_pid = info->si_pid; // Get the sender's PID
    char log_msg[100];

    if (sender_pid == egypt_pid)
    {
        last_time_egypt = time(NULL);
        snprintf(log_msg, sizeof(log_msg), "Heartbeat received from Egypt (PID: %d).", sender_pid);
        kill(egypt_pid, SIGUSR2);
    }
    else if (sender_pid == italy_pid)
    {
        last_time_italy = time(NULL);
        snprintf(log_msg, sizeof(log_msg), "Heartbeat received from Italy (PID: %d).", sender_pid);
        kill(italy_pid, SIGUSR2);
    }
    else if (sender_pid == japan_pid)
    {
        last_time_japan = time(NULL);
        snprintf(log_msg, sizeof(log_msg), "Heartbeat received from Japan (PID: %d).", sender_pid);
        kill(japan_pid, SIGUSR2);
    }
    else
    {
        snprintf(log_msg, sizeof(log_msg), "Unexpected SIGUSR2 from unknown PID: %d", sender_pid);
    }

    log_message(log_msg);
}

// Main function
int main()
{
    // Set up SIGUSR1, SIGUSR2 and SIGINT signal handlers
    signal(SIGINT, terminate_task);
    signal(SIGUSR1, handle_sigusr1);
    signal(SIGUSR2, SIG_IGN);

    log_file = fopen("watchdog_log.txt", "a");
    if (!log_file)
    {
        perror("Error opening log file");
        return 1;
    }

    log_message("Watchdog started and waiting for PIDs...");

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

    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;         // Use extended signal information
    sa.sa_sigaction = handle_sigusr2; // Set the handler function

    // Register handler for SIGUSR2
    sigaction(SIGUSR2, &sa, NULL);

    // Buffer to store the read data
    char buffer[20];
    ssize_t bytesRead;

    last_time_egypt = time(NULL);
    last_time_italy = time(NULL);
    last_time_japan = time(NULL);

    while (1)
    {
        time_t current_time = time(NULL);

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
