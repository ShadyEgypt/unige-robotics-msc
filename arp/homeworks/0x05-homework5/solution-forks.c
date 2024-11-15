#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/wait.h>
#include <errno.h>

#define PIPE_NAME "/tmp/watchdog_pipe"
#define TIMEOUT 5 // n seconds - time to wait before alerting if no signal from a process

typedef struct
{
    pid_t pid;
    time_t last_active;
} ProcessInfo;

ProcessInfo processes[3];

// Function to initialize the processes array with PIDs and start time
void init_processes(pid_t pids[])
{
    time_t current_time = time(NULL);
    for (int i = 0; i < 3; i++)
    {
        processes[i].pid = pids[i];
        processes[i].last_active = current_time;
    }
}

// Function for the watchdog to monitor activity
void watchdog()
{
    int fd;

    // Open the named pipe for reading
    if ((fd = open(PIPE_NAME, O_RDONLY)) < 0)
    {
        perror("Error opening pipe");
        exit(1);
    }

    while (1)
    {
        pid_t pid;
        int bytes_read = read(fd, &pid, sizeof(pid_t));

        if (bytes_read == sizeof(pid_t))
        {
            // Update the last active time for this PID
            time_t current_time = time(NULL);
            for (int i = 0; i < 3; i++)
            {
                if (processes[i].pid == pid)
                {
                    processes[i].last_active = current_time;
                    printf("Watchdog: Received ping from process %d\n", pid);
                    break;
                }
            }
        }

        // Check for timeouts
        time_t current_time = time(NULL);
        for (int i = 0; i < 3; i++)
        {
            if (difftime(current_time, processes[i].last_active) > TIMEOUT)
            {
                printf("ALERT: Process %d has been inactive for more than %d seconds!\n", processes[i].pid, TIMEOUT);
            }
        }

        sleep(1); // Reduce CPU usage
    }

    close(fd);
}

// Function representing work done by a child process
void child_process()
{
    int fd;

    // Open the named pipe for writing
    if ((fd = open(PIPE_NAME, O_WRONLY)) < 0)
    {
        perror("Error opening pipe for writing");
        exit(1);
    }

    pid_t pid = getpid();

    while (1)
    {
        // Simulate work
        sleep(rand() % 3 + 1); // Random delay between 1 and 3 seconds

        // Write PID to the pipe
        write(fd, &pid, sizeof(pid_t));
        printf("Process %d wrote to pipe\n", pid);
    }

    close(fd);
}

int main()
{
    // Seed random for delays
    srand(time(NULL));

    // Create the named pipe (FIFO)
    mkfifo(PIPE_NAME, 0666);

    // Array to store PIDs of child processes
    pid_t pids[3];

    // Fork process A, B, and C
    for (int i = 0; i < 3; i++)
    {
        pid_t pid = fork();
        if (pid < 0)
        {
            perror("Fork failed for child process");
            exit(1);
        }
        else if (pid == 0)
        {
            // Child process
            child_process();
            exit(0);
        }
        else
        {
            // Parent process stores child PID
            pids[i] = pid;
        }
    }

    // Initialize the last active times for each child process
    init_processes(pids);

    // Fork the watchdog process
    pid_t pid = fork();
    if (pid < 0)
    {
        perror("Fork failed for watchdog");
        exit(1);
    }
    else if (pid == 0)
    {
        // Watchdog process
        watchdog();
        exit(0);
    }

    // Wait for all child processes and the watchdog (they run indefinitely)
    for (int i = 0; i < 4; i++)
    { // 3 child processes + 1 watchdog
        wait(NULL);
    }

    // Remove the named pipe on exit
    unlink(PIPE_NAME);

    return 0;
}
