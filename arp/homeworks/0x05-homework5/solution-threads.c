#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>

#define PIPE_NAME "/tmp/watchdog_pipe"
#define TIMEOUT 5 // n seconds - time to wait before alerting if no signal from a process
#define BUF_SIZE 2

// Tracks the last active time of each process
time_t last_active[3]; // 0 for process A, 1 for B, 2 for C

// Function for the watchdog to monitor activity
void *watchdog(void *arg)
{
    int fd;
    char buf[BUF_SIZE];

    // Open the named pipe for reading
    if ((fd = open(PIPE_NAME, O_RDONLY)) < 0)
    {
        perror("Error opening pipe");
        exit(1);
    }

    while (1)
    {
        // Read data from the pipe
        if (read(fd, buf, BUF_SIZE - 1) > 0)
        {
            buf[1] = '\0';          // Null-terminate the string
            int idx = buf[0] - 'A'; // Process A -> index 0, B -> 1, C -> 2
            if (idx >= 0 && idx < 3)
            {
                last_active[idx] = time(NULL); // Update last active time
                printf("Watchdog: Process %c is active\n", buf[0]);
            }
        }

        // Check for timeouts
        time_t current_time = time(NULL);
        for (int i = 0; i < 3; i++)
        {
            if (difftime(current_time, last_active[i]) > TIMEOUT)
            {
                printf("ALERT: Process %c has been inactive for more than %d seconds!\n", 'A' + i, TIMEOUT);
            }
        }

        sleep(1); // Reduce CPU usage
    }

    close(fd);
    return NULL;
}

// Function representing work done by process A, B, or C
void *process(void *arg)
{
    char identifier = *(char *)arg;
    int fd;

    // Open the named pipe for writing
    if ((fd = open(PIPE_NAME, O_WRONLY)) < 0)
    {
        perror("Error opening pipe for writing");
        exit(1);
    }

    while (1)
    {
        // Simulate work
        sleep(rand() % 3 + 1); // Random delay between 1 and 3 seconds

        // Write identifier to the pipe
        write(fd, &identifier, 1);
        printf("Process %c wrote to pipe\n", identifier);
    }

    close(fd);
    return NULL;
}

int main()
{
    // Seed random for delays
    srand(time(NULL));

    // Initialize last active time
    time_t start_time = time(NULL);
    for (int i = 0; i < 3; i++)
    {
        last_active[i] = start_time;
    }

    // Create the named pipe (FIFO)
    mkfifo(PIPE_NAME, 0666);

    // Start the watchdog in a separate thread
    pthread_t watchdog_thread;
    pthread_create(&watchdog_thread, NULL, watchdog, NULL);

    // Start each process (A, B, C) in separate threads
    pthread_t threads[3];
    char process_ids[3] = {'A', 'B', 'C'};
    for (int i = 0; i < 3; i++)
    {
        pthread_create(&threads[i], NULL, process, &process_ids[i]);
    }

    // Join threads (they run indefinitely)
    pthread_join(watchdog_thread, NULL);
    for (int i = 0; i < 3; i++)
    {
        pthread_join(threads[i], NULL);
    }

    // Remove the named pipe on exit
    unlink(PIPE_NAME);

    return 0;
}
