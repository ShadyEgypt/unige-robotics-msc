#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <signal.h>
#include <time.h>

#define FIFO1 "fifo1"
#define FIFO2 "fifo2"
#define OUTPUT_FILE "output.txt"
#define MAX_RUNTIME 10

void child_process(const char *fifo_name, char character, int interval_ms)
{
    printf("Child process started: Writing '%c' to %s every %d ms\n", character, fifo_name, interval_ms);

    int fifo_fd = open(fifo_name, O_WRONLY);
    if (fifo_fd == -1)
    {
        perror("Failed to open FIFO in child");
        exit(1);
    }

    for (int i = 0; i < MAX_RUNTIME * 1000 / interval_ms; ++i)
    {
        write(fifo_fd, &character, 1);
        printf("Child writing '%c' to %s\n", character, fifo_name);
        usleep(interval_ms * 1000); // Sleep for interval in milliseconds
    }

    printf("Child process writing '%c' finished.\n", character);
    close(fifo_fd);
    exit(0);
}

int main()
{
    printf("Creating FIFOs...\n");

    if (mkfifo(FIFO1, 0666) == -1)
    {
        perror("Failed to create FIFO1");
        exit(1);
    }
    if (mkfifo(FIFO2, 0666) == -1)
    {
        perror("Failed to create FIFO2");
        exit(1);
    }

    printf("Forking first child...\n");
    pid_t child1 = fork();
    if (child1 == 0)
    {
        child_process(FIFO1, 'A', 1000); // Writes 'A' every 1 second
    }

    printf("Forking second child...\n");
    pid_t child2 = fork();
    if (child2 == 0)
    {
        child_process(FIFO2, 'B', 500); // Writes 'B' every 0.5 seconds
    }

    printf("Opening FIFOs in parent...\n");
    int fifo1_fd = open(FIFO1, O_RDONLY | O_NONBLOCK);
    int fifo2_fd = open(FIFO2, O_RDONLY | O_NONBLOCK);

    if (fifo1_fd == -1 || fifo2_fd == -1)
    {
        perror("Failed to open FIFOs in parent");
        exit(1);
    }

    printf("Opening output file in parent...\n");
    FILE *output_file = fopen(OUTPUT_FILE, "w");
    if (!output_file)
    {
        perror("Failed to open output file");
        exit(1);
    }

    fd_set read_fds;
    struct timeval timeout;
    int max_fd = (fifo1_fd > fifo2_fd) ? fifo1_fd : fifo2_fd;
    time_t start_time = time(NULL);

    int child1_done = 0, child2_done = 0;

    printf("Entering select loop to read from FIFOs...\n");

    while (!child1_done || !child2_done)
    {
        FD_ZERO(&read_fds);
        FD_SET(fifo1_fd, &read_fds);
        FD_SET(fifo2_fd, &read_fds);

        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ready = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        if (ready == -1)
        {
            perror("select() failed");
            break;
        }
        else if (ready == 0)
        {
            if (time(NULL) - start_time >= MAX_RUNTIME)
            {
                printf("Max runtime reached, exiting loop.\n");
                break;
            }
            printf("No data available, timeout occurred. Checking child status...\n");
        }

        if (FD_ISSET(fifo1_fd, &read_fds))
        {
            char buf;
            if (read(fifo1_fd, &buf, 1) > 0)
            {
                fputc(buf, output_file);
                printf("Parent read '%c' from FIFO1 and wrote to file.\n", buf);
            }
        }

        if (FD_ISSET(fifo2_fd, &read_fds))
        {
            char buf;
            if (read(fifo2_fd, &buf, 1) > 0)
            {
                fputc(buf, output_file);
                printf("Parent read '%c' from FIFO2 and wrote to file.\n", buf);
            }
        }

        // Check if each child has exited
        if (!child1_done)
        {
            int status;
            if (waitpid(child1, &status, WNOHANG) > 0)
            {
                printf("Child1 finished.\n");
                child1_done = 1;
            }
        }
        if (!child2_done)
        {
            int status;
            if (waitpid(child2, &status, WNOHANG) > 0)
            {
                printf("Child2 finished.\n");
                child2_done = 1;
            }
        }
    }

    printf("Closing output file and FIFOs in parent...\n");
    fclose(output_file);
    close(fifo1_fd);
    close(fifo2_fd);

    printf("Cleaning up FIFOs...\n");
    unlink(FIFO1);
    unlink(FIFO2);

    printf("Parent process finished.\n");
    return 0;
}
