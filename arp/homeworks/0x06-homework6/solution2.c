#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <time.h>

#define OUTPUT_FILE "output2.txt"
#define MAX_RUNTIME 10

void child_process(int write_fd, char character, int interval_ms)
{
    printf("Child process started: Writing '%c' every %d ms\n", character, interval_ms);

    for (int i = 0; i < MAX_RUNTIME * 1000 / interval_ms; ++i)
    {
        write(write_fd, &character, 1);
        printf("Child writing '%c'\n", character);
        usleep(interval_ms * 1000); // Sleep for interval in milliseconds
    }

    printf("Child process writing '%c' finished.\n", character);
    close(write_fd);
    exit(0);
}

int main()
{
    printf("Creating unnamed pipes...\n");

    int pipe1[2], pipe2[2];
    if (pipe(pipe1) == -1 || pipe(pipe2) == -1)
    {
        perror("Failed to create pipes");
        exit(1);
    }

    printf("Forking first child...\n");
    pid_t child1 = fork();
    if (child1 == 0)
    {
        close(pipe1[0]);                    // Close read end in child
        child_process(pipe1[1], 'A', 1000); // Writes 'A' every 1 second
    }

    printf("Forking second child...\n");
    pid_t child2 = fork();
    if (child2 == 0)
    {
        close(pipe2[0]);                   // Close read end in child
        child_process(pipe2[1], 'B', 500); // Writes 'B' every 0.5 seconds
    }

    // Close write ends in the parent
    close(pipe1[1]);
    close(pipe2[1]);

    printf("Opening output file in parent...\n");
    FILE *output_file = fopen(OUTPUT_FILE, "w");
    if (!output_file)
    {
        perror("Failed to open output file");
        exit(1);
    }

    fd_set read_fds;
    struct timeval timeout;
    int max_fd = (pipe1[0] > pipe2[0]) ? pipe1[0] : pipe2[0];
    time_t start_time = time(NULL);

    int child1_done = 0, child2_done = 0;

    printf("Entering select loop to read from pipes...\n");

    while (!child1_done || !child2_done)
    {
        FD_ZERO(&read_fds);
        FD_SET(pipe1[0], &read_fds);
        FD_SET(pipe2[0], &read_fds);

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

        if (FD_ISSET(pipe1[0], &read_fds))
        {
            char buf;
            if (read(pipe1[0], &buf, 1) > 0)
            {
                fputc(buf, output_file);
                printf("Parent read '%c' from pipe1 and wrote to file.\n", buf);
            }
            else
            {
                printf("Child1 finished writing.\n");
                child1_done = 1;
            }
        }

        if (FD_ISSET(pipe2[0], &read_fds))
        {
            char buf;
            if (read(pipe2[0], &buf, 1) > 0)
            {
                fputc(buf, output_file);
                printf("Parent read '%c' from pipe2 and wrote to file.\n", buf);
            }
            else
            {
                printf("Child2 finished writing.\n");
                child2_done = 1;
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

    printf("Closing output file in parent...\n");
    fclose(output_file);

    printf("Parent process finished.\n");
    return 0;
}
