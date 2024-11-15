#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>

int pipe_fd;
int A, B;
int result = 0;

void handle_termination(int sig)
{
    printf("\n");
}

// Handle SIGUSR2 to compute (A + B)++ and print result
void handle_calculation(int sig)
{
    if (sig == SIGUSR2)
    {
        result = A + B;
        printf("Computed result: %d\n", result);
        A++;
        B++;
    }
}

// Terminate P2 process
void terminate_process(int sig)
{
    close(pipe_fd);
    exit(0);
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        fprintf(stderr, "Usage: %s <write_fd> <pid_pipe_fd>\n", argv[0]);
        return 1;
    }

    pipe_fd = open(argv[1], O_RDONLY); // Open the named pipe for reading from P1
    if (pipe_fd == -1)
    {
        perror("Failed to open pipe for reading from P1");
        return 1; // Exit if pipe opening fails
    }

    int pid_pipe_fd = open(argv[2], O_WRONLY); // Open the named pipe for sending P2's PID
    if (pid_pipe_fd == -1)
    {
        perror("Failed to open pipe for writing P2's PID");
        return 1; // Exit if pipe opening fails
    }

    // Send P2's PID to P1
    pid_t pid = getpid();
    if (write(pid_pipe_fd, &pid, sizeof(pid_t)) == -1)
    {
        perror("write");
        return 1;
    }
    close(pid_pipe_fd); // Close pid_pipe_fd after sending PID

    // Set up signal handlers
    signal(SIGUSR2, handle_calculation); // Handle SIGUSR2 for calculations
    signal(SIGTERM, terminate_process);  // Terminate on SIGTERM
    signal(SIGINT, handle_termination);
    while (1)
    {
        // Read A
        if (read(pipe_fd, &A, sizeof(int)) <= 0)
        {
            perror("Failed to read A or end of input");
            break;
        }
        printf("Read A: %d\n", A);

        // Read B
        if (read(pipe_fd, &B, sizeof(int)) <= 0)
        {
            perror("Failed to read B or end of input");
            break;
        }
        printf("Read B: %d\n", B);

        // Wait for signal from P1 to compute and print result
        pause();
    }

    return 0;
}
