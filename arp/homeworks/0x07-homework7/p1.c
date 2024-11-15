#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <string.h>

int pipe_fd;     // Pipe for communication to P2
int pid_pipe_fd; // Pipe for receiving P2's PID
pid_t p2_pid;

// Handle Ctrl-C (SIGINT) to send SIGUSR1 to P2
void forward_signal(int sig)
{
    printf("\n");
    kill(p2_pid, SIGUSR2); // Forward SIGINT to P2 as SIGUSR2
}

// Terminate both processes
void terminate_processes(int sig)
{
    kill(p2_pid, SIGTERM); // Send termination signal to P2
    close(pipe_fd);
    exit(0);
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        fprintf(stderr, "Usage: %s <write_fd> <pid_pipe_read_fd>\n", argv[0]);
        return 1;
    }

    pipe_fd = open(argv[1], O_WRONLY); // Open the named pipe for writing to P2
    if (pipe_fd == -1)
    {
        perror("Failed to open pipe for writing to P2");
        return 1; // Exit if pipe opening fails
    }

    pid_pipe_fd = open(argv[2], O_RDONLY); // Open the named pipe for reading P2's PID
    if (pid_pipe_fd == -1)
    {
        perror("Failed to open pipe for reading P2's PID");
        return 1; // Exit if pipe opening fails
    }

    printf("Waiting for P2 to send the PID\n");
    // Read P2's PID
    if (read(pid_pipe_fd, &p2_pid, sizeof(pid_t)) <= 0)
    {
        perror("Failed to read P2 PID");
        return 1;
    }
    close(pid_pipe_fd); // Close the pid_pipe after reading PID

    printf("P2 PID received: %d\n", p2_pid);

    signal(SIGINT, forward_signal);       // Handle SIGINT and forward to P2 as SIGUSR2
    signal(SIGTERM, terminate_processes); // Handle termination signal

    while (1)
    {
        int A, B;
        printf("Enter two numbers (A B) or -1 -1 to quit: ");
        scanf("%d %d", &A, &B);

        if (A == -1 || B == -1)
        {
            terminate_processes(0);
        }

        write(pipe_fd, &A, sizeof(int));
        write(pipe_fd, &B, sizeof(int));
    }

    return 0;
}
