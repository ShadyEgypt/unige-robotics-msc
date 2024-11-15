#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>    // For fork() and execlp()
#include <sys/types.h> // For pid_t
#include <sys/wait.h>  // For waitpid()

int main()
{
    // Create named pipes (FIFOs)
    if (mkfifo("pipe1", 0666) == -1)
    {
        perror("Failed to create named pipe1");
        return 1;
    }
    if (mkfifo("pipe2", 0666) == -1)
    {
        perror("Failed to create named pipe2");
        return 1;
    }
    signal(SIGINT, SIG_IGN);

    // Now fork P1 and P2
    pid_t p1_pid = fork();
    if (p1_pid == -1)
    {
        perror("Failed to fork P1");
        return 1;
    }

    if (p1_pid == 0)
    {
        // P1 logic here
        execlp("konsole", "konsole", "-e", "./p1", "pipe1", "pipe2", NULL);
        perror("execlp failed for P1");
        return 1;
    }

    pid_t p2_pid = fork();
    if (p2_pid == -1)
    {
        perror("Failed to fork P2");
        return 1;
    }

    if (p2_pid == 0)
    {
        // P2 logic here
        execlp("konsole", "konsole", "-e", "./p2", "pipe1", "pipe2", NULL);
        perror("execlp failed for P2");
        return 1;
    }

    // Parent process: wait for both P1 and P2 to terminate
    int status;
    waitpid(p1_pid, &status, 0); // Wait for P1 to terminate
    printf("P1 terminated with status %d\n", status);

    waitpid(p2_pid, &status, 0); // Wait for P2 to terminate
    printf("P2 terminated with status %d\n", status);

    // Parent can now terminate after both child processes have finished

    printf("Parent terminating...\n");

    // Parent deletes the FIFOs before terminating
    if (unlink("pipe1") == -1)
    {
        perror("Failed to delete pipe1");
    }
    else
    {
        printf("pipe1 deleted successfully\n");
    }

    if (unlink("pipe2") == -1)
    {
        perror("Failed to delete pipe2");
    }
    else
    {
        printf("pipe2 deleted successfully\n");
    }
    return 0;
}
