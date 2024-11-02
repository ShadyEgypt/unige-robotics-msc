#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

int main()
{
    int fd_write, fd_read;

    char *myfifo = "/tmp/myfifo";
    char *ackfifo = "/tmp/ackfifo";

    // Create FIFOs
    mkfifo(myfifo, 0666);
    mkfifo(ackfifo, 0666);

    char input_string[80], ack_message[80];

    while (1)
    {
        fd_write = open(myfifo, O_WRONLY);

        printf("Please, write two integer numbers, separated by commas (,), or q to quit\n");
        fflush(stdout);

        // Read input from the user
        fgets(input_string, 80, stdin);
        write(fd_write, input_string, strlen(input_string) + 1);
        close(fd_write);

        // If the user enters 'q', exit
        if (input_string[0] == 'q')
            exit(EXIT_SUCCESS);

        // Wait for acknowledgment from second.c
        fd_read = open(ackfifo, O_RDONLY);
        read(fd_read, ack_message, 80);
        printf("Acknowledgment from second: %s\n", ack_message);
        close(fd_read);
    }

    return 0;
}
