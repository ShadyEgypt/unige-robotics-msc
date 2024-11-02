#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

int main()
{
    int fd_read, fd_write;

    char *myfifo = "/tmp/myfifo";
    char *ackfifo = "/tmp/ackfifo";

    // Create FIFOs
    mkfifo(myfifo, 0666);
    mkfifo(ackfifo, 0666);

    char str1[80], ack_message[80] = "Data received successfully.";
    char format_string[80] = "%d,%d";
    int n1, n2;
    double mean;

    while (1)
    {
        fd_read = open(myfifo, O_RDONLY);
        read(fd_read, str1, 80);
        close(fd_read);

        // If the user entered 'q', exit
        if (str1[0] == 'q')
            exit(EXIT_SUCCESS);

        // Read numbers from input
        sscanf(str1, format_string, &n1, &n2);
        mean = (n1 + n2) / 2.0;
        printf("mean value is: %f, sum is: %d\n", mean, n1 + n2);

        // Send acknowledgment to first.c
        fd_write = open(ackfifo, O_WRONLY);
        write(fd_write, ack_message, strlen(ack_message) + 1);
        close(fd_write);
    }

    return 0;
}
