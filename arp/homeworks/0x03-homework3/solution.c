#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <sys/file.h>

char *myfifo = "/tmp/myfifo";

void sender()
{
    int fd;
    mkfifo(myfifo, 0666);
    char input_string[120], send_string[120];
    int toggle = 0;
    char format_string[80] = "%d,%d";
    int n1, n2;
    while (1)
    {
        fd = open(myfifo, O_WRONLY);
        if (fd == -1)
        {
            perror("Encountered error while opening the numbers fifo\n");
            close(fd);
            exit(EXIT_FAILURE);
        }
        printf("\nPlease, write two integer numbers, separated by commas (,), or q to quit\n"); /* to be sure that the previous is executed immediately */
        fflush(stdout);                                                                         /* read a full input line */
        fgets(input_string, 120, stdin);
        if (input_string[0] == 'q')
        {
            strcpy(send_string, input_string);
            write(fd, send_string, strlen(send_string) + 1);
            close(fd);
            exit(EXIT_SUCCESS);
        }
        else
        {
            /* Attempt to read numbers from input line */
            if (sscanf(input_string, format_string, &n1, &n2) == 2)
            {
                // Successfully parsed both numbers
                printf("Sender received %d and %d\n", n1, n2);
            }
            else
            {
                // Error handling if parsing failed
                fprintf(stderr, "Error: Failed to parse two integers from input.\n");
                // You might want to handle the error here, e.g., by requesting input again
                continue;
            }
        }
        char tmp[120];

        // Choose which process will have to compute the two numbers
        switch (toggle)
        {
        case 0:
            strcpy(tmp, "1,");
            break;
        case 1:
            strcpy(tmp, "2,");
            break;
        default:
            perror("Unkown error, Gian has made some mistakes.... =(\n");
            close(fd);
            exit(EXIT_FAILURE);
            break;
        }
        toggle = 1 - toggle;
        strcat(tmp, input_string);
        strcpy(send_string, tmp);
        write(fd, send_string, strlen(send_string) + 1);
        printf("[Sender] The string sent is %s", send_string);
        fflush(stdout);
        close(fd);
    }
}

void receiver(int which)
{
    int fd;
    mkfifo(myfifo, 0666);
    char input_string[120], send_string[120];
    char format_string[120];
    snprintf(format_string, sizeof(format_string), "%d,%s,%s", which, "%d", "%d");
    int n1, n2;
    double mean;
    while (1)
    {
        fd = open(myfifo, O_RDWR);
        if (fd == -1)
        {
            perror("Encountered error while opening the numbers fifo\n");
            close(fd);
            exit(EXIT_FAILURE);
        }
        if (read(fd, input_string, 120) == -1)
        {
            perror("Encountered error reading the FIFO\n");
            close(fd);
            exit(EXIT_FAILURE);
        }
        if (input_string[0] == 'q')
        {
            write(fd, input_string, strlen(input_string) + 1);
            close(fd);
            exit(EXIT_SUCCESS);
        } /* read numbers from input line */
        switch (sscanf(input_string, format_string, &n1, &n2))
        {
        case 2:
            mean = (n1 + n2) / 2.0;
            printf("[Receiver %d] Mean value is: %f, sum is: %d\n", which, mean, n1 + n2);
            fflush(stdout);
            close(fd);
            break;
        case 1:
            printf("[Receiver %d] The received string is: %s\n", which, input_string);
            perror("Encountered error reading the FIFO message\n");
            fflush(stdout);
            close(fd);
            exit(EXIT_FAILURE);
            break;
        case 0:
            write(fd, input_string, strlen(input_string) + 1);
            close(fd);
            continue;
            break;
        default:
            char tmp[13];
            snprintf(tmp, sizeof(tmp), "[Receiver %d]", which);
            perror("tmp");
            break;
        }
    }
}
int main(int argc, char *argv[])
{
    unlink(myfifo);
    pid_t first = fork();
    if (first == -1)
    {
        perror("first");
        return 1;
    }
    if (first == 0)
    { // Child process
        sender();
        exit(EXIT_SUCCESS);
    }
    else
    {
        // Parent process
        pid_t sec1 = fork();
        if (sec1 == -1)
        {
            perror("sec1");
            return 1;
        }
        if (sec1 == 0)
        { // Child process
            receiver(1);
            exit(EXIT_SUCCESS);
        }
        else
        {
            pid_t sec2 = fork();
            if (sec2 == -1)
            {
                perror("sec2");
                return 1;
            }
            if (sec2 == 0)
            { // Child process
                receiver(2);
                exit(EXIT_SUCCESS);
            }
        }
    }
    for (int i = 0; i < 3; i++)
        wait(NULL); // ! Delete the FIFO [DO IT]
    unlink(myfifo);
    return 0;
}