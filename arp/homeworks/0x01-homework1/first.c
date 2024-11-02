#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

int main()
{
    int fd;

    char *myfifo = "/tmp/myfifo";
    mkfifo(myfifo, 0666);

    char input_string[80], send_string[80];

    while (1)
    {

        fd = open(myfifo, O_WRONLY);

        printf("Please, write two integer numbers, separated by commas (,), or q to quit\n");
        /* to be sure that the previous is executed immediately */
        fflush(stdout);
        /* read a full input line */
        fgets(input_string, 80, stdin);
        write(fd, input_string, strlen(input_string) + 1);
        close(fd);
        /* if the first input char is q, exit  */
        if (input_string[0] == 'q')
            exit(EXIT_SUCCESS);
    }
    return 0;
}
