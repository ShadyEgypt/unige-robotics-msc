
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

    char *myfifo2 = "/tmp/myfifo2";
    mkfifo(myfifo2, 0666);

    char str1[80];
    char format_string[80] = "%d,%d";
    int n1, n2;
    double mean;

    while (1)
    {

        fd = open(myfifo2, O_RDONLY);
        read(fd, str1, 80);
        /* if the first input char is q, exit  */
        if (str1[0] == 'q')
            exit(EXIT_SUCCESS);
        /* read numbers from input line */
        sscanf(str1, format_string, &n1, &n2);

        printf("the two values are: %d and %d\n", n1, n2);
        close(fd);
    }
    return 0;
}
