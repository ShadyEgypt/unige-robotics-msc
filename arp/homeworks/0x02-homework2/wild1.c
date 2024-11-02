#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

int main()
{
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); // Set stdin to non-blocking mode

    char input_string[5];

    while (1)
    {
        usleep(567000);
        printf("%c", '|');
        fflush(stdout);

        // Non-blocking read
        if (read(STDIN_FILENO, input_string, sizeof(input_string) - 1) > 0)
        {
            if (input_string[0] == 'q')
            {
                fcntl(STDIN_FILENO, F_SETFL, flags); // Reset stdin to blocking mode before exit
                exit(EXIT_SUCCESS);
            }
        }
    }

    return 0;
}
