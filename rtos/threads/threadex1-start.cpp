/*
compile with:
    g++ threadex1start.cpp -lpthread -o thread1

run with:
    .\thread1
*/

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>

void *print_message_function(void *ptr);

int main()
{
    // Declare the ids of thread 1 and thread 2
    pthread_t thread1, thread2;
    const char *message1 = "1";
    const char *message2 = "2";
    int iret1, iret2;

    // Create two independent threads each of which will execute the function
    // print_message_function, by passing a different string as the fourth parameter.
    // That is, the first thread has message1 as the fourth parameter, the second thread has
    // message2 as the fourth parameter

    // Wait till threads are complete before main continues. Unless we
    // wait we run the risk of executing an exit which will terminate
    // the process and all threads before the threads have completed.

    // OPTIONAL: try to get the return state of threads using the second parameter of pthread_join, and print it
}

void *print_message_function(void *ptr)
{
    char *message;
    message = (char *)ptr;
    int i;
    for (i = 0; i < 10000; i++)
    {
        // print the message and flush output on the screen
    }

    // OPTIONAL: when exiting from the thread,
    // use pthread_exit and properly set the error code,
    // such that this value can be retrieved in the main thread
    // when using the join function
}