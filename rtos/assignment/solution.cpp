// compile with:
// g++ -lpthread solution.cpp -o solution
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>

// Task code for periodic tasks J1, J2, J3
void J1_task_code();
void J2_task_code();
void J3_task_code();

// Task code for aperiodic task J4
void J4_task_code();

// Thread functions for periodic tasks J1, J2, J3
void *J1_task(void *);
void *J2_task(void *);
void *J3_task(void *);

// Thread function for aperiodic task J4
void *J4_task(void *);

// Initialization of mutexes and conditions (only for aperiodic scheduling)
pthread_mutex_t mutex_J4 = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond_J4 = PTHREAD_COND_INITIALIZER;

#define INNERLOOP 5000
#define OUTERLOOP 2500

#define NPERIODICTASKS 3
#define NAPERIODICTASKS 1
#define NTASKS (NPERIODICTASKS + NAPERIODICTASKS)

long int periods[NPERIODICTASKS];
struct timespec next_arrival_time[NTASKS];
double WCET[NTASKS];
pthread_attr_t attributes[NTASKS];
pthread_t thread_id[NTASKS];
struct sched_param parameters[NTASKS];
int missed_deadlines[NTASKS];

int fd, result, len;
char buf[10];
const char *str;

int main()
{

    // Set task periods in nanoseconds
    periods[0] = 700000000;  // J1
    periods[1] = 600000000;  // J2
    periods[2] = 1000000000; // J3

    // For aperiodic tasks we set the period to 0
    periods[3] = 0; // J4

    struct sched_param priomax;
    priomax.sched_priority = sched_get_priority_max(SCHED_FIFO);
    struct sched_param priomin;
    priomin.sched_priority = sched_get_priority_min(SCHED_FIFO);

    if (getuid() == 0)
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &priomax);

    for (int i = 0; i < NTASKS; i++)
    {
        struct timespec time_1, time_2;
        clock_gettime(CLOCK_REALTIME, &time_1);

        if (i == 0)
            J1_task_code();
        else if (i == 1)
            J2_task_code();
        else if (i == 2)
            J3_task_code();
        else if (i == 3)
            J4_task_code();

        clock_gettime(CLOCK_REALTIME, &time_2);
        WCET[i] = 1000000000 * (time_2.tv_sec - time_1.tv_sec) + (time_2.tv_nsec - time_1.tv_nsec);
    }

    double U = WCET[0] / periods[0] + WCET[1] / periods[1] + WCET[2] / periods[2];
    double Ulub = 1; // Harmonic relationships assumed here for simplicity

    if (U > Ulub)
    {
        printf("\n U=%lf Ulub=%lf Non schedulable Task Set", U, Ulub);
        return (-1);
    }
    printf("\n U=%lf Ulub=%lf Scheduable Task Set", U, Ulub);
    fflush(stdout);
    sleep(5);

    if (getuid() == 0)
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &priomin);

    for (int i = 0; i < NPERIODICTASKS; i++)
    {
        pthread_attr_init(&(attributes[i]));
        pthread_attr_setinheritsched(&(attributes[i]), PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&(attributes[i]), SCHED_FIFO);
        parameters[i].sched_priority = priomax.sched_priority - i;
        pthread_attr_setschedparam(&(attributes[i]), &(parameters[i]));
    }

    for (int i = NPERIODICTASKS; i < NTASKS; i++)
    {
        pthread_attr_init(&(attributes[i]));
        pthread_attr_setschedpolicy(&(attributes[i]), SCHED_FIFO);

        // set minimum priority (background scheduling)
        parameters[i].sched_priority = 0;
        pthread_attr_setschedparam(&(attributes[i]), &(parameters[i]));
    }
    int iret[NTASKS];
    struct timespec time_1;
    clock_gettime(CLOCK_REALTIME, &time_1);

    for (int i = 0; i < NPERIODICTASKS; i++)
    {
        long int next_arrival_nanoseconds = time_1.tv_nsec + periods[i];
        next_arrival_time[i].tv_nsec = next_arrival_nanoseconds % 1000000000;
        next_arrival_time[i].tv_sec = time_1.tv_sec + next_arrival_nanoseconds / 1000000000;
        missed_deadlines[i] = 0;
    }

    iret[0] = pthread_create(&(thread_id[0]), &(attributes[0]), J1_task, NULL);
    iret[1] = pthread_create(&(thread_id[1]), &(attributes[1]), J2_task, NULL);
    iret[2] = pthread_create(&(thread_id[2]), &(attributes[2]), J3_task, NULL);
    iret[3] = pthread_create(&(thread_id[3]), &(attributes[3]), J4_task, NULL);

    for (int i = 0; i < NTASKS; i++)
    {
        pthread_join(thread_id[i], NULL);
    }
    exit(0);
}

void open_write_close_simple(const char *str)
{
    // printf("%s", str); // Print the string
    // fflush(stdout);    // Ensure the output is flushed immediately
    int fd;
    ssize_t result;
    size_t len;

    // Open the device
    fd = open("/dev/simple", O_RDWR);
    if (fd == -1)
    {
        perror("open failed");
        return; // Exit if the device cannot be opened
    }

    // Calculate the length of the string to write, including the null terminator
    len = strlen(str) + 1;

    // Write the string to the device
    result = write(fd, str, len);
    if (result != len)
    {
        perror("write failed");
        close(fd); // Ensure the file descriptor is closed even if write fails
        return;
    }

    // Print what was written
    printf("Successfully wrote '%s' to /dev/simple\n", str);

    // Close the device
    close(fd);
}

// Define the task code functions here
void J1_task_code()
{
    str = "[1";
    open_write_close_simple(str);
    int i, j;
    for (i = 0; i < OUTERLOOP; i++)
    {
        for (j = 0; j < INNERLOOP; j++)
        {
            double waste = sin(i * j);
        }
    }
    str = "1]";
    open_write_close_simple(str);
}

void J2_task_code()
{
    str = "[2";
    open_write_close_simple(str);
    int i, j;
    double uno;
    for (i = 0; i < OUTERLOOP; i++)
    {
        for (j = 0; j < INNERLOOP; j++)
        {
            uno = cos(i * j);
        }
    }
    if (uno)
    {
        // printf(":ex(4)");
        fflush(stdout);
        // pthread_mutex_lock(&mutex_J4);
        pthread_cond_signal(&cond_J4);
        // pthread_mutex_unlock(&mutex_J4);
    }
    str = "2]";
    open_write_close_simple(str);
}

void J3_task_code()
{
    str = "[3";
    open_write_close_simple(str);
    int i, j;
    for (i = 0; i < OUTERLOOP; i++)
    {
        for (j = 0; j < INNERLOOP; j++)
        {
            double waste = tan(i * j);
        }
    }
    str = "3]";
    open_write_close_simple(str);
}

void J4_task_code()
{
    str = "[4";
    open_write_close_simple(str);
    int i, j;
    for (i = 0; i < OUTERLOOP; i++)
    {
        for (j = 0; j < INNERLOOP; j++)
        {
            double waste = sqrt(i * j);
        }
    }
    str = "4]";
    open_write_close_simple(str);
}

// task1
void *J1_task(void *ptr)
{
    // set thread affinity, that is the processor on which threads shall run
    cpu_set_t cset;
    CPU_ZERO(&cset);
    CPU_SET(0, &cset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset);

    int i = 0;
    for (i = 0; i < 100; i++)
    {
        J1_task_code();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_arrival_time[1], NULL);
        long int next_arrival_nanoseconds = next_arrival_time[1].tv_nsec + periods[0];
        next_arrival_time[1].tv_nsec = next_arrival_nanoseconds % 1000000000;
        next_arrival_time[1].tv_sec = next_arrival_time[1].tv_sec + next_arrival_nanoseconds / 1000000000;
    }
    return NULL;
}

// task2
void *J2_task(void *ptr)
{
    // set thread affinity, that is the processor on which threads shall run
    cpu_set_t cset;
    CPU_ZERO(&cset);
    CPU_SET(0, &cset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset);

    int i = 0;
    for (i = 0; i < 100; i++)
    {
        J2_task_code();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_arrival_time[1], NULL);
        long int next_arrival_nanoseconds = next_arrival_time[1].tv_nsec + periods[1];
        next_arrival_time[1].tv_nsec = next_arrival_nanoseconds % 1000000000;
        next_arrival_time[1].tv_sec = next_arrival_time[1].tv_sec + next_arrival_nanoseconds / 1000000000;
    }
    return NULL;
}

// task3
void *J3_task(void *ptr)
{
    // set thread affinity, that is the processor on which threads shall run
    cpu_set_t cset;
    CPU_ZERO(&cset);
    CPU_SET(0, &cset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset);

    int i = 0;
    for (i = 0; i < 100; i++)
    {
        J3_task_code();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_arrival_time[2], NULL);
        long int next_arrival_nanoseconds = next_arrival_time[2].tv_nsec + periods[2];
        next_arrival_time[2].tv_nsec = next_arrival_nanoseconds % 1000000000;
        next_arrival_time[2].tv_sec = next_arrival_time[2].tv_sec + next_arrival_nanoseconds / 1000000000;
    }
    return NULL;
}

// task4
void *J4_task(void *)
{
    // set thread affinity, that is the processor on which threads shall run
    cpu_set_t cset;
    CPU_ZERO(&cset);
    CPU_SET(0, &cset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset);

    // add an infinite loop
    while (1)
    {
        pthread_cond_wait(&cond_J4, &mutex_J4);
        J4_task_code();
    }
    return NULL;
}