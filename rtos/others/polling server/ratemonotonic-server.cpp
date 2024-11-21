// compile with: g++ -lpthread ratemonotonic-server.cpp -o ratemonotonic-server

// This exercise show how to schedule threads with Rate Monotonic with aperiodic tasks in background

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/types.h>

// code of periodic tasks
void task1_code();
void task2_code();
void task3_code();

// code of aperiodic tasks
void task4_code();
void task5_code();

// characteristic function of the thread, only for timing and synchronization
// periodic tasks
void *task1(void *);
void *task2(void *);
void *task3(void *);

// aperiodic tasks: we don't need thread functions for aperiodic tasks
void *polling_server(void *);

// initialization of mutexes and conditions (only for aperiodic scheduling)
pthread_mutex_t mutex_task_4 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_task_5 = PTHREAD_MUTEX_INITIALIZER;

bool flag4 = false;
bool flag5 = false;
int qrequests[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int qindex1 = 0;
int qindex2 = 0;

#define INNERLOOP 1000
#define OUTERLOOP 100

#define NPERIODICTASKS 4
#define NAPERIODICTASKS 2
#define NTASKS NPERIODICTASKS + NAPERIODICTASKS

long int periods[NTASKS];
struct timespec next_arrival_time[NTASKS];
double WCET[NTASKS];
pthread_attr_t attributes[NTASKS];
pthread_t thread_id[NTASKS];
struct sched_param parameters[NTASKS];
int missed_deadlines[NTASKS];

int main()
{
    // set task periods in nanoseconds
    // the first task has period 100 millisecond
    // the second task has period 200 millisecond
    // the third task has period 400 millisecond
    // you can already order them according to their priority;
    // if not, you will need to sort them
    periods[0] = 50000000;  // in nanoseconds
    periods[1] = 100000000; // in nanoseconds
    periods[2] = 200000000; // in nanoseconds
    periods[3] = 400000000; // in nanoseconds

    // this is not strictly necessary, but it is convenient to
    // assign a name to the maximum and the minimum priotity in the
    // system. We call them priomin and priomax.

    struct sched_param priomax;
    priomax.sched_priority = sched_get_priority_max(SCHED_FIFO);
    struct sched_param priomin;
    priomin.sched_priority = sched_get_priority_min(SCHED_FIFO);

    // set the maximum priority to the current thread (you are required to be
    // superuser). Check that the main thread is executed with superuser privileges
    // before doing anything else.

    if (getuid() == 0)
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &priomax);

    // execute all tasks in standalone modality in order to measure execution times
    // (use gettimeofday). Use the computed values to update the worst case execution
    // time of each task.

    int i;
    for (i = 0; i < NTASKS; i++)
    {

        // initializa time_1 and time_2 required to read the clock
        struct timespec time_1, time_2;
        clock_gettime(CLOCK_REALTIME, &time_1);

        // we should execute each task more than one for computing the WCET
        // periodic tasks
        if (i == 0)
            task1_code();
        if (i == 1)
            task2_code();
        if (i == 2)
            task3_code();

        // aperiodic tasks
        if (i == 3)
            task4_code();
        if (i == 4)
            task5_code();

        clock_gettime(CLOCK_REALTIME, &time_2);

        // compute the Worst Case Execution Time (in a real case, we should repeat this many times under
        // different conditions, in order to have reliable values

        WCET[i] = 1000000000 * (time_2.tv_sec - time_1.tv_sec) + (time_2.tv_nsec - time_1.tv_nsec);
        printf("\nWorst Case Execution Time %d=%f \n", i, WCET[i]);
        if (i == 4 || i == 5)
        {
            // Here the Capacity of the Server is updated to guarantee that Aperiodic Tasks can
            // be scheduled within the same period of the Server while keeping the set of
            // tasks schedulable.
            if (WCET[i] > WCET[0])
                WCET[0] = WCET[i];
        }
    }
    // print on screen  the capacity of the Polling Server
    printf("\nPolling Server Capacity is %lf", WCET[0]);
    fflush(stdout);

    // compute U
    double U = WCET[0] / periods[0] + WCET[1] / periods[1] + WCET[2] / periods[2];

    // compute Ulub by considering the fact that we have harmonic relationships between periods
    double Ulub = 1;

    // if there are no harmonic relationships, use the following formula instead
    // double Ulub = NPERIODICTASKS*(pow(2.0,(1.0/NPERIODICTASKS)) -1);

    // check the sufficient conditions: if they are not satisfied, exit
    if (U > Ulub)
    {
        printf("\n U=%lf Ulub=%lf Non schedulable Task Set", U, Ulub);
        return (-1);
    }
    printf("\n U=%lf Ulub=%lf Scheduable Task Set", U, Ulub);
    fflush(stdout);
    sleep(5);

    // set the minimum priority to the current thread: this is now required because
    // we will assign higher priorities to periodic threads to be soon created
    // pthread_setschedparam

    if (getuid() == 0)
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &priomin);

    // set the attributes of each task, including scheduling policy and priority
    for (i = 0; i < NPERIODICTASKS; i++)
    {
        // initializa the attribute structure of task i
        pthread_attr_init(&(attributes[i]));

        // set the attributes to tell the kernel that the priorities and policies are explicitly chosen,
        // not inherited from the main thread (pthread_attr_setinheritsched)
        pthread_attr_setinheritsched(&(attributes[i]), PTHREAD_EXPLICIT_SCHED);

        // set the attributes to set the SCHED_FIFO policy (pthread_attr_setschedpolicy)
        pthread_attr_setschedpolicy(&(attributes[i]), SCHED_FIFO);

        // properly set the parameters to assign the priority inversely proportional
        // to the period
        // parameters[i].sched_priority = priomin.sched_priority+NTASKS - i;
        parameters[i].sched_priority = sched_get_priority_max(SCHED_FIFO) - i;

        // set the attributes and the parameters of the current thread (pthread_attr_setschedparam)
        pthread_attr_setschedparam(&(attributes[i]), &(parameters[i]));
    }

    // delare the variable to contain the return values of pthread_create
    int iret[NTASKS];

    // declare variables to read the current time
    struct timespec time_1;
    clock_gettime(CLOCK_REALTIME, &time_1);

    // set the next arrival time for each task. This is not the beginning of the first
    // period, but the end of the first period and beginning of the next one.
    for (i = 0; i < NPERIODICTASKS; i++)
    {
        long int next_arrival_nanoseconds = time_1.tv_nsec + periods[i];
        // then we compute the end of the first period and beginning of the next one
        next_arrival_time[i].tv_nsec = next_arrival_nanoseconds % 1000000000;
        next_arrival_time[i].tv_sec = time_1.tv_sec + next_arrival_nanoseconds / 1000000000;
        missed_deadlines[i] = 0;
    }

    // create all threads(pthread_create)
    iret[0] = pthread_create(&(thread_id[0]), &(attributes[0]), polling_server, NULL);
    iret[1] = pthread_create(&(thread_id[1]), &(attributes[1]), task1, NULL);
    iret[2] = pthread_create(&(thread_id[2]), &(attributes[2]), task2, NULL);
    iret[3] = pthread_create(&(thread_id[3]), &(attributes[3]), task3, NULL);

    // join all threads (pthread_join)
    pthread_join(thread_id[0], NULL);
    pthread_join(thread_id[1], NULL);
    pthread_join(thread_id[2], NULL);
    pthread_join(thread_id[3], NULL);

    // set the next arrival time for each task. This is not the beginning of the first
    // period, but the end of the first period and beginning of the next one.
    for (i = 0; i < NTASKS; i++)
    {
        printf("\nMissed Deadlines Task %d=%d", i, missed_deadlines[i]);
        fflush(stdout);
    }
    exit(0);
}

// application specific task_1 code
void task1_code()
{
    // print the id of the current task
    printf(" 1[ ");
    fflush(stdout);

    // this double loop with random computation is only required to waste time
    int i, j;
    double uno;
    for (i = 0; i < OUTERLOOP; i++)
    {
        for (j = 0; j < INNERLOOP; j++)
        {
            uno = rand() * rand() % 10;
        }
    }

    // when the random variable uno=0, then aperiodic task 5 must
    // be executed
    if (uno == 0)
    {
        printf(":ex(4)");
        fflush(stdout);
        pthread_mutex_lock(&mutex_task_4);

        flag4 = true;
        // We do not need to signal a variable here, since we are not
        // waking up a thread. If we want to execute the aperiodic task 4,
        // it is sufficient to properly set the value of a standard variable, that
        // will be read by the Polling Server at the beginning of its execution to check
        // if it is required to execute the aperiodic task

        pthread_mutex_unlock(&mutex_task_4);
    }

    // when the random variable uno=1, then aperiodic task 5 must
    // be executed
    if (uno == 1)
    {
        printf(":ex(5)");
        fflush(stdout);
        pthread_mutex_lock(&mutex_task_5);

        flag5 = true;
        // We do not need to signal a variable here, since we are not
        // waking up a thread. If we want to execute the aperiodic task 4,
        // it is sufficient to properly set the value of a standard variable, that
        // will be read by the Polling Server at the beginning of its execution to check
        // if it is required to execute the aperiodic task

        pthread_mutex_unlock(&mutex_task_5);
    }

    // print the id of the current task
    printf(" ]1 ");
    fflush(stdout);
}
// thread code for task_1 (used only for temporization)
void *task1(void *ptr)
{
    // set thread affinity, that is the processor on which threads shall run
    cpu_set_t cset;
    CPU_ZERO(&cset);
    CPU_SET(0, &cset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset);

    // execute the task one hundred times... it should be an infinite loop (too dangerous)
    int i = 0;
    for (i = 0; i < 100; i++)
    {
        // execute application specific code
        task1_code();

        // it would be nice to check if we missed a deadline here... why don't
        // you try by yourself?

        // sleep until the end of the current period (which is also the start of the
        // new one
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_arrival_time[0], NULL);

        // the thread is ready and can compute the end of the current period for
        // the next iteration

        long int next_arrival_nanoseconds = next_arrival_time[0].tv_nsec + periods[0];
        next_arrival_time[0].tv_nsec = next_arrival_nanoseconds % 1000000000;
        next_arrival_time[0].tv_sec = next_arrival_time[0].tv_sec + next_arrival_nanoseconds / 1000000000;
    }
}

void task2_code()
{
    // print the id of the current task
    printf(" 2[ ");
    fflush(stdout);
    int i, j;
    double uno;
    for (i = 0; i < OUTERLOOP; i++)
    {
        for (j = 0; j < INNERLOOP; j++)
        {
            uno = rand() * rand() % 10;
        }
    }
    // print the id of the current task
    printf(" ]2 ");
    fflush(stdout);
}

void *task2(void *ptr)
{
    // set thread affinity, that is the processor on which threads shall run
    cpu_set_t cset;
    CPU_ZERO(&cset);
    CPU_SET(0, &cset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset);

    int i = 0;
    for (i = 0; i < 100; i++)
    {
        task2_code();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_arrival_time[1], NULL);
        long int next_arrival_nanoseconds = next_arrival_time[1].tv_nsec + periods[1];
        next_arrival_time[1].tv_nsec = next_arrival_nanoseconds % 1000000000;
        next_arrival_time[1].tv_sec = next_arrival_time[1].tv_sec + next_arrival_nanoseconds / 1000000000;
    }
}

void task3_code()
{
    // print the id of the current task
    printf(" 3[ ");
    fflush(stdout);
    int i, j;
    double uno;
    for (i = 0; i < OUTERLOOP; i++)
    {
        for (j = 0; j < INNERLOOP; j++)
            ;
        double uno = rand() * rand() % 10;
    }
    // print the id of the current task
    printf(" ]3 ");
    fflush(stdout);
}

void *task3(void *ptr)
{
    // set thread affinity, that is the processor on which threads shall run
    cpu_set_t cset;
    CPU_ZERO(&cset);
    CPU_SET(0, &cset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset);

    int i = 0;
    for (i = 0; i < 100; i++)
    {
        task3_code();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_arrival_time[2], NULL);
        long int next_arrival_nanoseconds = next_arrival_time[2].tv_nsec + periods[2];
        next_arrival_time[2].tv_nsec = next_arrival_nanoseconds % 1000000000;
        next_arrival_time[2].tv_sec = next_arrival_time[2].tv_sec + next_arrival_nanoseconds / 1000000000;
    }
}

void task4_code()
{
    printf(" 4[ ");
    fflush(stdout);
    for (int i = 0; i < OUTERLOOP; i++)
    {
        for (int j = 0; j < INNERLOOP; j++)
            double uno = rand() * rand();
    }
    printf(" ]4 ");
    fflush(stdout);
    fflush(stdout);
}

void task5_code()
{
    printf(" 5[ ");
    fflush(stdout);
    for (int i = 0; i < OUTERLOOP; i++)
    {
        for (int j = 0; j < INNERLOOP; j++)
            double uno = rand() * rand();
    }
    printf(" ]5 ");
    fflush(stdout);
}

void *polling_server(void *ptr)
{
    // set thread affinity, that is the processor on which threads shall run
    cpu_set_t cset;
    CPU_ZERO(&cset);
    CPU_SET(0, &cset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset);

    int i = 0;
    for (i = 0; i < 200; i++)
    {
        printf(" PS[ ");
        fflush(stdout);

        // we check if flag 4 has been set to true
        // if yes, we properly add the request to the queue and set
        // flag4 to false
        pthread_mutex_lock(&mutex_task_4);
        if (flag4)
        {
            qrequests[qindex1] = 4;
            qindex1 = (qindex1 + 1) % 20;
            flag4 = false;
        }
        pthread_mutex_unlock(&mutex_task_4);

        // we check if flag 5 has been set to true
        // if yes, we properly add the request to the queue and set
        // flag5 to false
        pthread_mutex_lock(&mutex_task_5);
        if (flag5)
        {
            qrequests[qindex1] = 5;
            qindex1 = (qindex1 + 1) % 20;
            flag5 = false;
        }
        pthread_mutex_unlock(&mutex_task_5);

        // if the queue contains a request to execute task4,
        // do it and move the index to the next element of the queue
        if (qrequests[qindex2] == 4)
        {
            task4_code();
            qindex2 = (qindex2 + 1) % 20;
        }
        // otherwise if the queue contains a request to execute task5,
        // do it and move the index to the next element of the queue
        else if (qrequests[qindex2] == 5)
        {
            task5_code();
            qindex2 = (qindex2 + 1) % 20;
        }
        printf(" ]PS ");
        fflush(stdout);

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_arrival_time[0], NULL);
        long int next_arrival_nanoseconds = next_arrival_time[0].tv_sec * 1000000000 + next_arrival_time[0].tv_nsec + periods[0];
        next_arrival_time[0].tv_sec = next_arrival_nanoseconds / 1000000000;
        next_arrival_time[0].tv_nsec = next_arrival_nanoseconds % 1000000000;
    }
}