/*
compile with:
	g++ threadex2start.cpp -lpthread -o thread2
   
run with:
	.\thread2
*/

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

void *functionC(void *);

// initialize mutex

int  counter = 0;
int temp;

int main()
{
   	int rc1, rc2;
   
 //declare the ids of thread 1 and thread 2

 // Create two independent threads each of which will execute functionC 


// Wait till threads are completed before main continues (notice however that threads do never return).   
 
}
 
void *functionC(void *)
{
	while(1)
	{
 	// lock mutex (try to see how the behaviour changes if the mutex is present or absent)

   	temp = counter; 
   	temp = temp + 1;
   	
	// introduce a delay of one second to increase the probability of a preemption to occur
	sleep(1);
   	counter=temp;

	// print counter value
	
	// unlock mutex (try to see how the behaviour changes if the mutex is present or absent)
	}
}

// OPTIONAL: try to define the mutex as a "recursive" (see the slides about the concept of a "recursive mutex"). Each
// thread must call the lock function and the unlock function twice to test the mechanism.