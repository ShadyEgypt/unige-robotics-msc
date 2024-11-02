
#include <stdio.h> 
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h> 
#include <stdlib.h> 
  
int main() 
{ 
  
    char input_string[80]; 
    char format_string[80]="%d,%d"; 
    int n1, n2;
    //
    double mean;
    while (1) 
    { 
      
	printf("Please, write two integer numbers, separated by commas (,), or q to quit\n");
        /* to be sure that the previous is executed immediately */
	fflush(stdout);
        /* read a full input line */
        fgets(input_string, 80 , stdin); 
        /* if the first input char is q, exit  */
        if (input_string[0] == 'q') exit(EXIT_SUCCESS) ;
        /* read nubmers from input line */
        sscanf(input_string, format_string, &n1, &n2);
        mean = (n1 + n2) / 2.0; 
        printf("mean value is: %f, sum is: %d\n", mean, n1 + n2); 
    } 
    /* cannot be here! */
    exit(EXIT_FAILURE);
} 
