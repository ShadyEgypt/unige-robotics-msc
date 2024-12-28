/* readtertest.c
 */

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>

int main()
{
	int fd, result;
	int len = 11;
	char buf[len];

	// open file for reading
	if ((fd = open("/dev/simple", O_RDWR)) == -1)
	{
		perror("open failed");
		return -1;
	}
	// read 11 bytes
	if ((result = read(fd, &buf, sizeof(buf))) != len)
	{
		perror("read failed");
		return -1;
	}
	// print the result on the screen
	fprintf(stdout, "Read %s \n", buf);

	// close file
	close(fd);

	return 0;
}
