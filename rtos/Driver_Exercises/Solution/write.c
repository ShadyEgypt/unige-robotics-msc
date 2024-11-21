/* writertest.c
 */

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>

int main()
{
  int fd, result, len;
  char buf[10];
  const char *str;
  if ((fd = open("/dev/simple", O_RDWR)) == -1)
  {
    perror("open failed");
    return -1;
  }
  str = "cavolfiore";
  len = strlen(str) + 1;
  if ((result = write(fd, str, len)) != len)
  {
    perror("write failed");
    return -1;
  }
  printf("%d bytes written \n", result);
  close(fd);
}
