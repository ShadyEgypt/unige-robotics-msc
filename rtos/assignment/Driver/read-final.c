#include <stdio.h>
#include <errno.h> // Must be included explicitly for errno
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

int main()
{
  int fd, result;
  int len = 9; // Match the written size
  char buf[len];

  memset(buf, 0, sizeof(buf)); // Initialize buffer

  fd = open("/dev/simple", O_RDWR);
  if (fd == -1)
  {
    perror("open /dev/simple failed");
    return -1;
  }

  result = read(fd, buf, len);
  if (result == -1)
  {
    perror("read failed");
    fprintf(stderr, "Error code: %d\n", errno);
    close(fd);
    return -1;
  }

  buf[result] = '\0'; // Safely null-terminate the string
  printf("Read: %s\n", buf);

  close(fd);
  return 0;
}
