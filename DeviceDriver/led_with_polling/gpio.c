#include <stdio.h>
#include <fcntl.h>
#include <string.h>


int main(void)
{
	char buf[BUFSIZ];
	char i = 0;
	int fd = -1;
	memset(buf, 0, BUFSIZ);

	fd = open("/dev/gpiopolling", O_RDWR);
	printf("fd = %d\n", fd);

	read(fd, buf, NULL, NULL);

	while(1)
	{
		write(fd, NULL, NULL, NULL);
	}

	close(fd);

	return 0;
}
