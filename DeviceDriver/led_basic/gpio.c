#include <stdio.h>
#include <fcntl.h>
#include <string.h>


int main(int argc, char** argv)
{
	char buf[BUFSIZ];
	char i = 0;
	int fd = -1;
	memset(buf, 0, BUFSIZ);

	printf("GPIO Set : %s\n", argv[1]);
	fd = open("/dev/gpioled", O_RDWR);
	printf("fd = %d\n", fd);
	write(fd, argv[1], strlen(argv[1]), NULL);

	read(fd, buf, strlen(argv[1]), NULL);


	printf("read data : %s\n", buf);

	close(fd);

	return 0;

}
