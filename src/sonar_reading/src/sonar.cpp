#include <unistd.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
int dID = 0x3a;
int fd, reg;
int main(int argc , char **argv) {
	if((fd=wiringPiI2CSetup(dID))<0)
		printf("error opening i2c channel\n");

	int e = wiringPiI2CWrite(fd, 0x51);

	usleep(100000);

	int r = wiringPiI2CReadReg16(fd, 0xe1);

	int val = (r >> 8) & 0xff | (r << 8) & 0x1;
	printf("%d\n", val);

}

