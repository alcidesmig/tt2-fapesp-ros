#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <pthread.h>

static int fd;

unsigned short i2c_read(unsigned char addr, unsigned char reg, int delay) {
    static struct i2c_msg msgs[1];
    int r;

    struct i2c_rdwr_ioctl_data msgset = { msgs, sizeof(msgs) / sizeof(*msgs) };
    unsigned char buf[4];
	
    buf[0] = reg;
    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].buf = (void *)buf;
    msgs[0].len = 1;

    r = ioctl(fd, I2C_RDWR, &msgset);
    if (r < 0) return 0xffff;
  
    if (delay) usleep(delay);
  
    msgs[0].addr = addr;
    msgs[0].flags = I2C_M_RD;
    msgs[0].buf = (void *)buf;
    msgs[0].len = 2;

    r = ioctl(fd, I2C_RDWR, &msgset);
    if (r<0) return 0xffff;

    return buf[0]*256 + buf[1];
}

void  * thread_func(void *arg) {
    fd = open("/dev/i2c-1", O_RDWR);
    
    while(1){
	unsigned short temp = i2c_read(0x40,0,20000);
	unsigned short hum  = i2c_read(0x40,1,20000);

	double temp_value = temp * (165.0/65536.0) - 40;
	double humidity_value = hum  * (100.0/65536.0);
	printf("%.3f\n", temp_value);	
	if (temp!=0xffff)
	    printf("Temperatura: %.3f C\n", temp_value);
	if (hum!=0xffff)
	    printf("Umidade:    %.3f %%RH\n", humidity_value);
	sleep(1);
    }
}


int main(int argc, char *argv[]) {
    pthread_t threads[2];
    pthread_create(&(threads[0]), NULL, thread_func, NULL);
    pthread_join(threads[0], NULL);
    return 0;
}

