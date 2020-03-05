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
#include <math.h>


static int fd;
typedef unsigned char   u8;


float CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C = 1.225;
float CONSTANTS_AIR_GAS_CONST = 287.1;
float CONSTANTS_ABSOLUTE_NULL_CELSIUS = -273.15;


 float calc_indicated_airspeed(float differential_pressure) {
       if (differential_pressure > 0.0)
       {
             return sqrtf((2.0 * differential_pressure) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);
       }
       else
       {
              return -sqrtf((2.0 * fabsf(differential_pressure)) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);
       }
}

void read_airspeed() {
    fd = open("/dev/i2c-1", O_RDWR);
    printf("Abrindo a i2c"); 
    FILE * fp;
    
    while(1){
	unsigned char bytes[4] = {0, 0, 0, 0};
	ioctl(fd, I2C_SLAVE, 0x28);
	read(fd, bytes, 4);
	char status = (bytes[0] & 0xC0) >> 6;
	int dp_raw = 0, dT_raw = 0;
	dp_raw = (bytes[0] << 8) + bytes[1];
	/* mask the used bits */
	dp_raw = 0x3FFF & dp_raw;
	dT_raw = (bytes[2] << 8) + bytes[3];
	dT_raw = (0xFFE0 & dT_raw) >> 5;

	// dT max is almost certainly an inbytesid reading
	if (dT_raw == 2047) {
		printf("erro dtraw");
	}

	float temperature = ((200.0f * dT_raw) / 2047) - 50;
	

	// Calculate differential pressure. As its centered around 8000
	// and can go positive or negative
	const float P_min = -1.0f;
	const float P_max = 1.0f;
	const float PSI_to_Pa = 6894.757f;
	/*
	  this equation is an inversion of the equation in the
	  pressure transfer function figure on page 4 of the datasheet
	  We negate the result so that positive differential pressures
	  are generated when the bottom port is used as the static
	  port on the pitot and top port is used as the dynamic port
	 */
	float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
	float diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;

	//printf("DiffPressPsi> %f\nStatus> %d \n", diff_press_PSI, status);
        //printf("Temp> %f\n", temperature);
	//for(int i = 0; i < 4; i++) printf("%d: %d\n", i, bytes[i]);
	if(status == 0) printf("Velocidade do ar: %f, temperatura: %f\n", calc_indicated_airspeed(diff_press_pa_raw), temperature);
    }
}


int main(){
	read_airspeed();
	return 0;
}

