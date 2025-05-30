/* Kernel includes. */
#include <FreeRTOS.h>
#include <task.h>

/* Library includes. */
#include <math.h>
#include <stdio.h>
#include <cstring>
#include <pico/mutex.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/binary_info.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>

// I²C Pins
// I²C 0 - SDA: GP0/GP4/ GP8/GP12/GP16/GP20
// I²C 0 - SCL: GP1/GP5/ GP9/GP13/GP17/GP21
// I²C 1 - SDA: GP2/GP6/GP10/GP14/GP18/GP26
// I²C 1 - SCL: GP3/GP7/GP11/GP15/GP19/GP27
#define I2C_SDA_PIN 	4
#define I2C_SCL_PIN 	5

// Pico slave address
#define I2C_SLAVE_ADDR 	0x0A

// GPIOs for control
#define PARACHUTE	 	3
#define AIR_BRAKE		20

// States
#define ON_LAND			1
#define ASCENDING		2
#define DESCENDING		3
#define NOT_DEPLOYED	0
#define DEPLOYED 		1
 				

// Shared variables between cores
float svHeight;				// in [m]
float svTotal_Accel;		// in [G]
float svVelocity;			// in [m/s]

// State of the rocket
int current_state;
int airbrake_state;
bool meco;

// Sample Period (sample frequency was ~2.4Hz)
float dt;

// mutex to sincronize access to shared variables
static mutex_t mutex;

/**
 * Configures the hardware
 */
static void setup();

/**
 * Configures the I²C
 */
static void setup_i2c();

/**
 * In charge of reading I2C bus, reads 8 bytes.
 * @param  Parameter passed to the task. Not used.
 */
static void readI2C( void *param );

/**
 * Checks if current_height < max_height to deploy parachute.
 * @param  Parameter passed to the task. Not used.
 */
static void recovery( void *param );

/**
 * Checks for MECO, if so, pushes height to core_1.
 * @param  Parameter passed to the task. Not used.
 */
static void ifMECO( void *param );

/**
 * Converts pressure readings in [Pa] to height in [m].
 * @param  Parameter passed to the task. Pressure in [Pa].
 * @return Height in [m]
 */
static int pressureToHeight( int pressure );

/**
 * Converts total acceleration readings to [G].
 * @param  Parameter passed to the task. raw tota acceleration.
 * @return Total Acceleration in [G]
 */
static float accelInGs( float raw_t_a );

/**
 * Core_1 program.
 * @param  Parameter passed to the task. Not used.
 */
void core1_main();



int main( void ){
	setup();
	dt = 0.416; // aproximation of sample period
	meco = false;

	multicore_launch_core1(core1_main);

	xTaskCreate(readI2C, "Readi2c", configMINIMAL_STACK_SIZE, NULL, 1, NULL); // &i2cTaskHandle );
	xTaskCreate(recovery, "Recovery", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(ifMECO, "MECO", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	mutex_init(&mutex);

	vTaskStartScheduler();

	for( ;; );
	return 0;
}

void core1_main(){
	/* ---------- Rocket parameters [Komodo AP] ---------- */
	
	// natural parameters
	float m = 6.718;					// dry mass [kg]
	float rho = 1.2;					// air density
	float C_d = 0.497;					// drag coefficient
	float A = M_PI * pow(0.1113/2, 2);  // cross section area
	float g = 9.8;                   	// gravity's local acceleration
	float K_d = C_d * A; 				// drag coefficient times area

	// airbrake parameters
	float A_AB =  M_PI * pow(0.1150/2, 2);	// cross section area with airbrake deployed
	float C_d_AB = 1.5;						// drag coefficient with airbrake deployed
	float K_dAB = C_d_AB * A_AB;			// drag coefficient times area with airbrake deployed

	float h_max;
	uint32_t apogee = 700;
	
	// terminal velocity with airbrake deployed
	float Vt_AB;
	Vt_AB = sqrt((2 * m * g) / (C_d_AB * A_AB * rho));

	while(true){
		uint32_t raw_h0 = multicore_fifo_pop_blocking();
		uint32_t raw_v0 = multicore_fifo_pop_blocking();

		float h0 = *(float *)&raw_h0;
		float v0 = *(float *)&raw_v0;

		h_max = (m / (rho * C_d_AB * A_AB)) * log(1 + (pow(v0,2)) / (pow(Vt_AB,2))) + h0;

		if(h_max > apogee && airbrake_state == NOT_DEPLOYED){
			gpio_put(AIR_BRAKE, 1);
			airbrake_state = DEPLOYED;
			printf("-------------------------------------------------------------------\n");
			printf("Airbrake desplegado a: %.2f\n", h0);
			printf("-------------------------------------------------------------------\n");
		}
	}
}


static void setup(){
	stdio_init_all();
	setup_i2c();

	gpio_init(PARACHUTE);
	gpio_set_dir(PARACHUTE, GPIO_OUT);
	gpio_put(PARACHUTE, 0);

	gpio_init(AIR_BRAKE);
	gpio_set_dir(AIR_BRAKE, GPIO_OUT);
	gpio_put(AIR_BRAKE, 0);
}


static void setup_i2c(){
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
	#warning i2c/lcd_1602_i2c example requires a board with I2C pins
#else
	// Initialize I²C-0 at 400kHz for pins 4 & 5
	i2c_init(&i2c0_inst, 200000);
	// Enable pins 4 & 5 to run as I²C SDA and SCL (pull-up)
	gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_SDA_PIN);
	gpio_pull_up(I2C_SCL_PIN);
	bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));
#endif
}


static void recovery( void *param ){
	int current_height;
	int last_height;
	float velocity;

	current_state = ON_LAND; 

	mutex_enter_blocking(&mutex);
	last_height = svHeight;
	mutex_exit(&mutex);
	
	printf("Altura Inicial: %f[msnm]\n", last_height);

	while(true){

		mutex_enter_blocking(&mutex);
		current_height = svHeight;
		svVelocity = (current_height - last_height) / dt;
		velocity = svVelocity;
		mutex_exit(&mutex);

		if(current_height > last_height && current_state == 1) {
   			// now ascending
   			current_state = ASCENDING;
  		}

		// descending
		if(current_height < last_height && current_state == 2){
			printf("KBOOM\n");
			gpio_put(PARACHUTE, 1);
			current_state = DESCENDING;

		// still ascending	
		}else if(current_height > last_height){
			last_height = current_height;
		}

		if(current_state == DESCENDING)
			last_height = current_height;	

		printf("Altura: %d[msnm]\n", current_height);
		printf("Velocidad: %.2f[m/s]\n", velocity);
		printf("Estado: %d\n", current_state);

		vTaskDelay(pdMS_TO_TICKS(416));  
	}	
}


static void ifMECO( void *param ){
	int current_velocity;
	int last_velocity;

	mutex_enter_blocking(&mutex);
	last_velocity = svVelocity;
	mutex_exit(&mutex);

	while(true){
		mutex_enter_blocking(&mutex);
		current_velocity = svVelocity;
		mutex_exit(&mutex);

		// ascending, motor still on
		if(current_velocity > last_velocity && current_state == 2){
			last_velocity = current_velocity;

		// ascending, MECO
		}else if(current_velocity < last_velocity && current_state == 2 && !meco){
			meco = true;
			printf("-------------------------------------------------------------------\n");
			printf("M.E.C.O.\n");
			printf("-------------------------------------------------------------------\n");
		}

		if(meco){
			mutex_enter_blocking(&mutex);
			multicore_fifo_push_blocking(*(uint32_t *)&svHeight);
			multicore_fifo_push_blocking(*(uint32_t *)&svVelocity);
			mutex_exit(&mutex);
		}
		vTaskDelay(pdMS_TO_TICKS(416));  

	}
}


static void readI2C(void *param) {
	// waits just in case smth hasn't been initialized
	//vTaskDelay(pdMS_TO_TICKS(50));  

	// for recieving uint32_t + float (8 bytes)
	uint8_t buffer[8] = {0,0,0,0,0,0,0,0};
	uint32_t Pressure;		// in [Pa]
	float Raw_Total_Accel;	// needs to be converted to [G]


	while (true) {
		printf("Solicitando 8 bytes al esclavo I2C (0x%02X)...\n", I2C_SLAVE_ADDR);

		int result = i2c_read_blocking(&i2c0_inst, I2C_SLAVE_ADDR, buffer, sizeof(buffer), false);

		if (result < 0) {
			printf("Error al leer del esclavo I2C\n");
		} else {

			// copy bytes to respective variables
			memcpy(&Pressure, buffer, sizeof(Pressure));
			memcpy(&Raw_Total_Accel, buffer+4, sizeof(Raw_Total_Accel));
			
			mutex_enter_blocking(&mutex);
			svHeight = pressureToHeight(Pressure);
			svTotal_Accel = accelInGs(Raw_Total_Accel);
			mutex_exit(&mutex);

			//printf("Presion: %u\n", Pressure);
			//printf("Aceleracion: %f\n", Raw_Total_Accel);
			//printf("Altura: %f[msnm]\n", svHeight);
			//printf("Aceleracion Total: %f[G]\n", svTotal_Accel);
		}

		vTaskDelay(pdMS_TO_TICKS(416));  // Espera 1 segundo
	}
}


static int pressureToHeight( int pressure ){
	float sea_level_pressure = 101325;	// in [Pa]
	float h = 0;
	int height;

	h =  44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
	
	height = (int) h;

	// relative height
	return height - 1237;
}


static float accelInGs( float raw_t_a ){
	return raw_t_a / 16384.0;	// configured in +-2G's
}