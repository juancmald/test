// juan_maldonado_final.c

#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <rc/mpu.h>
#include <rc/time.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <unistd.h>
#include <stdlib.h>
#include <math.h> // for M_PI
#include <robotcontrol.h>

// function declarations
void on_pause_press();
void on_pause_release();

static void imu_interrupt_function(void); // MPU interrupt
static void* my_thread_1(void* ptr); //background thread 1

static rc_mpu_data_t mpu_data;

int main()
{

	pthread_t thread1=0;

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;	

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	   // set up mpu configuration
        rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_fetch_accel_gyro = 1;
        mpu_config.dmp_sample_rate = 200; // Hz to excecute IMU interrupt
        mpu_config.orient = ORIENTATION_Y_UP;

	  // start mpu
        if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
                fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
                rc_led_blink(RC_LED_RED, 5, 5);
                return -1;
        }

	// initialize pause button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);

	// initialize encoders
        if(rc_encoder_eqep_init()==-1){
                fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
                return -1;
        }
       
	 // initialize motors
	 if(rc_motor_init()==-1){
      	       fprintf(stderr,"ERROR: failed to initialize motors\n");
       	       return -1;
        }
       
       
	 // initialize adc
        if(rc_adc_init()==-1){
                fprintf(stderr, "failed to initialize adc\n");
        }

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start balance stack to control setpoints
        if(rc_pthread_create(&thread1,my_thread_1, (void*) NULL, SCHED_OTHER, 0)){
                fprintf(stderr, "failed to start thread1\n");
                return -1;
        }


	rc_mpu_set_dmp_callback(&imu_interrupt_function); //making sure other setup functions don't interfere with the imu_interrupt

	printf("\nPress and release pause button to turn green LED on and off\n");
	printf("hold pause button down for 2 seconds to exit\n");


	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);

	// Variable Declarations
//	double wheelL;
//	int wheelL_pol = -1; // left wheel polarity
//	double wheelR;
//	int wheelR_pol = 1; // right wheel polarity
//	int encoder_res = 2160; // resolution of encoder
//	double u1_D = 0;
//	double u2_D = 0;
//	double error_1 = 0;
//	double error_2 = 0;

	while(rc_get_state()!=EXITING){
	
		if(rc_get_state()==RUNNING){
			rc_led_set(RC_LED_GREEN, 1);
			rc_led_set(RC_LED_RED, 0);
		}
		else{
			rc_led_set(RC_LED_GREEN, 0);
			rc_led_set(RC_LED_RED, 1);
		}
		// always sleep at some point	

	
		rc_usleep(10000);
	}

	// turn off LEDs and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}


double accel_z;
double accel_y;
double gyro_x;
double accel_theta;
double ang_vel_gyro; 
double time_step = 0.005; 
double gyro_ang_0 = 0.00;
double gyro_theta;
double accel_comp_lp;
double gyro_comp_hp;
double theta_comp = 0;
double comp_gyro_1 = 0;
double comp_accel_1 = 0;
double theta_comp_gyro_1 = 0;
double theta_comp_accel_1 = 0;
double theta_gyro_1 = 0;
double theta_gyro_0;
double theta_accel_0;
double u0_D1 = 0;
double u1_D1 = 0;
double u2_D1 = 0;
double error0_D1 = 0;
double error1_D1 = 0;
double error2_D1 = 0;
//double theta_ref = 0;
int wL_pol = -1;
int wR_pol = 1;
int offset = 15; // MiP body angle offseti
double wheelL;
double wheelR;
double wheelavg;
int encoder_res = 2160;
double theta0_D2;
double K = 0.0019;
double Gr = 35.57;

// NOTE: 0 degrees corresponds to the eduMIP perfectely upright

static void imu_interrupt_function(){
//	if (abs(theta_comp) < 5) {

		accel_z = mpu_data.accel[2];
		accel_y = mpu_data.accel[1];
		gyro_x = mpu_data.gyro[0];
		double wc = 100;
		double alpha = (1.0 / wc) / ((1.0 / wc) + time_step);

		// Computing the tilt angle from the accelerometer
		accel_theta = atan2(-1.0*accel_z, accel_y) * 180/M_PI; // units of degrees
		theta_accel_0 = accel_theta;

		// Computing the tilt angle from the gyroscope
		ang_vel_gyro = gyro_x; // units of deg/s
		gyro_theta = gyro_ang_0 + (ang_vel_gyro * time_step); // Using Euler integration
		theta_gyro_0 = gyro_theta;
		gyro_ang_0 = gyro_theta; // stepping the state

		// Computing the  tilt angle using a complementary filter
		gyro_comp_hp = (alpha * theta_comp_gyro_1) + (alpha * (theta_gyro_0 - theta_gyro_1));
		theta_gyro_1 = theta_gyro_0;
		theta_comp_gyro_1 = gyro_comp_hp;

		accel_comp_lp = (alpha * theta_accel_0) + ((1 - alpha) * theta_comp_accel_1);
		theta_comp_accel_1 = accel_comp_lp;

		theta_comp = (gyro_comp_hp + accel_comp_lp) + offset;

	//	wheelL = (rc_encoder_eqep_read(2) * 2.0 * M_PI) / (encoder_res*35.57); // converting encoder values to wheel position in radians
	//	wheelR = (rc_encoder_eqep_read(3) * 2.0 * M_PI) / (encoder_res*35.57);
	//	wheelavg = ((abs(wheelL + wheelR))/2.0) - theta_comp;
		// wheelR is passive and wheelL is controlled
//		double error_0 = wheelR - wheelL; // defining error between reference and output

//		double u_0 = (-67.19*error_0)  + (113.6*error_1) - (47.24*error_2) + (1.138*u1_D) - (0.1421*u2_D); // difference equation of continuous discrete controller

//		u2_D = u1_D; // updating controller outputs
//		u1_D = u_0;

//		error_2 = error_1; // updating error signals
//		error_1 = error_0;

	//	printf("Controller output: %3f\n\n", u_0); // printing the controller output
	//	printf("Error: %3.3f\n", error_0); // printing the error signal
	//	rc_motor_set(2,wheelL_pol*u_0); // writing the controller output to the controlled wheel



	
		error0_D1 = -1*(theta0_D2 *180/M_PI) - theta_comp; // defining error between reference and output

		u0_D1 = (-67.19*error0_D1)  + (113.6*error1_D1) - (47.24*error2_D1) + (1.138*u1_D1) - (0.1421*u2_D1); // difference equation of continuous discrete controller	

		u2_D1 = u1_D1;
		u1_D1 = u0_D1;
		u0_D1 = u0_D1 * 0.05;

		error2_D1 = error1_D1;
		error1_D1 = error0_D1;

	//	if (u0_D1 > 1.0) {
		//	rc_motor_set(2,wL_pol*1); // writing the controller output to the controlled wheel
		//	rc_motor_set(3,wR_pol*1); // writing the controller output to the controlled wheel	
	//	}	

	//	else if (u0_D1 < 1.0) {
		//	rc_motor_set(2,wL_pol*-1); // writing the controller output to the controlled wheel
		//	rc_motor_set(3,wR_pol*-1); // writing the controller output to the controlled wheel	
	//	}


	//	else {
			rc_motor_set(2,wL_pol*u0_D1*K); // writing the controller output to the controlled wheel

		//	rc_motor_set(2,wL_pol*0); // writing the controller output to the controlled wheel
			rc_motor_set(3,wR_pol*u0_D1*K); // writing the controller output to the controlled wheel

		//	rc_motor_set(3,wR_pol*0); // writing the controller output to the controlled wheel	
	//	}

		if ((theta_comp > 70) || (theta_comp < -60)) {
			rc_motor_free_spin(0);
		}
	
	//	printf("u0_D1: %3f\n\n", u0_D1*K);
		return;
//	}
}


//double theta0_D2;
double theta1_D2 = 0;
double theta2_D2 = 0;
double error0_D2 = 0;
double error1_D2 = 0;
double error2_D2 = 0;
double theta_ref = 0;

static void* my_thread_1(__attribute__ ((unused)) void* ptr){
	while(rc_get_state()!=EXITING){

		wheelL = (rc_encoder_eqep_read(2) * 2.0 * M_PI) / (encoder_res*Gr); // converting encoder values to wheel position in radians

		wheelR = (rc_encoder_eqep_read(3) * 2.0 * M_PI) / (encoder_res*Gr);

		wheelavg = ((abs(wheelL + wheelR))/2.0) - theta_comp;

		error0_D2 = (theta_ref - wheelavg); // defining error between reference and output
	printf("Wheel avg val: %3f\n", wheelavg);
		theta0_D2 = (0.03625*error0_D2)  + (0.001202*error1_D2) - (0.03145*error2_D2) + (1.205*theta1_D2) - (0.359*theta2_D2); // difference equation of continuous discrete controller	

		theta2_D2 = theta1_D2;
		theta1_D2 = theta0_D2;

		error2_D2 = error1_D2;
		error1_D2 = error0_D2;

		
		printf("Theta ref: %3f\n\n", theta0_D2);
		printf("Theta comp: %3f\n\n", theta_comp);
		rc_usleep(50000);
	}
	return NULL;
}



/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_motor_cleanup();
	rc_set_state(EXITING);
	return;
}
