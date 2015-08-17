// Initial Sketch for EKF Navigation 
// Only reads raw data from inertial captors, GPS and magnetometers. 
// Filter will (for now) be implemented on another calculator reading from a file where all this info would have been stored. 


// RAW SKETCH 

#include <stdarg.h>
#include <stdlib.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_Declination.h>
#include <AP_Compass.h> 
#include <AP_GPS.h>
#include <AP_Math.h>


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;



// Declaring variables

GPS         *gps;
AP_GPS_Auto GPS(&gps);

#define A_LED_PIN 27
#define C_LED_PIN 25
#define A_LED_PIN 37
#define C_LED_PIN 35
#define T6 1000000
#define T7 10000000

AP_Compass_HMC5843 compass;
AP_InertialSensor_MPU6000 ins;
Vector3f accel;
Vector3f gyro;
uint32_t timer,global_timer_start, global_timer_end;
double mag_offset[3];





void setup(){
  
  hal.uartB->begin(38400);
  hal.console->println("Launching setup.\nInitializing captors");
  
  
    // Initializing GPS 
    hal.console->println("GPS initialisation..");
    gps = &GPS;
    gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);
    hal.console->println("GPS initialized");
    gps->update();
    hal.scheduler->delay(2000);
     
    

    
    // Initializing inertial captors
  
    hal.console->println("Inertial sensors startup...");
    
    hal.gpio->pinMode(A_LED_PIN, GPIO_OUTPUT);
    hal.gpio->pinMode(C_LED_PIN, GPIO_OUTPUT);

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, 1);
#endif

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ,
			 NULL);
    hal.console->println("Inertial sensors ready to read");
    
    
    
    
    // Implement a accumulation method for accel calibration (offset calculating)
    Vector3f acc_accumulator, gyro_accumulator;
    int i = 0;
    while (i<100)
    {
      while (ins.num_samples_available() == 0) /* noop */ ;
      ins.update();
      accel = ins.get_accel();
      gyro  = ins.get_gyro();
      acc_accumulator.x += accel.x;
      acc_accumulator.y += accel.y;
      acc_accumulator.z += accel.z;
      gyro_accumulator.x += gyro.x;
      gyro_accumulator.y += gyro.y;
      gyro_accumulator.z += gyro.z;
      hal.scheduler->delay(10);
      i++;
    }
    hal.console->printf("$AO,%4.3f,%4.3f,%4.3f\n",
                        (float)acc_accumulator.x / 100.0,
                        (float)acc_accumulator.y / 100.0,
                        (float)acc_accumulator.z / 100.0);
    hal.console->printf("$GO,%4.3f,%4.3f,%4.3f\n",
                        (float)gyro_accumulator.x / 100.0,
                        (float)gyro_accumulator.y / 100.0,
                        (float)gyro_accumulator.z / 100.0);
      
      
    
    
    // Initializing compass 
  
  if (!compass.init()){
    hal.console->println("Compass initialisation failed");
    while(1);
  }
  
   compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
   compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

  // Complete rotation of the device (2 axes) to calibrate both 3 axes. 
  // and getting the initial orientation of the board
  compass.read();
  double mag_min[3] = {5000,5000,5000};
  double mag_max[3] = {-5000,-5000,-5000};
  hal.console->println("Ready to calibrate magnetometer");
  hal.scheduler->delay(2000);
  hal.console->println("Make a 360° rotation on even surface");
  timer = hal.scheduler->micros();
  while((hal.scheduler->micros()-timer)<20000000L){
    compass.read();
    mag_min[0] = (mag_min[0] > compass.mag_x ? compass.mag_x : mag_min[0]);
    mag_min[1] = (mag_min[1] > compass.mag_y ? compass.mag_y : mag_min[1]);
    mag_max[0] = (mag_max[0] < compass.mag_x ? compass.mag_x : mag_max[0]);
    mag_max[1] = (mag_max[1] < compass.mag_y ? compass.mag_y : mag_max[1]);
    hal.scheduler->delay(10);
  }  
    hal.console->println("WAIT");
    hal.scheduler->delay(3000);
    hal.console->println("Make a 360° rotation NOSE DOWN");
    timer = hal.scheduler->micros();
  while((hal.scheduler->micros()-timer)<20000000L){
    compass.read();
    mag_min[2] = (mag_min[2] > compass.mag_z ? compass.mag_z : mag_min[2]);
    mag_max[2] = (mag_max[2] < compass.mag_z ? compass.mag_z : mag_max[2]);
    hal.scheduler->delay(10);
  }
  hal.console->println("WAIT");
  hal.scheduler->delay(5000);

    
    
  mag_offset[0] = -(mag_min[0]+mag_max[0])/2;
  mag_offset[1] = -(mag_min[1]+mag_max[1])/2;
  mag_offset[2] = -(mag_min[2]+mag_max[2])/2;
  hal.console->printf("Magnetometer offset: %3d,%3d,%3d \n", (int)mag_offset[0], (int)mag_offset[1], (int)mag_offset[2]);
  hal.console->printf("$MO,%3d,%3d,%3d \n", (int)mag_offset[0], (int)mag_offset[1], (int)mag_offset[2]);
 
    
    
    // Initializing GPS 
    /*hal.console->println("GPS initialisation..");
    gps = &GPS;
    gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);
    hal.console->println("GPS initialized");*/
    // Home method for EKF
    hal.scheduler->delay(2000);
    gps->update();
    hal.console->println("Waiting for GPS lock");
    for (int i=0;i<100;i++){  // Works better for some reason
      gps->update();
      hal.scheduler->delay(200);
    }
    gps->update();
    if (gps->new_data){
    hal.console->println("HOME");
    hal.console->print("$GPSI,");
    print_latlon(hal.console,gps->latitude);
    hal.console->print(",");
    print_latlon(hal.console,gps->longitude);
    hal.console->print(",");
    hal.console->printf("%.2f,%lu\n",(float)gps->altitude / 100.0,gps->time);
    gps->new_data = false;
    }
    hal.console->println("Ready");
    hal.scheduler->delay(2000);

}




/*-------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------*/



void loop(){
  
     global_timer_start = hal.scheduler->millis();
     
     
     // Inertial sensors reading 
      while (ins.num_samples_available() == 0) /* noop */ ;
      ins.update();
      accel = ins.get_accel();
      gyro = ins.get_gyro();
      

      // Compass reading 
      compass.read();
      hal.console->printf("$M,%3d,%3d,%3d \n",compass.mag_x,compass.mag_y,compass.mag_z);
      
      
      
      // Printing inertial sensors 
      hal.console->printf_P(PSTR("$A,%4.3f,%4.3f,%4.3f\n"),
                              accel.x,
                              accel.y,
                              accel.z);
      hal.scheduler->delay(10);  
      hal.console->printf_P(PSTR("$G,%4.3f,%4.3f,%4.3f\n"),
                              gyro.x,
                              gyro.y,
                              gyro.z);
                                   
      
      
     // GPS reading (reads 1 out of 10 GPS reading)
     gps->update();
     if (gps->new_data) {
       hal.console->print("$GPS,");
       print_latlon(hal.console,gps->latitude);
       hal.console->print(",");
       print_latlon(hal.console,gps->longitude);
       hal.console->print(",");
       hal.console->printf("%.2f, %lu \n",
                          (float)gps->altitude / 100.0,
                          gps->time);
       gps->new_data = false;
       }
       else hal.console->print("$GPS,NA\n");
     
     
     
    
    // Short delay time to reach a given sample timing
   global_timer_end = hal.scheduler->millis()-global_timer_start;
   hal.scheduler->delay(20-global_timer_end);  
   //hal.console->printf("TIME : %i\n",hal.scheduler->millis()-global_timer_start);
      
}  
  
  

AP_HAL_MAIN();



