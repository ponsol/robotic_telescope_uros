#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "math.h"

#include "driver/ledc.h"
#include "esp_err.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>


#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif


#include "common.h"

#define PWM_FREQ  50

#define LEDC_MODE      LEDC_LOW_SPEED_MODE


const float pi = 3.14156;
const float deg2rad = pi/180.0 ;

#define NMOTOR 1

int motgpio[NMOTOR] = {14}; //for esp32s3

//motor properties
//motor-1 min-duty=206, max-duty=975
//motor-2 min-duty=420, max-duty=1079
const float motor_zero =  0*deg2rad;
const float motor_range = (180+45)*deg2rad;
const float motor_range_ms = 3.757 ; //adjusted value
const float motor_ref = 1.006 ; //adjusted value

float ms_per_rad =  motor_range_ms / ( motor_range);
float duty_per_ms  =  4096 * PWM_FREQ * 1.e-3 ;




void ledc_init(void)
{

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = PWM_FREQ,  // Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));


    // common channel config 
    ledc_channel_config_t _lchan = { 
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = motgpio[0],
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config_t ledc_channel =  _lchan  ;

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel ));

}


void servomot_init() {
   ledc_init();
}
void servomot_move(int i, double angle) {

   //this motor does not move
   if ( i == 6 ) {
     motval[i] = angle ;
     return ;
   }

   printf ("mot %d angin %f %f\n", i, angle/deg2rad, motval[i]/deg2rad );

   if  ( angle  <  (motor_zero - motor_range/2) )  angle =  -(motor_zero - motor_range/2) ;
   if  ( angle  >  (motor_range/2 + motor_zero)  )  angle =   (motor_range/2 + motor_zero) ;

   double tangle = angle + (motor_range/2 - motor_zero); 
   double msprad = ms_per_rad ;
   double mref = motor_ref ;

   if ( i == 1 ) mref =  motor_ref + 1.05 ; //adjusted values
   if ( i == 1 ) msprad = msprad / 1.1765 ; //adjusted values

   int duty =  duty_per_ms * msprad * tangle  ;

   printf ("ms_p_rad %f  duty_per_ms %f  tangle %f\n", ms_per_rad, duty_per_ms ,  tangle )  ;
   duty +=  duty_per_ms*mref ;

   printf ("here mspd %f %f  %f\n", ms_per_rad, angle, duty_per_ms * ms_per_rad*tangle  );
   printf("moving motor: %d angle: %.15g duty: %d\n", i, angle/deg2rad, duty );

   ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, i, duty));
   ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, i));
   motval[i] = angle ;

}

