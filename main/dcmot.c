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

#define NMOTOR 1

static const float pi = 3.14159;
static const float deg2rad = pi/180.0 ;

int dcmotgpio[NMOTOR] = {14}; //for esp32s3

void dcmot_init() {
}

void dcmot_move(int i, double angle) {

   motval[i] = angle ;

   angle = angle / deg2rad ;

}

