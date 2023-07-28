#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "math.h"

#include "esp_err.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "led_strip.h"



#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#define INTERNAL_LED_GPIO 38
static led_strip_handle_t led_strip;
#define AZMOT_GPIO  9

#include "common.h"

#include "dcmot.h"
#include "servomot.h"



#define NAME_LEN   25
#define   TOL_POS  1.0e-4
#define   TOL_VEL  1.0e-8
#define   TOL_EFF  1.0e-8

#define LEDC_MODE      LEDC_LOW_SPEED_MODE

#define PWM_FREQ  50 


					      
double motval[NJOINTS] = {0};

uint64_t time_offset = 0;


rcl_publisher_t tel_js_publisher;

rcl_subscription_t tel_jc_subscriber;

sensor_msgs__msg__JointState tel_jc_msg;
sensor_msgs__msg__JointState tel_js_msg;
sensor_msgs__msg__JointState tel_jsaved_msg;

rcl_subscription_t intsubscriber;
rcl_publisher_t intpublisher;
std_msgs__msg__Int32 recv_msg;
std_msgs__msg__Int32 pub_msg;


uint64_t  millis() {
  return (int) ( esp_timer_get_time()/1e3 );
}

void get_js_msg() {

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
        RCLC_UNUSED(last_call_time);

        if (timer != NULL) {

                get_js_msg();

                printf("Publishing: JointState\n" );
                printf("   at time: %ld %ld\n",  tel_js_msg.header.stamp.sec, tel_js_msg.header.stamp.nanosec );
                RCSOFTCHECK(rcl_publish(&tel_js_publisher, &tel_js_msg, NULL));

        }
}




void movebot() {

   double angle;

   for ( int i = 0; i < NJOINTS; i++ ) {
      if ( fabs(  tel_jsaved_msg.position.data[i] - tel_js_msg.position.data[i] ) > TOL_POS  ) {

	  angle = tel_jsaved_msg.position.data[i];
          if ( i == 0 ) servomot_move(i, angle);
          if ( i == 1 ) dcmot_move(i, angle);

      } else {
        motval[i] = tel_jsaved_msg.position.data[i] ;
      }
    }

}



void jc_subscription_callback(const void * msgin)
{
        const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

     if ( msg->name.size ==  NJOINTS ) printf("\nJoint State: ");
     printf(" at time %lf\n", msg->header.stamp.sec + (msg->header.stamp.nanosec)/1.0e9 );


    int new = 0;


    //to avoid duplicates
    //find whether  received msg is different from jsaved_msg
    for ( int i = 0; i < msg->name.size ; i++ ) {
       if ( fabs(msg->position.data[i] - tel_jsaved_msg.position.data[i] ) > TOL_POS ) {
              new = 1 ; 
       }
       if ( fabs(msg->velocity.data[i] - tel_jsaved_msg.velocity.data[i] ) > TOL_VEL ) {
              new = 1 ; 
       }

       if ( fabs(msg->effort.data[i] - tel_jsaved_msg.effort.data[i] ) > TOL_EFF ) {
              new = 1 ; 
       }
    }

    tel_jsaved_msg = *msg ;

    //if it is new 
    if ( new ) {
      for ( int i = 0; i < msg->name.size ; i++ ){
         printf("%s: %.15g %.15g %.15g \n", msg->name.data[i].data, 
                                   msg->position.data[i],
                                   msg->velocity.data[i],
                                   msg->effort.data[i] );
     
      }

       movebot();
    }

}


void   init_msg() {

      static micro_ros_utilities_memory_conf_t aconf =
            { .max_string_capacity = NAME_LEN,
              .max_ros2_type_sequence_capacity = NJOINTS,
              .max_basic_type_sequence_capacity = NJOINTS, };


      micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &tel_jc_msg,
            aconf);

      micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &tel_js_msg,
            aconf);

      micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &tel_jsaved_msg,
            aconf);

      return ;

}
         

void get_time_offset() {
     uint64_t now = millis();
     uint64_t ros_time_ms = rmw_uros_epoch_millis();
     time_offset = ros_time_ms - now;
     printf("time offsest %lld %lld %lld\n", now, ros_time_ms, time_offset ); 
}

void micro_ros_task(void * arg)
{
        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;

        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&init_options, allocator));

        printf("entering task\n");

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

        // Static Agent IP and port can be used instead of autodisvery.
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
        //RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

        //set init_options
        RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

        // create node
        rcl_node_t node;
        RCCHECK(rclc_node_init_default(&node, "robotic_telescope_uros", "", &support));

        printf("created node\n");

        // Create subscriber.
        RCCHECK(rclc_subscription_init_default(
                &tel_jc_subscriber,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                "/telescope_joint_commands"));


        printf("created jc subscriber\n");


        // create publisher
        RCCHECK(rclc_publisher_init_default(
                &tel_js_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                "/telescope_joint_states"));


        printf("created js publisher\n");



        // create timer,
        rcl_timer_t timer;
        const unsigned int timer_timeout = 1000;
        RCCHECK(rclc_timer_init_default(
                &timer,
                &support,
                RCL_MS_TO_NS(timer_timeout),
                timer_callback));

        printf("created timer\n");

        // create executor
        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
        RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

        //unsigned int rcl_wait_timeout = 1000;   // in ms
        //RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
        printf("created executor\n");

        RCCHECK(rclc_executor_add_timer(&executor, &timer));

        RCCHECK(rclc_executor_add_subscription(&executor, &tel_jc_subscriber, &tel_jc_msg, &jc_subscription_callback, ON_NEW_DATA));


        printf("ready to spin\n");

        init_msg();
        printf("created msg\n");

        RCCHECK( rmw_uros_sync_session(20) );
        get_time_offset();

	led_strip_set_pixel(led_strip, 0, 0, 8, 0);
        led_strip_refresh(led_strip);

        printf("Start spinning\n");
        while(1){
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                usleep(10000);
        }

        // free resources
        printf("Free resources\n");
        RCCHECK(rcl_subscription_fini(&tel_jc_subscriber, &node));
        RCCHECK(rcl_publisher_fini(&tel_js_publisher, &node));

        RCCHECK(rcl_node_fini(&node));

        vTaskDelete(NULL);
}


static void led_init(void)
{

    led_strip_config_t strip_config = {
        .strip_gpio_num = INTERNAL_LED_GPIO,
        .max_leds = 1, 
    };


    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

void app_main(void)
{
    led_init();
    led_strip_set_pixel(led_strip, 0, 8, 0, 0);
    led_strip_refresh(led_strip);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    servomot_init();


    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}

