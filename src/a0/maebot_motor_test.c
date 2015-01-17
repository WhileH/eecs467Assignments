#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "common/timestamp.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"

#define CMD_PRD 50000 //us  -> 20Hz
#define MTR_SPD 0.25f
#define MTR_STOP 0.0f
#define MTR_TURN_SPD 0.17f
#define LEFT_MOTOR_COEFF 1
#define RIGHT_MOTOR_COEFF 0.96
#define FORWARD1 1280000
#define FORWARD2 2580000
#define FORWARD3 3840000
#define TURNTIME 600000
maebot_motor_command_t msg;
pthread_mutex_t msg_mutex;


void *
diff_drive_thread (void *arg)
{
    lcm_t *lcm = lcm_create (NULL);

    uint64_t utime_start;
    while(1) {
        utime_start = utime_now ();

        pthread_mutex_lock (&msg_mutex);
        maebot_motor_command_t_publish (lcm, "MAEBOT_MOTOR_COMMAND", &msg);
        pthread_mutex_unlock (&msg_mutex);

        usleep (CMD_PRD - (utime_now() - utime_start));
    }

    return NULL;
}

/*static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const maebot_motor_feedback_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");
    left_ticks = msg->encoder_left_ticks;
    right_ticks = msg->encoder_right_ticks;
}*/

int
main (int argc, char *argv[])
{
    //lcm_t *lcm = lcm_create (NULL);
//	if(!lcm)
//		return 1;
    if (pthread_mutex_init (&msg_mutex, NULL)) {
        printf ("mutex init failed\n");
        return 1;
    }

    // Init msg
    // no need for mutex here, as command thread hasn't started yet.
    //msg.motor_left_speed = MTR_STOP;
    //msg.motor_right_speed = MTR_STOP;
    // Start sending motor commands
    pthread_t diff_drive_thread_pid;
    pthread_create (&diff_drive_thread_pid, NULL, diff_drive_thread, NULL);
    //maebot_motor_feedback_t_subscribe (lcm,
    //                                   "MAEBOT_MOTOR_FEEDBACK",
    //                                   motor_feedback_handler,
    //                                   NULL);

   // while (1)
   //     lcm_handle (lcm);
    //int right_tick_init = right_ticks;
    //int left_tick_init = left_ticks;
    //int dist = 20;//in cm
    //int goal_right_ticks = right_tick_init+dist*48;
    //int goal_left_ticks = left_tick_init+dist*48;
    /*while(right_ticks<goal_right_ticks && left_ticks<goal_left_ticks){
        pthread_mutex_lock (&msg_mutex);
        msg.motor_left_speed = MTR_SPD*LEFT_MOTOR_COEFF;
        msg.motor_right_speed = MTR_SPD*RIGHT_MOTOR_COEFF;
        pthread_mutex_unlock (&msg_mutex);
        usleep(1000000);
    }*/
    int round = 0;
    for (round = 0; round < 6; ++round) {
        //forward2
        pthread_mutex_lock (&msg_mutex);
        msg.motor_left_speed = MTR_SPD*LEFT_MOTOR_COEFF;
        msg.motor_right_speed = MTR_SPD*RIGHT_MOTOR_COEFF;
        pthread_mutex_unlock (&msg_mutex);
        usleep(FORWARD1);
        //right turn
        pthread_mutex_lock (&msg_mutex);
        msg.motor_left_speed = MTR_TURN_SPD*LEFT_MOTOR_COEFF;
        msg.motor_right_speed = -MTR_TURN_SPD*RIGHT_MOTOR_COEFF;
        pthread_mutex_unlock (&msg_mutex);
        usleep (TURNTIME);
        //forward3
        pthread_mutex_lock (&msg_mutex);
        msg.motor_left_speed = MTR_SPD*LEFT_MOTOR_COEFF;
        msg.motor_right_speed = MTR_SPD*RIGHT_MOTOR_COEFF;
        pthread_mutex_unlock (&msg_mutex);
        usleep(FORWARD1);
        //right turn
        pthread_mutex_lock (&msg_mutex);
        msg.motor_left_speed = MTR_TURN_SPD*LEFT_MOTOR_COEFF;
        msg.motor_right_speed = -MTR_TURN_SPD*RIGHT_MOTOR_COEFF;
        pthread_mutex_unlock (&msg_mutex);
        usleep (TURNTIME);
    }

    return 0;
}
