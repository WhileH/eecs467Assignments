#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>

#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/math_util.h"

#define NUM_SERVOS 6

typedef struct state state_t;
struct state
{
    getopt_t *gopt;

    // LCM
    lcm_t *lcm;
    const char *command_channel;
    const char *status_channel;

    pthread_t status_thread;
    pthread_t command_thread;
};


static void
status_handler (const lcm_recv_buf_t *rbuf,
                const char *channel,
                const dynamixel_status_list_t *msg,
                void *user)
{
    // Print out servo positions
    for (int id = 0; id < msg->len; id++) {
        dynamixel_status_t stat = msg->statuses[id];
        printf ("[id %d]=%6.3f ",id, stat.position_radians);
    }
    printf ("\n");
}

void *
status_loop (void *data)
{
    state_t *state = data;
    dynamixel_status_list_t_subscribe (state->lcm,
                                       state->status_channel,
                                       status_handler,
                                       state);
    const int hz = 15;
    while (1) {
        // Set up the LCM file descriptor for waiting. This lets us monitor it
        // until something is "ready" to happen. In this case, we are ready to
        // receive a message.
        int status = lcm_handle_timeout (state->lcm, 1000/hz);
        if (status <= 0)
            continue;

        // LCM has events ready to be processed
    }

    return NULL;
}

void *
command_loop (void *user)
{
    state_t *state = user;
    const int hz = 30;

    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = calloc (NUM_SERVOS, sizeof(dynamixel_command_t));

    while (1) {
        // Send LCM commands to arm. Normally, you would update positions, etc,
        // but here, we will just home the arm.
        for (int id = 0; id < NUM_SERVOS; id++) {
            if (getopt_get_bool (state->gopt, "idle")) {
                cmds.commands[id].utime = utime_now ();
                cmds.commands[id].position_radians = 0.0;
                cmds.commands[id].speed = 0.0;
                cmds.commands[id].max_torque = 0.0;
            }
            else {
                // home servos slowly
                cmds.commands[id].utime = utime_now ();
                cmds.commands[id].position_radians = 0.0;
                cmds.commands[id].speed = 0.05;
                cmds.commands[id].max_torque = 0.35;

            }
        }
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (10000000/hz);
	
	//rotate the base 15 degrees
	cmds.commands[0].utime = utime_now ();
        cmds.commands[0].position_radians = 0.26;
        cmds.commands[0].speed = 0.05;
        cmds.commands[0].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);

	//rotate base back to home
	cmds.commands[0].utime = utime_now ();
        cmds.commands[0].position_radians = 0.0;
        cmds.commands[0].speed = 0.05;
        cmds.commands[0].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);

	//rotate the shoulder 15 degrees
	cmds.commands[1].utime = utime_now ();
        cmds.commands[1].position_radians = 0.26;
        cmds.commands[1].speed = 0.05;
        cmds.commands[1].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);

	//rotate shoulder back to home
	cmds.commands[1].utime = utime_now ();
        cmds.commands[1].position_radians = 0.0;
        cmds.commands[1].speed = 0.05;
        cmds.commands[1].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);
	
	//rotate the elbow 15 degrees
	cmds.commands[2].utime = utime_now ();
        cmds.commands[2].position_radians = 1.2;
        cmds.commands[2].speed = 0.1;
        cmds.commands[2].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);

	//rotate elbow back to home
	cmds.commands[2].utime = utime_now ();
        cmds.commands[2].position_radians = 0.0;
        cmds.commands[2].speed = 0.1;
        cmds.commands[2].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);
	
	//rotate the wrist join 15 degrees
	cmds.commands[3].utime = utime_now ();
        cmds.commands[3].position_radians = 0.26;
        cmds.commands[3].speed = 0.05;
        cmds.commands[3].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);

	//rotate wrist joint back to home
	cmds.commands[3].utime = utime_now ();
        cmds.commands[3].position_radians = 0.0;
        cmds.commands[3].speed = 0.05;
        cmds.commands[3].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);

	//rotate the wrist 15 degrees
	cmds.commands[4].utime = utime_now ();
        cmds.commands[4].position_radians = 0.26;
        cmds.commands[4].speed = 0.05;
        cmds.commands[4].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);

	//rotate wrist back to home
	cmds.commands[4].utime = utime_now ();
        cmds.commands[4].position_radians = 0.0;
        cmds.commands[4].speed = 0.05;
        cmds.commands[4].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);

	//rotate the fingers 15 degrees
	cmds.commands[5].utime = utime_now ();
        cmds.commands[5].position_radians = 0.26;
        cmds.commands[5].speed = 0.05;
        cmds.commands[5].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);

	//rotate fingers back to home
	cmds.commands[5].utime = utime_now ();
        cmds.commands[5].position_radians = 0.0;
        cmds.commands[5].speed = 0.05;
        cmds.commands[5].max_torque = 0.35;
	dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
	usleep (50000000/hz);
	

        dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);

        usleep (1000000/hz);
    }

    free (cmds.commands);

    return NULL;
}

// This subscribes to the status messages sent out by the arm, displaying servo
// state in the terminal. It also sends messages to the arm ordering it to the
// "home" position (all servos at 0 radians).
int
main (int argc, char *argv[])
{
    getopt_t *gopt = getopt_create ();
    getopt_add_bool (gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_bool (gopt, 'i', "idle", 0, "Command all servos to idle");
    getopt_add_string (gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");
    getopt_add_string (gopt, '\0', "command-channel", "ARM_COMMAND", "LCM command channel");

    if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt);
        exit (EXIT_FAILURE);
    }

    state_t *state = calloc (1, sizeof(*state));
    state->gopt = gopt;
    state->lcm = lcm_create (NULL);
    state->command_channel = getopt_get_string (gopt, "command-channel");
    state->status_channel = getopt_get_string (gopt, "status-channel");

    pthread_create (&state->status_thread, NULL, status_loop, state);
    pthread_create (&state->command_thread, NULL, command_loop, state);

    // Probably not needed, given how this operates
    pthread_join (state->status_thread, NULL);
    pthread_join (state->command_thread, NULL);

    lcm_destroy (state->lcm);
    free (state);
    getopt_destroy (gopt);
}
