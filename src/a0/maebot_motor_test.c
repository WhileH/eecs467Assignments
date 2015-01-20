#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include "common/getopt.h"

#include "image_u8x3.h"
#include "image_source.h"
#include "image_convert.h"
#include "maebot/rplidar.h"

#include "common/timestamp.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_laser_scan_t.h"

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
FILE *fp_lcm, *fp_data, *fp_real_data;
pthread_mutex_t run_mutex;
int running;

typedef struct state state_t;
struct state {
    char     *url; // image_source url
    image_source_t *isrc;
    int fidx;
    getopt_t *gopt;
    int isTaking;
    pthread_t runthread;

    GtkWidget *window;
    GtkWidget *image;

    pthread_mutex_t mutex;

    FILE *record_islog;
};

void
my_gdkpixbufdestroy (guchar *pixels, gpointer data)
{
    free (pixels);
}

gint
callback_func (GtkWidget *widget, GdkEventKey *event, gpointer callback_data)
{
    return 0;
}

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

void *run_feedback(void *input)
{
    lcm_t *lcm = (lcm_t *) input;
	pthread_mutex_lock(&run_mutex);
    while (running){
		pthread_mutex_unlock(&run_mutex);
        lcm_handle (lcm);
		pthread_mutex_lock(&run_mutex);
	}
	pthread_mutex_unlock(&run_mutex);

	return NULL;
}

static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const maebot_motor_feedback_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");
    int left_ticks = msg->encoder_left_ticks;
    int right_ticks = msg->encoder_right_ticks;
	fprintf(fp_data, "%d %d\n", left_ticks, right_ticks);
}

static void
sensor_data_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                     const maebot_sensor_data_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");

    //fprintf (fp_real_data, "utime: %"PRId64"\n", msg->utime);
    //fprintf (fp_real_data, "accel[0, 1, 2]:        %d,\t%d,\t%d\n",
    //        msg->accel[0], msg->accel[1], msg->accel[2]);
    //fprintf (fp_real_data, "gyro[0, 1, 2]:         %d,\t%d,\t%d\n",
    //        msg->gyro[0], msg->gyro[1], msg->gyro[2]);
    fprintf(fp_real_data,"%lld %d %d %d %d %d %d\n",msg->utime,msg->accel[0],msg->accel[1],msg->accel[2],msg->gyro[0],msg->gyro[1],msg->gyro[2]);
}

static void
maebot_laser_scan_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                     const maebot_laser_scan_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");
	//fprintf(fp_lcm, "Subscribed to channel: MAEBOT_LASER_SCAN\n");
	int i;
	//fprintf(fp_lcm, "Num Ranges: %d\n", msg->num_ranges);
    fprintf(fp_lcm,"%d\n",msg->num_ranges);
	for(i = 0; i < msg->num_ranges; i++)
	{
		fprintf(fp_lcm, "%f %f\n", msg->ranges[i],msg->thetas[i]);
	}
	fprintf(fp_lcm, "\n");
}

int
main (int argc, char *argv[])
{
    state_t *state = calloc(1, sizeof(*state));
    state->gopt = getopt_create();
    state->isTaking = 0;
    getopt_add_bool(state->gopt, 'h', "--help", 0, "Show this help");

    if (!getopt_parse(state->gopt, argc, argv, 0)) {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    const zarray_t *args = getopt_get_extra_args(state->gopt);
    if (zarray_size(args) > 0) {
        zarray_get(args, 0, &state->url);
    } else {
        zarray_t *urls = image_source_enumerate();

        printf("Cameras:\n");
        for (int i = 0; i < zarray_size(urls); i++) {
            char *url;
            zarray_get(urls, i, &url);
            printf("  %3d: %s\n", i, url);
        }

        if (zarray_size(urls) == 0) {
            printf("No cameras found.\n");
            exit(0);
        }
        zarray_get(urls, 0, &state->url);
    }

    state->isrc = image_source_open(state->url);
    if (state->isrc == NULL) {
        printf("Unable to open device %s\n", state->url);
        exit(-1);
    }

    image_source_t *isrc = state->isrc;

    if (isrc->start(isrc))
        exit(-1);

    state->fidx = isrc->get_current_format(isrc);
    if (pthread_mutex_init (&msg_mutex, NULL)) {
        printf ("mutex init failed\n");
        return 1;
    }

	fp_lcm = fopen("lcm_data.txt", "w");
	fp_data = fopen("movement_data.txt", "w");
	fp_real_data = fopen("real_movement_data.txt", "w");
    lcm_t *lcm = lcm_create (NULL);
    lcm_t *lcm_sensor = lcm_create (NULL);
    lcm_t *lcm_lidar = lcm_create (NULL);
	if(!lcm)
		return 1;
	if (pthread_mutex_init (&msg_mutex, NULL)) {
        printf ("msg mutex init failed\n");
        return 1;
    }

	if (pthread_mutex_init (&run_mutex, NULL)) {
        printf ("running mutex init failed\n");
        return 1;
    }

    // Start sending motor commands
    pthread_t diff_drive_thread_pid;
    pthread_create (&diff_drive_thread_pid, NULL, diff_drive_thread, NULL);
    maebot_motor_feedback_t_subscribe (lcm,
                                       "MAEBOT_MOTOR_FEEDBACK",
                                       motor_feedback_handler,
                                       NULL);

    maebot_sensor_data_t_subscribe (lcm_sensor,
                                    "MAEBOT_SENSOR_DATA",
                                    sensor_data_handler,
                                    NULL);

    maebot_laser_scan_t_subscribe(lcm_lidar, "MAEBOT_LASER_SCAN", maebot_laser_scan_handler, NULL);
	pthread_mutex_lock(&run_mutex);
	running = 1;
	pthread_mutex_unlock(&run_mutex);

	pthread_t run_feedback_pid;
    pthread_create (&run_feedback_pid, NULL, run_feedback, (void *)lcm);

	pthread_t run_feedback_sensor_pid;
    pthread_create (&run_feedback_sensor_pid, NULL, run_feedback, (void *)lcm_sensor);

    int round = 0;
	int pic_num = 0;
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
		if(round == 0 || round == 1)
		{
	        lcm_handle (lcm_lidar);
			if(round == 0 || round == 1){
				image_source_data_t isdata;
				image_u8x3_t *im = NULL;
				char buffer[50];
				sprintf(buffer,"pic%d.ppm",pic_num);
				int res = isrc->get_frame(isrc, &isdata);
				if(!res){
					im = image_convert_u8x3(&isdata);
					res = image_u8x3_write_pnm(im,buffer);
				}
				//printf("take pic&d.ppm",pic_num);
				++pic_num;
			}
		}
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
		if(round == 0 || round == 1)
		{
	        lcm_handle (lcm_lidar);
			if(round == 0 || round == 1){
				image_source_data_t isdata;
				image_u8x3_t *im = NULL;
				char buffer[50];
				sprintf(buffer,"pic%d.ppm",pic_num);
				int res = isrc->get_frame(isrc, &isdata);
				if(!res){
					im = image_convert_u8x3(&isdata);
					res = image_u8x3_write_pnm(im,buffer);
				}
				//printf("take pic&d.ppm",pic_num);
				++pic_num;
			}
		}
    }
	pthread_mutex_lock(&run_mutex);
	running = 0;
	pthread_mutex_unlock(&run_mutex);
    return 0;
}
