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

#include "common/timestamp.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_laser_scan_t.h"
#include "maebot/rplidar.h"


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

static int
write_u32 (FILE *f, uint32_t v)
{
    uint8_t buf[4];
    buf[0] = (v>>24) & 0xff;
    buf[1] = (v>>16) & 0xff;
    buf[2] = (v>>8) & 0xff;
    buf[3] = (v>>0) & 0xff;

    int res = fwrite(buf, 1, 4, f);
    if (res != 4)
        return -1;
    return 0;
}

static int
write_u64 (FILE *f, uint64_t v)
{
    if (write_u32(f, (v>>32)) || write_u32(f, v & 0xffffffff))
        return -1;

    return 0;
}

void
my_gdkpixbufdestroy (guchar *pixels, gpointer data)
{
    free (pixels);
}

gint
callback_func (GtkWidget *widget, GdkEventKey *event, gpointer callback_data)
{
/*state_t *state = (state_t*) callback_data;
    image_source_t *isrc = state->isrc;

    switch (event->keyval) {
        case GDK_KEY_Tab: {
            printf("tab\n");
            state->fidx = (state->fidx + 1) % isrc->num_formats(isrc);
            pthread_mutex_lock(&state->mutex);
            isrc->stop(isrc);
            isrc->set_format(isrc, state->fidx);
            printf("set format %d\n", state->fidx);
            isrc->start(isrc);
            pthread_mutex_unlock(&state->mutex);
            break;
        }
        case GDK_KEY_s:
        case GDK_KEY_S:{
            if(state->isTaking == 0){
                state->record_islog = fopen("pic.ppm","w");
                state->isTaking = 1;
            }
            break;
        }
    }
    printf("got callback\n");*/
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

static void
maebot_laser_scan_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                     const maebot_laser_scan_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");

    /*printf ("Subscribed to channel: MAEBOT_SENSOR_DATA\n");
    printf ("utime: %"PRId64"\n", msg->utime);
    printf ("accel[0, 1, 2]:        %d,\t%d,\t%d\n",
            msg->accel[0], msg->accel[1], msg->accel[2]);
    printf ("gyro[0, 1, 2]:         %d,\t%d,\t%d\n",
            msg->gyro[0], msg->gyro[1], msg->gyro[2]);
    printf ("gyro_int[0, 1, 2]:     %"PRId64",\t%"PRId64",\t%"PRId64"\n",
            msg->gyro_int[0], msg->gyro_int[1], msg->gyro_int[2]);
    printf ("line_sensors[0, 1, 2]: %d,\t%d,\t%d\n",
            msg->line_sensors[0], msg->line_sensors[1], msg->line_sensors[2]);
    printf ("range: %d\n", msg->range);
    printf ("user_button_pressed: %s\n", msg->user_button_pressed ? "true" : "false");
    printf ("power_button_pressed: %s\n", msg->power_button_pressed ? "true" : "false");*/
    printf("Subscribed to channel: MAEBOT_LASER_SCAN\n");
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

    //g_type_init();
    //gtk_init (&argc, &argv);
    //gdk_threads_init();

    //state->window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    //gtk_window_set_title(GTK_WINDOW(state->window), state->url);

    //pthread_mutex_init(&state->mutex, NULL);
    //state->image = gtk_image_new();

    //gtk_container_add(GTK_CONTAINER(state->window), state->image);
    //gtk_widget_show(state->image);
    //gtk_widget_show(state->window);

    //g_signal_connect(state->window, "key_press_event",G_CALLBACK(callback_func), state);

    //////////////////////////////////////////////////////////
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

    //pthread_create(&state->runthread, NULL, runthread, state);
    //lcm_t *lcm_lidar = lcm_create(NULL);
    //if(!lcm_lidar)
    //return 1;
    //maebot_laser_scan_t_subscribe(lcm_lidar, "MAEBOT_LASER_SCAN", maebot_laser_scan_handler, NULL);
    // Init msg
    // no need for mutex here, as command thread hasn't started yet.
    msg.motor_left_speed = MTR_STOP;
    msg.motor_right_speed = MTR_STOP;
    // Start sending motor commands
    pthread_t diff_drive_thread_pid;
    pthread_create (&diff_drive_thread_pid, NULL, diff_drive_thread, NULL);

    int round = 0;
    int pic_num = 0;
    for (round = 0; round < 6; ++round) {
        //forward2
        pthread_mutex_lock (&msg_mutex);
        msg.motor_left_speed = MTR_SPD*LEFT_MOTOR_COEFF;
        msg.motor_right_speed = MTR_SPD*RIGHT_MOTOR_COEFF;
        pthread_mutex_unlock (&msg_mutex);
        usleep(FORWARD1);
        //take pic
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
        //take pic
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
        //right turn
        pthread_mutex_lock (&msg_mutex);
        msg.motor_left_speed = MTR_TURN_SPD*LEFT_MOTOR_COEFF;
        msg.motor_right_speed = -MTR_TURN_SPD*RIGHT_MOTOR_COEFF;
        pthread_mutex_unlock (&msg_mutex);
        usleep (TURNTIME);
    }
    pthread_mutex_lock (&msg_mutex);
    msg.motor_left_speed = MTR_STOP;
    msg.motor_right_speed = MTR_STOP;
    pthread_mutex_unlock (&msg_mutex);
    printf("stop");
    return 0;
}
