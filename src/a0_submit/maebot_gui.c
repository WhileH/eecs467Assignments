#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <inttypes.h>
#include <stdint.h>
// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"

typedef struct
{
    int running;
    image_u32_t *img;
    vx_application_t app;
    vx_world_t * world;
    vx_event_handler_t *vxeh;
    zhash_t * layers;
    pthread_mutex_t mutex; // for accessing the arrays
    pthread_t animate_thread;
} state_t;

typedef struct
{
    int64_t utime;
    float x;
    float y;
    float theta;
} odometry_t;

typedef struct
{
    float theta;
    float v_xr;
    float v_yr;
    float px;
    float py;
} imu_t;

typedef struct
{
    int64_t utime;
    int num_ranges;
    float ranges[500];
    float thetas[500];
} lidar_t;


imu_t *imu_data;
int imu_data_size;
odometry_t *odometry_data;
int odometry_data_size;
lidar_t lidar[4];
int left_tick_start, right_tick_start;
int corner_position[4];
int is_show_lidar;

static int calcPointX(float range,float theta,float origin_x){
    //printf("x :%d ", origin_x+cosf(theta)*range);
    return origin_x+cosf(theta)*range;
}

static int calcPointY(float range,float theta,float origin_y){
    //printf("y :%d\n", origin_y+sinf(theta)*range);
    return origin_y+sinf(theta)*range;
}

static void get_lidar_data(){
    FILE *lcm_data_fp = fopen("lcm_data.txt","r");
    if(lcm_data_fp == NULL){
        printf("Can't open lcm_data.txt file");
    }
    int i,j;
    for (i = 0; i < 4; ++i) {
        int check = fscanf(lcm_data_fp,"%"SCNd64"\n",&(lidar[i].utime));
        check = fscanf(lcm_data_fp,"%d\n",&(lidar[i].num_ranges));
        for (j = 0; j < lidar[i].num_ranges; ++j) {
            check = fscanf(lcm_data_fp,"%f %f\n",&(lidar[i].ranges[j]),&(lidar[i].thetas[j]));
            //printf("%f %f\n",lidar[i].ranges[j],lidar[i].thetas[j]);
        }
    }
    fclose(lcm_data_fp);
}

//function to calculate the distance
static void calc_move(int sL, int sR, int index){
	float dtheta, s, alpha;
    float x_prev = odometry_data[index-1].x;
    float y_prev = odometry_data[index-1].y;
    float theta_prev = odometry_data[index-1].theta;

    int sL_i = sL - left_tick_start;
    int sR_i = sR - right_tick_start;

    // increment left and right starting tick
    left_tick_start = sL;
    right_tick_start = sR;

    //convert ticks to meters
    float sL_f = sL_i/4800.0;
    float sR_f = sR_i/4800.0;

    s = (sR_f + sL_f)/2;
    dtheta = (sR_f - sL_f)/0.08;
    alpha = dtheta/2;
    odometry_data[index].x = s*(cosf(theta_prev + alpha)) + x_prev;
    odometry_data[index].y = s*(sinf(theta_prev + alpha)) + y_prev;
    odometry_data[index].theta = dtheta + theta_prev;
}

static void get_odometry_data()
{
    FILE *fp;
    int64_t timestamp;
    odometry_data = calloc(100000, sizeof(odometry_t));
    fp = fopen("movement_data.txt", "r");
    if(fp == NULL)
    {
        printf("Can't open movement_data.txt!\n");
        exit(1);
    }
    FILE *ofp;
    ofp = fopen("output_odometer.txt", "w");
    if(ofp == NULL)
    {
        printf("Can't open output_odometer.txt!\n");
        exit(1);
    }
    int check = fscanf(fp, "%"SCNd64" %d %d",&timestamp, &left_tick_start, &right_tick_start);
    if(check != 3) {
        printf("error reading movement_data.txt\n");
        exit(1);
    }
    int left_tick, right_tick;
    int index = 0,i = 0,curr = 0;
    odometry_data[index].utime = timestamp;
    odometry_data[index].x = 0;
    odometry_data[index].y = 0;
    odometry_data[index].theta = 0;
    //fprintf(ofp, "%f %f %f\n", odometry_data[index].x, odometry_data[index].y, odometry_data[index].theta);
    index++;

    while(fscanf(fp, "%"SCNd64" %d %d", &timestamp,&left_tick, &right_tick) == 3)
    {
        calc_move(left_tick, right_tick, index);
        odometry_data[index].utime = timestamp;
        for (i = 0; i < 4; i++) {
            if(abs(lidar[i].utime - timestamp) < 30000 && abs(odometry_data[corner_position[curr]].utime - timestamp) > 250000){
                if(curr < 4){
                    corner_position[curr] = index;
                    ++curr;
                    break;
                }
            }
        }
        fprintf(ofp, "%f %f %f\n", odometry_data[index].x, odometry_data[index].y, odometry_data[index].theta);
        index++;
    }
    fprintf(ofp,"%d %d %d %d \n",corner_position[0],corner_position[1],corner_position[2],corner_position[3]);
    fclose(ofp);
    fclose(fp);
    odometry_data_size = index;
}

static void calc_move_acc(int64_t delta_timestamp, int x_acc,int y_acc, int gyro_z, int index)
{
    imu_data[index].theta = imu_data[index-1].theta + (gyro_z/131.0)*delta_timestamp/1000000.0;
    //imu_data[index].v_xr = imu_data[index-1].v_xr + (x_acc/16384.0)*delta_timestamp/1000000.0;
    float delta_v_xw = ((x_acc/16384.0)*cosf(imu_data[index].theta) - (y_acc/16384.0)*sinf(imu_data[index].theta))*delta_timestamp/1000000.0;
    float delta_v_yw = ((x_acc/16384.0)*sinf(imu_data[index].theta) + (y_acc/16384.0)*cosf(imu_data[index].theta))*delta_timestamp/1000000.0;
    imu_data[index].v_xr = imu_data[index-1].v_xr + delta_v_xw;
    imu_data[index].v_yr = imu_data[index-1].v_yr + delta_v_yw;
        //float v_xw = imu_data[index].v_xr*cosf(imu_data[index].theta);
        //float v_yw = imu_data[index].v_xr*sinf(imu_data[index].theta);
    //imu_data[index].px = imu_data[index-1].px + v_xw*delta_timestamp/1000000.0;
    //imu_data[index].py = imu_data[index-1].py + v_yw*delta_timestamp/1000000.0;
    imu_data[index].px = imu_data[index-1].px + imu_data[index].v_xr*delta_timestamp/1000000.0;
    imu_data[index].py = imu_data[index-1].py + imu_data[index].v_yr*delta_timestamp/1000000.0;
}

static void get_imu_data()
{
    int64_t timestamp;
    int x_acc, y_acc, z_acc, gyro_x, gyro_y, gyro_z;
    FILE *fp;
    imu_data = calloc(100000, sizeof(imu_t));
    fp = fopen("real_movement_data.txt", "r");
    if(fp == NULL)
    {
        printf("Can't open real_movement_data.txt!\n");
        exit(1);
    }

    FILE *ofp;
    ofp = fopen("output.txt", "w");
    if(ofp == NULL)
    {
        printf("Can't open output.txt\n");
        exit(1);
    }

    int index = 0;
    imu_data[index].theta = 0;
    imu_data[index].v_xr = 0;
    imu_data[index].v_yr = 0;
    imu_data[index].px = 0;
    imu_data[index].py = 0;
    index++;

    fprintf(ofp, "%"SCNd64" %f %f %f %f\n", 0, imu_data[index].theta, imu_data[index].v_xr, imu_data[index].px, imu_data[index].py);
    int first = fscanf(fp, "%"SCNd64" %d %d %d %d %d %d", &timestamp, &x_acc, &y_acc, &z_acc, &gyro_x, &gyro_y, &gyro_z);
    if(first != 7)
    {
        printf("error reading real_movement_data.txt\n");
        exit(1);
    }
    int64_t prev_timestamp = timestamp;

    while(fscanf(fp, "%"SCNd64" %d %d %d %d %d %d", &timestamp, &x_acc, &y_acc, &z_acc, &gyro_x, &gyro_y, &gyro_z) == 7)
    {
        calc_move_acc(timestamp - prev_timestamp, x_acc, y_acc,gyro_z, index);
        fprintf(ofp, "%"SCNd64" %f %f %f %f\n", (timestamp - prev_timestamp), imu_data[index].theta, imu_data[index].v_xr, imu_data[index].px, imu_data[index].py);
        index++;
        prev_timestamp = timestamp;
    }
    imu_data_size = index;
}

static void draw(state_t * state, vx_world_t * world)
{
    if(!is_show_lidar){
        // draw intended route
        vx_buffer_t *wrld = vx_world_get_buffer(world, "origin");
        float x0 = 0*15,y0 = 0*15,x1 = 0.6096*15,y1=0*15,x2 = 0.6096*15,y2 = -0.9144*15,x3 = 0*15,y3 = -0.9144*15;
        float origin_pts[] = {x0,y0,0,x1,y1,0,x1,y1,0,x2,y2,0,x2,y2,0,x3,y3,0,x3,y3,0,x0,y0,0};
        int origin_pts_size = 8;
        vx_resc_t *origin_verts = vx_resc_copyf(origin_pts, origin_pts_size*3);
        vx_buffer_add_back(wrld, vxo_lines(origin_verts, origin_pts_size, GL_LINES, vxo_lines_style(vx_orange, 2.0f)));
        //vx_buffer_swap(vx_world_get_buffer(world, "origin"));
        // make legend
        int lgnd_pts = 2;
        vx_object_t *lgnd = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> Legend\n");
        vx_object_t *odm = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> Odometry: \n");
        float odm_ln[6] = {20, 0, 0, 0, 0, 0};
        vx_resc_t *odm_verts = vx_resc_copyf(odm_ln, lgnd_pts*3);
        vx_object_t *imu = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> IMU: \n");
        float imu_ln[6] = {20, 0, 0, 0, 0, 0};
        vx_resc_t *imu_verts = vx_resc_copyf(imu_ln, lgnd_pts*3);
        vx_object_t *cmd = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> Command Path: \n");
        float cmd_ln[6] = {20, 0, 0, 0, 0, 0};
        vx_resc_t *cmd_verts = vx_resc_copyf(cmd_ln, lgnd_pts*3);
        vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(100, 180), vxo_mat_scale(0.8), lgnd)));
        vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(100, 165), vxo_mat_scale(0.6), odm)));
        vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(140, 165), vxo_mat_scale(1.0),
                    vxo_lines(odm_verts, lgnd_pts, GL_LINES, vxo_points_style(vx_blue, 2.0f)))));
        vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(117, 150), vxo_mat_scale(0.6), imu)));
        vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(140, 150), vxo_mat_scale(1.0),
                    vxo_lines(imu_verts, lgnd_pts, GL_LINES, vxo_points_style(vx_red, 2.0f)))));
        vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(88, 135), vxo_mat_scale(0.6), cmd)));
        vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(140, 135), vxo_mat_scale(1.0),
                    vxo_lines(cmd_verts, lgnd_pts, GL_LINES, vxo_points_style(vx_orange, 2.0f)))));
        vx_buffer_swap(wrld);
    }
    if(is_show_lidar){
        int i,j,index;
        float px,py;
        float pts[12];
        vx_buffer_t *wrld = vx_world_get_buffer(world, "lidar");
        for (i = 0; i < 4; ++i) {
            float points[3000];
            memset(points,0,3000);
            //printf("plot lidar %d with range %d\n",i,lidar[i].num_ranges);
            index = corner_position[i];
            px = odometry_data[index].x;
            py = odometry_data[index].y;
            pts[3*i+0] = px*15;
            pts[3*i+1] = py*15;
            pts[3*i+2] = 0;
            //printf("%f %f\n",px,py);
            for (j = 0; j < lidar[i].num_ranges; ++j) {
                points[j*6+0] = px*15;
                points[j*6+1] = py*15;
                points[j*6+2] = 0;
                points[j*6+3] = calcPointX(lidar[i].ranges[j]*100,lidar[i].thetas[j],px*15);
                points[j*6+4] = calcPointY(lidar[i].ranges[j]*100,lidar[i].thetas[j],py*15);
                points[j*6+5] = 0;
                //printf("%f %f\n",points[j*6+3],points[j*6+4]);
            }
            vx_resc_t *verts = vx_resc_copyf(points, 500*3);
            vx_buffer_add_back(wrld,vxo_lines(verts, 500, GL_LINES, vxo_lines_style(vx_green, 2.0f)));
        }
        vx_resc_t *vpts = vx_resc_copyf(pts, 4*3);
        vx_buffer_add_back(wrld,vxo_points(vpts, 4, vxo_points_style(vx_red, 5.0f)));
        vx_buffer_swap(wrld);
    }
}

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;
    pthread_mutex_lock(&state->mutex);

    vx_layer_t * layer = NULL;

    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_remove(state->layers, &disp, NULL, &layer);

    vx_layer_destroy(layer);

    pthread_mutex_unlock(&state->mutex);
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;

    vx_layer_t * layer = vx_layer_create(state->world);
    vx_layer_set_display(layer, disp);

    pthread_mutex_lock(&state->mutex);
    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_put(state->layers, &disp, &layer, NULL, NULL);
    pthread_mutex_unlock(&state->mutex);
}

static void state_destroy(state_t * state)
{

    if (state->img != NULL)
        image_u32_destroy(state->img);
    if (state->vxeh)
        free(state->vxeh);
    vx_world_destroy(state->world);
    assert(zhash_size(state->layers) == 0);

    zhash_destroy(state->layers);
    free(state);

    pthread_mutex_destroy(&state->mutex);

}

static state_t * state_create()
{
    state_t * state = calloc(1, sizeof(state_t));
    state->running = 1;
    state->app.impl=state;
    state->app.display_started=display_started;
    state->app.display_finished=display_finished;
    state->world = vx_world_create();
    state->layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
    pthread_mutex_init (&state->mutex, NULL);

    return state;
}

void * render_loop(void * data)
{
    state_t * state = data;
    int count = 0;
    while(state->running && !is_show_lidar) {
        int i=0;
        vx_buffer_t *buf = vx_world_get_buffer(state->world, "odometer");
        while(i < count-1 && i < imu_data_size-1 && count < imu_data_size)
        {
            float pts[] = {imu_data[i].px*15, imu_data[i].py*15, 0,
                           imu_data[i+1].px*15, imu_data[i+1].py*15, 0};
            vx_resc_t *verts = vx_resc_copyf(pts, 6);
            vx_buffer_add_back(buf, vxo_lines(verts, 2, GL_LINES, vxo_lines_style(vx_red, 2.0f)));
            i++;
        }
        i = 0;
        while(i < count-1 && i < odometry_data_size-1 && count < odometry_data_size)
        {
            float pts[] = {odometry_data[i].x*15, odometry_data[i].y*15, 0,
                           odometry_data[i+1].x*15, odometry_data[i+1].y*15, 0};
            vx_resc_t *verts = vx_resc_copyf(pts, 6);
            vx_buffer_add_back(buf, vxo_lines(verts, 2, GL_LINES, vxo_lines_style(vx_blue, 2.0f)));
            i++;
        }
        count++;
        if(count == imu_data_size || count == odometry_data_size)
        {
            count--;
        }
        vx_buffer_swap(buf);
        usleep(5000);
    }

    return NULL;
}

int main(int argc, char ** argv)
{
    get_lidar_data();
    get_odometry_data();
    get_imu_data();

    getopt_t *gopt = getopt_create();
    getopt_add_bool   (gopt, 'h', "help", 0, "Show help");
    getopt_add_bool (gopt, '\0', "no-gtk", 0, "Don't show gtk window, only advertise remote connection");
    getopt_add_string (gopt, '\0', "pnm", "", "Path for pnm file to render as texture (.e.g BlockM.pnm)");
    getopt_add_bool (gopt, '\0', "stay-open", 0, "Stay open after gtk exits to continue handling remote connections");
    getopt_add_bool (gopt, 'l',"lidar",0,"Show lidar image" );
    // parse and print help
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help")) {
        printf ("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage (gopt);
        exit (1);
    }
    if(getopt_get_bool(gopt,"lidar")){
        is_show_lidar = 1;
    }
    vx_global_init(); // Call this to initialize the vx-wide lock. Required to start the GL thread or to use the program library
    //eecs467_init (argc, argv);
    state_t * state = state_create();

    // Load a pnm from file, and repack the data so that it's understandable by vx
    if (strcmp(getopt_get_string(gopt,"pnm"),"")) {
        state->img = image_u32_create_from_pnm(getopt_get_string(gopt, "pnm"));

        printf("Loaded image %d x %d from %s\n",
               state->img->width, state->img->height,
               getopt_get_string(gopt, "pnm"));
    }

    draw(state, state->world);

    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init(&remote_attr);
    remote_attr.advertise_name = "Vx Demo";
    vx_remote_display_source_t * cxn = vx_remote_display_source_create_attr(&state->app, &remote_attr);
    pthread_create(&state->animate_thread, NULL, render_loop, state);

    if (!getopt_get_bool(gopt,"no-gtk")) {
        gdk_threads_init ();
        gdk_threads_enter ();
        gtk_init (&argc, &argv);

        vx_gtk_display_source_t * appwrap = vx_gtk_display_source_create(&state->app);
        GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
        GtkWidget * canvas = vx_gtk_display_source_get_widget(appwrap);
        gtk_window_set_default_size (GTK_WINDOW (window), 1024, 576);
        gtk_container_add(GTK_CONTAINER(window), canvas);
        gtk_widget_show (window);
        gtk_widget_show (canvas); // XXX Show all causes errors!

        g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);
        gtk_main (); // Blocks as long as GTK window is open
        gdk_threads_leave ();

        vx_gtk_display_source_destroy(appwrap);

        // quit when gtk closes? Or wait for remote displays/Ctrl-C
        if (!getopt_get_bool(gopt, "stay-open"))
            state->running = 0;
    }

    pthread_join(state->animate_thread, NULL);
    vx_remote_display_source_destroy(cxn);

    state_destroy(state);
    vx_global_destroy();
    getopt_destroy(gopt);
}
