#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

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
    zhash_t * layers;
    pthread_mutex_t mutex; // for accessing the arrays
    pthread_t animate_thread;
} state_t;

typedef struct
{
    float x;
    float y;
    float theta;
} odometry_t;

odometry_t *odometry_data;
int left_tick_start, right_tick_start;

//function to calculate the distance
void calc_move(int sL, int sR, int index, FILE*fp){
	float dtheta, s, alpha;
    float x_prev = odometry_data[index-1].x;
    float y_prev = odometry_data[index-1].y;
    float theta_prev = odometry_data[index-1].theta;

    int sL_i = sL - left_tick_start;
    int sR_i = sR - right_tick_start;
    fprintf(fp, "%d %d\n", sL_i, sR_i);

    // increment left and right starting tick
    left_tick_start = sL;
    right_tick_start = sR;

    //convert ticks to meters
    float sL_f = sL_i/4800.0;
    float sR_f = sR_i/4800.0;
    fprintf(fp, "%f %f\n", sL_f, sR_f);

    s = (sR_f + sL_f)/2;
    dtheta = (sR_f - sL_f)/0.08;
    alpha = dtheta/2;
    fprintf(fp, "%f %f %f\n", s, dtheta, alpha);
    odometry_data[index].x = s*(cosf(theta_prev + alpha)) + x_prev;
    odometry_data[index].y = s*(sinf(theta_prev + alpha)) + y_prev;
    odometry_data[index].theta = dtheta + theta_prev;
}

static void get_odometry_data()
{
    FILE *fp;
    odometry_data = calloc(100000, sizeof(odometry_t));
    fp = fopen("movement_data.txt", "r");
    if(fp == NULL)
    {
        printf("Can't open movement_data.txt!\n");
        exit(1);
    }
    
    FILE *ofp;
    ofp = fopen("output.txt", "w");
    if(ofp == NULL)
    {
        printf("Can't open output.txt!\n");
        exit(1);
    }
    
    int check = fscanf(fp, "%d %d", &left_tick_start, &right_tick_start);
    if(check != 2) {
        printf("error reading movement_data.txt\n");
        exit(1);
    }
    int left_tick, right_tick;
    int index = 0;

    odometry_data[index].x = 0;
    odometry_data[index].y = 0;
    odometry_data[index].theta = 0;
    fprintf(ofp, "%f %f %f\n", odometry_data[index].x, odometry_data[index].y, odometry_data[index].theta);
    index++;

    while(fscanf(fp, "%d %d", &left_tick, &right_tick) == 2)
    {
        calc_move(left_tick, right_tick, index, ofp);
        fprintf(ofp, "%f %f %f\n\n", odometry_data[index].x, odometry_data[index].y, odometry_data[index].theta);
        index++;
    }
    printf("final index: %d", index);
}

static void draw(state_t * state, vx_world_t * world)
{
    // draw origin
    float x1=1, x_1=-1, y1=1, y_1=-1, z0=0, x0=0, y0=0;
    float origin_pts[] = {x0, y_1, z0, x0, y1, z0, x_1, y0, z0, x1, y0, z0};
    int origin_pts_size = 4;
    vx_resc_t *origin_verts = vx_resc_copyf(origin_pts, origin_pts_size*3);
    vx_buffer_add_back(vx_world_get_buffer(world, "origin"), vxo_lines(origin_verts, origin_pts_size, GL_LINES, vxo_lines_style(vx_red, 2.0f)));
    vx_buffer_swap(vx_world_get_buffer(world, "origin"));

    // make legend
    int lgnd_pts = 2;
    vx_object_t *lgnd = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<middle, #000000>> Legend\n");
    vx_object_t *odm = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<middle, #000000>> Odometry: \n");
    float odm_ln[6] = {20, 0, 0, 0, 0, 0};
    vx_resc_t *odm_verts = vx_resc_copyf(odm_ln, lgnd_pts*3);
    vx_object_t *imu = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<middle, #000000>> IMU: \n");
    float imu_ln[6] = {20, 0, 0, 0, 0, 0};
    vx_resc_t *imu_verts = vx_resc_copyf(imu_ln, lgnd_pts*3);
    vx_buffer_t *wrld = vx_world_get_buffer(world, "text");
    
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(100, 180), vxo_mat_scale(0.8), lgnd)));
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(100, 165), vxo_mat_scale(0.6), odm)));
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(140, 165), vxo_mat_scale(1.0), 
			     vxo_lines(odm_verts, lgnd_pts, GL_LINES, vxo_points_style(vx_red, 2.0f))))); 
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(117, 150), vxo_mat_scale(0.6), imu)));
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(140, 150), vxo_mat_scale(1.0), 
			     vxo_lines(imu_verts, lgnd_pts, GL_LINES, vxo_points_style(vx_blue, 2.0f)))));
    vx_buffer_swap(wrld);
    
   
    

    // Draw from the vx shape library
    /*if (1) {
        vx_buffer_add_back(vx_world_get_buffer(world, "fixed-cube"),
                           vxo_chain(vxo_mat_translate3(3.0,0,0),
                                     vxo_mat_scale3(2,2,2),
                                     // vxo_box(vxo_mesh_style(vx_orange)))); 
                                     vxo_box(vxo_mesh_style_fancy(vx_orange, vx_orange, vx_white, 1.0, 400.0, 2))));


        vx_buffer_add_back(vx_world_get_buffer(world, "fixed-cube"),
                           vxo_chaiwhile (fscanf(ifp, "%s %d", username, &score) == 2) {n(vxo_mat_translate3(0,3.0,0),
                                     vxo_mat_scale3(1,1,1),
                                     vxo_depth_test(0,
                                                    vxo_box(vxo_mesh_style_solid(vx_green)))));
        vx_buffer_swap(vx_world_get_buffer(world, "fixed-cube"));
    }

    if (1) {
        float Tr = .2;
        float amb[] = {0.0,0.0,0.0};
        float diff[] = {0.0,0.0,0.0};
        float spec[] = {1.0,1.0,1.0};
        float specularity = 1.0;

        int type = 2;

        vx_buffer_add_back(vx_world_get_buffer(world, "window"),
                           vxo_chain(vxo_mat_translate3(0,0,2.5),
                                     vxo_mat_rotate_y(-M_PI/7),
                                     vxo_mat_rotate_z(M_PI/5),
                                     vxo_mat_rotate_x(M_PI/2),
                                     vxo_mat_scale3(10,10,1),
                                     vxo_rect(vxo_mesh_style_fancy(amb, diff, spec, Tr, specularity, type),
                                              vxo_lines_style(vx_black,2))));
        vx_buffer_swap(vx_world_get_buffer(world, "window"));
        vx_buffer_set_draw_order(vx_world_get_buffer(world, "window"), 100);
    }


    if (1) {
        // Draw a custom ellipse:
        int npoints = 35;
        float points[npoints*3];
        for (int i = 0; i < npoints; i++) {
            float angle = 2*M_PI*i/npoints;

            float x = 5.0f*cosf(angle);
            float y = 8.0f*sinf(angle);
            float z = 0.0f;

            points[3*i + 0] = x;
            points[3*i + 1] = y;
            points[3*i + 2] = z;
        }

        vx_buffer_add_back(vx_world_get_buffer(world, "ellipse"), vxo_lines(vx_resc_copyf (points, npoints*3),
                                                                            npoints, GL_LINE_LOOP,
                                                                            vxo_lines_style(vx_purple, 1.0f) ));
        vx_buffer_swap(vx_world_get_buffer(world, "ellipse"));
    }

    if (1) {
        vx_object_t *vt = vxo_text_create(VXO_TEXT_ANCHOR_TOP_RIGHT, "<<right,#0000ff>>Heads Up!\n");
        vx_buffer_t *vb = vx_world_get_buffer(world, "text");
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP_RIGHT,vt));
        vx_buffer_swap(vb);
    }

    // Draw a texture
    if (state->img != NULL){
        image_u32_t * img = state->img;
        vx_object_t * o3 = vxo_image_texflags(vx_resc_copyui(img->buf, img->stride*img->height),
                                              img->width, img->height, img->stride,
                                              GL_RGBA, VXO_IMAGE_FLIPY,
                                              VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

        // pack the image into the unit square
        vx_buffer_t * vb = vx_world_get_buffer(world, "texture");
        vx_buffer_add_back(vb,vxo_chain(
                               vxo_mat_scale(1.0/img->height),
                               vxo_mat_translate3(0, - img->height, 0),
                                        o3));
        vx_buffer_swap(vb);
    }*/
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
    while(state->running) {
        double rad = (vx_util_mtime() % 5000) * 2* M_PI / 5e3;

        vx_object_t * vo = vxo_chain(vxo_mat_rotate_z(rad),
                                     vxo_mat_translate2(0,10),
                                     vxo_box(vxo_mesh_style(vx_blue)));

        vx_buffer_add_back(vx_world_get_buffer(state->world, "rotating-square"), vo);
        vx_buffer_swap(vx_world_get_buffer(state->world, "rotating-square"));
        usleep(5000);
    }

    return NULL;
}

int main(int argc, char ** argv)
{
    get_odometry_data();

    getopt_t *gopt = getopt_create();
    getopt_add_bool   (gopt, 'h', "help", 0, "Show help");
    getopt_add_bool (gopt, '\0', "no-gtk", 0, "Don't show gtk window, only advertise remote connection");
    getopt_add_string (gopt, '\0', "pnm", "", "Path for pnm file to render as texture (.e.g BlockM.pnm)");
    getopt_add_bool (gopt, '\0', "stay-open", 0, "Stay open after gtk exits to continue handling remote connections");

    // parse and print help
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help")) {
        printf ("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage (gopt);
        exit (1);
    }

    vx_global_init(); // Call this to initialize the vx-wide lock. Required to start the GL thread or to use the program library

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
        gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
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
