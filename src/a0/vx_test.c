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

#define NUM_SERVOS 6

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



static void draw(state_t * state, vx_world_t * world)
{
    //draw text
    if (1) {
        vx_object_t *vt0 = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<middle,#000000>>Servo 0\n");
        vx_object_t *vt1 = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<middle,#000000>>Servo 1\n");
        vx_object_t *vt2 = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<middle,#000000>>Servo 2\n");
        vx_object_t *vt3 = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<middle,#000000>>Servo 3\n");
        vx_object_t *vt4 = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<middle,#000000>>Servo 4\n");
        vx_object_t *vt5 = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<middle,#000000>>Servo 5\n");
        vx_buffer_t *vb = vx_world_get_buffer(world, "text");
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-365,0),vxo_mat_scale(1.0),vt0)));
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-219,0),vxo_mat_scale(1.0),vt1)));
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-73,0),vxo_mat_scale(1.0),vt2)));
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(73,0),vxo_mat_scale(1.0),vt3)));
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(219,0),vxo_mat_scale(1.0),vt4)));
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(365,0),vxo_mat_scale(1.0),vt5)));
        vx_buffer_swap(vb);
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
    //modify this
    state_t * state = data;
    while(state->running) {
        int angle[6] = {-15,30,-45,60,-75,90};
        int position[6] = {-365,-219,-73,73,219,365};
        vx_object_t * vo[6];
        for(int id=0;id<NUM_SERVOS;++id){
            if(angle[id] < 0){
                vo[id] = vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(position[id],angle[id]/2-10),vxo_mat_scale2(60,angle[id]),vxo_rect(vxo_mesh_style(vx_red))));
            }
            else{
                 vo[id] = vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(position[id],angle[id]/2+10),vxo_mat_scale2(60,angle[id]),vxo_rect(vxo_mesh_style(vx_blue))));
            }
        }
        vx_buffer_t *vb = vx_world_get_buffer(state->world, "bar");
        for (int id=0; id < NUM_SERVOS; ++id) {
             vx_buffer_add_back(vb, vo[id]);
        }
        vx_buffer_swap(vb);
        usleep(5000);
    }
    return NULL;
}

int main(int argc, char ** argv)
{
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