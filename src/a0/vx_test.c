#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "eecs467_util.h"    // This is where a lot of the internals live

typedef struct state state_t;
struct state{
    bool running;
    parameter_gui_t *pg;
    vx_application_t vxapp;
    vx_world_t      *vxworld;
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    pthread_t animate_thread;
};

state_t *
state_create(void)
{
    state_t *state = calloc(1,sizeof(*state));
    state->vxworld = vx_world_create();
    state->vxeh = calloc (1, sizeof(*state->vxeh));
    //state->vxeh->key_event = key_event;
    //state->vxeh->mouse_event = mouse_event;
    //state->vxeh->touch_event = touch_event;
    //state->vxeh->dispatch_order = 100;
    state->vxeh->impl = state; // this gets passed to events, so store useful struct here!
    state->vxapp.display_started = eecs467_default_display_started;
    state->vxapp.display_finished = eecs467_default_display_finished;
    state->vxapp.impl = eecs467_default_implementation_create (state->vxworld, state->vxeh);
    state->running = 1;
    return state;
}

void
state_destroy (state_t *state)
{
    if(!state)
        return;
    if (state->vxeh)
        free (state->vxeh);
    if (state->pg)
        pg_destroy (state->pg);
    free(state);
}

void *
animate_thread (void *data)
{
    state_t *state = data;
    vx_object_t *vxo_sphere = vxo_chain (vxo_mat_rotate_z (1),
        vxo_mat_translate2 (0, 0.5),
        vxo_mat_scale (0.1),
        vxo_sphere (vxo_mesh_style (vx_blue)));
    // Then, we add this object to a buffer awaiting a render order
    vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "rot-sphere"), vxo_sphere);
    vx_buffer_swap (vx_world_get_buffer (state->vxworld, "rot-sphere"));
    return NULL;
}

int
main(int argc, char *argv[])
{
    eecs467_init(argc,argv);
    state_t *state = state_create();
    state->pg = pg_create ();
     // Launch our worker threads
    pthread_create (&state->animate_thread, NULL, animate_thread, state);
    // This is the main loop
    eecs467_gui_run (&state->vxapp, state->pg, 1024, 768);

    state->running = 0;
    pthread_join (state->animate_thread, NULL);
    state_destroy(state);
    vx_global_destroy();
    return 0;
}
