include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
     $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES) \
     $(CFLAGS_VX_GL) \
	 $(CFLAGS_VX) \
	 $(CFLAGS_GTK) \
	 -O2

LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GL) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_LCMTYPES) \
	  $(LDFLAGS_VX) \
	  $(LDFLAGS_GTK)

LIBDEPS = $(call libdeps, $(LDFLAGS))
LIB_VXGTK = $(LIB_PATH)/libvxgtk.a
LIB_EECS467 = $(LIB_PATH)/libeecs467.a
LIBEECS467_OBJS = eecs467_util.o
VXGTK_OBJS = vx_gtk_display_source.o vx_gtk_display.o gtkuimagepane.o vx_gtk_buffer_manager.o
BIN_REXARM_DEBUG_GUI = $(BIN_PATH)/rexarm_debug_gui
BIN_REXARM_SEQUENCE_TEST = $(BIN_PATH)/rexarm_sequence_test
BIN_EECS467_MAEBOT_EXAMPLE = $(BIN_PATH)/eecs467_maebot_example
BIN_EECS467_MAEBOT_VIDEO_TELOP = $(BIN_PATH)/eecs467_maebot_video_telop
BIN_REXARM_LISTEN = $(BIN_PATH)/rexarm_listening

ALL = $(LIB_EECS467) $(BIN_VX_TEST) $(BIN_REXARM_SEQUENCE_TEST)

all: $(ALL)

$(LIB_EECS467): $(LIBEECS467_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $(LIBEECS467_OBJS)

$(BIN_REXARM_DEBUG_GUI): rexarm_debug_gui.o $(LIB_EECS467) $(LIBDEPS) $(LIB_VXGTK) $(LIB_VXGL) $(LIB_VX)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_REXARM_SEQUENCE_TEST): rexarm_sequence_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_REXARM_LISTEN): rexarm_listening.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)