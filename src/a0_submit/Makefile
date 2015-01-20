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
	 $(CFLAGS_USB) \
	 $(CFLAGS_PNG) \
	 $(CFLAGS_DC1394) \
	 -O2

LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GL) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_LCMTYPES) \
	  $(LDFLAGS_VX) \
	  $(LDFLAGS_GTK) \
	  $(LDFLAGS_USB) \
	  $(LDFLAGS_PNG) \
	  $(LDFLAGS_DC1394)

LIBDEPS = $(call libdeps, $(LDFLAGS))
LIB_VXGTK = $(LIB_PATH)/libvxgtk.a
LIB_EECS467 = $(LIB_PATH)/libeecs467.a
LIBEECS467_OBJS = eecs467_util.o
LIB_IMAGESOURCE = $(LIB_PATH)/libimagesource.a
VXGTK_OBJS = vx_gtk_display_source.o vx_gtk_display.o gtkuimagepane.o vx_gtk_buffer_manager.o
LIBIMAGESOURCE_OBJS = \
	image_convert.o \
	image_source.o \
	image_source_dc1394.o \
	image_source_filedir.o \
	image_source_islog.o \
	image_source_null.o \
	image_source_pgusb.o \
	image_source_tcp.o \
	image_source_v4l2.o \
	image_u32.o \
	image_u8.o \
	image_u8x3.o \
	image_util.o \
        pnm.o
BIN_REXARM_DEBUG_GUI = $(BIN_PATH)/rexarm_debug_gui
BIN_REXARM_SEQUENCE_TEST = $(BIN_PATH)/rexarm_sequence_test
BIN_ISVIEW_PPM = $(BIN_PATH)/isview_ppm
BIN_MAEBOT_TEST = $(BIN_PATH)/maebot_test
BIN_MAEBOT_GUI = $(BIN_PATH)/maebot_gui

ALL = $(LIB_EECS467) $(BIN_REXARM_DEBUG_GUI) $(BIN_REXARM_SEQUENCE_TEST) $(BIN_ISVIEW_PPM) $(BIN_MAEBOT_TEST) $(BIN_MAEBOT_GUI)

all: $(ALL)

$(LIB_EECS467): $(LIBEECS467_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $(LIBEECS467_OBJS)

$(BIN_REXARM_DEBUG_GUI): rexarm_debug_gui.o $(LIB_EECS467) $(LIBDEPS) $(LIB_VXGTK) $(LIB_VXGL) $(LIB_VX)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_MAEBOT_GUI): maebot_gui.o $(LIB_EECS467) $(LIBDEPS) $(LIB_VXGTK) $(LIB_VXGL) $(LIB_VX)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_REXARM_SEQUENCE_TEST): rexarm_sequence_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_ISVIEW_PPM): isview_ppm.o $(LIB_IMAGESOURCE) $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_MAEBOT_TEST): maebot_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
