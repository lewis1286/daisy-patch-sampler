TARGET      = daisy_patch_sampler
CPP_SOURCES = daisy_patch_sampler.cpp

LIBDAISY_DIR = ../libDaisy
DAISYSP_DIR  = ../DaisySP

OPT = -O2 -g

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

ui-only: C_DEFS += -DPROJECT_UI_ONLY
ui-only: all

debug: OPT = -Og -g
debug: all

debug-ui-only: OPT = -Og -g
debug-ui-only: C_DEFS += -DPROJECT_UI_ONLY
debug-ui-only: all
