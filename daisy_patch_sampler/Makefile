TARGET      = daisy_patch_sampler
CPP_SOURCES = daisy_patch_sampler.cpp

# Minimal ff_convert / ff_wtoupper stubs for ASCII-only IR filenames.
# Replaces the full ccsbcs.c code-page table (~1.1 KB) to stay within FLASH budget.
C_SOURCES  += ffascii.c

# CMSIS-DSP: floating-point FIR filter (block-based convolution for CONV mode).
# Not pre-compiled into libdaisy.a, so we compile these two files directly.
# ARM_MATH_LOOPUNROLL enables the 8-accumulator inner loop in arm_fir_f32, which
# breaks the serial VMLA dependency chain. Without it, M7's 5-cycle VMLA latency
# serialises each MAC, costing ~6 cycles/tap instead of ~1 cycle/tap, and the
# 1024-tap × 2-channel FIR overruns the audio budget (~590K vs 480K cycles).
CMSIS_DSP_SRC = ../libDaisy/Drivers/CMSIS-DSP/Source/FilteringFunctions
C_SOURCES     += $(CMSIS_DSP_SRC)/arm_fir_f32.c
C_SOURCES     += $(CMSIS_DSP_SRC)/arm_fir_init_f32.c
C_INCLUDES    += -I../libDaisy/Drivers/CMSIS-DSP/PrivateInclude
C_DEFS        += -DARM_MATH_LOOPUNROLL

LIBDAISY_DIR = ../libDaisy
DAISYSP_DIR  = ../DaisySP

# Run from external QSPI flash (XIP). Requires Daisy bootloader.
# Flash with: make program-dfu  (NOT make program — OpenOCD can't target QSPI)
APP_TYPE = BOOT_QSPI

OPT = -Os -g

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

ui-only: C_DEFS += -DPROJECT_UI_ONLY
ui-only: all

debug: OPT = -Og -g
debug: all

debug-ui-only: OPT = -Og -g
debug-ui-only: C_DEFS += -DPROJECT_UI_ONLY
debug-ui-only: all
