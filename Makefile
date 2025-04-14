# Project Name
TARGET = Tap-O-Matic

USE_DAISYSP_LGPL=1
#DEBUG=1

# Sources
CPP_SOURCES = time_machine_hardware.cpp TimeMachine.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy/
DAISYSP_DIR = ../../DaisySP/

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
