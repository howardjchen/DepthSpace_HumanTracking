include ../Build/Common/CommonDefs.mak

BIN_DIR = ../Hersy

INC_DIRS = ../../Include =/usr/include/ni

SRC_FILES = ./*.cpp

EXE_NAME = hersy

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut GL
endif

USED_LIBS += OpenNI

LIB_DIRS += ../../Lib
include ../Build/Common/CommonCppMakefile

