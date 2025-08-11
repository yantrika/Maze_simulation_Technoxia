# Corrected Makefile for khepra4 controller

WEBOTS_HOME = C:/Webots
INCLUDE = -I"$(WEBOTS_HOME)/include/controller/c"
LIBS = -L"$(WEBOTS_HOME)/lib/controller" -lController
CC = "$(WEBOTS_HOME)/msys64/mingw64/bin/gcc.exe"

TARGET = khepra4
SRC = $(TARGET).c
BIN = $(TARGET).exe

all:
	$(CC) -Wall -Wextra -O2 $(INCLUDE) $(SRC) -o $(BIN) $(LIBS)

clean:
	del $(BIN)
