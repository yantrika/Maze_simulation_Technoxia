# Compiler and flags
CC = gcc
CFLAGS = -O2 -Wall -Wextra
WEBOTS_HOME_PATH = $(WEBOTS_HOME)
INCLUDE = -I"$(WEBOTS_HOME_PATH)/include/controller/c"
LIBS = -L"$(WEBOTS_HOME_PATH)/lib/controller" -lController -lm

# Sources and output
TARGET = LHA_2
SOURCES = main.c movement.c
OBJECTS = $(SOURCES:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)
