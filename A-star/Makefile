# Makefile for your SDL2 project

# Compiler and flags
CC = g++
CFLAGS = -Isrc/include -Isdl_image/include -Iinclude -Lsrc/lib -Lsdl_image/lib  
LIBS = -lmingw32 -lSDL2_image -lSDL2main -lSDL2 

# Source files and output binary
SRCS = main.cpp
OBJS = $(SRCS:.cpp=.o)
OUT = main

#Headers and dependencies
HEADERS = SDL_helper_functions.h include/A_Star.hpp

# Targets
all: $(OUT)

$(OUT): $(OBJS)
	$(CC) $(CFLAGS) -o $(OUT) $(OBJS) $(LIBS)

%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(OBJS) $(OUT)
