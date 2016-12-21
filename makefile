UNAME := $(shell uname)


ifeq ($(UNAME), Linux)
    CC=clang++
    LFLAGS =
    TESTFILE=
endif
ifeq ($(UNAME), Darwin)
    CC=g++-6
    LFLAGS = -I"include"
    TESTFILE=
endif
CFLAGS =  -O2 -std=c++11 -I"include" 

CPP_FILES := $(wildcard src/*.cpp)
OBJ_DIR := obj
OBJ_FILES := $(addprefix $(OBJ_DIR)/,$(notdir $(CPP_FILES:.cpp=.o)))


all: $(OBJ_FILES)	
	mkdir -p bin
	${CC} $(LFLAGS) -o bin/main $^

$(OBJ_FILES) : | $(OBJ_DIR)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: src/%.cpp
	${CC} $(CFLAGS) -c -o $@ $<

clean: 
	rm ${OBJ_FILES}

test:
	./bin/main

