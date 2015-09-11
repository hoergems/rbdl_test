# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-g -c -std=c++11

INCLUDEFLAGS=-I/usr/local/include/rbdl/ -I/usr/include/eigen3
LDFLAGS=-lrbdl -lrbdl_urdfreader

all: rbdl clean

rbdl: rbdl.o
	$(CC) $(INCLUDEFLAGS) $^ -o $@ $(LDFLAGS)
	
rbdl.o: rbdl_test.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) $< -o $@
	
clean:
	rm *.o
