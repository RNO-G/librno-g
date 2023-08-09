CC=g++
CFLAGS= -Wall -lgsl -lgslcblas -lfftw3

all: test_offsets

test_offsets:  test_block_offsets.o block_offsets.o
	$(CC) -o test_offsets test_block_offsets.o block_offsets.o $(CFLAGS)

test_block_offsets.o: test_block_offsets.c
	$(CC) -c $(CFLAGS) test_block_offsets.c 

block_offsets.o: block_offsets.c
	$(CC) -c $(CFLAGS) block_offsets.c

clean:
	rm -rf *.o