#Makefile
#char code "LF(UNIX)" is required!!

OPTIONS = -g -Wall
PROGS = ixgbe

all: $(PROGS)

clean :
	rm -f $(PROGS)
	rm -f *.o

ixgbe : main.o
	$(CC) $(OPTIONS) -o $@ main.o
	
main.o : main.c
	$(CC) $(OPTIONS) -c main.c

