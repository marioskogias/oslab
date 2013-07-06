CC = gcc


CFLAGS = -Wall 
CFLAGS += -g
# CFLAGS += -O2 -fomit-frame-pointer -finline-functions

LIBS = 

BINS = cryptosock decryptosock

all: $(BINS)

cryptosock: cryptosock.c
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)

decryptosock: decryptosock.c
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)
clean:
	rm -f *.o *~ $(BINS)
