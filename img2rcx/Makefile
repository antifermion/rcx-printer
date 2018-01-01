CC=gcc
CFLAGS=-I. -lpthread -lm -O2
DEPS = lnphost.h stb_image.h
OBJ = img2rcx.o lnphost.o stb_image.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

img2rcx: $(OBJ)
	gcc -o $@ $^ $(CFLAGS)