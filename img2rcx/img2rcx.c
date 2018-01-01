#include <stdio.h>
#include <stdlib.h>
#include <semaphore.h>
#include <errno.h>
#include <string.h>
#include "lnphost.h"
#include "stb_image.h"

#define DEVICE_NAME "/dev/usb/legousbtower0"
#define RCX_ADDRESS 0x01
#define TOWER_ADDRESS 0x81
#define MAX_BLOCK_SIZE 251

#define SETUP_BLOCK_NUM 0

#define ASSEMBLE_INT(a, b) (((a) << 8) | (b))
#define UPPER_BYTE(a) ((a) >> 8)
#define LOWER_BYTE(a) ((a) & 0xff)

int width, height;
unsigned char *image;

struct lnptower tower;
unsigned char block_size;

volatile enum {
	sending = 0, awaiting_block_request, finished
} state = awaiting_block_request;

volatile unsigned int block_num = SETUP_BLOCK_NUM, block_count;
sem_t sem_waiting;

void send_block() {
	unsigned char packet[block_size + 2];
	memset(packet + 2, 0, block_size);
	packet[0] = UPPER_BYTE(block_num);
	packet[1] = LOWER_BYTE(block_num);
	printf("Sending block %d.\n", block_num);
	int bit = 0, byte = 0;
	while (byte < block_size) {
		int pixel = ((block_num - 1) * block_size + byte) * 8 + bit;
		packet[byte + 2] |= (image[pixel] != 255) << bit;
		if (++bit == 8) {
			bit = 0;
			++byte;
		}
		if (pixel == width * height - 1) break;
	}
	if (lnp_addressing_send(&tower, packet, block_size + 2, RCX_ADDRESS, TOWER_ADDRESS)) {
		fprintf(stderr, "Error while sending block.\n");
	}
}

void send_setup() {
	printf("Sending setup packet.\n");
	unsigned char packet[9] = {
		UPPER_BYTE(block_num), LOWER_BYTE(block_num),
		UPPER_BYTE(width), LOWER_BYTE(width),
		UPPER_BYTE(height), LOWER_BYTE(height),
		UPPER_BYTE(block_count), LOWER_BYTE(block_count),
		block_size
	};
	if(lnp_addressing_send(&tower, packet, 9, RCX_ADDRESS, TOWER_ADDRESS)) {
		fprintf(stderr, "Error while sending setup.\n");
	}
}

void packet_handler(unsigned char *data, unsigned char length,
		unsigned char source, unsigned char dest) {
	int not_waiting;
	sem_getvalue(&sem_waiting, &not_waiting);
	if (source != RCX_ADDRESS) {
		fprintf(stderr, "Received packet from %#04x.\n", source);
	} else if (length != 2) {
		fprintf(stderr, "Unexpected packet length: %u\n", length);
	} else if (not_waiting || state != awaiting_block_request) {
		fprintf(stderr, "Not awaiting packet.\n");
	} else {
		unsigned int requested_block = ASSEMBLE_INT(data[0], data[1]);
		printf("Recieved request for block %u.\n", requested_block);
		if (requested_block > block_count) {
			state = finished;
		} else {
			state = sending;
			block_num = requested_block;
		}
		sem_post(&sem_waiting);
	}
}

void print_image() {
	printf("Image: width = %d, height = %d, block_size = %u, block_count = %d.\n",
		width, height, block_size, block_count);
	if (width > 40) return;
	int pixel = 0;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			printf("%c", image[pixel++] == 255 ? '.' : 'X');
		}
		printf("\n");
	}
}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		fprintf(stderr, "Pass path to image as parameter.\n");
		return 1;
	}

	int channels;
	image = stbi_load(argv[1], &width, &height, &channels, 1);
	
	block_size = MAX_BLOCK_SIZE;
	block_count = (width * height + block_size * 8 - 1) / (block_size * 8);

	if (lnp_open(&tower, DEVICE_NAME, 0)) {
		fprintf(stderr, "Failed to connect to tower.\n");
		return 1;
	}

	if (sem_init(&sem_waiting, 0, 1)){
		perror("Error during sem_init.");
		return 1;
	}

	lnp_addressing_set_handler(&tower, TOWER_ADDRESS, packet_handler);

	print_image();

	printf("Awaiting first block request...\n");
	while (state != finished) {
		if(sem_wait(&sem_waiting)) {
			perror("Error during sem_wait.");
		}
		if (state == sending) {
			if (block_num == SETUP_BLOCK_NUM) {
				send_setup();
			} else {
				send_block();
			}
			state = awaiting_block_request;
		}
	}

	printf("Image transmitted.\n");

	lnp_close(&tower);
	sem_destroy(&sem_waiting);
	stbi_image_free(image);
}