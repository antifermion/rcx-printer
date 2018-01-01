#include <conio.h>
#include <unistd.h>
#include <dmotor.h>
#include <dsensor.h>
#include <tm.h>
#include <time.h>
#include <string.h>
#include <dsound.h>
#include <lnp/lnp.h>

#define motor_x_dir motor_c_dir
#define motor_y_dir motor_a_dir
#define motor_z_dir motor_b_dir

#define PEN_DOWN_SENSOR SENSOR_3
#define PEN_DOWN TOUCH(PEN_DOWN_SENSOR)
#define PEN_UP_DIR fwd
#define PEN_DOWN_DIR rev

#define X_PLUS_DIR fwd
#define X_MINUS_DIR rev
#define X_HOME PEN_DOWN
#define X_ROTATION_SENSOR SENSOR_2
#define X_POS ROTATION_2
#define MAX_X 1180

#define Y_LIGHT_SENSOR SENSOR_1
#define BLACK_THRESH 0xc600
#define WHITE_THRESH 0xb000
#define BLACK 0
#define WHITE 1
#define UNDEFINED 2
#define Y_PLUS_DIR fwd
#define Y_MINUS_DIR rev

#define X_PIXEL_SIZE 7
#define Y_PIXEL_SIZE 3
#define IMG_START_X 0
#define X_WIGGLE 10
#define Y_WIGGLE 10
#define Y_FEED 20
#define INTERMEDIATE_SLEEP 100
#define PEN_UP_DURATION 300

#define RCX_PORT 1
#define TOWER_ADDRESS 0x81
#define MAX_BLOCK_SIZE 251

#define SETUP_BLOCK_NUM 0

#define TIMEOUT 3000

#define ASSEMBLE_INT(a, b) (((a) << 8) | (b))
#define UPPER_BYTE(a) ((a) >> 8)
#define LOWER_BYTE(a) ((a) & 0xff)

int block_size;
int width, height, x = 0, y = 0;
unsigned int block_count;
volatile unsigned int block_num = SETUP_BLOCK_NUM;
unsigned char block[MAX_BLOCK_SIZE];

volatile enum {
	awaiting_block, sending_block_request, printing_block, finished
} state = sending_block_request;

void show_error(char *err) {
	dsound_system(0);
	cputs(err);
}

void packet_handler(const unsigned char* data, unsigned char length, unsigned char src) {
	if (src != TOWER_ADDRESS) {
	} else if (state != awaiting_block) {
		show_error("eup");
	} else if (length < 9) {
		show_error("eul");
	} else {
		unsigned received_block_num = ASSEMBLE_INT(data[0], data[1]);
		if (received_block_num == block_num) {
			if (block_num == SETUP_BLOCK_NUM) {
				if (length != 9) {
					show_error("euls");
				} else {
					cputs("rset");
					width = ASSEMBLE_INT(data[2], data[3]);
					height = ASSEMBLE_INT(data[4], data[5]);
					block_count = ASSEMBLE_INT(data[6], data[7]);
					block_size = data[8];
					++block_num;
					state = sending_block_request;
				}
			} else {
				if (length != block_size + 2) {
					show_error("eulb");
				} else {
					cputs("rblk");
					memcpy(block, data + 2, block_size);
					state = printing_block;
				}
			}
		} else {
			show_error("ewb");
			state = sending_block_request;
		}
	}
}

wakeup_t received_block_or_timeout(time_t send_time) {
	return state != awaiting_block || get_system_up_time() - send_time > TIMEOUT;
}

void send_block_request() {
	const unsigned char packet[2] = {
		UPPER_BYTE(block_num), LOWER_BYTE(block_num)};
	lnp_addressing_write(packet, 2, TOWER_ADDRESS, RCX_PORT);
}

void send_block_request_timeout() {
	cputs("sbr");
	state = awaiting_block;
	send_block_request();
	wait_event(received_block_or_timeout, get_system_up_time());
	if (state == awaiting_block) {
		show_error("to");
		state = sending_block_request;
	}
}

inline char y_color() {
	unsigned sensor_val = Y_LIGHT_SENSOR;
	if (sensor_val > BLACK_THRESH) return BLACK;
	if (sensor_val < WHITE_THRESH) return WHITE;
	return UNDEFINED;
}

void home_x() {
	motor_x_dir(X_PLUS_DIR);
	while(!X_HOME);
	motor_x_dir(X_MINUS_DIR);
	while(X_HOME);
	motor_x_dir(brake);
	ds_rotation_set(&X_ROTATION_SENSOR, MAX_X);
}

char color = BLACK;
void init_y_sensor() {
	motor_y_dir(Y_PLUS_DIR);
	while(y_color() != WHITE);
	while(y_color() != BLACK);
	motor_y_dir(brake);
}

void pen_tap() {
	motor_z_dir(PEN_DOWN_DIR);
	while(!PEN_DOWN);
	motor_z_dir(PEN_UP_DIR);
	msleep(PEN_UP_DURATION);
	motor_z_dir(brake);
}

void set_x(int x) {
	if (x < X_POS) {
		motor_x_dir(X_MINUS_DIR);
	} else if (x > X_POS) {
		motor_x_dir(X_PLUS_DIR);
	}
	while (X_POS != x);
	motor_x_dir(brake);
}

void move_y(int dy) {
	if (dy > 0) {
		motor_y_dir(Y_PLUS_DIR);
	} else {
		motor_y_dir(Y_MINUS_DIR);
		dy = -dy;
	}

	while (dy > 0) {
		char new_color = y_color();
		if (new_color != UNDEFINED && new_color != color) {
			--dy;
			color = new_color;
		}
	}

	motor_y_dir(brake);
}

void setup() {
	motor_a_speed(255);
	motor_b_speed(255);
	motor_c_speed(255);

	ds_passive(&PEN_DOWN_SENSOR);
	ds_active(&X_ROTATION_SENSOR);
	ds_rotation_on(&X_ROTATION_SENSOR);
	ds_active(&Y_LIGHT_SENSOR);

	home_x();

	init_y_sensor();
	move_y(Y_FEED);
	set_x(IMG_START_X - X_WIGGLE);
}

void print_block() {
	cputs("prnt");
	msleep(500);
	lcd_int(block_num);

	int pixel;
	for (pixel = 0; pixel < block_size * 8 && y < height; ++pixel) {
		if (block[pixel >> 3] & (1 << (pixel & 7))) {
			set_x(IMG_START_X + x * X_PIXEL_SIZE);
			//msleep(INTERMEDIATE_SLEEP);
			pen_tap();
		}
		if (++x == width) {
			++y;
			move_y(Y_PIXEL_SIZE);
			//msleep(INTERMEDIATE_SLEEP);
			x = 0;
			set_x(IMG_START_X - X_WIGGLE);
		}
	}
	if (++block_num > block_count) {
		cputs("fin");
		state = finished;
		send_block_request();
	} else {
		state = sending_block_request;
	}
}

int main() {
	setup();
	msleep(500);

	lnp_addressing_set_handler(RCX_PORT, packet_handler);

	while (state != finished) {
		switch(state) {
			case sending_block_request: send_block_request_timeout(); break;
			case printing_block: print_block(); break;
			default:;
		}
		msleep(1000);
	}

	lnp_addressing_set_handler(RCX_PORT, LNP_DUMMY_ADDRESSING);
	return 0;
}
