/* =========================================================================== 

    lnphost.h

    Routines to send and receive data packets via the Lego Mindstorms
    IR tower. This library is intended to be a replacement for the core
    functionality of lnpd. Packets in the input stream are detected by a
    heuristic algorithm.

    This is free software, released under the terms of the GNU General
    Public License.

    Stephan Höhrmann, August 2004

=========================================================================== */

#ifndef LNPHOST_H
#define LNPHOST_H

#include <pthread.h>

#if defined(LINUX) || defined(linux)
#define LNP_DEFAULTDEVICE "/dev/ttyS0"
#elif defined (__CYGWIN__) || (_WIN32)
#define LNP_DEFAULTDEVICE "com1"
#else
#define LNP_DEFAULTDEVICE "/dev/ttya"
#endif

#if defined (__CYGWIN__) || defined (_WIN32)
#include <windows.h>
#define FILEHANDLE HANDLE
#else
#define FILEHANDLE int
#endif

#define LNP_RCXTTY_ENV "RCXTTY"
#define LNP_KEEPALIVE 4000
#define RINGBUFSIZE 1024

struct ringbuf {
  int head,tail,fill;
  unsigned char data[RINGBUFSIZE];
};

typedef void (*lnp_raw_handler_t) (unsigned char *data, unsigned length,
				int isvalid);
typedef void (*lnp_lego_handler_t) (unsigned char *data, unsigned length);
typedef void (*lnp_integrity_handler_t)(unsigned char *data,
					unsigned char length);
typedef void (*lnp_addressing_handler_t) (unsigned char *data,
					  unsigned char length,
					  unsigned char source,
                                          unsigned char dest);

struct lnptower {
  FILEHANDLE fd;
  int usb,keepalive;
  unsigned timeout;
  volatile int keepalivetimer;
  volatile int threadsactive;
  volatile int callbacks;
  pthread_t keepalive_thread;
  pthread_t receiving_thread;
  struct ringbuf buffer;
  struct ringbuf rejected;
  lnp_lego_handler_t lnp_lego_handler;
  lnp_raw_handler_t lnp_raw_handler;
  lnp_integrity_handler_t lnp_integrity_handler;
  lnp_addressing_handler_t lnp_addressing_handler[0x100];
};


#define LNP_SLOW 0
#define LNP_FAST 1
#define LNP_PARITY 0
#define LNP_NOPARITY 2
#define LNP_NOKEEPALIVE 4

#define LNP_LOWRANGE 0
#define LNP_MIDRANGE 1
#define LNP_HIGHRANGE 2

extern int  lnp_open(struct lnptower *tower, char *device, unsigned flags);
extern void lnp_close(struct lnptower *tower);
extern int  lnp_set_range(struct lnptower *tower, int range);

#define LNP_NOHANDLER NULL

extern void lnp_raw_set_handler(struct lnptower *tower,
				lnp_raw_handler_t handler); 
extern void lnp_lego_set_handler(struct lnptower *tower,
				 lnp_lego_handler_t handler);
extern void lnp_integrity_set_handler(struct lnptower *tower,
				      lnp_integrity_handler_t handler);
extern void lnp_addressing_set_handler(struct lnptower *tower,
				       unsigned char address,
				       lnp_addressing_handler_t handler);
extern void lnp_addressing_set_multi(struct lnptower *tower,
				     unsigned char mask, unsigned char value,
				     lnp_addressing_handler_t handler);



extern void lnp_block(struct lnptower *tower);
extern void lnp_unblock(struct lnptower *tower);



extern int  lnp_raw_send(struct lnptower *tower, unsigned char *data,
			 unsigned length);
extern int  lnp_lego_send(struct lnptower *tower, unsigned char *data,
			  unsigned char length);
extern int  lnp_integrity_send(struct lnptower *tower, unsigned char *data,
			       unsigned char length);
extern int  lnp_addressing_send(struct lnptower *tower, unsigned char *data,
				unsigned char length,
				unsigned char dest, unsigned char src);



extern unsigned char lnp_addressing_host(unsigned char mask,
					 unsigned char address);
extern unsigned char lnp_addressing_port(unsigned char mask,
					 unsigned char address);
extern unsigned char lnp_addressing_addr(unsigned char mask,
					 unsigned char host,
					 unsigned char port);



extern void lnp_hexdump(char *prefix, void *data, unsigned length);

#endif
