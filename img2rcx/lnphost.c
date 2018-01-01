/* =========================================================================== 

    lnphost.c

    Routines to send and receive data packets via the Lego Mindstorms
    IR tower. This library is intended to be a replacement for the core
    functionality of lnpd. Packets in the input stream are detected by a
    heuristic algorithm.

    This is free software, released under the terms of the GNU General
    Public License.

    (C) 2004 Stephan Höhrmann

=========================================================================== */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include "lnphost.h"


/* ---- Simple ringbuffer to store received data -------------------------- */

static void ringbuf_init(struct ringbuf *buf)
{
  buf->fill=buf->head=buf->tail=0;
}

static void ringbuf_clear(struct ringbuf *buf)
{
  buf->tail=buf->head;
  buf->fill=0;
}

static void ringbuf_put(struct ringbuf *buf, unsigned char value)
{
  assert(buf->fill<RINGBUFSIZE);
  buf->data[buf->head]=value;
  buf->head=(buf->head+1)%RINGBUFSIZE;
  (buf->fill)++;
}

static unsigned char ringbuf_get(struct ringbuf *buf)
{
  unsigned char res;

  assert(buf->fill>0);
  res=buf->data[buf->tail];
  buf->tail=(buf->tail+1)%RINGBUFSIZE;
  (buf->fill)--;
  return(res);
}

static unsigned char ringbuf_sneak(struct ringbuf *buf, int ahead)
{
  assert(buf->fill>ahead);
  assert(ahead>=0);
  return(buf->data[(buf->tail+ahead)%RINGBUFSIZE]);
}

static void ringbuf_del(struct ringbuf *buf, int count)
{
  assert(count>=0);
  assert(buf->fill>=count);
  buf->tail=((buf->tail)+count)%RINGBUFSIZE;
  buf->fill-=count;
}

static inline int ringbuf_filled(struct ringbuf *buf)
{
  return(buf->fill);
}


/* ---- Access bits of an unsigned value ---------------------------------- */

static unsigned bitmask_get(unsigned mask, unsigned value)
{
  unsigned res,inmask,outmask;

  res=0;
  inmask=outmask=1;
  while ((value & mask)!=0) {
    while ((mask & inmask)==0) inmask=inmask<<1;
    if ((value & inmask)!=0) res|=outmask;
    mask&=(~inmask);
    outmask=outmask<<1;
  }
  return(res);
}

static unsigned bitmask_put(unsigned mask, unsigned value)
{
  unsigned res;
  unsigned inmask,outmask;

  res=0;
  inmask=outmask=1;
  while (mask!=0) {
    while ((mask & outmask)==0) outmask=outmask<<1;
    if ((value & inmask)!=0) res|=outmask;
    mask&=(~outmask);
    inmask=inmask<<1;
  }
  return(res);
}


/* ---- Scan received data to find valid packet --------------------------- */

#define UNKNOWN 0
#define VALID 1
#define BROKEN 2

static int legoscanpacket(struct ringbuf *buf, int *size, int finished)
{
  unsigned char checksum,lastbyte;
  int pos=0,avail;

  avail=ringbuf_filled(buf);
  if (avail>=RINGBUFSIZE-16) return(BROKEN);
  if (avail<7) return(((finished) && (avail>0))?BROKEN:UNKNOWN);
  if (ringbuf_sneak(buf,pos)!=0x55) return(BROKEN); else pos++;
  if (ringbuf_sneak(buf,pos)!=0xff) return(BROKEN); else pos++;
  if (ringbuf_sneak(buf,pos)!=0x00) return(BROKEN); else pos++;
  avail-=pos;

  checksum=lastbyte=0;
  while ((avail>=2) &&
         (ringbuf_sneak(buf,pos)==(0xff^ringbuf_sneak(buf,pos+1)))) {
    checksum=(checksum+lastbyte) & 0xff;
    lastbyte=ringbuf_sneak(buf,pos);
    pos+=2;
    avail-=2;
  }
  if ((avail<2) && (!finished)) return(UNKNOWN);
  if ((checksum==lastbyte) && (pos>=7)) {
    *size=pos;
    return(VALID);
  } else return(BROKEN);
}

static int lnpscanpacket(struct ringbuf *buf, int *size, int finished)
{
  int pos=0,avail;
  unsigned char length;
  unsigned char checksum=0xff;

  avail=ringbuf_filled(buf);
  if (avail>255+3) return(BROKEN);
  if (avail<3) return(((finished) && (avail>0))?BROKEN:UNKNOWN);
  if (ringbuf_sneak(buf,pos)==0xf0) {
    length=ringbuf_sneak(buf,pos+1);
    if (avail<3+length) return(((finished) && (avail>0))?BROKEN:UNKNOWN);
    while (pos<3+length-1)
      checksum=(checksum+ringbuf_sneak(buf,pos++)) & 0xff;
    if ((checksum==ringbuf_sneak(buf,pos)) && (pos>=3)) {
      *size=pos+1;
      return(VALID);
    } else return(BROKEN);
  }
  if (ringbuf_sneak(buf,pos)==0xf1) {
    length=ringbuf_sneak(buf,pos+1);
    if (avail<3+length) return(((finished) && (avail>0))?BROKEN:UNKNOWN);
    while (pos<3+length-1)
      checksum=(checksum+ringbuf_sneak(buf,pos++)) & 0xff;
    if ((checksum==ringbuf_sneak(buf,pos)) && (pos>=5)) {
      *size=pos+1;
      return(VALID);
    } else return(BROKEN);
  }
  return(BROKEN);
}

void extractdata(struct ringbuf *buf, int size,
              int headerlen, int footerlen, int steplen,
              unsigned char *databuf, int *datasize)
{
  int src,dest;

  dest=0;
  for (src=headerlen; src<size-footerlen; src+=steplen)
    databuf[dest++]=ringbuf_sneak(buf,src);
  *datasize=dest;
}


/* ---- Send and receive data --------------------------------------------- */

int lnp_raw_send(struct lnptower *tower, unsigned char *data, unsigned length)
{
  unsigned completed;
  int retries=1000;
#if defined (__CYGWIN__) || defined (_WIN32)
  DWORD res=0;
#else
  int res=0;
#endif

  tower->keepalivetimer=LNP_KEEPALIVE;
  if (length<1) return(length);
  completed=0;
  while ((completed<length) && (retries>0)) {
#if defined (__CYGWIN__) || defined (_WIN32)
    if (WriteFile(tower->fd,data+completed,length-completed,&res,NULL)==0) {
      fprintf(stderr, "Error writing to tower: %lu\n",
              (unsigned long)GetLastError());
      return(-1);
    }
#else
    res=write(tower->fd,data+completed,length-completed);
    if (res<0) {
      if ((errno==EINTR) || (errno==EAGAIN)) {
        res=0;
        usleep(10);
        retries--;
      } else {
        perror("write");
        return(-1);
      }
    }
#endif
    completed+=res;
  }
  tower->keepalivetimer=LNP_KEEPALIVE;
  return(completed);
}

int lnp_raw_recvchar(struct lnptower *tower)
{
  unsigned char data;

#if defined (__CYGWIN__) || defined (_WIN32)
  struct timeval timebegin,timenow;
  unsigned long elapsed;
  COMMTIMEOUTS CommTimeouts;
  DWORD res=0;
  
  if (tower->usb) {
    gettimeofday(&timebegin,0);
    while (1) {
      if (ReadFile(tower->fd,&data,1,&res,NULL)==0) {
        fprintf(stderr, "Error reading tower: %lu\n",
                (unsigned long)GetLastError());
      }
      gettimeofday(&timenow,0);
      elapsed = (timenow.tv_sec-timebegin.tv_sec)+
        (timenow.tv_usec-timebegin.tv_usec);
      if (res==1) return(data);
      if(elapsed>tower->timeout) return(-1);
    }
  } else {
    GetCommTimeouts(tower->fd,&CommTimeouts);
    CommTimeouts.ReadIntervalTimeout=MAXDWORD;
    CommTimeouts.ReadTotalTimeoutMultiplier=0;
    CommTimeouts.ReadTotalTimeoutConstant=tower->timeout;
    CommTimeouts.WriteTotalTimeoutMultiplier=10;
    CommTimeouts.WriteTotalTimeoutConstant=1000;
    SetCommTimeouts(tower->fd,&CommTimeouts);
    if (ReadFile(tower->fd,&data,1,&res,NULL)==0) {
      fprintf(stderr, "Error reading tower: %lu\n",
              (unsigned long)GetLastError());
      return(-1);
    }
    if (res==0) return(-1);
    return(data);
  }
#else
  struct timeval tv;
  fd_set fds;

  FD_ZERO(&fds);
  FD_SET(tower->fd, &fds);
  tv.tv_sec=(tower->timeout)/1000;
  tv.tv_usec=((tower->timeout)%1000)*1000;
  if (select(tower->fd+1,&fds,NULL,NULL,&tv)<0) {
    perror("select");
    return(-1);
  }
  if (!FD_ISSET(tower->fd, &fds)) return(-1);
  if (read(tower->fd,&data,1)!=1) {
    perror("read");
    return(-1);
  }
  return(data);
#endif
}


/* ---- Receiving thread -------------------------------------------------- */

static void *receiver(void *args)
{
  struct lnptower *tower=(struct lnptower*)args;
  unsigned char userdata[RINGBUFSIZE];
  int usersize;
  int legostatus,legosize;
  int lnpstatus,lnpsize;
  int data;
  unsigned char source,dest;

  while (tower->threadsactive) {
    data=lnp_raw_recvchar(tower);
    if (data>=0) ringbuf_put(&(tower->buffer),data);

  checkpacket:
    if (ringbuf_filled(&(tower->buffer))>0) {
      /* Scan received data for valid packets */
      legostatus=legoscanpacket(&(tower->buffer),&legosize,data<0);
      lnpstatus=lnpscanpacket(&(tower->buffer),&lnpsize,data<0);

      /* Flush invalid data received before the current packet */
      if ((legostatus==VALID) || (lnpstatus==VALID)) {
        if (ringbuf_filled(&(tower->rejected))>0) {
          if ((tower->lnp_raw_handler) && (tower->callbacks)) {
            extractdata(&(tower->rejected),ringbuf_filled(&(tower->rejected)),
                    0,0,1,userdata,&usersize);
              tower->lnp_raw_handler(userdata,usersize,0);
          }
        }
        ringbuf_clear(&(tower->rejected));
      }

      /* Valid LNP packet received */
      if (lnpstatus==VALID) {
        if ((tower->lnp_raw_handler) && (tower->callbacks)) {
          extractdata(&(tower->buffer),lnpsize,0,0,1,userdata,&usersize);
          tower->lnp_raw_handler(userdata,usersize,1);
        }
        switch (ringbuf_sneak(&(tower->buffer),0)) {
          case 0xf0:
            if ((tower->lnp_integrity_handler) && (tower->callbacks)) {
              extractdata(&(tower->buffer),lnpsize,2,1,1,userdata,&usersize);
              tower->lnp_integrity_handler(userdata,usersize);
            }
            break;
          case 0xf1:
            dest=ringbuf_sneak(&(tower->buffer),2);
            source=ringbuf_sneak(&(tower->buffer),3);
            if ((tower->lnp_addressing_handler[dest]) && (tower->callbacks)) {
                extractdata(&(tower->buffer),lnpsize,4,1,1,userdata,&usersize);
                tower->lnp_addressing_handler[dest](userdata,usersize,
                                                    source,dest);
            }
        }
        ringbuf_del(&(tower->buffer),lnpsize);
        continue;
      }

      /* Valid LEGO packet received */
      if (legostatus==VALID) {
        if ((tower->lnp_raw_handler) && (tower->callbacks)) {
          extractdata(&(tower->buffer),legosize,0,0,1,userdata,&usersize);
          tower->lnp_raw_handler(userdata,usersize,1);
        }
        if ((tower->lnp_lego_handler) && (tower->callbacks)) {
          extractdata(&(tower->buffer),legosize,3,2,2,userdata,&usersize);
          tower->lnp_lego_handler(userdata,usersize);
        }
        ringbuf_del(&(tower->buffer),legosize);
        continue;
      }

      /* Move first byte of received data to rejected buffer */
      if ((lnpstatus==BROKEN) && (legostatus==BROKEN)) {
        ringbuf_put(&(tower->rejected),ringbuf_get(&(tower->buffer)));
        if (ringbuf_filled(&(tower->rejected))>=
            (RINGBUFSIZE/4>256?256:RINGBUFSIZE/4)) {
          if ((tower->lnp_raw_handler) && (tower->callbacks)) {
            extractdata(&(tower->rejected),ringbuf_filled(&(tower->rejected)),
                    0,0,1,userdata,&usersize);
              tower->lnp_raw_handler(userdata,usersize,0);
          }
          ringbuf_clear(&(tower->rejected));
        }
        goto checkpacket;
      }
    }

    /* Read error or timeout, move all received bytes to rejected buffer */
    if (data<0) {
      while (ringbuf_filled(&(tower->buffer))>0) {
        ringbuf_put(&(tower->rejected),ringbuf_get(&(tower->buffer)));
        if (ringbuf_filled(&(tower->rejected))==RINGBUFSIZE) {
          if ((tower->lnp_raw_handler) && (tower->callbacks)) {
            extractdata(&(tower->rejected),ringbuf_filled(&(tower->rejected)),
                    0,0,1,userdata,&usersize);
              tower->lnp_raw_handler(userdata,usersize,0);
          }
          ringbuf_clear(&(tower->rejected));
        }
      }
      if (ringbuf_filled(&(tower->rejected))>0) {
        if ((tower->lnp_raw_handler) && (tower->callbacks)) {
          extractdata(&(tower->rejected),ringbuf_filled(&(tower->rejected)),
                      0,0,1,userdata,&usersize);
          tower->lnp_raw_handler(userdata,usersize,0);
        }
      }
      ringbuf_clear(&(tower->rejected));
    }
  }
  return(NULL);
}


/* ---- Keepalive thread -------------------------------------------------- */

static void *keepalive(void *args)
{
  static unsigned char keepalivebyte = 255;
  struct lnptower *tower=(struct lnptower*)args;

  while (tower->threadsactive) {
    if (tower->keepalivetimer>=100) {
      usleep(100*1000);
      tower->keepalivetimer-=100;
    } else {
      usleep((tower->keepalivetimer)*1000);
      if (tower->keepalive) lnp_raw_send(tower,&keepalivebyte,1);
      tower->keepalivetimer=LNP_KEEPALIVE;
    }
  }
  return(NULL);
}


/* ---- User API ---------------------------------------------------------- */

int lnp_open(struct lnptower *tower, char *device, unsigned flags)
{
  pthread_attr_t attr;
  int i;
#if defined (__CYGWIN__) || defined (_WIN32)
  DCB dcb;
#else
  struct termios ios;
#endif

  /* Select device to communicate with the tower */
  if (!device) device=getenv(LNP_RCXTTY_ENV);
  if (!device) device=LNP_DEFAULTDEVICE;

  /* Open device */
#if defined (__CYGWIN__) || defined (_WIN32)
  if ((tower->fd=CreateFile(device,GENERIC_READ|GENERIC_WRITE,0,
                        NULL,OPEN_EXISTING,0,NULL))==INVALID_HANDLE_VALUE) {
    fprintf(stderr,"Error %lu while opening %s\n",
           (unsigned long)GetLastError(),device);
    return(-1);
  }
  if (GetCommState(tower->fd,&dcb)) {
    tower->usb=0;
    FillMemory(&dcb,sizeof(dcb),0);
    dcb.fBinary=TRUE;
    dcb.ByteSize=8;
    dcb.StopBits=0; /* 0=1 Bit */
    dcb.Parity=((flags&(LNP_FAST|LNP_NOPARITY))==0)?1:0; /* 0=no, 1=odd */
    dcb.BaudRate=((flags&LNP_FAST)==0)?CBR_2400:CBR_4800;
    dcb.fParity=((flags&(LNP_FAST|LNP_NOPARITY))==0)?TRUE:FALSE;
    dcb.fAbortOnError=FALSE;
    if (!SetCommState(tower->fd,&dcb)) {
      fprintf(stderr,"Error %lu: SetCommState\n",
              (unsigned long)GetLastError());
      CloseHandle(tower->fd);
      return(-1);
    }
    PurgeComm(tower->fd,PURGE_RXABORT|PURGE_TXABORT|
              PURGE_RXCLEAR|PURGE_TXCLEAR);
  } else {
    tower->usb=1;
  }
#else
  if ((tower->fd=open(device,O_RDWR))<0) {
    perror(device);
    return(-1);
  }
  if (isatty(tower->fd)) {
    tower->usb=0;
    memset(&ios,0,sizeof(ios));
    ios.c_cflag=CREAD|CLOCAL|CS8;
    if ((flags & LNP_FAST)==0) {
      cfsetispeed(&ios,B2400);
      cfsetospeed(&ios,B2400);
      if ((flags & LNP_NOPARITY)==0) ios.c_cflag|=(PARENB|PARODD);
    } else {
      cfsetispeed(&ios,B4800);
      cfsetospeed(&ios,B4800);
    }
    if (tcsetattr(tower->fd,TCSANOW,&ios)) {
      perror("tcsetattr");
      close(tower->fd);
      return(-1);
    }
    tcflush(tower->fd,TCIOFLUSH);
  } else {
    tower->usb=1;
  }
#endif
  /* Set up variables for the threads before they start */
  tower->keepalive=((flags & LNP_NOKEEPALIVE)==0)?(!(tower->usb)):0;
  if ((flags&LNP_FAST)!=0)
    tower->timeout=34;
  else if ((flags&LNP_NOPARITY)!=0)
    tower->timeout=67;
  else tower->timeout=74;
  tower->callbacks=1;
  tower->threadsactive=1;
  tower->keepalivetimer=20;
  ringbuf_init(&(tower->buffer));
  ringbuf_init(&(tower->rejected));
  tower->lnp_lego_handler=LNP_NOHANDLER;
  tower->lnp_raw_handler=LNP_NOHANDLER;
  tower->lnp_integrity_handler=LNP_NOHANDLER;
  for (i=0; i<0x100; i++)
    tower->lnp_addressing_handler[i]=LNP_NOHANDLER;

  /* Create threads for keepalive mechanism and data receiver */
  pthread_attr_init(&attr);
  pthread_attr_setscope(&attr,PTHREAD_SCOPE_SYSTEM);
  if (pthread_create(&(tower->receiving_thread),&attr,receiver,tower)) {
    perror("pthread_create");
#if defined (__CYGWIN__) || defined (_WIN32)
  CloseHandle(tower->fd);
#else
  close(tower->fd);
#endif
    return(-1);
  }
  if (pthread_create(&(tower->keepalive_thread),&attr,keepalive,tower)) {
    perror("pthread_create");
    tower->threadsactive=0;
    pthread_join(tower->receiving_thread,NULL);
#if defined (__CYGWIN__) || defined (_WIN32)
  CloseHandle(tower->fd);
#else
  close(tower->fd);
#endif
    return(-1);
  }
  return(0);
}

void lnp_close(struct lnptower *tower)
{
  /* terminate threads */
  tower->threadsactive=0;
  pthread_join(tower->keepalive_thread,NULL);
  pthread_join(tower->receiving_thread,NULL);
  /* Close device */
#if defined (__CYGWIN__) || defined (_WIN32)
  CloseHandle(tower->fd);
#else
  close(tower->fd);
#endif
}

int lnp_set_range(struct lnptower *tower, int range)
{
  /* function is not supported by the seriel tower (use the switch) and
     -currently- not by the legousbtower kernel module */
  return(1);
}


/* ---- Set callback routines --------------------------------------------- */

void lnp_raw_set_handler(struct lnptower *tower, lnp_raw_handler_t handler)
{
  tower->lnp_raw_handler=handler;
}

void lnp_lego_set_handler(struct lnptower *tower, lnp_lego_handler_t handler)
{
  tower->lnp_lego_handler=handler;
}

void lnp_integrity_set_handler(struct lnptower *tower,
                               lnp_integrity_handler_t handler)
{
  tower->lnp_integrity_handler=handler;
}

void lnp_addressing_set_handler(struct lnptower *tower, unsigned char address,
                                lnp_addressing_handler_t handler)
{
  tower->lnp_addressing_handler[address&0xff]=handler;
}

void lnp_addressing_set_multi(struct lnptower *tower,
                              unsigned char mask, unsigned char value,
                              lnp_addressing_handler_t handler)
{
  int i;

  for (i=0; i<0x100; i++)
    if ((i&mask)==value)
      tower->lnp_addressing_handler[i]=handler;
}

void lnp_block(struct lnptower *tower)
{
  tower->callbacks=0;
}

void lnp_unblock(struct lnptower *tower)
{
  tower->callbacks=1;
}


/* ---- Send data --------------------------------------------------------- */

int lnp_lego_send(struct lnptower *tower, unsigned char *data,
                   unsigned char length)
{
  unsigned char sendbuf[517];
  unsigned char checksum=0x00;
  int i;

  sendbuf[0]=0x55;
  sendbuf[1]=0xff;
  sendbuf[2]=0x00;
  for (i=0; i<length; i++) {
    sendbuf[2*i+3]=data[i];
    sendbuf[2*i+4]=data[i]^0xff;
    checksum+=data[i];
  }
  sendbuf[3+2*length]=checksum;
  sendbuf[4+2*length]=checksum^0xff;
  return(lnp_raw_send(tower,sendbuf,2*length+5)!=2*length+5);
}

int lnp_integrity_send(struct lnptower *tower, unsigned char *data,
                        unsigned char length)
{
  unsigned char sendbuf[259];
  unsigned char checksum=0xff;
  int i;

  sendbuf[0]=0xf0;
  sendbuf[1]=length;
  for (i=0; i<length; i++)
    sendbuf[i+2]=data[i];
  for (i=0; i<length+2; i++)
    checksum+=sendbuf[i];
  sendbuf[2+length]=checksum;
  return(lnp_raw_send(tower,sendbuf,length+3)!=length+3);
}

int lnp_addressing_send(struct lnptower *tower, unsigned char *data,
                         unsigned char length,
                         unsigned char dest, unsigned char src)
{
  unsigned char sendbuf[261];
  unsigned char checksum=0xff;
  int i;

  sendbuf[0]=0xf1;
  sendbuf[1]=length+2;
  sendbuf[2]=dest;
  sendbuf[3]=src;
  for (i=0; i<length; i++)
    sendbuf[i+4]=data[i];
  for (i=0; i<length+4; i++)
    checksum+=sendbuf[i];
  sendbuf[4+length]=checksum;
  return(lnp_raw_send(tower,sendbuf,length+5)!=length+5);
}


/* ---- LNP addressing layer helpers -------------------------------------- */

unsigned char lnp_addressing_host(unsigned char mask, unsigned char address)
{
  return(bitmask_get(mask,address));
}

unsigned char lnp_addressing_port(unsigned char mask, unsigned char address)
{
  return(bitmask_get(0xff^mask,address));
}


unsigned char lnp_addressing_addr(unsigned char mask, unsigned char host,
                                  unsigned char port)
{
  return(bitmask_put(mask,host) | bitmask_put(0xff^mask,port));
}


/* ---- hexdump routine --------------------------------------------------- */

#define HEXDUMP_LINE 16  /* Bytes per line */
#define HEXDUMP_GROUP 8  /* Number of bytes in a line to be grouped */
#define HEXDUMP_ASCII 1  /* Show ASCII characters */

void lnp_hexdump(char *prefix, void *data, unsigned length)
{
  unsigned offset;
  unsigned char *dataptr=(unsigned char *)data;
  unsigned linesize,lineindex;

  for (offset=0; offset<length; offset+=HEXDUMP_LINE) {
    linesize=(length-offset<HEXDUMP_LINE)?(length-offset):HEXDUMP_LINE;
    if (prefix) printf("%s ",prefix);
    printf("%04X ",offset);
    for (lineindex=0; lineindex<HEXDUMP_LINE; lineindex++) {
      if (lineindex<linesize) printf(" %02X",*(dataptr++));
      else if (HEXDUMP_ASCII) printf("   ");
      if ((lineindex%HEXDUMP_GROUP)==(HEXDUMP_GROUP-1)) printf(" ");
    }
    if (HEXDUMP_ASCII) {
      printf(" ");
      dataptr-=linesize;
      for (lineindex=0; lineindex<HEXDUMP_LINE; lineindex++) {
        if (lineindex<linesize)
          printf("%c",(isprint(*dataptr))?(*dataptr):'.');
        dataptr++;
      }
    }
    printf("\n");
  }
}
