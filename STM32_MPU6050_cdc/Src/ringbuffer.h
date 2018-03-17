
#ifndef _RINGBUFFER_H
#define _RINGBUFFER_H

/*
 * example usage:
 *
 * declare an array of u32 (for memory alignment).. cast this to ringbuffer_t..
 * call ringbuffer_init() passing this array of u32 as the instance data..
 */

#if 0
typedef unsigned int    u32;
typedef unsigned char   u8;
#endif

typedef struct sRingBuffer_t ringbuffer_t;

void ringbuffer_init(ringbuffer_t *inst, u32 bufsize);
void ringbuffer_reset(ringbuffer_t *inst);
int ringbuffer_isempty(ringbuffer_t *inst);
int ringbuffer_isfull(ringbuffer_t *inst);
int ringbuffer_available(ringbuffer_t *inst);
int ringbuffer_write(ringbuffer_t *inst, u8 *buff, u32 size);
int ringbuffer_read(ringbuffer_t *inst, u8 *buff, u32 size);

#endif // _RINGBUFFER_H
