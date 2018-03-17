
#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"
#include "ringbuffer.h"

/*
 * lock free ringbuffer.
 * Critical region protection, write only changes write index, read only changes read index.
 *
 */

struct sRingBuffer_t
{
   volatile u32   rd;         // index to buffer[] where next byte will be read.
   volatile u32   wr;         // index to buffer[] where next byte will be written.
   u32            size;
   u8             buffer[1];  // define as array of size 1, but in reality, user can provide a contiguous region of memory..
};


#define RINGBUFFER_OVERHEAD   (((u32)((struct sRingBuffer_t *)(0))->buffer) - ((u32)&(((struct sRingBuffer_t *)(0))->rd)))


/*
 * Initialize ringbuffer instance.
 * IN: inst - pointer to ringbuffer_t structure
 * IN: bufsize - size of actual storage, in bytes..
 */
void ringbuffer_init(ringbuffer_t *inst, u32 bufsize)
{
   //printf("Ringbuffer overhead = %d\n", RINGBUFFER_OVERHEAD);

   memset((char*)inst, 0, sizeof(ringbuffer_t));
   inst->size = bufsize - RINGBUFFER_OVERHEAD;
}


/*
 * Reset ringbuffer. This assumes that the ringbuffer has already been initialized..
 */
void ringbuffer_reset(ringbuffer_t *inst)
{
   inst->wr = inst->rd = 0;
   if (inst->size)
   {
      memset((char*)inst->buffer, 0, inst->size);
   }
}



/*
 * returns 1 if empty, 0 if not
 */
int ringbuffer_isempty(ringbuffer_t *inst)
{
   //printf("**wr=%d, rd=%d, size=%d\n", inst->wr, inst->rd, inst->size);
   return (inst->rd == inst->wr);
}


/*
 * returns 1 if full, 0 if not
 */
int ringbuffer_isfull(ringbuffer_t *inst)
{
   //printf("++wr=%d, rd=%d, size=%d\n", inst->wr, inst->rd, inst->size);
   return (((inst->wr + 1) % inst->size) == inst->rd);
}



/*
 * returns number of free storage available (in bytes)
 */
int ringbuffer_available(ringbuffer_t *inst)
{
   u32 avail;
   u32 rd, wr;

   rd = inst->rd;
   wr = inst->wr;
   avail = (rd > wr) ? (rd - wr) : ((inst->size - wr) + rd);
   return avail;
}


/*
 * returns number of bytes written..
 */
int ringbuffer_write(ringbuffer_t *inst, u8 *buff, u32 size)
{
   u32 avail = 0;
   u32 written = 0;
   u32 towrite;
   u32 tmp;
   u32 rd, wr;

   if (ringbuffer_isfull(inst))
      return 0;

   rd = inst->rd;
   wr = inst->wr;

   // compute available space..
   avail = (rd > wr) ? (rd - wr) : ((inst->size - wr) + rd);
   if (avail) avail -= 1;
   towrite = size > avail ? avail : size;  // actual number of bytes to write.

   written = 0;
   if (rd <= wr)
   {
      // possible wrapping needed..
      tmp = inst->size - wr; // free space to end of buffer.
      if (tmp >= towrite)
      {
         // no wrapping needed..
         memcpy(&(inst->buffer[wr]), buff, towrite);
         written += towrite;
         wr += towrite;
         wr %= inst->size;
      }
      else
      {
         // wrapping needed..
         memcpy(&(inst->buffer[wr]), buff, tmp);
         memcpy(&(inst->buffer[0]), &buff[tmp], towrite-tmp);
         written = towrite;
         wr = towrite-tmp;
      }
   }
   else
   {
      // no wrapping needed.
      memcpy(&(inst->buffer[wr]), buff, towrite);
      written += towrite;
      wr += towrite;
      wr %= inst->size;
   }

   inst->wr = wr; // update wr index
   return written;
}




/*
 * returns number of bytes read..
 */
int ringbuffer_read(ringbuffer_t *inst, u8 *buff, u32 size)
{
   u32 avail = 0;
   u32 read = 0;
   u32 toread;
   u32 tmp;
   u32 rd, wr;

   if (ringbuffer_isempty(inst))
      return 0;

   rd = inst->rd;
   wr = inst->wr;

   avail = (wr > rd) ? (wr - rd) : ((inst->size - rd) + wr);
   toread = size > avail ? avail : size;  // actual number of bytes to read.

   read = 0;
   if (wr <= rd)
   {
      // possible wrapping needed..
      tmp = inst->size - rd; // available data to end of buffer.
      if (tmp >= toread)
      {
         // no wrapping needed..
         memcpy(buff, &(inst->buffer[rd]), toread);
         read += toread;
         rd += toread;
         rd %= inst->size;
      }
      else
      {
         // wrapping needed..
         memcpy(buff, &(inst->buffer[rd]), tmp);
         memcpy(&buff[tmp], &(inst->buffer[0]), toread-tmp);
         read = toread;
         rd = toread-tmp;
      }
   }
   else
   {
      // no wrapping needed.
      memcpy(buff, &(inst->buffer[rd]), toread);
      read += toread;
      rd += toread;
      rd %= inst->size;
   }

   inst->rd = rd; // update rd index
   return read;
}












