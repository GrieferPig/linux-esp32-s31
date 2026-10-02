/* SPDX-License-Identifier: GPL-2.0-only */
/* RV32 little-endian bounded copy. All word/halfword accesses are aligned,
 * and every load remains inside [source, source + length). Keep the caller's
 * skb alignment: the DMA frame and its Ethernet payload need not co-align. */
static void s31_sm_rx_copy(void *destination, const void *source, size_t length)
{
 unsigned char *d = destination;
 const unsigned char *s = source;
 _Static_assert(sizeof(unsigned int) == 4, "RV32 word size");
 _Static_assert(sizeof(unsigned short) == 2, "RV32 halfword size");
 while (length && ((unsigned long)d & 3)) {
  *d++ = *s++; length--;
 }
 if (!((unsigned long)s & 3)) {
  while (length >= 16) {
   const unsigned int *p = (const void *)s;
   unsigned int *q = (void *)d;
   unsigned int a=p[0], b=p[1], c=p[2], e=p[3];
   q[0]=a; q[1]=b; q[2]=c; q[3]=e;
   s+=16; d+=16; length-=16;
  }
  while (length >= 4) {
   *(unsigned int *)(void *)d = *(const unsigned int *)(const void *)s;
   s+=4; d+=4; length-=4;
  }
 } else if (!((unsigned long)s & 1)) {
  while (length >= 16) {
   const unsigned short *p = (const void *)s;
   unsigned int *q = (void *)d;
   unsigned int a=p[0] | ((unsigned int)p[1]<<16);
   unsigned int b=p[2] | ((unsigned int)p[3]<<16);
   unsigned int c=p[4] | ((unsigned int)p[5]<<16);
   unsigned int e=p[6] | ((unsigned int)p[7]<<16);
   q[0]=a; q[1]=b; q[2]=c; q[3]=e;
   s+=16; d+=16; length-=16;
  }
  while (length >= 4) {
   const unsigned short *p = (const void *)s;
   *(unsigned int *)(void *)d = p[0] | ((unsigned int)p[1]<<16);
   s+=4; d+=4; length-=4;
  }
 }
 while (length--) *d++ = *s++;
}
