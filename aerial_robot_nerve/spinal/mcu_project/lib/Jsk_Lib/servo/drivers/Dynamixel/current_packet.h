#ifndef SPINAL_CURRENT_PACKET_H
#define SPINAL_CURRENT_PACKET_H
#include <stdint.h>
#include <stddef.h>
namespace spinal_current {
inline uint16_t crc(const uint8_t* data, size_t size) {
  uint16_t value=0;
  for (size_t i=0; i<size; ++i) {
    value^=static_cast<uint16_t>(data[i])<<8;
    for (unsigned bit=0; bit<8; ++bit) value=(value & 0x8000) ? (value<<1)^0x8005 : value<<1;
  }
  return value;
}
inline size_t encode(uint8_t id,uint8_t instruction,const uint8_t* parameters,size_t count,
                     uint8_t* out,size_t capacity) {
  if (!out || capacity<10 || (count && !parameters)) return 0;
  out[0]=255;out[1]=255;out[2]=253;out[3]=0;out[4]=id;
  size_t index=7;
  unsigned ff=0;
  for (size_t i=0;i<=count;++i) {
    const uint8_t value=i ? parameters[i-1] : instruction;
    if (index+2>=capacity) return 0;
    out[index++]=value;
    if (ff==2 && value==253) {
      if (index+2>=capacity) return 0;
      out[index++]=253; ff=0;
    } else if (value==255) ff=ff<2 ? ff+1 : 2;
    else ff=0;
  }
  // Protocol LENGTH counts stuffed bytes, instruction and CRC.
  const size_t length=index-7+2;
  if (length>65535) return 0;
  out[5]=length&255;out[6]=(length>>8)&255;
  const uint16_t checksum=crc(out,index);
  out[index++]=checksum&255;out[index++]=checksum>>8;
  return index;
}
inline bool decode(const uint8_t* p, size_t length, uint8_t id, unsigned size, uint32_t& value) {
  if (length<11 || length>32 || size<1 || size>4 || p[0]!=255 || p[1]!=255 ||
      p[2]!=253 || p[3]!=0 || p[4]!=id || (p[5] | (p[6]<<8))+7u!=length ||
      crc(p,length-2)!=(p[length-2] | (p[length-1]<<8))) return false;
  uint8_t body[12];
  size_t count=0;
  for (size_t i=7; i<length-2; ++i) {
    if (count>=sizeof(body)) return false;
    body[count++]=p[i];
    if (count>=3 && body[count-3]==255 && body[count-2]==255 && body[count-1]==253) {
      if (++i>=length-2 || p[i]!=253) return false;
    }
  }
  if (count!=size+2 || body[0]!=0x55 || body[1]!=0) return false;
  value=0;
  for (unsigned i=0; i<size; ++i) value|=static_cast<uint32_t>(body[2+i])<<(8*i);
  return true;
}
}
#endif
