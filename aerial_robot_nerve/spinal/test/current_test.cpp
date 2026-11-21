#include "current_session.h"
#include "current_packet.h"
#include <assert.h>
#include <map>
#include <tuple>
#include <vector>
#include <iostream>
#include <limits>
using namespace spinal_current;
struct Bus {
  std::map<unsigned,uint32_t> regs{{0,1020},{6,46},{11,3},{13,2},{38,1193},{64,1},{70,0},
    {80,123},{82,45},{84,678},{98,0},{102,999},{108,20},{112,50},{116,2048},{132,2050}};
  std::vector<std::tuple<unsigned,unsigned,uint32_t>> writes;
  uint32_t time=0;
  int fail_address=-1;
  bool fail_reads=false, fail_writes=false;
  uint32_t now() {return time;}
  bool read(unsigned a,unsigned n,uint32_t& value) {
    if (fail_reads && int(a)==fail_address) return false;
    value=regs[a]; return true;
  }
  bool write(unsigned a,unsigned n,uint32_t value) {
    writes.emplace_back(a,n,value);
    if (fail_writes && int(a)==fail_address) return false;
    if (a==11) {assert(regs[64]==0); regs[102]=1193; regs[80]=regs[82]=regs[84]=0; regs[108]=regs[112]=0;}
    regs[a]=value; return true;
  }
};
void arm(Session& s,Bus& b,uint32_t token=7) {
  assert(s.prepare(b,100,token)); assert(!s.armed && b.regs[64]==0 && b.regs[102]==0);
  assert(s.arm(b,token)); assert(s.armed && b.regs[64]==1 && b.regs[102]==0);
}
int main() {
  {
    Bus b; Session s; arm(s,b);
    assert(s.command(b,7,1,-80)); b.time=20; s.tick(b); assert(b.regs[102]==uint16_t(-80));
    b.time=200; assert(!s.command(b,7,1,0));
    b.time=250; s.tick(b); assert(!s.armed && !s.ready && s.token==0 && b.regs[64]==0 && b.regs[102]==0);
    assert(!s.command(b,7,2,10));
    arm(s,b,8); assert(!s.command(b,7,2,10)); assert(s.command(b,8,1,10));
    assert(!s.command(b,8,2,101)); assert(!s.armed && b.regs[64]==0);
    assert(s.restore(b)); assert(!s.managed && b.regs[64]==0 && b.regs[11]==3);
    assert(b.regs[80]==123 && b.regs[82]==45 && b.regs[84]==678 && b.regs[108]==20 && b.regs[112]==50);
    assert(b.regs[116]==b.regs[132]);
    b.regs[84]=222; arm(s,b,9); s.stop(b); assert(s.restore(b)); assert(b.regs[84]==222);
  }
  {
    Bus b; Session s; b.time=std::numeric_limits<uint32_t>::max()-100; arm(s,b);
    b.time+=249; assert(s.command(b,7,1,0)); b.time+=250; s.tick(b);
    assert(!s.armed && s.status==EXPIRED);
  }
  for (unsigned address : {0u,6u,11u,13u,38u,64u,70u,80u,82u,84u,98u,102u,108u,112u,132u}) {
    Bus b; Session s; b.fail_address=address; b.fail_reads=true;
    assert(!s.prepare(b,100,7)); assert(!s.ready && !s.armed);
  }
  for (unsigned address : {11u,64u,98u,102u}) {
    Bus b; Session s; b.fail_address=address; b.fail_writes=true;
    assert(!s.prepare(b,100,7)); assert(!s.ready && !s.armed);
  }
  for (unsigned address : {11u,64u,70u}) {
    Bus b; Session s; arm(s,b); b.regs[address]=(address==11) ? 3 : (address==64 ? 0 : 4);
    b.time=80; s.tick(b); assert(!s.armed && b.regs[64]==0);
  }
  {
    Bus b; Session s; b.regs[0]=999; assert(!s.prepare(b,100,1)); assert(b.writes.empty());
    b.regs[0]=1020; assert(!s.prepare(b,0,1)); assert(!s.prepare(b,1194,1)); assert(b.writes.empty());
  }
  {
    Bus b; Session s; assert(s.prepare(b,100,10));
    b.time=1000; s.tick(b); // A prepared motor must remain OFF and ready while the user edits Nm.
    assert(s.ready && s.token==10 && !s.armed && b.regs[64]==0);
    assert(s.arm(b,10));
    b.fail_address=64; b.fail_writes=true;
    assert(!s.stop(b)); assert(!s.armed && !s.ready && s.token==0);
    b.fail_writes=false; b.time+=80; s.tick(b);
    assert(b.regs[64]==0 && b.regs[102]==0);
  }
  {
    const uint8_t ping[]={255,255,253,0,1,3,0,1};
    assert(crc(ping,sizeof(ping))==0x4e19); // ROBOTIS Protocol 2.0 reference vector.
    // Status data 0x00fdffff requires byte stuffing.
    std::vector<uint8_t> packet{255,255,253,0,7,9,0,0x55,0,255,255,253,253,0};
    auto checksum=crc(packet.data(),packet.size());
    packet.push_back(checksum&255); packet.push_back(checksum>>8);
    uint32_t value=0; assert(decode(packet.data(),packet.size(),7,4,value)); assert(value==0x00fdffff);
    assert(!decode(packet.data(),packet.size(),8,4,value));
    assert(!decode(packet.data(),packet.size(),7,2,value));
    for (unsigned i=0; i<packet.size(); ++i) {
      auto corrupt=packet; corrupt[i]^=1;
      assert(!decode(corrupt.data(),corrupt.size(),7,4,value));
    }
    for (unsigned length=0; length<packet.size(); ++length)
      assert(!decode(packet.data(),length,7,4,value));
  }
  std::cout << "PASS: current sessions, fault injection, deadline/rollover, replay rejection, restoration, CRC/stuffing\n";
}
