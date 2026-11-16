#ifndef SPINAL_CURRENT_SESSION_H
#define SPINAL_CURRENT_SESSION_H
#include <stdint.h>

namespace spinal_current {
enum Action { QUERY=0, PREPARE=1, ARM=2, STOP=3, RESTORE=4 };
enum Status { OK=0, IO_ERROR=1, DENIED=2, UNSUPPORTED=3, RANGE_ERROR=4, EXPIRED=5 };
struct Snapshot {
  uint16_t model=0, limit=0;
  uint8_t firmware=0, mode=255;
  bool torque=false;
  int32_t position=0;
};

// Bus methods read(address,size,value), write(address,size,value), now().
// All methods are called under the driver's UART mutex. No HAL/ROS dependencies.
class Session {
public:
  static constexpr uint32_t TIMEOUT_MS = 250;
  bool managed=false, ready=false, armed=false;
  uint32_t token=0;
  uint16_t software_limit=0;
  Snapshot snapshot;
  uint8_t status=OK;

  template<class Bus> bool inspect(Bus& bus) {
    uint32_t v;
    if (!bus.read(0,2,v)) return fail(IO_ERROR);
    snapshot.model=v;
    if (!bus.read(6,1,v)) return fail(IO_ERROR);
    snapshot.firmware=v;
    if (!bus.read(11,1,v)) return fail(IO_ERROR);
    snapshot.mode=v;
    if (!bus.read(38,2,v)) return fail(IO_ERROR);
    snapshot.limit=v;
    if (!bus.read(64,1,v)) return fail(IO_ERROR);
    snapshot.torque=(v != 0);
    if (!bus.read(132,4,v)) return fail(IO_ERROR);
    snapshot.position=static_cast<int32_t>(v);
    status=OK;
    return true;
  }

  template<class Bus> bool prepare(Bus& bus, uint16_t cap, uint32_t new_token) {
    if (armed) return fail(DENIED);
    if (!inspect(bus)) return false;
    uint32_t v;
    if (snapshot.model != 1020 || snapshot.firmware < 38 ||
        !bus.read(13,1,v) || v != 2) return fail(UNSUPPORTED);
    if (!cap || cap > snapshot.limit || cap > 1193 || !new_token) return fail(RANGE_ERROR);
    if (snapshot.mode != 0 && snapshot.mode != 3 && snapshot.mode != 4) return fail(UNSUPPORTED);
    if (!bus.read(70,1,v) || v) return fail(DENIED);
    // Preserve the original position settings exactly once, before the EEPROM switch.
    if (!saved_ && snapshot.mode != 0) {
      saved_mode_=snapshot.mode;
      for (unsigned i=0; i<5; ++i)
        if (!bus.read(addresses_[i], sizes_[i], saved_values_[i])) return fail(IO_ERROR);
      saved_=true;
    }
    managed=true; // Legacy angle/torque commands remain blocked even on partial failure.
    stop(bus);
    if (!verify(bus,64,1,0) ||
        (snapshot.mode != 0 && !bus.write(11,1,0)) || !verify(bus,11,1,0) ||
        !bus.write(102,2,0) || !verify(bus,102,2,0) ||
        !bus.write(98,1,0) || !bus.write(98,1,5) || !verify(bus,98,1,5)) {
      stop(bus); return fail(IO_ERROR);
    }
    snapshot.mode=0;
    snapshot.torque=false;
    software_limit=cap;
    token=new_token;
    ready=true;
    sequence_=0;
    status=OK;
    return true;
  }

  template<class Bus> bool arm(Bus& bus, uint32_t session) {
    if (!ready || !session || session != token || armed) return fail(DENIED);
    uint32_t error;
    if (!verify(bus,11,1,0) || !verify(bus,64,1,0) || !verify(bus,98,1,5) ||
        !bus.read(70,1,error) || error || !bus.write(102,2,0) ||
        !verify(bus,102,2,0) || !bus.write(64,1,1) || !verify(bus,64,1,1)) {
      stop(bus); return fail(IO_ERROR);
    }
    armed=true;
    snapshot.torque=true;
    last_command_=last_write_=last_check_=bus.now();
    goal_=0;
    status=OK;
    return true;
  }

  template<class Bus> bool command(Bus& bus, uint32_t session, uint32_t sequence, int16_t raw) {
    if (!armed || session != token || !session) return fail(DENIED);
    if (static_cast<uint32_t>(bus.now()-last_command_) >= TIMEOUT_MS) {
      stop(bus); return fail(EXPIRED);
    }
    // Duplicate/out-of-order messages never renew the command deadline.
    if (static_cast<int32_t>(sequence-sequence_) <= 0) return fail(DENIED);
    if (raw > software_limit || raw < -static_cast<int32_t>(software_limit)) {
      stop(bus); return fail(RANGE_ERROR);
    }
    goal_=raw;
    sequence_=sequence;
    last_command_=bus.now();
    status=OK;
    return true;
  }

  template<class Bus> void tick(Bus& bus) {
    if (!armed) {
      // Retry OFF after a transient UART failure; unrelated position reads would
      // otherwise keep the actuator Bus Watchdog alive with the last current.
      if (managed && static_cast<uint32_t>(bus.now()-last_check_) >= 80) {
        if (!ensureOff(bus)) { ready=false; token=0; status=IO_ERROR; }
      }
      return;
    }
    if (static_cast<uint32_t>(bus.now()-last_command_) >= TIMEOUT_MS) {
      stop(bus); status=EXPIRED; return;
    }
    if (static_cast<uint32_t>(bus.now()-last_check_) >= 80) {
      uint32_t error;
      if (!verify(bus,11,1,0) || !verify(bus,64,1,1) || !bus.read(70,1,error) || error) {
        stop(bus); status=IO_ERROR; return;
      }
      last_check_=bus.now();
    }
    // Recheck after bus I/O, which itself may have consumed the deadline.
    if (static_cast<uint32_t>(bus.now()-last_command_) >= TIMEOUT_MS) {
      stop(bus); status=EXPIRED; return;
    }
    if (static_cast<uint32_t>(bus.now()-last_write_) >= 20) {
      if (!bus.write(102,2,static_cast<uint16_t>(goal_))) {
        stop(bus); status=IO_ERROR; return;
      }
      last_write_=bus.now();
    }
  }

  template<class Bus> bool stop(Bus& bus) {
    armed=ready=false;
    token=0;
    goal_=0;
    return ensureOff(bus);
  }

  template<class Bus> bool restore(Bus& bus) {
    if (!managed || armed || !saved_) return fail(DENIED);
    stop(bus);
    if (!verify(bus,64,1,0) || !bus.write(11,1,saved_mode_) || !verify(bus,11,1,saved_mode_))
      return fail(IO_ERROR);
    for (unsigned i=0; i<5; ++i)
      if (!bus.write(addresses_[i],sizes_[i],saved_values_[i]) ||
          !verify(bus,addresses_[i],sizes_[i],saved_values_[i])) return fail(IO_ERROR);
    uint32_t position;
    if (!bus.read(132,4,position) || !bus.write(116,4,position) ||
        !verify(bus,116,4,position) || !bus.write(98,1,0) || !verify(bus,98,1,0) ||
        !verify(bus,64,1,0)) return fail(IO_ERROR);
    snapshot.mode=saved_mode_;
    snapshot.position=static_cast<int32_t>(position);
    managed=false;
    saved_=false;
    status=OK;
    return true; // Leaves torque OFF. Legacy controls require a separate action.
  }

private:
  bool saved_=false;
  uint8_t saved_mode_=3;
  uint32_t saved_values_[5]={};
  const uint16_t addresses_[5]={80,82,84,108,112};
  const uint8_t sizes_[5]={2,2,2,4,4};
  uint32_t last_command_=0, last_write_=0, last_check_=0, sequence_=0;
  int16_t goal_=0;
  bool fail(uint8_t code) { status=code; return false; }
  template<class Bus> bool ensureOff(Bus& bus) {
    // Attempt OFF independently even if zero-current transmission failed.
    bool zero=bus.write(102,2,0);
    bool off=bus.write(64,1,0);
    bool confirmed_off=verify(bus,64,1,0);
    bool confirmed_zero=verify(bus,102,2,0);
    snapshot.torque=!confirmed_off;
    last_check_=bus.now();
    return zero && off && confirmed_off && confirmed_zero;
  }
  template<class Bus> bool verify(Bus& bus, uint16_t address, uint8_t size, uint32_t expected) {
    uint32_t value;
    return bus.read(address,size,value) && value==expected;
  }
};
}
#endif
