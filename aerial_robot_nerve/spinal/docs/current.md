# Spinal direct UART XM430 current sessions

This extension is for Spinal → UART/RS485 → XM430-W350 (model 1020), Protocol 2.0,
firmware >=38. Neuron/CAN firmware is unchanged. The direct UART implementation
supports at most `MAX_SERVO_NUM` (14) motors. The host adapter must use the same
capacity. A previously built 10-axis MCU image does not support indexes 10–13.
The instruction TX buffer is 256 bytes with a pre-encoding capacity check;
the servo task stack is increased accordingly. Fourteen-axis timing, power,
wiring and shutdown behavior still require hardware validation.
The shared packet encoder bounds stuffing, updates the stuffed LENGTH field,
and computes CRC after that update. CubeMX task settings match the larger stack.

The existing ten-slot flash layout is preserved. Extra slots 10–13 do not
persist per-slot driver settings in MCU flash: position PID/profile are read
from the servo, while external-encoder/resolution settings use direct-drive
defaults. Do not use those four slots for an unconfigured transmission or
external encoder. Host anatomical/current calibration remains explicit.

## New interfaces (existing message/service MD5s unchanged)

- `direct_servo_mode`: `spinal/DirectServoMode`
- `servo/direct_current`: `spinal/DirectServoCurrent`
- `index` is the scan-order index, **not** the DYNAMIXEL bus ID. Requests also carry
  the expected bus `servo_id`, checked before touching the motor.

Actions: QUERY=0, PREPARE=1, ARM=2, STOP=3, RESTORE=4.
PREPARE supplies a nonzero random host nonce in session; the response returns the
actual token mixed with a Spinal-local generation. ARM/current messages use the
returned token, not the request nonce.
PREPARE checks model, Protocol, firmware, error, device Current Limit, and the
requested software current cap; disables torque; saves position PID/profile;
writes/read-verifies mode 0, zero Goal Current and a 100ms Bus Watchdog.
PREPARE never enables torque. ARM requires the returned nonzero session token,
readback checks, and a verified zero Goal Current before torque ON.

Every current message carries equal-length `index`, `session`, `sequence`,
`current` arrays, with unique valid indexes. `current` is signed 2.69mA units.
The sequence must increase per session. Replays do not renew the deadline.
Commands are handed to the servo worker; ROS callbacks do not perform bus I/O.

250ms without an accepted new command causes zero current and torque OFF,
invalidates the token, and requires another PREPARE/ARM. This deadline runs on
Spinal's monotonic HAL clock, independent of host ROS time and ROS process health.
Mode/torque/hardware error are rechecked during current sessions. Bus Watchdog
additionally covers a stalled Spinal worker. This is software protection, not a
certified hardware emergency stop. Physical power cutoff remains required.

Legacy angle, torque-enable, and configuration callbacks do not take over a
current-managed motor. STOP keeps ownership and mode 0, with torque OFF. RESTORE
is a separate explicit action after STOP: restores the captured position mode
(3/4), PID and profile, adopts the post-switch encoder position, clears Bus
Watchdog, and leaves torque OFF before releasing legacy ownership. If the servo
was already mode 0 at the first PREPARE, no prior position setup is known and
RESTORE is refused. EEPROM is not automatically restored on disconnect.

## Build and checks

`catkin build spinal` regenerates host messages and MCU rosserial headers.
Build the appropriate STM32CubeIDE F7/H7/H7_v2 target afterwards. Do not flash
without identifying the board and backing up existing firmware/servo settings.
The host and MCU must both include these new interfaces; existing interfaces are
kept but an old MCU does not provide `direct_servo_mode`.

`test/current_test.cpp` exercises the actual HAL-independent current-session and
packet encoder/decoder headers, including 14-axis sessions and stuffed Sync Write,
with a fake register bus. Run `ctest -R spinal_current`
in the package build directory. Real UART latency, RS485 direction wiring,
servo behavior and failure timing require hardware testing before operation.

Reference: [ROBOTIS XM430-W350 manual](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/).
Nm must be converted using measured joint Nm/A and current/torque limits, not
the stall-torque/current ratio. A commanded Nm value is not a torque measurement.

Direct UART API is linked on the H7/H7_v2 projects. The existing F7 main does not
initialize DirectServo; an F7 build is a compatibility check, not direct-UART support.
The GUI/ROS adapter and calibration instructions are in
[Dracomancer hardware instructions](../../../robots/dracomancer/docs/hardware.md).
