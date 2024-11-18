// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (C) 2013 Henner Zeller <h.zeller@acm.org>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://gnu.org/licenses/gpl-2.0.txt>

#ifdef REMI_PI

#ifndef GPIO_INTERNAL_H
#define GPIO_INTERNAL_H

#include "gpio-bits.h"

#include <vector>

#if __ARM_ARCH >= 7
#define LED_MATRIX_ALLOW_BARRIER_DELAY 1
#else
#define LED_MATRIX_ALLOW_BARRIER_DELAY 0
#endif

// Putting this in our namespace to not collide with other things called like
// this.
namespace rgb_matrix {

namespace internal {

struct InternalHardwareMapping {
  const char *name;
  int max_parallel_chains;

  uint32_t output_enable;
  uint32_t clock;
  uint32_t strobe;

  uint32_t a, b, c, d, e;

  uint32_t p0_r1, p0_g1, p0_b1;
  uint32_t p0_r2, p0_g2, p0_b2;

  uint32_t p1_r1, p1_g1, p1_b1;
  uint32_t p1_r2, p1_g2, p1_b2;

  uint32_t p2_r1, p2_g1, p2_b1;
  uint32_t p2_r2, p2_g2, p2_b2;

  uint32_t p3_r1, p3_g1, p3_b1;
  uint32_t p3_r2, p3_g2, p3_b2;

  uint32_t p4_r1, p4_g1, p4_b1;
  uint32_t p4_r2, p4_g2, p4_b2;

  uint32_t p5_r1, p5_g1, p5_b1;
  uint32_t p5_r2, p5_g2, p5_b2;
};

struct InternalHardwareModeMapping {
  const char *name;
  int max_parallel_chains;

  uint32_t output_enable;
  uint32_t clock;
  uint32_t strobe;

  uint32_t a, b, c, d, e;

  uint32_t p0_r1, p0_g1, p0_b1;
  uint32_t p0_r2, p0_g2, p0_b2;

  uint32_t p1_r1, p1_g1, p1_b1;
  uint32_t p1_r2, p1_g2, p1_b2;

  uint32_t p2_r1, p2_g1, p2_b1;
  uint32_t p2_r2, p2_g2, p2_b2;

  uint32_t p3_r1, p3_g1, p3_b1;
  uint32_t p3_r2, p3_g2, p3_b2;

  uint32_t p4_r1, p4_g1, p4_b1;
  uint32_t p4_r2, p4_g2, p4_b2;

  uint32_t p5_r1, p5_g1, p5_b1;
  uint32_t p5_r2, p5_g2, p5_b2;
};

struct RGBPinMap {
  uint32_t bit;
  uint32_t pin;
};

extern struct InternalHardwareMapping internal_hardware_mappings;
extern struct RGBPinMap rgb_pin_mapping[3][6];
}

// For now, everything is initialized as output.
class GPIO {
public:
  GPIO();

  // Initialize before use. Returns 'true' if successful, 'false' otherwise
  // (e.g. due to a permission problem).
  bool Init(int slowdown);

  inline void SetBit(uint32_t addr, uint32_t value) {
    uint32_t register_addr = (addr >> 8) & 0xffff;
    uint8_t pin = addr & 0xff;

    if(value) {
      *(gpio_register+register_addr) |= (0x01 << pin);
    } else {
      *(gpio_register+register_addr) &= ~(0x01 << pin);
    }
  }

  inline void SetPinMode(uint32_t addr, uint8_t value) {
    uint32_t register_addr = (addr >> 8) & 0xff;
    uint32_t pin = addr & 0xff;

    uint32_t mask = 0x02 << (pin << 2);

    uint32_t v = (value << (pin << 2));

    *(gpio_register+register_addr) &= ~mask;

    *(gpio_register+register_addr) |= v;
  }

  void delay() const {
    for (int n = 0; n < slowdown_; n++) {
      asm("nop");
    }
  }

private:
  int slowdown_;

  volatile uint8_t *gpio_register;
  uint8_t gpio_tm_[48];
};

// A PinPulser is a utility class that pulses a GPIO pin. There can be various
// implementations.
class PinPulser {
public:
  // Factory for a PinPulser. Chooses the right implementation depending
  // on the context (CPU and which pins are affected).
  // "gpio_mask" is the mask that should be output (since we only
  //   need negative pulses, this is what it does)
  // "nano_wait_spec" contains a list of time periods we'd like
  //   invoke later. This can be used to pre-process timings if needed.
  static PinPulser *Create(GPIO *io, gpio_bits_t gpio_mask,
                           bool allow_hardware_pulsing,
                           const std::vector<int> &nano_wait_spec);

  virtual ~PinPulser() {}

  // Send a pulse with a given length (index into nano_wait_spec array).
  virtual void SendPulse(int time_spec_number) = 0;

  // If SendPulse() is asynchronously implemented, wait for pulse to finish.
  virtual void WaitPulseFinished() {}
};

// Get rolling over microsecond counter. We get this from a hardware register
// if possible and a terrible slow fallback otherwise.
uint32_t GetMicrosecondCounter();

uint64_t GetNanosecondCounter();

void SleepMicroseconds(long);

uint32_t JitterAllowanceMicroseconds();

}  // end namespace rgb_matrix

#endif  // REMI_PI_GPIO_INTERNAL_H

#endif
