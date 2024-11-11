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

#ifndef RPI_GPIO_INTERNAL_H
#define RPI_GPIO_INTERNAL_H

#include "gpio-bits.h"

#include <vector>

#if __ARM_ARCH >= 7
#define LED_MATRIX_ALLOW_BARRIER_DELAY 1
#else
#define LED_MATRIX_ALLOW_BARRIER_DELAY 0
#endif

#define PERI_BASE 0x11030000

#define GPIO_REGISTER_OFFSET         0x10
#define GPIO_MODE_REGISTER_OFFSET 0x0210

#define COUNTER_1Mhz_REGISTER_OFFSET   0x3000

#define RZ_G2L_PIN(g, pin) ((GPIO_REGISTER_OFFSET + g) & 0xff << 8) + (pin & 0xff)
#define RZ_G2L_MODE(g, pin) ((GPIO_MODE_REGISTER_OFFSET + g * 2) & 0xffff << 8) + (pin & 0xff)

// Putting this in our namespace to not collide with other things called like
// this.
namespace rgb_matrix {
// For now, everything is initialized as output.
class GPIO {
public:
  GPIO();

  // Initialize before use. Returns 'true' if successful, 'false' otherwise
  // (e.g. due to a permission problem).
  bool Init(int slowdown);

  void SetBit(uint32_t addr, uint32_t value);
  void SetPinMode(uint32_t addr, uint8_t value);

private:
  inline void delay() const {
    for (int n = 0; n < slowdown_; n++) {

    }
  }

private:
  int slowdown_;
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

void SleepMicroseconds(long);

uint32_t JitterAllowanceMicroseconds();

}  // end namespace rgb_matrix

#endif  // RPI_GPIO_INGERNALH

#endif
