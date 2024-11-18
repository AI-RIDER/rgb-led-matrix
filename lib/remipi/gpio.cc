// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (C) 2013 Henner Zeller <h.zeller@acm.org>
// Modifications for Remi Pi by Neil <qwe17235@gmail.com> on 2024-11
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

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <remipi/gpio.h>

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

/*
 * nanosleep() takes longer than requested because of OS jitter.
 * In about 99.9% of the cases, this is <= 25 microcseconds on
 * the Raspberry Pi (empirically determined with a Raspbian kernel), so
 * we substract this value whenever we do nanosleep(); the remaining time
 * we then busy wait to get a good accurate result.
 *
 * You can measure the overhead using DEBUG_SLEEP_JITTER below.
 *
 * Note: A higher value here will result in more CPU use because of more busy
 * waiting inching towards the real value (for all the cases that nanosleep()
 * actually was better than this overhead).
 *
 * This might be interesting to tweak in particular if you have a realtime
 * kernel with different characteristics.
 */
#define EMPIRICAL_NANOSLEEP_OVERHEAD_US 12

/*
 * In case of non-hardware pulse generation, use nanosleep if we want to wait
 * longer than these given microseconds beyond the general overhead.
 * Below that, just use busy wait.
 */
#define MINIMUM_NANOSLEEP_TIME_US 5

/* In order to determine useful values for above, set this to 1 and use the
 * hardware pin-pulser.
 * It will output a histogram atexit() of how much how often we were over
 * the requested time.
 * (The full histogram will be shifted by the EMPIRICAL_NANOSLEEP_OVERHEAD_US
 *  value above. To get a full histogram of OS overhead, set it to 0 first).
 */

#define GPIO_REGISTER_BASE 0x11030000

#define PORT_REGISTER_OFFSET         0x10
#define PORT_MODE_REGISTER_OFFSET 0x0120

#define PIN_INPUT_MODE 0x01
#define PIN_OUTPUT_MODE 0x02

#define REGISTER_BLOCK_SIZE 0x1000

#define RZ_G2L_PIN(g, pin) ((((uint32_t)PORT_REGISTER_OFFSET + g) & 0xffff) << 8) + ((uint32_t)pin & 0xff)
#define RZ_G2L_MODE(g, pin) ((((uint32_t)PORT_MODE_REGISTER_OFFSET + g * 2) & 0xffff) << 8) + ((uint32_t)pin & 0xff)

#define OSTM_REGISTER_BASE 0x12801000
#define OSTM1_REGISTER_OFFSET 0x400
#define COUNTER_100Mhz_REGISTER_OFFSET 0x04

// We're pre-mapping all the registers on first call of GPIO::Init(),
// so that it is possible to drop privileges afterwards and still have these
// usable.
static volatile uint8_t *s_GPIO_registers = NULL;
static volatile uint32_t *s_Timer100Mhz = NULL;
static volatile uint8_t *s_PWM_registers = NULL;
static volatile uint8_t *s_CLK_registers = NULL;

namespace rgb_matrix {

namespace internal {

struct InternalHardwareMapping internal_hardware_mappings = {
  .name          = "regular",

  .output_enable = RZ_G2L_PIN(32, 1),
  .clock         = RZ_G2L_PIN(40, 2),
  .strobe        = RZ_G2L_PIN(22, 1),

  /* Address lines */
  .a             = RZ_G2L_PIN(23, 0),
  .b             = RZ_G2L_PIN(44, 1),
  .c             = RZ_G2L_PIN(44, 0),
  .d             = RZ_G2L_PIN(32, 0),
  .e             = RZ_G2L_PIN(2, 1),  /* RxD kept free unless 1:64 */

  /* Parallel chain 0, RGB for both sub-panels */
  .p0_r1         = RZ_G2L_PIN(47, 0),
  .p0_g1         = RZ_G2L_PIN(4, 1),
  .p0_b1         = RZ_G2L_PIN(33, 0),
  .p0_r2         = RZ_G2L_PIN(47, 3),
  .p0_g2         = RZ_G2L_PIN(47, 2),
  .p0_b2         = RZ_G2L_PIN(47, 1),

  /* Chain 1 */
  .p1_r1         = RZ_G2L_PIN(36, 1),
  .p1_g1         = RZ_G2L_PIN(23, 1),
  .p1_b1         = RZ_G2L_PIN(27, 0),
  .p1_r2         = RZ_G2L_PIN(48, 3),
  .p1_g2         = RZ_G2L_PIN(46, 3),
  .p1_b2         = RZ_G2L_PIN(19, 1),

  /* Chain 2 */
  .p2_r1         = RZ_G2L_PIN(2, 0),
  .p2_g1         = RZ_G2L_PIN(3, 0),
  .p2_b1         = RZ_G2L_PIN(3, 1),
  .p2_r2         = RZ_G2L_PIN(48, 0),
  .p2_g2         = RZ_G2L_PIN(42, 4),
  .p2_b2         = RZ_G2L_PIN(48, 1),
};

struct InternalHardwareModeMapping internal_hardware_mode_mappings = {
  .name          = "regular",

  .output_enable = RZ_G2L_MODE(32, 1),
  .clock         = RZ_G2L_MODE(40, 2),
  .strobe        = RZ_G2L_MODE(22, 1),

  /* Address lines */
  .a             = RZ_G2L_MODE(23, 0),
  .b             = RZ_G2L_MODE(44, 1),
  .c             = RZ_G2L_MODE(44, 0),
  .d             = RZ_G2L_MODE(32, 0),
  .e             = RZ_G2L_MODE(2, 1),  /* RxD kept free unless 1:64 */

  /* Parallel chain 0, RGB for both sub-panels */
  .p0_r1         = RZ_G2L_MODE(47, 0),
  .p0_g1         = RZ_G2L_MODE(4, 1),
  .p0_b1         = RZ_G2L_MODE(33, 0),
  .p0_r2         = RZ_G2L_MODE(47, 3),
  .p0_g2         = RZ_G2L_MODE(47, 2),
  .p0_b2         = RZ_G2L_MODE(47, 1),

  /* Chain 1 */
  .p1_r1         = RZ_G2L_MODE(36, 1),
  .p1_g1         = RZ_G2L_MODE(23, 1),
  .p1_b1         = RZ_G2L_MODE(27, 0),
  .p1_r2         = RZ_G2L_MODE(48, 3),
  .p1_g2         = RZ_G2L_MODE(46, 3),
  .p1_b2         = RZ_G2L_MODE(19, 1),

  /* Chain 2 */
  .p2_r1         = RZ_G2L_MODE(2, 0),
  .p2_g1         = RZ_G2L_MODE(3, 0),
  .p2_b1         = RZ_G2L_MODE(3, 1),
  .p2_r2         = RZ_G2L_MODE(48, 0),
  .p2_g2         = RZ_G2L_MODE(42, 4),
  .p2_b2         = RZ_G2L_MODE(48, 1),
};

struct RGBPinMap rgb_pin_mapping[3][6] = {
  {
    {
      .bit = 11,
      .pin = internal_hardware_mappings.p0_r1
    },
    {
      .bit = 27,
      .pin = internal_hardware_mappings.p0_g1
    },
    {
      .bit = 7,
      .pin = internal_hardware_mappings.p0_b1
    },
    {
      .bit = 8,
      .pin = internal_hardware_mappings.p0_r2
    },
    {
      .bit = 9,
      .pin = internal_hardware_mappings.p0_g2
    },
    {
      .bit = 10,
      .pin = internal_hardware_mappings.p0_b2
    },
  },
  {
    {
      .bit = 11,
      .pin = internal_hardware_mappings.p1_r1
    },
    {
      .bit = 27,
      .pin = internal_hardware_mappings.p1_g1
    },
    {
      .bit = 7,
      .pin = internal_hardware_mappings.p1_b1
    },
    {
      .bit = 8,
      .pin = internal_hardware_mappings.p1_r2
    },
    {
      .bit = 9,
      .pin = internal_hardware_mappings.p1_g2
    },
    {
      .bit = 10,
      .pin = internal_hardware_mappings.p1_b2
    },
  },
  {
    {
      .bit = 11,
      .pin = internal_hardware_mappings.p1_r1
    },
    {
      .bit = 27,
      .pin = internal_hardware_mappings.p1_g1
    },
    {
      .bit = 7,
      .pin = internal_hardware_mappings.p1_b1
    },
    {
      .bit = 8,
      .pin = internal_hardware_mappings.p1_r2
    },
    {
      .bit = 9,
      .pin = internal_hardware_mappings.p1_g2
    },
    {
      .bit = 10,
      .pin = internal_hardware_mappings.p1_b2
    },
  },
};

static void print_internal_hardware_mapping(InternalHardwareMapping *mapping) {
  printf("Name: %s\n", mapping->name);
  printf("Output Enable: 0x%x\n", mapping->output_enable);
  printf("Clock: 0x%x\n", mapping->clock);
  printf("Strobe: 0x%x\n", mapping->strobe);
  printf("A: 0x%x\n", mapping->a);
  printf("B: 0x%x\n", mapping->b);
  printf("C: 0x%x\n", mapping->c);
  printf("D: 0x%x\n", mapping->d);
  printf("E: 0x%x\n", mapping->e);
  printf("P0 R1: 0x%x\n", mapping->p0_r1);
  printf("P0 G1: 0x%x\n", mapping->p0_g1);
  printf("P0 B1: 0x%x\n", mapping->p0_b1);
  printf("P0 R2: 0x%x\n", mapping->p0_r2);
  printf("P0 G2: 0x%x\n", mapping->p0_g2);
  printf("P0 B2: 0x%x\n", mapping->p0_b2);
  printf("P1 R1: 0x%x\n", mapping->p1_r1);
  printf("P1 G1: 0x%x\n", mapping->p1_g1);
  printf("P1 B1: 0x%x\n", mapping->p1_b1);
  printf("P1 R2: 0x%x\n", mapping->p1_r2);
  printf("P1 G2: 0x%x\n", mapping->p1_g2);
  printf("P1 B2: 0x%x\n", mapping->p1_b2);
  printf("P2 R1: 0x%x\n", mapping->p2_r1);
  printf("P2 G1: 0x%x\n", mapping->p2_g1);
  printf("P2 B1: 0x%x\n", mapping->p2_b1);
  printf("P2 R2: 0x%x\n", mapping->p2_r2);
  printf("P2 G2: 0x%x\n", mapping->p2_g2);
  printf("P2 B2: 0x%x\n", mapping->p2_b2);
}

static void print_internal_hardware_mode_mapping(InternalHardwareModeMapping *mapping) {
  printf("Name: %s\n", mapping->name);
  printf("Output Enable: 0x%x\n", mapping->output_enable);
  printf("Clock: 0x%x\n", mapping->clock);
  printf("Strobe: 0x%x\n", mapping->strobe);
  printf("A: 0x%x\n", mapping->a);
  printf("B: 0x%x\n", mapping->b);
  printf("C: 0x%x\n", mapping->c);
  printf("D: 0x%x\n", mapping->d);
  printf("E: 0x%x\n", mapping->e);
  printf("P0 R1: 0x%x\n", mapping->p0_r1);
  printf("P0 G1: 0x%x\n", mapping->p0_g1);
  printf("P0 B1: 0x%x\n", mapping->p0_b1);
  printf("P0 R2: 0x%x\n", mapping->p0_r2);
  printf("P0 G2: 0x%x\n", mapping->p0_g2);
  printf("P0 B2: 0x%x\n", mapping->p0_b2);
  printf("P1 R1: 0x%x\n", mapping->p1_r1);
  printf("P1 G1: 0x%x\n", mapping->p1_g1);
  printf("P1 B1: 0x%x\n", mapping->p1_b1);
  printf("P1 R2: 0x%x\n", mapping->p1_r2);
  printf("P1 G2: 0x%x\n", mapping->p1_g2);
  printf("P1 B2: 0x%x\n", mapping->p1_b2);
  printf("P2 R1: 0x%x\n", mapping->p2_r1);
  printf("P2 G1: 0x%x\n", mapping->p2_g1);
  printf("P2 B1: 0x%x\n", mapping->p2_b1);
  printf("P2 R2: 0x%x\n", mapping->p2_r2);
  printf("P2 G2: 0x%x\n", mapping->p2_g2);
  printf("P2 B2: 0x%x\n", mapping->p2_b2);
}

}

static bool LinuxHasModuleLoaded(const char *name) {
  FILE *f = fopen("/proc/modules", "r");
  if (f == NULL) return false; // don't care.
  char buf[256];
  const size_t namelen = strlen(name);
  bool found = false;
  while (fgets(buf, sizeof(buf), f) != NULL) {
    if (strncmp(buf, name, namelen) == 0) {
      found = true;
      break;
    }
  }
  fclose(f);
  return found;
}

#define GPIO_BIT(x) (1ull << x)

GPIO::GPIO() : slowdown_(1)
{
}

static int ReadBinaryFileToBuffer(uint8_t *buffer, size_t size,
                                  const char *filename) {
  const int fd = open(filename, O_RDONLY);
  if (fd < 0) return -1;
  const ssize_t r = read(fd, buffer, size); // assume one read enough.
  close(fd);
  return r;
}

// Like ReadBinaryFileToBuffer(), but adds null-termination.
static int ReadTextFileToBuffer(char *buffer, size_t size,
                                const char *filename) {
  int r = ReadBinaryFileToBuffer((uint8_t *)buffer, size - 1, filename);
  buffer[r >= 0 ? r : 0] = '\0';
  return r;
}

// Read a 32-bit big-endian number from a 4-byte buffer.
static uint32_t read_be32(const uint8_t *p) {
  return p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3];
}

static int GetNumCores() {
  return 2;
}

static uint8_t *mmap_g2l_register(off_t register_offset) {
  int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (mem_fd < 0) {
    perror("open /dev/mem failed");
    return NULL;
  }

  void *result =    mmap(NULL,                  // Any adddress in our space will do
                     REGISTER_BLOCK_SIZE,   // Map length
                     PROT_READ | PROT_WRITE,  // Enable r/w on GPIO registers.
                     MAP_SHARED,
                     mem_fd,                // File to map
                     register_offset
                     );
  close(mem_fd);

  if (result == MAP_FAILED) {
    perror("mmap error: ");
    fprintf(stderr, "MMapping register 0x%lx",
            register_offset);
    return NULL;
  }

  return (uint8_t*)result;
}

static bool mmap_all_g2l_registers_once() {
  if (s_GPIO_registers != NULL) return true;  // alrady done.

  // The common GPIO registers.
  s_GPIO_registers = mmap_g2l_register(GPIO_REGISTER_BASE);
  if (s_GPIO_registers == NULL) {
    return false;
  }

  // Time measurement. Might fail when run as non-root.
  uint8_t *ostm_register = mmap_g2l_register(OSTM_REGISTER_BASE);
  if (ostm_register != NULL) {
    s_Timer100Mhz = (uint32_t *)(ostm_register + OSTM1_REGISTER_OFFSET + COUNTER_100Mhz_REGISTER_OFFSET);
  }

  // Hardware pin-pulser. Might fail when run as non-root.
  // s_PWM_registers  = mmap_g2l_register(GPIO_PWM_BASE_OFFSET);
  // s_CLK_registers  = mmap_g2l_register(GPIO_CLK_BASE_OFFSET);

  return true;
}


bool GPIO::Init(int slowdown) {
  slowdown_ = slowdown;

  // Pre-mmap all bcm registers we need now and possibly in the future, as to
  // allow  dropping privileges after GPIO::Init() even as some of these
  // registers might be needed later.
  if (!mmap_all_g2l_registers_once())
    return false;

  gpio_register = s_GPIO_registers;

  static uint32_t pins[26] = {
    internal::internal_hardware_mode_mappings.output_enable,
    internal::internal_hardware_mode_mappings.a,
    internal::internal_hardware_mode_mappings.b,
    internal::internal_hardware_mode_mappings.c,
    internal::internal_hardware_mode_mappings.d,
    internal::internal_hardware_mode_mappings.e,
    internal::internal_hardware_mode_mappings.clock,
    internal::internal_hardware_mode_mappings.strobe,
    internal::internal_hardware_mode_mappings.p0_r1,
    internal::internal_hardware_mode_mappings.p0_g1,
    internal::internal_hardware_mode_mappings.p0_b1,
    internal::internal_hardware_mode_mappings.p0_r2,
    internal::internal_hardware_mode_mappings.p0_g2,
    internal::internal_hardware_mode_mappings.p0_b2,
    internal::internal_hardware_mode_mappings.p1_r1,
    internal::internal_hardware_mode_mappings.p1_g1,
    internal::internal_hardware_mode_mappings.p1_b1,
    internal::internal_hardware_mode_mappings.p1_r2,
    internal::internal_hardware_mode_mappings.p1_g2,
    internal::internal_hardware_mode_mappings.p1_b2,
    internal::internal_hardware_mode_mappings.p2_r1,
    internal::internal_hardware_mode_mappings.p2_g1,
    internal::internal_hardware_mode_mappings.p2_b1,
    internal::internal_hardware_mode_mappings.p2_r2,
    internal::internal_hardware_mode_mappings.p2_g2,
    internal::internal_hardware_mode_mappings.p2_b2
  };

  for(int i = 0; i < 26; i++){
    SetPinMode(pins[i], PIN_OUTPUT_MODE);
  }

  internal::print_internal_hardware_mapping(&internal::internal_hardware_mappings);
  internal::print_internal_hardware_mode_mapping(&internal::internal_hardware_mode_mappings);

  return true;
}

uint32_t JitterAllowanceMicroseconds() {
  return EMPIRICAL_NANOSLEEP_OVERHEAD_US;
}

/*
 * We support also other pinouts that don't have the OE- on the hardware
 * PWM output pin, so we need to provide (impefect) 'manual' timing as well.
 * Hence all various busy_wait_nano() implementations depending on the hardware.
 */

// --- PinPulser. Private implementation parts.
namespace {
// Manual timers.
class Timers {
public:
  static bool Init();
  static void sleep_nanos(long t);
};

// Simplest of PinPulsers. Uses somewhat jittery and manual timers
// to get the timing, but not optimal.
class TimerBasedPinPulser : public PinPulser {
public:
  TimerBasedPinPulser(GPIO *io, uint32_t addr,
                      const std::vector<int> &nano_specs)
    : io_(io), addr_(addr), nano_specs_(nano_specs) {
    if (!s_Timer100Mhz) {
      fprintf(stderr, "FYI: not running as root which means we can't properly "
              "control timing unless this is a real-time kernel. Expect color "
              "degradation. Consider running as root with sudo.\n");
    }
  }

  virtual void SendPulse(int time_spec_number) {
    io_->SetBit(addr_, 1);
    struct timespec sleep_time = { 0, (long)nano_specs_[time_spec_number] * 1000 };
    nanosleep(&sleep_time, NULL);
    io_->SetBit(addr_, 0);
  }

private:
  GPIO *const io_;
  const uint32_t addr_;
  const std::vector<int> nano_specs_;
};

// Check that 1 shows up in isolcpus
static bool HasIsolCPUs() {
  char buf[256];
  ReadTextFileToBuffer(buf, sizeof(buf), "/sys/devices/system/cpu/isolated");
  return index(buf, '1') != NULL;
}

static void busy_wait_impl(long nanos) {
  if (nanos < 20) return;

  uint64_t last = GetNanosecondCounter();
  uint64_t pass = 0;

  do {
    uint64_t c = GetNanosecondCounter();
    uint64_t elapse = c - last;

    if(c < last) {
      // current nanosecond small than last time
      elapse = 0xffffffff - last + c;
    }

    pass += elapse;

    last = c;
  } while(nanos - pass);
}

// Best effort write to file. Used to set kernel parameters.
static void WriteTo(const char *filename, const char *str) {
  const int fd = open(filename, O_WRONLY);
  if (fd < 0) return;
  (void) write(fd, str, strlen(str));  // Best effort. Ignore return value.
  close(fd);
}

// By default, the kernel applies some throtteling for realtime
// threads to prevent starvation of non-RT threads. But we
// really want all we can get iff the machine has more cores and
// our RT-thread is locked onto one of these.
// So let's tell it not to do that.
static void DisableRealtimeThrottling() {
  if (GetNumCores() == 1) return;   // Not safe if we don't have > 1 core.
  // We need to leave the kernel a little bit of time, as it does not like
  // us to hog the kernel solidly. The default of 950000 leaves 50ms that
  // can generate visible flicker, so we reduce that to 10ms.
  WriteTo("/proc/sys/kernel/sched_rt_runtime_us", "990000");
}

bool Timers::Init() {
  if (!mmap_all_g2l_registers_once())
    return false;

  DisableRealtimeThrottling();
  // If we have it, we run the update thread on core1. No perf-compromises:
  WriteTo("/sys/devices/system/cpu/cpu1/cpufreq/scaling_governor",
          "performance");

  if (!HasIsolCPUs()) {
    fprintf(stderr, "Suggestion: to slightly improve display update, add\n\tisolcpus=3\n"
            "at the end of /boot/cmdline.txt and reboot (see README.md)\n");
  }

  return true;
}

void Timers::sleep_nanos(long nanos) {
  // For smaller durations, we go straight to busy wait.

  // For larger duration, we use nanosleep() to give the operating system
  // a chance to do something else.

  // However, these timings have a lot of jitter, so if we have the 1Mhz timer
  // available, we use that to accurately mesure time spent and do the
  // remaining time with busy wait. If we don't have the timer available
  // (not running as root), we just use nanosleep() for larger values.

  if (s_Timer100Mhz) {
    static long kJitterAllowanceNanos = JitterAllowanceMicroseconds() * 1000;
    if (nanos > kJitterAllowanceNanos + MINIMUM_NANOSLEEP_TIME_US*1000) {
      const uint32_t before = *s_Timer100Mhz;
      struct timespec sleep_time = { 0, nanos - kJitterAllowanceNanos };
      nanosleep(&sleep_time, NULL);
      const uint32_t after = *s_Timer100Mhz;
      const long nanoseconds_passed = 10 * (uint32_t)(after - before);
      if (nanoseconds_passed > nanos) {
        return;  // darn, missed it.
      } else {
        nanos -= nanoseconds_passed; // remaining time with busy-loop
      }
    }
  } else {
    // Not running as root, not having access to 1Mhz timer. Approximate large
    // durations with nanosleep(); small durations are done with busy wait.
    if (nanos > (EMPIRICAL_NANOSLEEP_OVERHEAD_US + MINIMUM_NANOSLEEP_TIME_US)*1000) {
      struct timespec sleep_time
        = { 0, nanos - EMPIRICAL_NANOSLEEP_OVERHEAD_US*1000 };
      nanosleep(&sleep_time, NULL);
      return;
    }
  }

  busy_wait_impl(nanos);  // Use model-specific busy-loop for remaining time.
}

// A PinPulser that uses the PWM hardware to create accurate pulses.
// It only works on GPIO-12 or 18 though.
class HardwarePinPulser : public PinPulser {
public:
  static bool CanHandle(gpio_bits_t gpio_mask) {
#ifdef DISABLE_HARDWARE_PULSES
    return false;
#else
    const bool can_handle = gpio_mask==GPIO_BIT(18) || gpio_mask==GPIO_BIT(12);
    if (can_handle && (s_PWM_registers == NULL || s_CLK_registers == NULL)) {
      // Instead of silently not using the hardware pin pulser and falling back
      // to timing based loops, complain loudly and request the user to make
      // a choice before continuing.
      fprintf(stderr, "Need root. You are configured to use the hardware pulse "
              "generator "
              "for\n\tsmooth color rendering, however the necessary hardware\n"
              "\tregisters can't be accessed because you probably don't run\n"
              "\twith root permissions or privileges have been dropped.\n"
              "\tSo you either have to run as root (e.g. using sudo) or\n"
              "\tsupply the --led-no-hardware-pulse command-line flag.\n\n"
              "\tExiting; run as root or with --led-no-hardware-pulse\n\n");
      exit(1);
    }
    return can_handle;
#endif
  }

  HardwarePinPulser(gpio_bits_t pins, const std::vector<int> &specs)
    : triggered_(false) {
    assert(CanHandle(pins));
    // assert(s_CLK_registers && s_PWM_registers && s_Timer1Mhz);

    // for (size_t i = 0; i < specs.size(); ++i) {
    //   // Hints how long to nanosleep, already corrected for system overhead.
    //   sleep_hints_us_.push_back(specs[i]/1000 - JitterAllowanceMicroseconds());
    // }

    // const int base = specs[0];
    // // Get relevant registers
    // fifo_ = s_PWM_registers + PWM_FIFO;

    // if (pins == GPIO_BIT(18)) {
    //   // set GPIO 18 to PWM0 mode (Alternative 5)
    //   SetGPIOMode(s_GPIO_registers, 18, 2);
    // } else if (pins == GPIO_BIT(12)) {
    //   // set GPIO 12 to PWM0 mode (Alternative 0)
    //   SetGPIOMode(s_GPIO_registers, 12, 4);
    // } else {
    //   assert(false); // should've been caught by CanHandle()
    // }
    // InitPWMDivider((base/2) / PWM_BASE_TIME_NS);
    // for (size_t i = 0; i < specs.size(); ++i) {
    //   pwm_range_.push_back(2 * specs[i] / base);
    // }
  }

  virtual void SendPulse(int c) {
    // if (pwm_range_[c] < 16) {
    //   s_PWM_registers[PWM_RNG1] = pwm_range_[c];

    //   *fifo_ = pwm_range_[c];
    // } else {
    //   // Keep the actual range as short as possible, as we have to
    //   // wait for one full period of these in the zero phase.
    //   // The hardware can't deal with values < 2, so only do this when
    //   // have enough of these.
    //   s_PWM_registers[PWM_RNG1] = pwm_range_[c] / 8;

    //   *fifo_ = pwm_range_[c] / 8;
    //   *fifo_ = pwm_range_[c] / 8;
    //   *fifo_ = pwm_range_[c] / 8;
    //   *fifo_ = pwm_range_[c] / 8;
    //   *fifo_ = pwm_range_[c] / 8;
    //   *fifo_ = pwm_range_[c] / 8;
    //   *fifo_ = pwm_range_[c] / 8;
    //   *fifo_ = pwm_range_[c] / 8;
    // }

    // /*
    //  * We need one value at the end to have it go back to
    //  * default state (otherwise it just repeats the last
    //  * value, so will be constantly 'on').
    //  */
    // *fifo_ = 0;   // sentinel.

    // /*
    //  * For some reason, we need a second empty sentinel in the
    //  * fifo, otherwise our way to detect the end of the pulse,
    //  * which relies on 'is the queue empty' does not work. It is
    //  * not entirely clear why that is from the datasheet,
    //  * but probably there is some buffering register in which data
    //  * elements are kept after the fifo is emptied.
    //  */
    // *fifo_ = 0;

    // sleep_hint_us_ = sleep_hints_us_[c];
    // start_time_ = *s_Timer1Mhz;
    // triggered_ = true;
    // s_PWM_registers[PWM_CTL] = PWM_CTL_USEF1 | PWM_CTL_PWEN1 | PWM_CTL_POLA1;
  }

  virtual void WaitPulseFinished() {
    // if (!triggered_) return;
    // // Determine how long we already spent and sleep to get close to the
    // // actual end-time of our sleep period.
    // //
    // // TODO(hzeller): find if it is possible to get some sort of interrupt from
    // //   the hardware once it is done with the pulse. Sounds silly that there is
    // //   not (so far, only tested GPIO interrupt with a feedback line, but that
    // //   is super-slow with 20μs overhead).
    // if (sleep_hint_us_ > 0) {
    //   const uint32_t already_elapsed_usec = *s_Timer1Mhz - start_time_;
    //   const int to_sleep_us = sleep_hint_us_ - already_elapsed_usec;
    //   if (to_sleep_us > 0) {
    //     struct timespec sleep_time = { 0, 1000 * to_sleep_us };
    //     nanosleep(&sleep_time, NULL);
    //   }
    // }

    // while ((s_PWM_registers[PWM_STA] & PWM_STA_EMPT1) == 0) {
    //   // busy wait until done.
    // }

    // s_PWM_registers[PWM_CTL] = PWM_CTL_USEF1 | PWM_CTL_POLA1 | PWM_CTL_CLRF1;
    // triggered_ = false;
  }

private:
  void SetGPIOMode(volatile uint32_t *gpioReg, unsigned gpio, unsigned mode) {
    // const int reg = gpio / 10;
    // const int mode_pos = (gpio % 10) * 3;
    // gpioReg[reg] = (gpioReg[reg] & ~(7 << mode_pos)) | (mode << mode_pos);
  }

  void InitPWMDivider(uint32_t divider) {
    assert(divider < (1<<12));  // we only have 12 bits.
  }

private:
  std::vector<uint32_t> pwm_range_;
  std::vector<int> sleep_hints_us_;
  volatile uint32_t *fifo_;
  uint32_t start_time_;
  int sleep_hint_us_;
  bool triggered_;
};

} // end anonymous namespace

// Public PinPulser factory
PinPulser *PinPulser::Create(GPIO *io, gpio_bits_t gpio_mask,
                             bool allow_hardware_pulsing,
                             const std::vector<int> &nano_wait_spec) {
  if (!Timers::Init()) return NULL;
  if (allow_hardware_pulsing && HardwarePinPulser::CanHandle(gpio_mask)) {
    return new HardwarePinPulser(gpio_mask, nano_wait_spec);
  } else {
    return new TimerBasedPinPulser(io, gpio_mask, nano_wait_spec);
  }
}

// For external use, e.g. in the matrix for extra time.
uint32_t GetMicrosecondCounter() {
  if (s_Timer100Mhz) *s_Timer100Mhz / 100;

  // When run as non-root, we can't read the timer. Fall back to slow
  // operating-system ways.
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const uint64_t micros = ts.tv_nsec / 1000;
  const uint64_t epoch_usec = (uint64_t)ts.tv_sec * 1000000 + micros;
  return epoch_usec & 0xFFFFFFFF;
}

uint64_t GetNanosecondCounter() {
  if (s_Timer100Mhz) return (uint64_t)*s_Timer100Mhz * 10;

  // When run as non-root, we can't read the timer. Fall back to slow
  // operating-system ways.
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const uint64_t nsec = ts.tv_nsec;
  const uint64_t epoch_usec = (uint64_t)ts.tv_sec * 1000000000 + nsec;
  return epoch_usec & 0xFFFFFFFF;
}

// For external use, e.g. to lessen busy waiting.
void SleepMicroseconds(long t) {
  Timers::sleep_nanos(t * 1000);
}

} // namespace rgb_matrix

#endif
