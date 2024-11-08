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

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "gpio.h"

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

// TODO: read from doc
#define PERI_BASE 0x00000

#define GPIO_REGISTER_OFFSET         0x11030000
#define COUNTER_1Mhz_REGISTER_OFFSET   0x3000

#define GPIO_PWM_BASE_OFFSET	(GPIO_REGISTER_OFFSET + 0xC000)
#define GPIO_CLK_BASE_OFFSET	0x101000

#define REGISTER_BLOCK_SIZE (4*1024)

#define PWM_CTL      (0x00 / 4)
#define PWM_STA      (0x04 / 4)
#define PWM_RNG1     (0x10 / 4)
#define PWM_FIFO     (0x18 / 4)

#define PWM_CTL_CLRF1 (1<<6)	// CH1 Clear Fifo (1 Clears FIFO 0 has no effect)
#define PWM_CTL_USEF1 (1<<5)	// CH1 Use Fifo (0=data reg transmit 1=Fifo used for transmission)
#define PWM_CTL_POLA1 (1<<4)	// CH1 Polarity (0=(0=low 1=high) 1=(1=low 0=high)
#define PWM_CTL_SBIT1 (1<<3)	// CH1 Silence Bit (state of output when 0 transmission takes place)
#define PWM_CTL_MODE1 (1<<1)	// CH1 Mode (0=pwm 1=serialiser mode)
#define PWM_CTL_PWEN1 (1<<0)	// CH1 Enable (0=disable 1=enable)

#define PWM_STA_EMPT1 (1<<1)
#define PWM_STA_FULL1 (1<<0)

#define CLK_PASSWD  (0x5A<<24)

#define CLK_CTL_MASH(x)((x)<<9)
#define CLK_CTL_BUSY    (1 <<7)
#define CLK_CTL_KILL    (1 <<5)
#define CLK_CTL_ENAB    (1 <<4)
#define CLK_CTL_SRC(x) ((x)<<0)

#define CLK_CTL_SRC_PLLD 6  /* 500.0 MHz */

#define CLK_DIV_DIVI(x) ((x)<<12)
#define CLK_DIV_DIVF(x) ((x)<< 0)

#define CLK_PWMCTL 40
#define CLK_PWMDIV 41

// We want to have the last word in the fifo free
#define MAX_PWM_BIT_USE 224
#define PWM_BASE_TIME_NS 2

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x).
#define INP_GPIO(g) *(s_GPIO_registers+((g)/10)) &= ~(7ull<<(((g)%10)*3))
#define OUT_GPIO(g) *(s_GPIO_registers+((g)/10)) |=  (1ull<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

// We're pre-mapping all the registers on first call of GPIO::Init(),
// so that it is possible to drop privileges afterwards and still have these
// usable.
static volatile uint32_t *s_GPIO_registers = NULL;
static volatile uint32_t *s_Timer1Mhz = NULL;
static volatile uint32_t *s_PWM_registers = NULL;
static volatile uint32_t *s_CLK_registers = NULL;

namespace rgb_matrix {
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

GPIO::GPIO() : output_bits_(0), input_bits_(0), reserved_bits_(0),
               slowdown_(1)
{
}

gpio_bits_t GPIO::InitOutputs(gpio_bits_t outputs,
                              bool adafruit_pwm_transition_hack_needed) {
  if (s_GPIO_registers == NULL) {
    fprintf(stderr, "Attempt to init outputs but not yet Init()-ialized.\n");
    return 0;
  }

  // Hack: for the PWM mod, the user soldered together GPIO 18 (new OE)
  // with GPIO 4 (old OE).
  // Since they are connected inside the HAT, want to make extra sure that,
  // whatever the outside system set as pinmux, the old OE is _not_ also
  // set as output so that these GPIO outputs don't fight each other.
  //
  // So explicitly set both of these pins as input initially, so the user
  // can switch between the two modes "adafruit-hat" and "adafruit-hat-pwm"
  // without trouble.
  if (adafruit_pwm_transition_hack_needed) {
    // TODO: rm adafruit hat in future
  }

  outputs &= ~(output_bits_ | input_bits_ | reserved_bits_);

  // We don't know exactly what GPIO pins are occupied by 1-wire (can we
  // easily do that ?), so let's complain only about the default GPIO.
  if ((outputs & GPIO_BIT(4))
      && LinuxHasModuleLoaded("w1_gpio")) {
    // TODO: need more survey for this
    fprintf(stderr, "This Raspberry Pi has the one-wire protocol enabled.\n"
            "This will mess with the display if GPIO pins overlap.\n"
            "Disable 1-wire in raspi-config (Interface Options).\n\n");
  }

  const int kMaxAvailableBit = 31;

  for (int b = 0; b <= kMaxAvailableBit; ++b) {
    if (outputs & GPIO_BIT(b)) {
      INP_GPIO(b);   // for writing, we first need to set as input.
      OUT_GPIO(b);
    }
  }

  output_bits_ |= outputs;

  return outputs;
}

gpio_bits_t GPIO::RequestInputs(gpio_bits_t inputs) {
  if (s_GPIO_registers == NULL) {
    fprintf(stderr, "Attempt to init inputs but not yet Init()-ialized.\n");
    return 0;
  }

  inputs &= ~(output_bits_ | input_bits_ | reserved_bits_);

  const int kMaxAvailableBit = 31;

  for (int b = 0; b <= kMaxAvailableBit; ++b) {
    if (inputs & GPIO_BIT(b)) {
      INP_GPIO(b);
    }
  }

  input_bits_ |= inputs;

  return inputs;
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

static uint32_t *mmap_bcm_register(off_t register_offset) {
  off_t base = PERI_BASE;

  int mem_fd;
  if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
    // Try to fall back to /dev/gpiomem. Unfortunately, that device
    // is implemented in a way that it _only_ supports GPIO, not the
    // other registers we need, such as PWM or COUNTER_1Mhz, which means
    // we only can operate with degraded performance.
    //
    // But, instead of failing, mmap() then silently succeeds with the
    // unsupported offset. So bail out here.
    if (register_offset != GPIO_REGISTER_OFFSET)
      return NULL;

    mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC);
    if (mem_fd < 0) return NULL;
  }

  uint32_t *result =
    (uint32_t*) mmap(NULL,                  // Any adddress in our space will do
                     REGISTER_BLOCK_SIZE,   // Map length
                     PROT_READ|PROT_WRITE,  // Enable r/w on GPIO registers.
                     MAP_SHARED,
                     mem_fd,                // File to map
                     base + register_offset // Offset to bcm register
                     );
  close(mem_fd);

  if (result == MAP_FAILED) {
    perror("mmap error: ");
    fprintf(stderr, "MMapping from base 0x%lx, offset 0x%lx\n",
            base, register_offset);
    return NULL;
  }
  return result;
}

static bool mmap_all_bcm_registers_once() {
  if (s_GPIO_registers != NULL) return true;  // alrady done.

  // The common GPIO registers.
  s_GPIO_registers = mmap_bcm_register(GPIO_REGISTER_OFFSET);
  if (s_GPIO_registers == NULL) {
    return false;
  }

  // Time measurement. Might fail when run as non-root.
  uint32_t *timereg = mmap_bcm_register(COUNTER_1Mhz_REGISTER_OFFSET);
  if (timereg != NULL) {
    s_Timer1Mhz = timereg + 1;
  }

  // Hardware pin-pulser. Might fail when run as non-root.
  s_PWM_registers  = mmap_bcm_register(GPIO_PWM_BASE_OFFSET);
  s_CLK_registers  = mmap_bcm_register(GPIO_CLK_BASE_OFFSET);

  return true;
}

bool GPIO::Init(int slowdown) {
  slowdown_ = slowdown;

  // Pre-mmap all bcm registers we need now and possibly in the future, as to
  // allow  dropping privileges after GPIO::Init() even as some of these
  // registers might be needed later.
  if (!mmap_all_bcm_registers_once())
    return false;

  gpio_set_bits_low_ = s_GPIO_registers + (0x1C / sizeof(uint32_t));
  gpio_clr_bits_low_ = s_GPIO_registers + (0x28 / sizeof(uint32_t));
  gpio_read_bits_low_ = s_GPIO_registers + (0x34 / sizeof(uint32_t));

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
  TimerBasedPinPulser(GPIO *io, gpio_bits_t bits,
                      const std::vector<int> &nano_specs)
    : io_(io), bits_(bits), nano_specs_(nano_specs) {
    if (!s_Timer1Mhz) {
      fprintf(stderr, "FYI: not running as root which means we can't properly "
              "control timing unless this is a real-time kernel. Expect color "
              "degradation. Consider running as root with sudo.\n");
    }
  }

  virtual void SendPulse(int time_spec_number) {
    io_->ClearBits(bits_);
    Timers::sleep_nanos(nano_specs_[time_spec_number]);
    io_->SetBits(bits_);
  }

private:
  GPIO *const io_;
  const gpio_bits_t bits_;
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
  for (uint32_t i = (nanos - 15) * 100 / 73; i != 0; --i) {
    asm("");
  }
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
  if (!mmap_all_bcm_registers_once())
    return false;

  DisableRealtimeThrottling();
  // If we have it, we run the update thread on core1. No perf-compromises:
  WriteTo("/sys/devices/system/cpu/cpu1/cpufreq/scaling_governor",
          "performance");

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

  if (s_Timer1Mhz) {
    static long kJitterAllowanceNanos = JitterAllowanceMicroseconds() * 1000;
    if (nanos > kJitterAllowanceNanos + MINIMUM_NANOSLEEP_TIME_US*1000) {
      const uint32_t before = *s_Timer1Mhz;
      struct timespec sleep_time = { 0, nanos - kJitterAllowanceNanos };
      nanosleep(&sleep_time, NULL);
      const uint32_t after = *s_Timer1Mhz;
      const long nanoseconds_passed = 1000 * (uint32_t)(after - before);
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
    assert(s_CLK_registers && s_PWM_registers && s_Timer1Mhz);

    for (size_t i = 0; i < specs.size(); ++i) {
      // Hints how long to nanosleep, already corrected for system overhead.
      sleep_hints_us_.push_back(specs[i]/1000 - JitterAllowanceMicroseconds());
    }

    const int base = specs[0];
    // Get relevant registers
    fifo_ = s_PWM_registers + PWM_FIFO;

    if (pins == GPIO_BIT(18)) {
      // set GPIO 18 to PWM0 mode (Alternative 5)
      SetGPIOMode(s_GPIO_registers, 18, 2);
    } else if (pins == GPIO_BIT(12)) {
      // set GPIO 12 to PWM0 mode (Alternative 0)
      SetGPIOMode(s_GPIO_registers, 12, 4);
    } else {
      assert(false); // should've been caught by CanHandle()
    }
    InitPWMDivider((base/2) / PWM_BASE_TIME_NS);
    for (size_t i = 0; i < specs.size(); ++i) {
      pwm_range_.push_back(2 * specs[i] / base);
    }
  }

  virtual void SendPulse(int c) {
    if (pwm_range_[c] < 16) {
      s_PWM_registers[PWM_RNG1] = pwm_range_[c];

      *fifo_ = pwm_range_[c];
    } else {
      // Keep the actual range as short as possible, as we have to
      // wait for one full period of these in the zero phase.
      // The hardware can't deal with values < 2, so only do this when
      // have enough of these.
      s_PWM_registers[PWM_RNG1] = pwm_range_[c] / 8;

      *fifo_ = pwm_range_[c] / 8;
      *fifo_ = pwm_range_[c] / 8;
      *fifo_ = pwm_range_[c] / 8;
      *fifo_ = pwm_range_[c] / 8;
      *fifo_ = pwm_range_[c] / 8;
      *fifo_ = pwm_range_[c] / 8;
      *fifo_ = pwm_range_[c] / 8;
      *fifo_ = pwm_range_[c] / 8;
    }

    /*
     * We need one value at the end to have it go back to
     * default state (otherwise it just repeats the last
     * value, so will be constantly 'on').
     */
    *fifo_ = 0;   // sentinel.

    /*
     * For some reason, we need a second empty sentinel in the
     * fifo, otherwise our way to detect the end of the pulse,
     * which relies on 'is the queue empty' does not work. It is
     * not entirely clear why that is from the datasheet,
     * but probably there is some buffering register in which data
     * elements are kept after the fifo is emptied.
     */
    *fifo_ = 0;

    sleep_hint_us_ = sleep_hints_us_[c];
    start_time_ = *s_Timer1Mhz;
    triggered_ = true;
    s_PWM_registers[PWM_CTL] = PWM_CTL_USEF1 | PWM_CTL_PWEN1 | PWM_CTL_POLA1;
  }

  virtual void WaitPulseFinished() {
    if (!triggered_) return;
    // Determine how long we already spent and sleep to get close to the
    // actual end-time of our sleep period.
    //
    // TODO(hzeller): find if it is possible to get some sort of interrupt from
    //   the hardware once it is done with the pulse. Sounds silly that there is
    //   not (so far, only tested GPIO interrupt with a feedback line, but that
    //   is super-slow with 20μs overhead).
    if (sleep_hint_us_ > 0) {
      const uint32_t already_elapsed_usec = *s_Timer1Mhz - start_time_;
      const int to_sleep_us = sleep_hint_us_ - already_elapsed_usec;
      if (to_sleep_us > 0) {
        struct timespec sleep_time = { 0, 1000 * to_sleep_us };
        nanosleep(&sleep_time, NULL);
      }
    }

    while ((s_PWM_registers[PWM_STA] & PWM_STA_EMPT1) == 0) {
      // busy wait until done.
    }

    s_PWM_registers[PWM_CTL] = PWM_CTL_USEF1 | PWM_CTL_POLA1 | PWM_CTL_CLRF1;
    triggered_ = false;
  }

private:
  void SetGPIOMode(volatile uint32_t *gpioReg, unsigned gpio, unsigned mode) {
    const int reg = gpio / 10;
    const int mode_pos = (gpio % 10) * 3;
    gpioReg[reg] = (gpioReg[reg] & ~(7 << mode_pos)) | (mode << mode_pos);
  }

  void InitPWMDivider(uint32_t divider) {
    assert(divider < (1<<12));  // we only have 12 bits.

    s_PWM_registers[PWM_CTL] = PWM_CTL_USEF1 | PWM_CTL_POLA1 | PWM_CTL_CLRF1;

    // reset PWM clock
    s_CLK_registers[CLK_PWMCTL] = CLK_PASSWD | CLK_CTL_KILL;

    // set PWM clock source as 500 MHz PLLD
    s_CLK_registers[CLK_PWMCTL] = CLK_PASSWD | CLK_CTL_SRC(CLK_CTL_SRC_PLLD);

    // set PWM clock divider
    s_CLK_registers[CLK_PWMDIV]
      = CLK_PASSWD | CLK_DIV_DIVI(divider) | CLK_DIV_DIVF(0);

    // enable PWM clock
    s_CLK_registers[CLK_PWMCTL]
      = CLK_PASSWD | CLK_CTL_ENAB | CLK_CTL_SRC(CLK_CTL_SRC_PLLD);
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
  if (s_Timer1Mhz) return *s_Timer1Mhz;

  // When run as non-root, we can't read the timer. Fall back to slow
  // operating-system ways.
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const uint64_t micros = ts.tv_nsec / 1000;
  const uint64_t epoch_usec = (uint64_t)ts.tv_sec * 1000000 + micros;
  return epoch_usec & 0xFFFFFFFF;
}

// For external use, e.g. to lessen busy waiting.
void SleepMicroseconds(long t) {
  Timers::sleep_nanos(t * 1000);
}

} // namespace rgb_matrix

#endif
