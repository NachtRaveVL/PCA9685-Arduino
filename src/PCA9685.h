/*  Arduino Library for the PCA9685 16-Channel PWM Driver Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    Copyright (C) 2012 Kasper Skårhøj       <kasperskaarhoj@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Created by Kasper Skårhøj, August 3rd, 2012.
    Forked by Vitska, June 18th, 2016.
    Forked by NachtRaveVL, July 29th, 2016.

    PCA9685-Arduino - Version 1.3.0
*/

#ifndef PCA9685_H
#define PCA9685_H

// Library Setup

// NOTE: While editing the main header file isn't ideal, it is often the easiest given
// the Arduino IDE's limited custom build flag support. Editing this header file directly
// will affect all projects compiled on your system using these library files.

// Uncomment or -D this define to enable usage of the software i2c library (min 4MHz+ processor).
//#define PCA9685_ENABLE_SOFTWARE_I2C             // https://github.com/felias-fogg/SoftI2CMaster

// Uncomment or -D this define to disable usage of the Scheduler library on SAM/SAMD architecures.
//#define PCA9685_DISABLE_SCHEDULER               // https://github.com/arduino-libraries/Scheduler

// Uncomment or -D this define to disable usage of the CoopTask library when Scheduler library not used.
//#define PCA9685_DISABLE_COOPTASK                // https://github.com/dok-net/CoopTask

// Uncomment or -D this define to swap PWM low(begin)/high(end) phase values in register reads/writes (needed for some chip manufacturers).
//#define PCA9685_SWAP_PWM_BEG_END_REGS

// Uncomment or -D this define to enable debug output.
//#define PCA9685_ENABLE_DEBUG_OUTPUT

// Hookup Callouts
// -PLEASE READ-
// Many digital servos run on a 20ms pulse width (50Hz update frequency) based duty cycle,
// and do not utilize the entire pulse width for their control. Typically, 2.5% of the
// 20ms pulse width (0.5ms) represents -90° offset, and 12.5% of the 20ms pulse width
// (2.5ms) represents +90° offset. This roughly translates to raw PCA9685 PWM values of
// 102 and 512 (out of the 4096/12-bit value range) for their -90°/+90° offset control.
// However, these may need to be adjusted to fit your specific servo (e.g. some we've
// tested run ~130 to ~525 for their -90°/+90° offset control). Be aware that driving
// some 180° servos too far past their -90°/+90° operational range can cause a little
// plastic limiter pin to break off and get stuck inside of the servo's gearing, which
// could potentially cause the servo to become jammed and no longer function. Continuous
// servos operate in much the same fashion as 180° servos, but instead of the 2.5%/12.5%
// pulse width controlling a -90°/+90° offset it controls a -1x/+1x speed multiplier,
// with 0x being parked/no-movement and -1x/+1x being maximum speed in either direction.

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#elif defined(__MBED__)
#include <mbed.h>
#else
#include <WProgram.h>
#endif
#include <assert.h>

#ifndef PCA9685_ENABLE_SOFTWARE_I2C
#include <Wire.h>
#if BUFFER_LENGTH
#define PCA9685_I2C_BUFFER_LENGTH   BUFFER_LENGTH
#elif I2C_BUFFER_LENGTH
#define PCA9685_I2C_BUFFER_LENGTH   I2C_BUFFER_LENGTH
#else
#warning "i2c buffer length not defined - using default value of 32, which may not be correct for your microcontroller. Check Wire.h (or similar) for your hardware and manually define BUFFER_LENGTH or I2C_BUFFER_LENGTH to remove this warning."
#define PCA9685_I2C_BUFFER_LENGTH   32
#endif // /if BUFFER_LENGTH
#else
#define USE_SOFT_I2C_MASTER_H_AS_PLAIN_INCLUDE
#include "SoftI2CMaster.h"
#undef USE_SOFT_I2C_MASTER_H_AS_PLAIN_INCLUDE
#define PCA9685_USE_SOFTWARE_I2C
#endif // /ifndef PCA9685_ENABLE_SOFTWARE_I2C

#if !defined(PCA9685_DISABLE_SCHEDULER) && (defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD))
#include "Scheduler.h"
#define PCA9685_USE_SCHEDULER
#endif
#if !defined(PCA9685_DISABLE_COOPTASK) && !defined(PCA9685_USE_SCHEDULER)
#include "CoopTask.h"
#define PCA9685_USE_COOPTASK
#endif


// Default proxy addresser i2c addresses
#define PCA9685_I2C_DEF_ALLCALL_PROXYADR    (byte)0xE0      // Default AllCall i2c proxy address
#define PCA9685_I2C_DEF_SUB1_PROXYADR       (byte)0xE2      // Default Sub1 i2c proxy address
#define PCA9685_I2C_DEF_SUB2_PROXYADR       (byte)0xE4      // Default Sub2 i2c proxy address
#define PCA9685_I2C_DEF_SUB3_PROXYADR       (byte)0xE8      // Default Sub3 i2c proxy address


// Output driver control mode (see datasheet Table 12 and Fig 13, 14, and 15 concerning correct
// usage of OUTDRV).
enum PCA9685_OutputDriverMode {
    PCA9685_OutputDriverMode_OpenDrain,         // Module outputs in an open-drain (aka direct connection) style structure with 400mA @5v total sink current, useful for LEDs and low-power Servos
    PCA9685_OutputDriverMode_TotemPole,         // Module outputs in a totem-pole (aka push-pull) style structure with 400mA @5v total sink current and 160mA total source current, useful for external drivers (default)

    PCA9685_OutputDriverMode_Count,             // Internal use only
    PCA9685_OutputDriverMode_Undefined = -1     // Internal use only
};
// NOTE: Totem-pole mode should be used when an external N-type or P-type driver is in
// use, which provides actual sourcing current while open-drain mode doesn't. At max
// channel capacity, the sink current limit is 25mA@5v per channel while the source
// current limit, in totem-pole mode, is 10mA@5v per channel. However, from datasheet
// Table 6. subnote [1]: "Some newer LEDs include integrated Zener diodes to limit
// voltage transients, reduce EMI, and protect the LEDs, and these -MUST- be driven only
// in the open-drain mode to prevent over-heating the IC." Also from datasheet, Section
// 10. question 5: "in the push-pull architecture there is a low resistance path to GND
// through the Zener and this [causes] the IC to overheat."

// Output-enabled/active-low-OE-pin=LOW driver output mode (see datasheet Table 12 and
// Fig 13, 14, and 15 concerning correct usage of INVRT).
enum PCA9685_OutputEnabledMode {
    PCA9685_OutputEnabledMode_Normal,           // When OE is enabled/LOW, channels output a normal signal, useful for N-type external drivers (default)
    PCA9685_OutputEnabledMode_Inverted,         // When OE is enabled/LOW, channels output an inverted signal, useful for P-type external drivers or direct connection

    PCA9685_OutputEnabledMode_Count,            // Internal use only
    PCA9685_OutputEnabledMode_Undefined = -1    // Internal use only
};
// NOTE: Polarity inversion is often set according to if an external N-type driver
// (should not use INVRT) or external P-type driver/direct connection (should use INVRT)
// is used. Most breakouts have just a 220Ω resistor between the individual channel
// outputs of the IC and PWM output pins, which is useful when powering LEDs. The V+ rail
// of most breakouts can connect through a 10v 1000μF decoupling capacitor, typically
// already installed on most breakouts, which can reduce voltage spikes and ground bounce
// during phase shifts at the start/end of the PWM high phase when many channel devices
// are connected together. See https://forums.adafruit.com/viewtopic.php?f=8&t=127421 and
// https://forums.adafruit.com/viewtopic.php?f=8&t=162688 for information on installing
// a decoupling capacitor if need arises.

// Output-not-enabled/active-low-OE-pin=HIGH driver output mode (see datasheet Section
// 7.4 concerning correct usage of OUTNE).
enum PCA9685_OutputDisabledMode {
    PCA9685_OutputDisabledMode_Low,             // When OE is disabled/HIGH, channels output a LOW signal (default)
    PCA9685_OutputDisabledMode_High,            // When OE is disabled/HIGH, channels output a HIGH signal (only available in totem-pole mode)
    PCA9685_OutputDisabledMode_Floating,        // When OE is disabled/HIGH, channel outputs go into a floating (aka high-impedance/high-Z) state, which may be further refined via external pull-up/pull-down resistors

    PCA9685_OutputDisabledMode_Count,           // Internal use only
    PCA9685_OutputDisabledMode_Undefined = -1   // Internal use only
};
// NOTE: Active-low-OE pin is typically used to synchronize multiple PCA9685 devices
// together, but can also be used as an external dimming control signal.

// Channel update strategy used when multiple channels are being updated in batch.
enum PCA9685_ChannelUpdateMode {
    PCA9685_ChannelUpdateMode_AfterStop,        // Channel updates commit after full-transmission STOP signal (default)
    PCA9685_ChannelUpdateMode_AfterAck,         // Channel updates commit after individual channel update ACK signal

    PCA9685_ChannelUpdateMode_Count,            // Internal use only
    PCA9685_ChannelUpdateMode_Undefined = -1    // Internal use only
};

// Software-based phase balancing scheme.
enum PCA9685_PhaseBalancer {
    PCA9685_PhaseBalancer_None,                 // Disables software-based phase balancing, relying on installed hardware (10v 1F decoupling capacitor) to handle current sinkage and sourcing (default)
    PCA9685_PhaseBalancer_Linear,               // Uses linear software-based phase balancing, with each channel's leading edge being offset a preset 256 steps away from previous channel, with PWM values growing their HIGH phase area bidirectionally from such offset (beg/end clamped at [0,4095])
    PCA9685_PhaseBalancer_Dynamic,              // Uses dynamic software-based phase balancing, with each modified channel entering an in-use pool that recalculates an automatic distribution based on total channels modified/in-use (uses more processor time and i2c updates)

    PCA9685_PhaseBalancer_Count,                // Internal use only
    PCA9685_PhaseBalancer_Undefined = -1        // Internal use only
};
// NOTE: Software-based phase balancing attempts to further mitigate ground bounce and
// voltage spikes during phase shifts at the start/end of the PWM high phase by shifting
// the leading edge of each successive PWM high phase by some amount. This helps make
// the current sinks occur over the entire duty cycle range instead of all together at
// once. Software-based phase balancing can be useful in certain situations, but often
// isn't required. 


class PCA9685 {
public:
#ifndef PCA9685_USE_SOFTWARE_I2C

    // Library constructor. Typically called during class instantiation, before setup().
    // The i2c address should be the value of the A5-A0 pins, as the class handles the
    // module's base i2c address. It should be a value between 0 and 61, which gives a
    // maximum of 62 modules that can be addressed on the same i2c line.
    // Boards with more than one i2c line (e.g. Due/Teensy/etc.) can supply a different
    // Wire instance, such as Wire1 (using SDA1/SCL1), Wire2 (using SDA2/SCL2), etc.
    // Supported i2c clock speeds are 100kHz, 400kHz (default), and 1000kHz.
    PCA9685(byte i2cAddress = B000000, TwoWire& i2cWire = Wire, uint32_t i2cSpeed = 400000U);

    // Convenience constructor for custom Wire instance. See main constructor.
    PCA9685(TwoWire& i2cWire, uint32_t i2cSpeed = 400000U, byte i2cAddress = B000000);

#else

    // Library constructor. Typically called during class instantiation, before setup().
    // The i2c address should be the value of the A5-A0 pins, as the class handles the
    // module's base i2c address. It should be a value between 0 and 61, which gives a
    // maximum of 62 modules that can be addressed on the same i2c line.
    // Minimum supported i2c clock speed is 100kHz, which sets minimum processor speed at
    // 4MHz+ running in i2c standard mode. For up to 400kHz i2c clock speeds, minimum
    // processor speed is 16MHz+ running in i2c fast mode.
    PCA9685(byte i2cAddress = B000000);

#endif

    // Resets modules. Typically called in setup(), before any init()'s. Calling will
    // perform a software reset on all PCA9685 devices on the Wire instance, ensuring
    // that all PCA9685 devices on that line are properly reset.
    void resetDevices();

    // Initializes module. Typically called in setup().
    // See individual enums for more info.
    void init(PCA9685_OutputDriverMode driverMode = PCA9685_OutputDriverMode_TotemPole,
              PCA9685_OutputEnabledMode enabledMode = PCA9685_OutputEnabledMode_Normal,
              PCA9685_OutputDisabledMode disabledMode = PCA9685_OutputDisabledMode_Low,
              PCA9685_ChannelUpdateMode updateMode = PCA9685_ChannelUpdateMode_AfterStop,
              PCA9685_PhaseBalancer phaseBalancer = PCA9685_PhaseBalancer_None);

    // Convenience initializer for custom phase balancer. See main init method.
    void init(PCA9685_PhaseBalancer phaseBalancer,
              PCA9685_OutputDriverMode driverMode = PCA9685_OutputDriverMode_TotemPole,
              PCA9685_OutputEnabledMode enabledMode = PCA9685_OutputEnabledMode_Normal,
              PCA9685_OutputDisabledMode disabledMode = PCA9685_OutputDisabledMode_Low,
              PCA9685_ChannelUpdateMode updateMode = PCA9685_ChannelUpdateMode_AfterStop);

    // Initializes module as a proxy addresser. Typically called in setup(). Used when
    // instance talks through to AllCall/Sub1-Sub3 instances as a proxy object. Using
    // this method will disable any method that performs a read or conflicts with certain
    // states. Proxy addresser i2c addresses must be >= 0xE0, with defaults provided via
    // PCA9685_I2C_DEF_[ALLCALL|SUB[1-3]]_PROXYADR defines.
    void initAsProxyAddresser();

    // Mode accessors
    byte getI2CAddress();                                   // i2c address
    uint32_t getI2CSpeed();                                 // i2c clock speed (Hz)
    PCA9685_OutputDriverMode getOutputDriverMode();         // Output driver mode
    PCA9685_OutputEnabledMode getOutputEnabledMode();       // Output enabled mode
    PCA9685_OutputDisabledMode getOutputDisabledMode();     // Output disabled mode
    PCA9685_ChannelUpdateMode getChannelUpdateMode();       // Channel update mode
    PCA9685_PhaseBalancer getPhaseBalancer();               // Software phase balancer

    typedef void(*UserDelayFunc)(unsigned int);             // Passes delay timeout (where 0 indicates inside long blocking call / yield attempt suggested)
    // Sets user delay functions to call when a delay has to occur for processing to
    // continue. User functions here can customize what this means - typically it would
    // mean to call into a thread barrier() or yield() mechanism. Default implementation
    // is to call yield() when timeout >= 1ms, unless Scheduler and CoopTask are disabled.
    void setUserDelayFuncs(UserDelayFunc delayMillisFunc, UserDelayFunc delayMicrosFunc);

    // Min: 24Hz, Max: 1526Hz, Default: 200Hz. As Hz increases channel resolution
    // diminishes, as raw pre-scaler value, computed per datasheet, starts to require
    // much larger frequency increases for single-digit increases of the raw pre-scaler
    // value that ultimately controls the PWM frequency produced.
    void setPWMFrequency(float pwmFrequency = 200);
    // Sets standard servo frequency of 50Hz.
    void setPWMFreqServo();
    // Sets standard LED frequency of 60Hz.
    void setPWMFreqLED();

    // Turns channel either full on or full off
    void setChannelOn(int channel);
    void setChannelOff(int channel);

    // PWM amounts 0 - 4096, 0 full off, 4096 full on
    void setChannelPWM(int channel, uint16_t pwmAmount);
    void setChannelsPWM(int begChannel, int numChannels, const uint16_t *pwmAmounts);

    // Sets all channels, but won't distribute phases
    void setAllChannelsPWM(uint16_t pwmAmount);

    // Returns PWM amounts 0 - 4096, 0 full off, 4096 full on
    uint16_t getChannelPWM(int channel);

    // Enables multiple talk-through paths via i2c bus (lsb/bit0 must stay 0). To use,
    // create a new proxy instance using initAsProxyAddresser() with proper proxy i2c
    // address >= 0xE0, and pass that instance's i2c address into desired method below.
    void enableAllCallAddress(byte i2cAddressAllCall = PCA9685_I2C_DEF_ALLCALL_PROXYADR);
    void enableSub1Address(byte i2cAddressSub1 = PCA9685_I2C_DEF_SUB1_PROXYADR);
    void enableSub2Address(byte i2cAddressSub2 = PCA9685_I2C_DEF_SUB2_PROXYADR);
    void enableSub3Address(byte i2cAddressSub3 = PCA9685_I2C_DEF_SUB3_PROXYADR);
    void disableAllCallAddress();
    void disableSub1Address();
    void disableSub2Address();
    void disableSub3Address();

    // Allows external clock line to be utilized (power reset required to disable)
    void enableExtClockLine();

    byte getLastI2CError();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    int getWireInterfaceNumber();
    void printModuleInfo();
    void checkForErrors();
#endif

protected:
    byte _i2cAddress;                                       // Module's i2c address (default: B000000)
#ifndef PCA9685_USE_SOFTWARE_I2C
    TwoWire* _i2cWire;                                      // Wire class instance (unowned) (default: Wire)
    uint32_t _i2cSpeed;                                     // Module's i2c clock speed (default: 400000)
#endif
    PCA9685_OutputDriverMode _driverMode;                   // Output driver mode
    PCA9685_OutputEnabledMode _enabledMode;                 // OE enabled output mode
    PCA9685_OutputDisabledMode _disabledMode;               // OE disabled output mode
    PCA9685_ChannelUpdateMode _updateMode;                  // Channel update mode
    PCA9685_PhaseBalancer _phaseBalancer;                   // Phase balancer scheme
    bool _isProxyAddresser;                                 // Proxy addresser flag (disables certain functionality)
    UserDelayFunc _uDelayMillisFunc;                        // User millisecond delay function
    UserDelayFunc _uDelayMicrosFunc;                        // User microsecond delay function

    byte _lastI2CError;                                     // Last module i2c error

    byte getMode2Value();
    void getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd);

    void writeChannelBegin(int channel);
    void writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd);
    void writeChannelEnd();

    void writeRegister(byte regAddress, byte value);
    byte readRegister(byte regAddress);

#ifdef PCA9685_USE_SOFTWARE_I2C
    uint8_t _readBytes;
#endif
    void i2cWire_beginTransmission(uint8_t);
    uint8_t i2cWire_endTransmission(void);
    uint8_t i2cWire_requestFrom(uint8_t, uint8_t);
    size_t i2cWire_write(uint8_t);
    uint8_t i2cWire_read(void);
};

// Class to assist with calculating Servo PWM values from angle/speed values
class PCA9685_ServoEval {
public:
    // Uses a linear interpolation method to quickly compute PWM output value. Uses
    // default values of 2.5% and 12.5% of phase length for -90/+90 (or -1x/+1x).
    PCA9685_ServoEval(uint16_t minPWMAmount = 102, uint16_t maxPWMAmount = 512);

    // Uses a cubic spline to interpolate due to an offsetted zero point that isn't
    // exactly between -90/+90 (or -1x/+1x). This takes more time to compute, but gives a
    // smoother PWM output value along the entire range.
    PCA9685_ServoEval(uint16_t minPWMAmount, uint16_t midPWMAmount, uint16_t maxPWMAmount);

    ~PCA9685_ServoEval();

    // Returns the PWM value to use given the angle offset (-90 to +90)
    uint16_t pwmForAngle(float angle);

    // Returns the PWM value to use given the speed multiplier (-1 to +1)
    uint16_t pwmForSpeed(float speed);

private:
    float *_coeff;      // a,b,c,d coefficient values
    bool _isCSpline;    // Cubic spline tracking, for _coeff length
};

#endif // /ifndef PCA9685_H
