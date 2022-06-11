# PCA9685-Arduino
Arduino Library for the PCA9685 16-Channel PWM Driver Module.

**PCA9685-Arduino - Version 1.3.0**

Library to control a PCA9685 16-channel PWM driver module from an Arduino board.  
Licensed under the copy-left GNU GPL v3 license.

Created by Kasper Skårhøj, August 3rd, 2012.  
Forked by Vitska, June 18th, 2016.  
Forked by NachtRaveVL, July 29th, 2016.

This library allows communication with boards running a PCA6985 16-channel PWM driver module. It supports a wide range of available functionality, from setting the output PWM frequecy, allowing multi-device proxy addressing, and provides an assistant class for working with Servos.

Made primarily for Arduino microcontrollers, but should work with PlatformIO, ESP32/8266, Teensy, and others - although one might experience turbulence until the bug reports get ironed out. Unknown architectures must ensure `BUFFER_LENGTH` (or `I2C_BUFFER_LENGTH`) and `WIRE_INTERFACES_COUNT` are properly defined.

Dependencies include: Scheduler (SAM/SAMD only, disableable), TaskScheduler (alternate to Scheduler, disableable), CoopTask (alternate to TaskScheduler, optional), and SoftI2CMaster (optional).

The datasheet for the IC is available at <http://www.nxp.com/documents/data_sheet/PCA9685.pdf>.

## Library Setup

### Installation

The easiest way to install this library is to utilize the Arduino IDE library manager, or through a package manager such as PlatformIO. Otherwise, simply download this library and extract its files into a `PCA9685-Arduino` folder in your Arduino custom libraries folder, typically found in your `[My ]Documents\Arduino\libraries` folder (Windows), or `~/Documents/Arduino/libraries/` folder (Linux/OSX).

### Header Defines

There are several defines inside of the library's main header file that allow for more fine-tuned control of the library. You may edit and uncomment these lines directly, or supply them via custom build flags. While editing the main header file isn't ideal, it is often easiest. Note that editing the library's main header file directly will affect all projects compiled on your system using those modified library files.

Alternatively, you may also refer to <https://forum.arduino.cc/index.php?topic=602603.0> on how to define custom build flags manually via modifying the platform[.local].txt file. Note that editing such directly will affect all other projects compiled on your system using those modified platform framework files, but at least you keep those changes to the same place.

From PCA9685.h:
```Arduino
// Uncomment or -D this define to enable usage of the software i2c library (min 4MHz+ processor).
//#define PCA9685_ENABLE_SOFTWARE_I2C             // https://github.com/felias-fogg/SoftI2CMaster

// Uncomment or -D this define to completely disable usage of any multitasking commands, such as yield().
//#define PCA9685_DISABLE_MULTITASKING

// Uncomment or -D this define to swap PWM low(begin)/high(end) phase values in register reads/writes (needed for some chip manufacturers).
//#define PCA9685_SWAP_PWM_BEG_END_REGS

// Uncomment or -D this define to enable debug output.
//#define PCA9685_ENABLE_DEBUG_OUTPUT
```

### Library Initialization

There are several initialization mode settings exposed through this library that are used for more fine-tuned control.

#### Class Instantiation

The library's class object must first be instantiated, commonly at the top of the sketch where pin setups are defined (or exposed through some other mechanism), which makes a call to the library's class constructor. The constructor allows one to set the module's i2c address, i2c Wire class instance, and lastly i2c clock speed (most i2c parameters being ommitted when in software i2c mode). The default constructor values of the library, if left unspecified, is i2c address `B000000`, and i2c Wire class instance `Wire` @`400k`Hz.

From PCA9685.h, in class PCA9685, when in hardware i2c mode:
```Arduino
    // Library constructor. Typically called during class instantiation, before setup().
    // The i2c address should be the value of the A5-A0 pins, as the class handles the
    // module's base i2c address. It should be a value between 0 and 61, which gives a
    // maximum of 62 modules that can be addressed on the same i2c line.
    // Boards with more than one i2c line (e.g. Due/Teensy/etc.) can supply a different
    // Wire instance, such as Wire1 (using SDA1/SCL1), Wire2 (using SDA2/SCL2), etc.
    // Supported i2c clock speeds are 100kHz, 400kHz (default), and 1000kHz.
    PCA9685(byte i2cAddress = B000000, TwoWire& i2cWire = Wire, uint32_t i2cSpeed = 400000);

    // Convenience constructor for custom Wire instance. See main constructor.
    PCA9685(TwoWire& i2cWire, uint32_t i2cSpeed = 400000, byte i2cAddress = B000000);
```

From PCA9685.h, in class PCA9685, when in software i2c mode (see examples for sample usage):
```Arduino
    // Library constructor. Typically called during class instantiation, before setup().
    // The i2c address should be the value of the A5-A0 pins, as the class handles the
    // module's base i2c address. It should be a value between 0 and 61, which gives a
    // maximum of 62 modules that can be addressed on the same i2c line.
    // Minimum supported i2c clock speed is 100kHz, which sets minimum processor speed at
    // 4MHz+ running in i2c standard mode. For up to 400kHz i2c clock speeds, minimum
    // processor speed is 16MHz+ running in i2c fast mode.
    PCA9685(byte i2cAddress = B000000);
```

#### Device Initialization

Additionally, a call is expected to be provided to the library class object's `init(…)` or `initAsProxyAddresser()` methods, commonly called inside of the sketch's `setup()` function. This allows one to set the module's driver mode, enabled/disabled output settings, channel update mode, and phase balancer scheme, while the `initAsProxyAddresser()` method allows one to setup the object as a proxy addresser (see examples for sample usage). The default init values of the library, if left unspecified, is `PCA9685_OutputDriverMode_TotemPole`, `PCA9685_OutputEnabledMode_Normal`, `PCA9685_OutputDisabledMode_Low`, `PCA9685_ChannelUpdateMode_AfterStop`, and `PCA9685_PhaseBalancer_None` which seems to work for most of the PCA9685 breakouts on market, but should be set according to your setup.

See Section 7.3.2 of the datasheet for more details.

From PCA9685.h, in class PCA9685, for standard init:
```Arduino
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
```

From PCA9685.h, in class PCA9685, for init as a proxy addresser (see examples for sample usage):
```Arduino
    // Initializes module as a proxy addresser. Typically called in setup(). Used when
    // instance talks through to AllCall/Sub1-Sub3 instances as a proxy object. Using
    // this method will disable any method that performs a read or conflicts with certain
    // states. Proxy addresser i2c addresses must be >= 0xE0, with defaults provided via
    // PCA9685_I2C_DEF_[ALLCALL|SUB[1-3]]_PROXYADR defines.
    void initAsProxyAddresser();
```

From PCA9685.h:
```Arduino
// Output driver control mode (see datasheet Table 12 and Fig 13, 14, and 15 concerning correct
// usage of OUTDRV).
enum PCA9685_OutputDriverMode {
    PCA9685_OutputDriverMode_OpenDrain,         // Module outputs in an open-drain (aka direct connection) style structure with 400mA @5v total sink current, useful for LEDs and low-power Servos
    PCA9685_OutputDriverMode_TotemPole,         // Module outputs in a totem-pole (aka push-pull) style structure with 400mA @5v total sink current and 160mA total source current, useful for external drivers (default)
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
};
// NOTE: Active-low-OE pin is typically used to synchronize multiple PCA9685 devices
// together, but can also be used as an external dimming control signal.

// Channel update strategy used when multiple channels are being updated in batch.
enum PCA9685_ChannelUpdateMode {
    PCA9685_ChannelUpdateMode_AfterStop,        // Channel updates commit after full-transmission STOP signal (default)
    PCA9685_ChannelUpdateMode_AfterAck,         // Channel updates commit after individual channel update ACK signal
};

// Software-based phase balancing scheme.
enum PCA9685_PhaseBalancer {
    PCA9685_PhaseBalancer_None,                 // Disables software-based phase balancing, relying on installed hardware (10v 1F decoupling capacitor) to handle current sinkage and sourcing (default)
    PCA9685_PhaseBalancer_Linear,               // Uses linear software-based phase balancing, with each channel being a preset 16 steps (out of the 4096/12-bit value range) away from previous channel, capped at a maximum HIGH phase end position of 4096 steps.
    PCA9685_PhaseBalancer_Dynamic,              // Uses dynamic software-based phase balancing, with each modified channel entering an in-use pool that recalculates an automatic distribution based on total channels modified/in-use (uses more processor time and i2c updates)
};
// NOTE: Software-based phase balancing attempts to further mitigate ground bounce and
// voltage spikes during phase shifts at the start/end of the PWM high phase by shifting
// the leading edge of each successive PWM high phase by some amount. This helps make
// the current sinks occur over the entire duty cycle range instead of all together at
// once. Software-based phase balancing can be useful in certain situations, but in
// practice has been the source of many problems, including the case whereby the PCA9685
// will skip a cycle between PWM changes when the leading/trailing edge is shifted past a
// certain point. While we may revisit this idea in the future, for now we're content on
// leaving None as the default, and limiting the shift that Linear applies.
```

#### Device Reset

If you are constantly re-building and re-uploading during development, it may be wise to include a call to the library's `resetDevices()` method in order to reset all devices shared across the supplied Wire instance. This way you can ensure all devices on that i2c line start from a clean state.

From PCA9685.h, in class PCA9685:
```Arduino
    // Resets modules. Typically called in setup(), before any init()'s. Calling will
    // perform a software reset on all PCA9685 devices on the Wire instance, ensuring
    // that all PCA9685 devices on that line are properly reset.
    void resetDevices();
```

## Hookup Callouts

### Servo Control

* Many digital servos run on a 20ms pulse width (50Hz update frequency) based duty cycle, and do not utilize the entire pulse width for their control.
* Typically, 2.5% of the 20ms pulse width (0.5ms) represents -90° offset, and 12.5% of the 20ms pulse width (2.5ms) represents +90° offset.
  * This roughly translates to raw PCA9685 PWM values of 102 and 512 (out of the 4096/12-bit value range) for their -90°/+90° offset control.
  * However, these may need to be adjusted to fit your specific servo (e.g. some we've tested run ~130 to ~525 for their -90°/+90° offset control).
* Be aware that driving some 180° servos too far past their -90°/+90° operational range can cause a little plastic limiter pin to break off and get stuck inside of the servo's gearing, which could potentially cause the servo to become jammed and no longer function.
* Continuous servos operate in much the same fashion as 180° servos, but instead of the 2.5%/12.5% pulse width controlling a -90°/+90° offset it controls a -1x/+1x speed multiplier, with 0x being parked/no-movement and -1x/+1x being maximum speed in either direction.

See the `PCA9685_ServoEval` helper class to assist with calculating PWM values from Servo angle/speed values.

## Example Usage

Below are several examples of library usage.

### Simple Example

```Arduino
#include "PCA9685.h"

PCA9685 pwmController;                  // Library using default B000000 (A5-A0) i2c address, and default Wire @400kHz

void setup() {
    Serial.begin(115200);               // Begin Serial and Wire interfaces
    Wire.begin();
    Wire.setClock(pwmController.getI2CSpeed());

    pwmController.resetDevices();       // Resets all PCA9685 devices on i2c line

    pwmController.init();               // Initializes module using default totem-pole driver mode

    pwmController.setPWMFrequency(100); // Set PWM freq to 100Hz (default is 200Hz, supports 24Hz to 1526Hz)

    pwmController.setChannelPWM(0, 128 << 4); // Set PWM to 128/255, shifted into 4096-land

    Serial.println(pwmController.getChannelPWM(0)); // Should output 2048, which is 128 << 4
}

void loop() {
}

```

### Batching Example

In this example, we randomly select PWM frequencies on all 12 outputs and allow them to drive for 5 seconds before changing them.

```Arduino
#include "PCA9685.h"

PCA9685 pwmController(B010101);         // Library using B010101 (A5-A0) i2c address, and default Wire @400kHz

void setup() {
    Serial.begin(115200);               // Begin Serial and Wire interfaces
    Wire.begin();
    Wire.setClock(pwmController.getI2CSpeed());

    pwmController.resetDevices();       // Resets all PCA9685 devices on i2c line

    pwmController.init();               // Initializes module using default totem-pole driver mode, and default phase balancer

    pwmController.setPWMFrequency(500); // Set PWM freq to 500Hz (default is 200Hz, supports 24Hz to 1526Hz)

    randomSeed(analogRead(0));          // Use white noise for our randomness
}

void loop() {
    uint16_t pwms[12];
    pwms[0] = random(0, 4096);
    pwms[1] = random(0, 4096);
    pwms[2] = random(0, 4096);
    pwms[3] = random(0, 4096);
    pwms[4] = random(0, 4096);
    pwms[5] = random(0, 4096);
    pwms[6] = random(0, 4096);
    pwms[7] = random(0, 4096);
    pwms[8] = random(0, 4096);
    pwms[9] = random(0, 4096);
    pwms[10] = random(0, 4096);
    pwms[11] = random(0, 4096);
    pwmController.setChannelsPWM(0, 12, pwms);
    delay(5000);

    // NOTE: Many chips use a BUFFER_LENGTH size of 32, and in that case writing 12
    // channels will take 2 i2c transactions because only 7 channels can fit in a single
    // i2c buffer transaction at a time. This may cause a slight offset flicker between
    // the first 7 and remaining 5 channels, but can be offset by experimenting with a
    // channel update mode of PCA9685_ChannelUpdateMode_AfterAck. This will make each
    // channel update immediately upon sending of the Ack signal after each PWM command
    // is executed rather than at the Stop signal at the end of the i2c transaction.
}

```

### Multi-Device Proxy Example

In this example, we use a special instance to control other modules attached to it via the `ALL_CALL` register.

```Arduino
#include "PCA9685.h"

PCA9685 pwmController1(B000000);        // Library using B000000 (A5-A0) i2c address, and default Wire @400kHz
PCA9685 pwmController2(B000001);        // Library using B000001 (A5-A0) i2c address, and default Wire @400kHz

// Not a real device, will act as a proxy to pwmController1 and pwmController2, using all-call i2c address 0xE0, and default Wire @400kHz
PCA9685 pwmControllerAll(PCA9685_I2C_DEF_ALLCALL_PROXYADR);

void setup() {
    Serial.begin(115200);               // Begin Serial and Wire interfaces
    Wire.begin();
    Wire.setClock(min(pwmController1.getI2CSpeed(), pwmController2.getI2CSpeed()));

    pwmControllerAll.resetDevices();    // Resets all PCA9685 devices on i2c line

    pwmController1.init();              // Initializes first module using default totem-pole driver mode
    pwmController2.init();              // Initializes second module using default totem-pole driver mode

    pwmControllerAll.initAsProxyAddresser(); // Initializes 'fake' module as all-call proxy addresser

    // Enables all-call support to module from 'fake' all-call proxy addresser
    pwmController1.enableAllCallAddress(pwmControllerAll.getI2CAddress());
    pwmController2.enableAllCallAddress(pwmControllerAll.getI2CAddress()); // On both

    pwmController1.setChannelOff(0);    // Turn channel 0 off
    pwmController2.setChannelOff(0);    // On both

    pwmControllerAll.setChannelPWM(0, 4096); // Enables full on on both pwmController1 and pwmController2

    Serial.println(pwmController1.getChannelPWM(0)); // Should output 4096
    Serial.println(pwmController2.getChannelPWM(0)); // Should also output 4096

    // Note: Various parts of functionality of the proxy class instance are actually
    // disabled - typically anything that involves a read command being issued.
}

void loop() {
}

```

### Servo Evaluator Example

In this example, we utilize the `PCA9685_ServoEval` helper class to assist with setting PWM frequencies when working with servos.

We will be using `Wire1`, which is only available on boards with SDA1/SCL1 (e.g. Due/Teensy/etc.) - change to `Wire` if `Wire1` is unavailable.

```Arduino
#include "PCA9685.h"

PCA9685 pwmController(Wire1);           // Library using Wire1 @400kHz, and default B000000 (A5-A0) i2c address

// Linearly interpolates between standard 2.5%/12.5% phase length (102/512) for -90°/+90°
PCA9685_ServoEval pwmServo1;

// Testing our second servo has found that -90° sits at 128, 0° at 324, and +90° at 526.
// Since 324 isn't precisely in the middle, a cubic spline will be used to smoothly
// interpolate PWM values, which will account for said discrepancy. Additionally, since
// 324 is closer to 128 than 526, there is slightly less resolution in the -90° to 0°
// range while slightly more in the 0° to +90° range.
PCA9685_ServoEval pwmServo2(128,324,526);

void setup() {
    Serial.begin(115200);               // Begin Serial and Wire1 interfaces
    Wire1.begin();
    Wire1.setClock(pwmController.getI2CSpeed());

    pwmController.resetDevices();       // Resets all PCA9685 devices on i2c line

    pwmController.init();               // Initializes module using default totem-pole driver mode

    pwmController.setPWMFreqServo();    // 50Hz provides standard 20ms servo phase length

    pwmController.setChannelPWM(0, pwmServo1.pwmForAngle(-90));
    Serial.println(pwmController.getChannelPWM(0)); // Should output 102 for -90°

    // Showing linearity for midpoint, 205 away from both -90° and 90°
    Serial.println(pwmServo1.pwmForAngle(0));   // Should output 307 for 0°

    pwmController.setChannelPWM(0, pwmServo1.pwmForAngle(90));
    Serial.println(pwmController.getChannelPWM(0)); // Should output 512 for +90°

    pwmController.setChannelPWM(1, pwmServo2.pwmForAngle(-90));
    Serial.println(pwmController.getChannelPWM(1)); // Should output 128 for -90°

    // Showing less resolution in the -90° to 0° range
    Serial.println(pwmServo2.pwmForAngle(-45)); // Should output 225 for -45°, 97 away from -90°

    pwmController.setChannelPWM(1, pwmServo2.pwmForAngle(0));
    Serial.println(pwmController.getChannelPWM(1)); // Should output 324 for 0°

    // Showing more resolution in the 0° to +90° range
    Serial.println(pwmServo2.pwmForAngle(45));  // Should output 424 for +45°, 102 away from +90°

    pwmController.setChannelPWM(1, pwmServo2.pwmForAngle(90));
    Serial.println(pwmController.getChannelPWM(1)); // Should output 526 for +90°
}

void loop() {
}

```

### LED Example

In this example, we use inverted open-drain driver mode to power a row of LEDs. Most breakouts have a 220Ω resistor between the individual channel outputs of the IC and PWM output pins. Inverted mode is used here so that the IC is set into a current drain mode, meaning that the PWM output pins should be connected to the negative/cathode side of the LED it's hooked up to (that's the shorter leg).

```Arduino
#include "PCA9685.h"

PCA9685 pwmController;                  // Library using default B000000 (A5-A0) i2c address, and default Wire @400kHz

void setup() {
    Serial.begin(115200);               // Begin Serial and Wire interfaces
    Wire.begin();
    Wire.setClock(pwmController.getI2CSpeed());

    pwmController.resetDevices();       // Resets all PCA9685 devices on i2c line

    // Initializes module using inverted open-drain style driver mode, optimal for powering LEDs
    pwmController.init(PCA9685_OutputDriverMode_OpenDrain,
                       PCA9685_OutputEnabledMode_Inverted);

    pwmController.setPWMFreqLED();      // 60Hz provides more than enough HIGH phase area for LEDs

    randomSeed(analogRead(0));          // Use white noise for our randomness
}

void loop() {
    pwmController.setChannelPWM(random(0, 15), random(0, 4096));
    delay(50);
}

```

### Software i2c Example

In this example, we utilize a popular software i2c library for chips that do not have a hardware i2c bus, available at <https://github.com/felias-fogg/SoftI2CMaster>.

If one uncomments the line below inside the main header file (or defines it via custom build flag), software i2c mode for the library will be enabled. Additionally, you will need to correctly define `SCL_PIN`, `SCL_PORT`, `SDA_PIN`, and `SDA_PORT` according to your setup. `I2C_FASTMODE=1` should be set for 16MHz+ processors. Lastly note that, while in software i2c mode, the i2c clock speed returned by the library (via `getI2CSpeed()`) is only an upper bound and may not represent the actual i2c clock speed set nor achieved.

In PCA9685.h:
```Arduino
// Uncomment or -D this define to enable usage of the software i2c library (min 4MHz+ processor).
#define PCA9685_ENABLE_SOFTWARE_I2C             // https://github.com/felias-fogg/SoftI2CMaster
```  
Alternatively, in platform[.local].txt:
```Arduino
build.extra_flags=-DPCA9685_ENABLE_SOFTWARE_I2C
```

In main sketch:
```Arduino
#include "PCA9685.h"

// Setup defines for SoftI2CMaster are written before library include. That is because
// its header contains the full code definition, and should thus be included only once.
// The values for SCL_PORT and SDA_PORT are dependent upon which pins are used - refer to
// http://www.arduino.cc/en/Reference/PortManipulation to determine what you should use.
#define SCL_PIN 2
#define SCL_PORT PORTD
#define SDA_PIN 0 
#define SDA_PORT PORTC

#if F_CPU >= 16000000
#define I2C_FASTMODE 1                  // Running a 16MHz processor allows us to use i2c fast mode
#endif

#include "SoftI2CMaster.h"              // Include must come after setup defines (see library setup)

PCA9685 pwmController;                  // Library using default B000000 (A5-A0) i2c address

void setup() {
    Serial.begin(115200);               // Begin Serial and SoftI2C interfaces
    i2c_init();

    pwmController.resetDevices();       // Resets all PCA9685 devices on i2c line

    // Initializes module using software linear phase balancer, and default totem-pole driver mode
    pwmController.init(PCA9685_PhaseBalancer_Linear);

    pwmController.setChannelPWM(0, 2048); // Should see a 50% duty cycle along the 5ms phase width
}

void loop() {
}

```

## Module Info

In this example, we enable debug output support to print out module diagnostic information.

If one uncomments the line below inside the main header file (or defines it via custom build flag), debug output support will be enabled and the `printModuleInfo()` method will become available. Calling this method will display information about the module itself, including initalized states, register values, current settings, etc. Additionally, all library calls being made will display internal debug information about the structure of the call itself. An example of this output is shown below.

In PCA9685.h:
```Arduino
// Uncomment or -D this define to enable debug output.
#define PCA9685_ENABLE_DEBUG_OUTPUT
```  
Alternatively, in platform[.local].txt:
```Arduino
build.extra_flags=-DPCA9685_ENABLE_DEBUG_OUTPUT
```

In main sketch:
```Arduino
#include "PCA9685.h"

PCA9685 pwmController;                  // Library using default B000000 (A5-A0) i2c address, and default Wire @400kHz

void setup() {
    Serial.begin(115200);               // Begin Serial and Wire interfaces
    Wire.begin();
    Wire.setClock(pwmController.getI2CSpeed());

    pwmController.init();               // Initializes module using default totem-pole driver mode

    pwmController.printModuleInfo();    // Prints module diagnostic information
}

void loop() {
}

```

In serial monitor:
```
 ~~~ PCA9685 Module Info ~~~

i2c Address: 0x40
i2c Instance: 0: Wire
i2c Speed: 400kHz

Phase Balancer: 0: PCA9685_PhaseBalancer_None

Proxy Addresser: false

Mode1 Register:
  PCA9685::readRegister regAddress: 0x0
    PCA9685::readRegister retVal: 0x20
0x20, Bitset: PCA9685_MODE1_AUTOINC

Mode2 Register:
  PCA9685::readRegister regAddress: 0x1
    PCA9685::readRegister retVal: 0x4
0x4, Bitset: PCA9685_MODE2_OUTDRV_TPOLE

SubAddress1 Register:
  PCA9685::readRegister regAddress: 0x2
    PCA9685::readRegister retVal: 0xE2
0xE2

SubAddress2 Register:
  PCA9685::readRegister regAddress: 0x3
    PCA9685::readRegister retVal: 0xE4
0xE4

SubAddress3 Register:
  PCA9685::readRegister regAddress: 0x4
    PCA9685::readRegister retVal: 0xE8
0xE8

AllCall Register:
  PCA9685::readRegister regAddress: 0x5
    PCA9685::readRegister retVal: 0xE0
0xE0

```
