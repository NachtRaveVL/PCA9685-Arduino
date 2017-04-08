# PCA9685-Arduino
Arduino Library for the PCA9685 16-Channel PWM Driver Module.

**PCA9685-Arduino - Version 1.2.9**

Library to control a PCA9685 16-channel PWM driver module from an Arduino board.  
Licensed under the copy-left GNU GPL v3 license.

Created by Kasper Skårhøj, August 3rd, 2012.  
Forked by Vitska, June 18th, 2016.  
Forked by NachtRaveVL, July 29th, 2016.

This library allows communication with boards running a PCA6985 16-channel PWM driver module. It supports a wide range of available functionality, from setting the output PWM frequecy, allowing multi-device proxy addressing, and provides an assistant class for working with Servos.

The datasheet for the IC is available from <http://www.nxp.com/documents/data_sheet/PCA9685.pdf>.

## Library Setup

There are several defines inside of the library's header file that allows for more fine-tuned control.

```Arduino
// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define PCA9685_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define if wanting to exclude extended functionality from compilation.
//#define PCA9685_EXCLUDE_EXT_FUNC        1

// Uncomment this define if wanting to exclude ServoEvaluator assistant from compilation.
//#define PCA9685_EXCLUDE_SERVO_EVAL      1

// Uncomment this define to enable debug output.
//#define PCA9685_ENABLE_DEBUG_OUTPUT     1
```

## Servo Control Note

Many 180 degree controlled digital servos run on a 20ms pulse width (50Hz update frequency) based duty cycle, and do not utilize the entire pulse width for their -90/+90 degree control. Typically, 2.5% of the 20ms pulse width (0.5ms) is considered -90 degrees, and 12.5% of the 20ms pulse width (2.5ms) is considered +90 degrees. This roughly translates to raw PCA9685 PWM values of 102 and 512 (out of the 4096 value range) for -90 to +90 degree control, but may need to be adjusted to fit your specific servo (e.g. some I've tested run ~130 to ~525 for their -90/+90 degree control). Also be aware that driving some servos past their -90/+90 degrees of movement can cause a little plastic limiter pin to break off and get stuck inside of the gearing, which could potentially cause the servo to become jammed. See the PCA9685_ServoEvaluator class to assist with calculating PWM values from Servo angle values.

## Example Usage

Below are several examples of library usage.

### Simple Example
```Arduino
#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwmController;                  // Library using default Wire and default linear phase balancing scheme

void setup() {
    Serial.begin(115200);

    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line

    pwmController.init(B000000);        // Address pins A5-A0 set to B000000
    pwmController.setPWMFrequency(100); // Default is 200Hz, supports 24Hz to 1526Hz

    pwmController.setChannelPWM(0, 128 << 4); // Set PWM to 128/255, but in 4096 land

    Serial.println(pwmController.getChannelPWM(0)); // Should output 2048, which is 128 << 4
}

```

### Batching Example

In this example, we randomly select PWM frequencies on all 12 outputs and allow them to drive for 5 seconds before changing them.

```Arduino
#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwmController;                  // Library using default Wire and default linear phase balancing scheme

void setup() {
	Serial.begin(115200);

    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line

    pwmController.init(B010101);        // Address pins A5-A0 set to B010101
    pwmController.setPWMFrequency(500); // Default is 200Hz, supports 24Hz to 1526Hz

    randomSeed(analogRead(0));          // Use white noise for our randomness
}

void loop() {
    word pwms[12];
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

    // Note: Only 7 channels can be written in one i2c transaction due to a
    // BUFFER_LENGTH limit of 32, so 12 channels will take two i2c transactions.
}

```

### Multi-Device Proxy Example

In this example, we use a special instance to control other modules attached to it via the ALL_CALL register.

```Arduino
#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwmController1;                 // Library using default Wire and default linear phase balancing scheme
PCA9685 pwmController2;                 // Library using default Wire and default linear phase balancing scheme

PCA9685 pwmControllerAll;               // Not a real device, will act as a proxy to pwmController1 and pwmController2

void setup() {
    Serial.begin(115200);

    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmController1.resetDevices();      // Software resets all PCA9685 devices on Wire line (including pwmController2 in this case)

    pwmController1.init(B000000);       // Address pins A5-A0 set to B000000
    pwmController2.init(B000001);       // Address pins A5-A0 set to B000001

    pwmController1.setChannelOff(0);    // Turn channel 0 off
    pwmController2.setChannelOff(0);    // On both

    pwmController1.enableAllCallAddress(); // Default address of 0xE0
    pwmController2.enableAllCallAddress(); // Same default address

    pwmControllerAll.initAsProxyAddresser(); // Same default address of 0x0E as used in enable above

    pwmControllerAll.setChannelPWM(0, 4096); // Enables full on on both pwmController1 and pwmController2

    Serial.println(pwmController1.getChannelPWM(0)); // Should output 4096
    Serial.println(pwmController2.getChannelPWM(0)); // Should also output 4096

    // Note: Various parts of functionality of the proxy class instance are actually
    // disabled - typically anything that involves a read command being issued.
}

```

### Servo Evaluator Example

In this example, we utilize the ServoEvaluator class to assist with setting PWM frequencies when working with servos. We will be using Wire1, which is only available on boards with SDA1/SCL1 (Due, Zero, etc.) - change to Wire if Wire1 is unavailable.

```Arduino
#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwmController(Wire1, PCA9685_PhaseBalancer_Weaved); // Library using Wire1 and weaved phase balancing scheme

// Linearly interpolates between standard 2.5%/12.5% phase length (102/512) for -90°/+90°
PCA9685_ServoEvaluator pwmServo1;

// Testing our second servo has found that -90° sits at 128, 0° at 324, and +90° at 526.
// Since 324 isn't precisely in the middle, a cubic spline will be used to smoothly
// interpolate PWM values, which will account for said discrepancy. Additionally, since
// 324 is closer to 128 than 526, there is less resolution in the -90° to 0° range, and
// more in the 0° to +90° range.
PCA9685_ServoEvaluator pwmServo2(128,324,526);

void setup() {
    Serial.begin(115200);

    Wire1.begin();                      // Wire must be started first
    Wire1.setClock(400000);             // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line

    pwmController.init(B000000);        // Address pins A5-A0 set to B000000
    pwmController.setPWMFrequency(50);  // 50Hz provides 20ms standard servo phase length

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

```

### Software I2C Example

In this example, we utilize the software I2C functionality for chips that do not have a hardware I2C bus. We must uncomment the PCA9685_ENABLE_SOFTWARE_I2C define in the libraries main header file for software I2C mode to be enabled.

In PCA9685.h:
```Arduino
// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
#define PCA9685_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary
```

In main sketch:
```Arduino
#include "PCA9685.h"

#define SCL_PIN 2                       // Setup defines are written before library include
#define SCL_PORT PORTD 
#define SDA_PIN 0 
#define SDA_PORT PORTC 

#if F_CPU >= 16000000
#define I2C_FASTMODE 1                  // Running a 16MHz processor allows us to use I2C fast mode
#endif

#include "SoftI2CMaster.h"              // Include must come after setup defines

PCA9685 pwmController;                  // Library using default linear phase balancing scheme

void setup() {
	Serial.begin(115200);

    i2c_init();                         // Software I2C must be started first

    pwmController.resetDevices();       // Software resets all PCA9685 devices on software I2C line

    pwmController.init(B000000);        // Address pins A5-A0 set to B000000

    pwmController.setChannelPWM(0, 2048); // Should see a 50% duty cycle along the 5ms phase width
}

```

## Module Info

If one uncomments the PCA9685_ENABLE_DEBUG_OUTPUT define in the libraries main header file (thus enabling debug output) the printModuleInfo() method becomes available, which will display information about the module itself, including initalized states, register values, current settings, etc. All calls being made will display internal debug information about the structure of the call itself. An example of this output is shown here:

In PCA9685.h:
```Arduino
// Uncomment this define to enable debug output.
#define PCA9685_ENABLE_DEBUG_OUTPUT     1
```

In main sketch:
```Arduino
#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwmController;

void setup() {
    Serial.begin(115200);

    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmController.printModuleInfo();
}

```

In serial monitor:
```
  ~~~ PCA9685 Module Info ~~~

i2c Address:
0x40

Phase Balancer:
PCA9685_PhaseBalancer_Weaved

Proxy Addresser:
false

Mode1 Register:
  PCA9685::readRegister regAddress: 0x0
    PCA9685::readRegister retVal: 0x20
0x20, Bitset: PCA9685_MODE_AUTOINC

Mode2 Register:
  PCA9685::readRegister regAddress: 0x1
    PCA9685::readRegister retVal: 0xC
0xC, Bitset: PCA9685_MODE_OUTPUT_ONACK PCA9685_MODE_OUTPUT_TPOLE

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
