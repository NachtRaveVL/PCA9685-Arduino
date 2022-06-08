// PCA9685-Arduino Software i2c Example
// In this example, we utilize a popular software i2c library for chips that do not have
// a hardware i2c bus. If one uncomments the line below inside the main header file (or
// defines it via custom build flag), software i2c mode for the library will be enabled.
// Additionally, you will need to correctly define SCL_PIN, SCL_PORT, SDA_PIN, and
// SDA_PORT according to your setup. I2C_FASTMODE=1 should be set for 16MHz+ processors.
// You may refer to https://forum.arduino.cc/index.php?topic=602603.0 on how to define
// custom build flags manually via modifying platform[.local].txt.
//
// In PCA9685.h:
// // Uncomment or -D this define to enable usage of the software i2c library (min 4MHz+ processor).
// #define PCA9685_ENABLE_SOFTWARE_I2C             // https://github.com/felias-fogg/SoftI2CMaster
//
// Alternatively, in platform[.local].txt:
// build.extra_flags=-DPCA9685_ENABLE_SOFTWARE_I2C

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
