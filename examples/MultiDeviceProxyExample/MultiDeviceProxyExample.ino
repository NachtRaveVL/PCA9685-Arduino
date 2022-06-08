// PCA9685-Arduino Multi-Device Proxy Example
// In this example, we use a special instance to control other modules attached to it via
// the ALL_CALL register.

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
