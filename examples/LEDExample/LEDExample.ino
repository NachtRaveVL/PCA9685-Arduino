// PCA9685-Arduino LED Example
// In this example, we use inverted open-drain driver mode to power a row of LEDs. Most
// breakouts have a 220Ω resistor between the individual channel outputs of the IC and
// PWM output pins. Inverted mode is used here so that the IC is set into a current drain
// mode, meaning that the PWM output pins should be connected to the negative/cathode
// side of the LED it's hooked up to (that's the shorter leg).

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
