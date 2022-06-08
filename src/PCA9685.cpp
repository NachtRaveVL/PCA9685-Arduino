/*  Arduino Library for the PCA9685 16-Channel PWM Driver Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    Copyright (C) 2012 Kasper Skårhøj       <kasperskaarhoj@gmail.com>
    PCA9685 Main
*/

#include "PCA9685.h"

#define PCA9685_I2C_BASE_MODULE_ADDRESS (byte)0x40
#define PCA9685_I2C_BASE_MODULE_ADRMASK (byte)0x3F
#define PCA9685_I2C_BASE_PROXY_ADDRESS  (byte)0xE0
#define PCA9685_I2C_BASE_PROXY_ADRMASK  (byte)0xFE

// Register addresses from data sheet
#define PCA9685_MODE1_REG               (byte)0x00
#define PCA9685_MODE2_REG               (byte)0x01
#define PCA9685_SUBADR1_REG             (byte)0x02
#define PCA9685_SUBADR2_REG             (byte)0x03
#define PCA9685_SUBADR3_REG             (byte)0x04
#define PCA9685_ALLCALL_REG             (byte)0x05
#define PCA9685_LED0_REG                (byte)0x06          // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_PRESCALE_REG            (byte)0xFE
#define PCA9685_ALLLED_REG              (byte)0xFA

// Mode1 register values
#define PCA9685_MODE1_RESTART           (byte)0x80
#define PCA9685_MODE1_EXTCLK            (byte)0x40
#define PCA9685_MODE1_AUTOINC           (byte)0x20
#define PCA9685_MODE1_SLEEP             (byte)0x10
#define PCA9685_MODE1_SUBADR1           (byte)0x08
#define PCA9685_MODE1_SUBADR2           (byte)0x04
#define PCA9685_MODE1_SUBADR3           (byte)0x02
#define PCA9685_MODE1_ALLCALL           (byte)0x01

// Mode2 register values
#define PCA9685_MODE2_OUTDRV_TPOLE      (byte)0x04
#define PCA9685_MODE2_INVRT             (byte)0x10
#define PCA9685_MODE2_OUTNE_TPHIGH      (byte)0x01
#define PCA9685_MODE2_OUTNE_HIGHZ       (byte)0x02
#define PCA9685_MODE2_OCH_ONACK         (byte)0x08

#define PCA9685_SW_RESET                (byte)0x06          // Sent to address 0x00 to reset all devices on Wire line
#define PCA9685_PWM_FULL                (uint16_t)0x1000    // Special value for full on/full off LEDx modes
#define PCA9685_PWM_MASK                (uint16_t)0x0FFF    // Mask for 12-bit/4096 possible phase positions

#define PCA9685_CHANNEL_COUNT           16
#define PCA9685_MIN_CHANNEL             0
#define PCA9685_MAX_CHANNEL             (PCA9685_CHANNEL_COUNT - 1)
#define PCA9685_ALLLED_CHANNEL          -1                  // Special value for ALLLED registers

#define PCA9685_PBLIN_STEPS             (PCA9685_PWM_FULL / PCA9685_CHANNEL_COUNT) // Step size per channnel for linear phase balancing


#ifdef PCA9685_USE_SOFTWARE_I2C
boolean __attribute__((noinline)) i2c_init(void);
bool __attribute__((noinline)) i2c_start(uint8_t addr);
void __attribute__((noinline)) PCA9685_i2c_stop(void) asm("ass_i2c_stop");
bool __attribute__((noinline)) PCA9685_i2c_write(uint8_t value) asm("ass_i2c_write");
uint8_t __attribute__((noinline)) i2c_read(bool last);
#endif

static void uDelayMillisFuncDef(unsigned int timeout) {
#if defined(PCA9685_YIELD)
    if (timeout > 0) {
        unsigned long currTime = millis();
        unsigned long endTime = currTime + (unsigned long)timeout;
        if (currTime < endTime) { // not overflowing
            while (millis() < endTime)
                PCA9685_YIELD();
        } else { // overflowing
            unsigned long begTime = currTime;
            while (currTime >= begTime || currTime < endTime) {
                PCA9685_YIELD();
                currTime = millis();
            }
        }
    } else
        PCA9685_YIELD();
#else
    delay(timeout);
#endif
}

static void uDelayMicrosFuncDef(unsigned int timeout) {
#if defined(PCA9685_YIELD)
    if (timeout > 1000) {
        unsigned long currTime = micros();
        unsigned long endTime = currTime + (unsigned long)timeout;
        if (currTime < endTime) { // not overflowing
            while (micros() < endTime)
                PCA9685_YIELD();
        } else { // overflowing
            unsigned long begTime = currTime;
            while (currTime >= begTime || currTime < endTime) {
                PCA9685_YIELD();
                currTime = micros();
            }
        }
    } else if (timeout > 0)
        delayMicroseconds(timeout);
    else
        PCA9685_YIELD();
#else
    delayMicroseconds(timeout);
#endif
}


#ifndef PCA9685_USE_SOFTWARE_I2C

PCA9685::PCA9685(byte i2cAddress, TwoWire& i2cWire, uint32_t i2cSpeed)
    // I2C 7-bit address is B 1 A5 A4 A3 A2 A1 A0
    // RW lsb bit added by Arduino core TWI library
    : _i2cAddress(i2cAddress),
      _i2cWire(&i2cWire),
      _i2cSpeed(i2cSpeed),
      _driverMode(PCA9685_OutputDriverMode_Undefined),
      _enabledMode(PCA9685_OutputEnabledMode_Undefined),
      _disabledMode(PCA9685_OutputDisabledMode_Undefined),
      _updateMode(PCA9685_ChannelUpdateMode_Undefined),
      _phaseBalancer(PCA9685_PhaseBalancer_Undefined),
      _isProxyAddresser(false),
      _uDelayMillisFunc(uDelayMillisFuncDef), _uDelayMicrosFunc(uDelayMicrosFuncDef),
      _lastI2CError(0)
{ }

PCA9685::PCA9685(TwoWire& i2cWire, uint32_t i2cSpeed, byte i2cAddress)
    : _i2cAddress(i2cAddress),
      _i2cWire(&i2cWire),
      _i2cSpeed(i2cSpeed),
      _driverMode(PCA9685_OutputDriverMode_Undefined),
      _enabledMode(PCA9685_OutputEnabledMode_Undefined),
      _disabledMode(PCA9685_OutputDisabledMode_Undefined),
      _updateMode(PCA9685_ChannelUpdateMode_Undefined),
      _phaseBalancer(PCA9685_PhaseBalancer_Undefined),
      _isProxyAddresser(false),
      _uDelayMillisFunc(uDelayMillisFuncDef), _uDelayMicrosFunc(uDelayMicrosFuncDef),
      _lastI2CError(0)
{ }

#else

PCA9685::PCA9685(byte i2cAddress)
    : _i2cAddress(i2cAddress),
      _readBytes(0),
      _driverMode(PCA9685_OutputDriverMode_Undefined),
      _enabledMode(PCA9685_OutputEnabledMode_Undefined),
      _disabledMode(PCA9685_OutputDisabledMode_Undefined),
      _updateMode(PCA9685_ChannelUpdateMode_Undefined),
      _phaseBalancer(PCA9685_PhaseBalancer_Undefined),
      _isProxyAddresser(false),
      _uDelayMillisFunc(uDelayMillisFuncDef), _uDelayMicrosFunc(uDelayMicrosFuncDef),
      _lastI2CError(0)
{ }

#endif // /ifndef PCA9685_USE_SOFTWARE_I2C

void PCA9685::resetDevices() {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::resetDevices");
#endif

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif

    i2cWire_beginTransmission(0x00);
    i2cWire_write(PCA9685_SW_RESET);
    i2cWire_endTransmission();

    _uDelayMicrosFunc(10);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void PCA9685::init(PCA9685_OutputDriverMode driverMode,
                   PCA9685_OutputEnabledMode enabledMode,
                   PCA9685_OutputDisabledMode disabledMode,
                   PCA9685_ChannelUpdateMode updateMode,
                   PCA9685_PhaseBalancer phaseBalancer) {
    if (_isProxyAddresser) return;

    _i2cAddress = PCA9685_I2C_BASE_MODULE_ADDRESS | (_i2cAddress & PCA9685_I2C_BASE_MODULE_ADRMASK);

    _driverMode = driverMode;
    _enabledMode = enabledMode;
    _disabledMode = disabledMode;
    _updateMode = updateMode;
    _phaseBalancer = phaseBalancer;

    assert(!(_driverMode == PCA9685_OutputDriverMode_OpenDrain && _disabledMode == PCA9685_OutputDisabledMode_High && "Unsupported combination"));

    byte mode2Val = getMode2Value();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::init mode2Val: 0x");
    Serial.print(mode2Val, HEX);
    Serial.print(", i2cAddress: 0x");
    Serial.print(_i2cAddress, HEX);
    Serial.print(", i2cWire#: ");
    Serial.print(getWireInterfaceNumber());
    Serial.print(", i2cSpeed: ");
    Serial.print(roundf(getI2CSpeed() / 1000.0f)); Serial.print("kHz");
    Serial.print(", driverMode: ");
    switch(_driverMode) {
        case PCA9685_OutputDriverMode_OpenDrain: Serial.print("OpenDrain"); break;
        case PCA9685_OutputDriverMode_TotemPole: Serial.print("TotemPole"); break;
        case PCA9685_OutputDriverMode_Count:
        case PCA9685_OutputDriverMode_Undefined:
            Serial.print(_driverMode); break;
    }
    Serial.print(", enabledMode: ");
    switch (_enabledMode) {
        case PCA9685_OutputEnabledMode_Normal: Serial.print("Normal"); break;
        case PCA9685_OutputEnabledMode_Inverted: Serial.print("Inverted"); break;
        case PCA9685_OutputEnabledMode_Count:
        case PCA9685_OutputEnabledMode_Undefined:
            Serial.print(_enabledMode); break;
    }
    Serial.print(", disabledMode: ");
    switch (_disabledMode) {
        case PCA9685_OutputDisabledMode_Low: Serial.print("Low"); break;
        case PCA9685_OutputDisabledMode_High: Serial.print("High"); break;
        case PCA9685_OutputDisabledMode_Floating: Serial.print("Floating"); break;
        case PCA9685_OutputDisabledMode_Count:
        case PCA9685_OutputDisabledMode_Undefined:
            Serial.print(_disabledMode); break;
    }
    Serial.print(", updateMode: ");
    switch (_updateMode) {
        case PCA9685_ChannelUpdateMode_AfterStop: Serial.print("AfterStop"); break;
        case PCA9685_ChannelUpdateMode_AfterAck: Serial.print("AfterAck"); break;
        case PCA9685_ChannelUpdateMode_Count:
        case PCA9685_ChannelUpdateMode_Undefined:
            Serial.print(_updateMode); break;
    }
    Serial.print(", phaseBalancer: ");
    switch(_phaseBalancer) {
        case PCA9685_PhaseBalancer_None: Serial.print("None"); break;
        case PCA9685_PhaseBalancer_Linear: Serial.print("Linear"); break;
        case PCA9685_PhaseBalancer_Dynamic: Serial.print("Dynamic"); break;
        case PCA9685_PhaseBalancer_Count:
        case PCA9685_PhaseBalancer_Undefined:
            Serial.print(_phaseBalancer); break;
    }
    Serial.println("");
#endif

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif

    writeRegister(PCA9685_MODE1_REG, PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC);
    writeRegister(PCA9685_MODE2_REG, mode2Val);
}

void PCA9685::init(PCA9685_PhaseBalancer phaseBalancer,
                   PCA9685_OutputDriverMode driverMode,
                   PCA9685_OutputEnabledMode enabledMode,
                   PCA9685_OutputDisabledMode disabledMode,
                   PCA9685_ChannelUpdateMode updateMode) {
    init(driverMode, enabledMode, disabledMode, updateMode, phaseBalancer);
}

void PCA9685::initAsProxyAddresser() {
    if (_driverMode != PCA9685_OutputDriverMode_Undefined) return;

    _i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (_i2cAddress & PCA9685_I2C_BASE_PROXY_ADRMASK);
    _isProxyAddresser = true;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::initAsProxyAddresser i2cAddress: 0x");
    Serial.print(_i2cAddress, HEX);
    Serial.print(", i2cWire#: ");
    Serial.print(getWireInterfaceNumber());
    Serial.print(", i2cSpeed: ");
    Serial.print(roundf(getI2CSpeed() / 1000.0f)); Serial.print("kHz");
    Serial.println("");
#endif

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

byte PCA9685::getI2CAddress() {
    return _i2cAddress;
}

uint32_t PCA9685::getI2CSpeed() {
#ifndef PCA9685_USE_SOFTWARE_I2C
    return _i2cSpeed;
#else
#if I2C_FASTMODE || F_CPU >= 16000000
    return 400000;
#else
    return 100000;
#endif
#endif // ifndef PCA9685_USE_SOFTWARE_I2C
}

PCA9685_OutputDriverMode PCA9685::getOutputDriverMode() {
    return _driverMode;
}

PCA9685_OutputEnabledMode PCA9685::getOutputEnabledMode() {
    return _enabledMode;
}

PCA9685_OutputDisabledMode PCA9685::getOutputDisabledMode() {
    return _disabledMode;
}

PCA9685_ChannelUpdateMode PCA9685::getChannelUpdateMode() {
    return _updateMode;
}

PCA9685_PhaseBalancer PCA9685::getPhaseBalancer() {
    return _phaseBalancer;
}

byte PCA9685::getMode2Value() {
    byte mode2Val = (byte)0x00;

    if (_driverMode == PCA9685_OutputDriverMode_TotemPole) {
        mode2Val |= PCA9685_MODE2_OUTDRV_TPOLE;
    }

    if (_enabledMode == PCA9685_OutputEnabledMode_Inverted) {
        mode2Val |= PCA9685_MODE2_INVRT;
    }

    if (_disabledMode == PCA9685_OutputDisabledMode_High) {
        mode2Val |= PCA9685_MODE2_OUTNE_TPHIGH;
    }
    else if (_disabledMode == PCA9685_OutputDisabledMode_Floating) {
        mode2Val |= PCA9685_MODE2_OUTNE_HIGHZ;
    }

    if (_updateMode == PCA9685_ChannelUpdateMode_AfterAck) {
        mode2Val |= PCA9685_MODE2_OCH_ONACK;
    }

    return mode2Val;
}

void PCA9685::setUserDelayFuncs(UserDelayFunc delayMillisFunc, UserDelayFunc delayMicrosFunc) {
    _uDelayMillisFunc = delayMillisFunc ? delayMillisFunc : uDelayMillisFuncDef;
    _uDelayMicrosFunc = delayMicrosFunc ? delayMicrosFunc : uDelayMicrosFuncDef;
}

void PCA9685::setPWMFrequency(float pwmFrequency) {
    if (pwmFrequency < 0 || _isProxyAddresser) return;

    // This equation comes from section 7.3.5 of the datasheet, but the rounding has been
    // removed because it isn't needed. Lowest freq is 23.84, highest is 1525.88.
    int preScalerVal = (25000000 / (4096 * pwmFrequency)) - 1;
    if (preScalerVal > 255) preScalerVal = 255;
    if (preScalerVal < 3) preScalerVal = 3;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::setPWMFrequency pwmFrequency: ");
    Serial.print(pwmFrequency);
    Serial.print(", preScalerVal: 0x");
    Serial.println(preScalerVal, HEX);
#endif

    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP));
    writeRegister(PCA9685_PRESCALE_REG, (byte)preScalerVal);

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART));
    _uDelayMicrosFunc(500);
}

void PCA9685::setPWMFreqServo() {
    setPWMFrequency(50);
}

void PCA9685::setPWMFreqLED() {
    setPWMFrequency(60);
}

void PCA9685::setChannelOn(int channel) {
    if (channel < 0 || channel > 15) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::setChannelOn");
#endif

    writeChannelBegin(channel);
    writeChannelPWM(PCA9685_PWM_FULL, 0);  // time_on = FULL; time_off = 0;
    writeChannelEnd();
}

void PCA9685::setChannelOff(int channel) {
    if (channel < 0 || channel > 15) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::setChannelOff");
#endif

    writeChannelBegin(channel);
    writeChannelPWM(0, PCA9685_PWM_FULL);  // time_on = 0; time_off = FULL;
    writeChannelEnd();
}

void PCA9685::setChannelPWM(int channel, uint16_t pwmAmount) {
    if (channel < 0 || channel > 15) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::setChannelPWM");
#endif

    writeChannelBegin(channel);

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(channel, pwmAmount, &phaseBegin, &phaseEnd);

    writeChannelPWM(phaseBegin, phaseEnd);

    writeChannelEnd();
}

void PCA9685::setChannelsPWM(int begChannel, int numChannels, const uint16_t *pwmAmounts) {
    if (begChannel < 0 || begChannel > 15 || numChannels < 0) return;
    if (begChannel + numChannels > 16) numChannels -= (begChannel + numChannels) - 16;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::setChannelsPWM numChannels: ");
    Serial.println(numChannels);
#endif

    // From avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many channels can be written at once. Therefore, we loop around until all
    // channels have been written out into their registers. I2C_BUFFER_LENGTH is used in
    // other architectures, so we rely on PCA9685_I2C_BUFFER_LENGTH logic to sort it out.

    while (numChannels > 0) {
        writeChannelBegin(begChannel);

#ifndef PCA9685_USE_SOFTWARE_I2C
        int maxChannels = min(numChannels, (PCA9685_I2C_BUFFER_LENGTH - 1) / 4);
#else // TODO: Software I2C doesn't have buffer length restrictions? -NR
        int maxChannels = numChannels;
#endif
        while (maxChannels-- > 0) {
            uint16_t phaseBegin, phaseEnd;
            getPhaseCycle(begChannel++, *pwmAmounts++, &phaseBegin, &phaseEnd);

            writeChannelPWM(phaseBegin, phaseEnd);
            --numChannels;
        }

        writeChannelEnd();
        if (_lastI2CError) return;
    }
}

void PCA9685::setAllChannelsPWM(uint16_t pwmAmount) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::setAllChannelsPWM");
#endif

    writeChannelBegin(PCA9685_ALLLED_CHANNEL);

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(PCA9685_ALLLED_CHANNEL, pwmAmount, &phaseBegin, &phaseEnd);

    writeChannelPWM(phaseBegin, phaseEnd);

    writeChannelEnd();
}

uint16_t PCA9685::getChannelPWM(int channel) {
    if (channel < 0 || channel > 15 || _isProxyAddresser) return 0;

    byte regAddress = PCA9685_LED0_REG + (channel << 2);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::getChannelPWM channel: ");
    Serial.print(channel);
    Serial.print(", regAddress: 0x");
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
    if (i2cWire_endTransmission()) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    int bytesRead = i2cWire_requestFrom((uint8_t)_i2cAddress, 4);
    if (bytesRead != 4) {
        while (bytesRead-- > 0)
            i2cWire_read();
#ifdef PCA9685_USE_SOFTWARE_I2C
        PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif
        _lastI2CError = 4;
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

#ifndef PCA9685_SWAP_PWM_BEG_END_REGS
    uint16_t phaseBegin = (uint16_t)i2cWire_read();
    phaseBegin |= (uint16_t)i2cWire_read() << 8;
    uint16_t phaseEnd = (uint16_t)i2cWire_read();
    phaseEnd |= (uint16_t)i2cWire_read() << 8;
#else
    uint16_t phaseEnd = (uint16_t)i2cWire_read();
    phaseEnd |= (uint16_t)i2cWire_read() << 8;
    uint16_t phaseBegin = (uint16_t)i2cWire_read();
    phaseBegin |= (uint16_t)i2cWire_read() << 8;
#endif

#ifdef PCA9685_USE_SOFTWARE_I2C
    PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  PCA9685::getChannelPWM phaseBegin: ");
    Serial.print(phaseBegin);
    Serial.print(", phaseEnd: ");
    Serial.println(phaseEnd);
#endif

    // See datasheet section 7.3.3
    uint16_t retVal;
    if (phaseEnd >= PCA9685_PWM_FULL)
        // Full OFF
        // Figure 11 Example 4: full OFF takes precedence over full ON
        // See also remark after Table 7
        retVal = 0;
    else if (phaseBegin >= PCA9685_PWM_FULL)
        // Full ON
        // Figure 9 Example 3
        retVal = PCA9685_PWM_FULL;
    else if (phaseBegin <= phaseEnd)
        // start and finish in same cycle
        // Section 7.3.3 example 1
        retVal = phaseEnd - phaseBegin;
    else
        // span cycles
        // Section 7.3.3 example 2
        retVal = (phaseEnd + PCA9685_PWM_FULL) - phaseBegin;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  PCA9685::getChannelPWM retVal: ");
    Serial.println(retVal);
#endif

    return retVal;
}

void PCA9685::enableAllCallAddress(byte i2cAddressAllCall) {
    if (_isProxyAddresser) return;

    byte i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressAllCall & PCA9685_I2C_BASE_PROXY_ADRMASK);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::enableAllCallAddress i2cAddressAllCall: 0x");
    Serial.println(i2cAddress, HEX);
#endif

    writeRegister(PCA9685_ALLCALL_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_ALLCALL));
}

void PCA9685::enableSub1Address(byte i2cAddressSub1) {
    if (_isProxyAddresser) return;

    byte i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressSub1 & PCA9685_I2C_BASE_PROXY_ADRMASK);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::enableSub1Address i2cAddressSub1: 0x");
    Serial.println(i2cAddress, HEX);
#endif

    writeRegister(PCA9685_SUBADR1_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_SUBADR1));
}

void PCA9685::enableSub2Address(byte i2cAddressSub2) {
    if (_isProxyAddresser) return;

    byte i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressSub2 & PCA9685_I2C_BASE_PROXY_ADRMASK);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::enableSub2Address i2cAddressSub2: 0x");
    Serial.println(i2cAddress, HEX);
#endif

    writeRegister(PCA9685_SUBADR2_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_SUBADR2));
}

void PCA9685::enableSub3Address(byte i2cAddressSub3) {
    if (_isProxyAddresser) return;

    byte i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressSub3 & PCA9685_I2C_BASE_PROXY_ADRMASK);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::enableSub3Address i2cAddressSub3: 0x");
    Serial.println(i2cAddress, HEX);
#endif

    writeRegister(PCA9685_SUBADR3_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_SUBADR3));
}

void PCA9685::disableAllCallAddress() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::disableAllCallAddress");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_ALLCALL));
}

void PCA9685::disableSub1Address() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::disableSub1Address");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_SUBADR1));
}

void PCA9685::disableSub2Address() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::disableSub2Address");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_SUBADR2));
}

void PCA9685::disableSub3Address() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::disableSub3Address");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_SUBADR3));
}

void PCA9685::enableExtClockLine() {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::enableExtClockLine");
#endif

    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP));
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_EXTCLK));

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART));
    _uDelayMicrosFunc(500);
}

byte PCA9685::getLastI2CError() {
    return _lastI2CError;
}

void PCA9685::getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd) {
    // See datasheet section 7.3.3
    if (pwmAmount == 0) {
        // Full OFF -> time_end[bit12] = 1
        *phaseBegin = 0;
        *phaseEnd = PCA9685_PWM_FULL;
    }
    else if (pwmAmount >= PCA9685_PWM_FULL) {
        // Full ON -> time_beg[bit12] = 1, time_end[bit12] = <ignored>
        *phaseBegin = PCA9685_PWM_FULL;
        *phaseEnd = 0;
    } else if (channel == PCA9685_ALLLED_CHANNEL) {
        // ALLLED should not receive a phase shifted begin value
        *phaseBegin = 0;
        *phaseEnd = (*phaseBegin + pwmAmount) & PCA9685_PWM_MASK;
    } else {
        // Get HIGH phase offset beg/end based on phase balancing scheme
        switch(_phaseBalancer) {
            case PCA9685_PhaseBalancer_None:
            case PCA9685_PhaseBalancer_Count:
            case PCA9685_PhaseBalancer_Undefined:
                *phaseBegin = 0;
                *phaseEnd = (*phaseBegin + pwmAmount) & PCA9685_PWM_MASK;
                break;

            case PCA9685_PhaseBalancer_Linear:
                // Distribute HIGH phase area over more of the duty cycle range to balance load
                *phaseBegin = (uint16_t)max((int_fast16_t)0, (int_fast16_t)(channel * PCA9685_PBLIN_STEPS) - (int_fast16_t)(pwmAmount >> 1));
                *phaseEnd = min(PCA9685_PWM_FULL - 1, *phaseBegin + pwmAmount);
                *phaseBegin = *phaseEnd - pwmAmount;
                break;

            case PCA9685_PhaseBalancer_Dynamic:
                // TODO: Dynamic phase balancing. -NR
                *phaseBegin = 0;
                *phaseEnd = (*phaseBegin + pwmAmount) & PCA9685_PWM_MASK;
                break;
        }
    }
}

void PCA9685::writeChannelBegin(int channel) {
    byte regAddress;

    if (channel != PCA9685_ALLLED_CHANNEL)
        regAddress = PCA9685_LED0_REG + (channel * 0x04);
    else
        regAddress = PCA9685_ALLLED_REG;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  PCA9685::writeChannelBegin channel: ");
    Serial.print(channel);
    Serial.print(", regAddress: 0x");
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
}

void PCA9685::writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  PCA9685::writeChannelPWM phaseBegin: ");
    Serial.print(phaseBegin);
    Serial.print(", phaseEnd: ");
    Serial.println(phaseEnd);
#endif

#ifndef PCA9685_SWAP_PWM_BEG_END_REGS
    i2cWire_write(lowByte(phaseBegin));
    i2cWire_write(highByte(phaseBegin));
    i2cWire_write(lowByte(phaseEnd));
    i2cWire_write(highByte(phaseEnd));
#else
    i2cWire_write(lowByte(phaseEnd));
    i2cWire_write(highByte(phaseEnd));
    i2cWire_write(lowByte(phaseBegin));
    i2cWire_write(highByte(phaseBegin));
#endif
}

void PCA9685::writeChannelEnd() {
    i2cWire_endTransmission();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void PCA9685::writeRegister(byte regAddress, byte value) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  PCA9685::writeRegister regAddress: 0x");
    Serial.print(regAddress, HEX);
    Serial.print(", value: 0x");
    Serial.println(value, HEX);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
    i2cWire_write(value);
    i2cWire_endTransmission();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

byte PCA9685::readRegister(byte regAddress) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  PCA9685::readRegister regAddress: 0x");
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
    if (i2cWire_endTransmission()) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    int bytesRead = i2cWire_requestFrom((uint8_t)_i2cAddress, 1);
    if (bytesRead != 1) {
        while (bytesRead-- > 0)
            i2cWire_read();
#ifdef PCA9685_USE_SOFTWARE_I2C
        PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif
        _lastI2CError = 4;
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    byte retVal = i2cWire_read();

#ifdef PCA9685_USE_SOFTWARE_I2C
    PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("    PCA9685::readRegister retVal: 0x");
    Serial.println(retVal, HEX);
#endif

    return retVal;
}

void PCA9685::i2cWire_beginTransmission(uint8_t addr) {
    _lastI2CError = 0;
#ifndef PCA9685_USE_SOFTWARE_I2C
    _i2cWire->beginTransmission(addr);
#else
    i2c_start(addr);
#endif
}

uint8_t PCA9685::i2cWire_endTransmission(void) {
#ifndef PCA9685_USE_SOFTWARE_I2C
    return (_lastI2CError = _i2cWire->endTransmission());
#else
    PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
    return (_lastI2CError = 0);
#endif
}

uint8_t PCA9685::i2cWire_requestFrom(uint8_t addr, uint8_t len) {
#ifndef PCA9685_USE_SOFTWARE_I2C
    return _i2cWire->requestFrom(addr, (size_t)len);
#else
    i2c_start(addr | 0x01);
    return (_readBytes = len);
#endif
}

size_t PCA9685::i2cWire_write(uint8_t data) {
#ifndef PCA9685_USE_SOFTWARE_I2C
    return _i2cWire->write(data);
#else
    return (size_t)PCA9685_i2c_write(data);
#endif
}

uint8_t PCA9685::i2cWire_read(void) {
#ifndef PCA9685_USE_SOFTWARE_I2C
    return (uint8_t)(_i2cWire->read() & 0xFF);
#else
    if (_readBytes > 1) {
        _readBytes -= 1;
        return (uint8_t)(i2c_read(false) & 0xFF);
    }
    else {
        _readBytes = 0;
        return (uint8_t)(i2c_read(true) & 0xFF);
    }
#endif
}

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT

int PCA9685::getWireInterfaceNumber() {
#ifndef PCA9685_USE_SOFTWARE_I2C
    if (_i2cWire == &Wire) return 0;
#if WIRE_INTERFACES_COUNT > 1
    if (_i2cWire == &Wire1) return 1;
#endif
#if WIRE_INTERFACES_COUNT > 2
    if (_i2cWire == &Wire2) return 2;
#endif
#if WIRE_INTERFACES_COUNT > 3
    if (_i2cWire == &Wire3) return 3;
#endif
#if WIRE_INTERFACES_COUNT > 4
    if (_i2cWire == &Wire4) return 4;
#endif
#if WIRE_INTERFACES_COUNT > 5
    if (_i2cWire == &Wire5) return 5;
#endif
#endif // /ifndef PCA9685_USE_SOFTWARE_I2C
    return -1;
}

static const char *textForWireInterfaceNumber(int wireNum) {
#ifndef PCA9685_USE_SOFTWARE_I2C
    switch (wireNum) {
        case 0: return "Wire";
        case 1: return "Wire1";
        case 2: return "Wire2";
        case 3: return "Wire3";
        case 4: return "Wire4";
        case 5: return "Wire5";
        default: return "<CustomWire>";
    }
#else
    return "SoftwareI2C";
#endif // /ifndef PCA9685_USE_SOFTWARE_I2C
}

void PCA9685::printModuleInfo() {
    Serial.println(""); Serial.println(" ~~~ PCA9685 Module Info ~~~");

    Serial.println(""); Serial.print("i2c Address: ");
    Serial.print("0x"); Serial.println(_i2cAddress, HEX);
    Serial.print("i2c Instance: ");
    Serial.print(getWireInterfaceNumber()); Serial.print(": ");
    Serial.println(textForWireInterfaceNumber(getWireInterfaceNumber()));
    Serial.print("i2c Speed: ");
    Serial.print(roundf(getI2CSpeed() / 1000.0f)); Serial.println("kHz");

    Serial.println(""); Serial.print("Phase Balancer: ");
    Serial.print(_phaseBalancer); Serial.print(": ");
    switch (_phaseBalancer) {
        case PCA9685_PhaseBalancer_None:
            Serial.println("PCA9685_PhaseBalancer_None"); break;
        case PCA9685_PhaseBalancer_Linear:
            Serial.println("PCA9685_PhaseBalancer_Linear"); break;
        case PCA9685_PhaseBalancer_Dynamic:
            Serial.println("PCA9685_PhaseBalancer_Dynamic"); break;
        case PCA9685_PhaseBalancer_Count:
        case PCA9685_PhaseBalancer_Undefined:
            Serial.println(""); break;
    }

    if (!_isProxyAddresser) {
        Serial.println(""); Serial.println("Proxy Addresser: false");

        Serial.println(""); Serial.println("Mode1 Register:");
        const byte mode1Reg = readRegister(PCA9685_MODE1_REG);
        Serial.print("0x"); Serial.print(mode1Reg, HEX);
        Serial.print(", Bitset:");
        if (mode1Reg & PCA9685_MODE1_RESTART)
            Serial.print(" PCA9685_MODE1_RESTART");
        if (mode1Reg & PCA9685_MODE1_EXTCLK)
            Serial.print(" PCA9685_MODE1_EXTCLK");
        if (mode1Reg & PCA9685_MODE1_AUTOINC)
            Serial.print(" PCA9685_MODE1_AUTOINC");
        if (mode1Reg & PCA9685_MODE1_SLEEP)
            Serial.print(" PCA9685_MODE1_SLEEP");
        if (mode1Reg & PCA9685_MODE1_SUBADR1)
            Serial.print(" PCA9685_MODE1_SUBADR1");
        if (mode1Reg & PCA9685_MODE1_SUBADR2)
            Serial.print(" PCA9685_MODE1_SUBADR2");
        if (mode1Reg & PCA9685_MODE1_SUBADR3)
            Serial.print(" PCA9685_MODE1_SUBADR3");
        if (mode1Reg & PCA9685_MODE1_ALLCALL)
            Serial.print(" PCA9685_MODE1_ALLCALL");
        Serial.println("");

        Serial.println(""); Serial.println("Mode2 Register:");
        const byte mode2Reg = readRegister(PCA9685_MODE2_REG);
        Serial.print("0x"); Serial.print(mode2Reg, HEX);
        Serial.print(", Bitset:");
        if (mode2Reg & PCA9685_MODE2_OUTDRV_TPOLE)
            Serial.print(" PCA9685_MODE2_OUTDRV_TPOLE");
        if (mode2Reg & PCA9685_MODE2_INVRT)
            Serial.print(" PCA9685_MODE2_INVRT");
        if (mode2Reg & PCA9685_MODE2_OUTNE_TPHIGH)
            Serial.print(" PCA9685_MODE2_OUTNE_TPHIGH");
        if (mode2Reg & PCA9685_MODE2_OUTNE_HIGHZ)
            Serial.print(" PCA9685_MODE2_OUTNE_HIGHZ");
        if (mode2Reg & PCA9685_MODE2_OCH_ONACK)
            Serial.print(" PCA9685_MODE2_OCH_ONACK");
        Serial.println("");

        Serial.println(""); Serial.println("SubAddress1 Register:");
        const byte subAdr1Reg = readRegister(PCA9685_SUBADR1_REG);
        Serial.print("0x"); Serial.println(subAdr1Reg, HEX);

        Serial.println(""); Serial.println("SubAddress2 Register:");
        const byte subAdr2Reg = readRegister(PCA9685_SUBADR2_REG);
        Serial.print("0x"); Serial.println(subAdr2Reg, HEX);

        Serial.println(""); Serial.println("SubAddress3 Register:");
        const byte subAdr3Reg = readRegister(PCA9685_SUBADR3_REG);
        Serial.print("0x"); Serial.println(subAdr3Reg, HEX);

        Serial.println(""); Serial.println("AllCall Register:");
        const byte allCallReg = readRegister(PCA9685_ALLCALL_REG);
        Serial.print("0x"); Serial.println(allCallReg, HEX);
    } else {
        Serial.println(""); Serial.println("Proxy Addresser: true");
    }
}

static const char *textForI2CError(byte errorCode) {
    switch (errorCode) {
    case 0:
        return "Success";
    case 1:
        return "Data too long to fit in transmit buffer";
    case 2:
        return "Received NACK on transmit of address";
    case 3:
        return "Received NACK on transmit of data";
    default:
        return "Other error";
    }
}

void PCA9685::checkForErrors() {
    if (_lastI2CError) {
        Serial.print("  PCA9685::checkErrors lastI2CError: ");
        Serial.print(_lastI2CError);
        Serial.print(": ");
        Serial.println(textForI2CError(getLastI2CError()));
    }
}

#endif // /ifdef PCA9685_ENABLE_DEBUG_OUTPUT

PCA9685_ServoEval::PCA9685_ServoEval(uint16_t minPWMAmount, uint16_t maxPWMAmount)
    : _coeff(NULL), _isCSpline(false)
{
    minPWMAmount = min(max(minPWMAmount, (uint16_t)0), PCA9685_PWM_FULL);
    maxPWMAmount = min(max(maxPWMAmount, minPWMAmount), PCA9685_PWM_FULL);

    _coeff = new float[2];
    _isCSpline = false;

    _coeff[0] = minPWMAmount;
    _coeff[1] = (maxPWMAmount - minPWMAmount) / 180.0f;
}

PCA9685_ServoEval::PCA9685_ServoEval(uint16_t minPWMAmount, uint16_t midPWMAmount, uint16_t maxPWMAmount)
    : _coeff(NULL), _isCSpline(false)
{
    minPWMAmount = min(max(minPWMAmount, (uint16_t)0), PCA9685_PWM_FULL);
    midPWMAmount = min(max(midPWMAmount, minPWMAmount), PCA9685_PWM_FULL);
    maxPWMAmount = min(max(maxPWMAmount, midPWMAmount), PCA9685_PWM_FULL);

    if (maxPWMAmount - midPWMAmount != midPWMAmount - minPWMAmount) {
        _coeff = new float[8];
        _isCSpline = true;

        // Cubic spline code adapted from: https://shiftedbits.org/2011/01/30/cubic-spline-interpolation/
        /* "THE BEER-WARE LICENSE" (Revision 42): Devin Lane wrote this [part]. As long as you retain
        * this notice you can do whatever you want with this stuff. If we meet some day, and you
        * think this stuff is worth it, you can buy me a beer in return. */
       // TODO: Looks like I owe Devin Lane a beer. -NR

        float x[3] = { 0, 90, 180 };
        float y[3] = { (float)minPWMAmount, (float)midPWMAmount, (float)maxPWMAmount };
        float c[3], b[2], d[2], h[2], l[1], u[2], a[1], z[2]; // n = 3

        h[0] = x[1] - x[0];
        u[0] = z[0] = 0;
        c[2] = 0;

        for (int i = 1; i < 2; ++i) {
            h[i] = x[i + 1] - x[i];
            l[i - 1] = (2 * (x[i + 1] - x[i - 1])) - h[i - 1] * u[i - 1];
            u[i] = h[i] / l[i - 1];
            a[i - 1] = (3 / h[i]) * (y[i + 1] - y[i]) - (3 / h[i - 1]) * (y[i] - y[i - 1]);
            z[i] = (a[i - 1] - h[i - 1] * z[i - 1]) / l[i - 1];
        }

        for (int i = 1; i >= 0; --i) {
            c[i] = z[i] - u[i] * c[i + 1];
            b[i] = (y[i + 1] - y[i]) / h[i] - (h[i] * (c[i + 1] + 2 * c[i])) / 3;
            d[i] = (c[i + 1] - c[i]) / (3 * h[i]);

            _coeff[4 * i + 0] = y[i]; // a
            _coeff[4 * i + 1] = b[i]; // b
            _coeff[4 * i + 2] = c[i]; // c
            _coeff[4 * i + 3] = d[i]; // d
        }
    }
    else {
        _coeff = new float[2];
        _isCSpline = false;

        _coeff[0] = minPWMAmount;
        _coeff[1] = (maxPWMAmount - minPWMAmount) / 180.0f;
    }
}

PCA9685_ServoEval::~PCA9685_ServoEval() {
    if (_coeff) { delete[] _coeff; _coeff = NULL; }
}

uint16_t PCA9685_ServoEval::pwmForAngle(float angle) {
    float retVal;
    angle = constrain(angle + 90, 0, 180);

    if (!_isCSpline) {
        retVal = _coeff[0] + (_coeff[1] * angle);
    }
    else {
        if (angle <= 90) {
            retVal = _coeff[0] + (_coeff[1] * angle) + (_coeff[2] * angle * angle) + (_coeff[3] * angle * angle * angle);
        }
        else {
            angle -= 90;
            retVal = _coeff[4] + (_coeff[5] * angle) + (_coeff[6] * angle * angle) + (_coeff[7] * angle * angle * angle);
        }
    }

    return (uint16_t)min(max((uint16_t)roundf(retVal), (uint16_t)0), PCA9685_PWM_FULL);
};

uint16_t PCA9685_ServoEval::pwmForSpeed(float speed) {
    return pwmForAngle(speed * 90.0f);
}
