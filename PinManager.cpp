/*
 *    Copyright (c) 2020 Sangchul Go <luke.go@hardkernel.com>
 *
 *    OdroidThings is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    OdroidThings is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with OdroidThings.
 *    If not, see <http://www.gnu.org/licenses/>.
 */

#include <wiringPi/wiringPi.h>
#include <cutils/properties.h>
#include <hardware/odroidThings.h>
#include <hardware/odroidthings-base.h>

#include <sstream>
#include <vector>
#include <map>

#include <cutils/log.h>

#include "PinManager.h"

// pin array number is based on physical pin number
static const pin_t n2_pin_support_list[PIN_MAX] = {
    {"", -1, 0},
    {"3.3V", -1, PIN_PWR}, {"5V", -1, PIN_PWR},
    {"3", 8, PIN_I2C}, {"5V", -1, PIN_PWR},
    {"5", 9, PIN_I2C}, {"GND", -1, PIN_GND},
    {"7", 7, PIN_GPIO}, {"8", 15, PIN_UART},
    {"GND", -1, PIN_GND}, {"10", 16, PIN_UART},
    {"11", 0, PIN_GPIO}, {"12", 1, PIN_GPIO|PIN_PWM},
    {"13", 2, PIN_GPIO}, {"GND", -1, PIN_GND},
    {"15", 3, PIN_GPIO|PIN_PWM}, {"16", 4, PIN_GPIO},
    {"3.3V", -1, PIN_PWR}, {"18", 5, PIN_GPIO},
    {"19", 12, PIN_GPIO}, {"GND", -1, PIN_GND},
    {"21", 13, PIN_GPIO}, {"22", 6, PIN_GPIO},
    {"23", 14, PIN_GPIO}, {"24", 10, PIN_GPIO},
    {"25", -1, PIN_GND}, {"26", 11, PIN_GPIO},
    {"27", 30, PIN_I2C}, {"28", 31, PIN_I2C},
    {"29", 21, PIN_GPIO}, {"GND", -1, PIN_GND},
    {"31", 22, PIN_GPIO}, {"32", 26, PIN_GPIO},
    {"33", 23, PIN_GPIO|PIN_PWM}, {"GND", -1, PIN_GND},
    {"35", 24, PIN_GPIO|PIN_PWM}, {"36", 27, PIN_GPIO},
    {"AIN0", 25, PIN_AIN}, {"1.8V", 28, PIN_PWR},
    {"GND", -1, PIN_GND}, {"AIN1", 29, PIN_AIN},
};

static const pin_t c4_pin_support_list[PIN_MAX] = {
    {"", -1, 0},
    {"3.3V", -1, PIN_PWR}, {"5V", -1, PIN_PWR},
    {"3", 8, PIN_I2C}, {"5V", -1, PIN_PWR},
    {"5", 9, PIN_I2C}, {"GND", -1, PIN_GND},
    {"7", 7, PIN_GPIO}, {"8", 15, PIN_UART},
    {"GND", -1, PIN_GND}, {"10", 16, PIN_UART},
    {"11", 0, PIN_GPIO}, {"12", 1, PIN_GPIO|PIN_PWM},
    {"13", 2, PIN_GPIO}, {"GND", -1, PIN_GND},
    {"15", 3, PIN_GPIO|PIN_PWM}, {"16", 4, PIN_GPIO},
    {"3.3V", -1, PIN_PWR}, {"18", 5, PIN_GPIO},
    {"19", 12, PIN_GPIO}, {"GND", -1, PIN_GND},
    {"21", 13, PIN_GPIO}, {"22", 6, PIN_GPIO},
    {"23", 14, PIN_GPIO}, {"24", 10, PIN_GPIO},
    {"25", -1, PIN_GND}, {"26", 11, PIN_GPIO},
    {"27", 30, PIN_I2C}, {"28", 31, PIN_I2C},
    {"29", 21, PIN_GPIO}, {"GND", -1, PIN_GND},
    {"31", 22, PIN_GPIO}, {"32", 26, PIN_GPIO},
    {"33", 23, PIN_GPIO|PIN_PWM}, {"GND", -1, PIN_GND},
    {"35", 24, PIN_GPIO|PIN_PWM}, {"36", 27, PIN_GPIO},
    {"AIN3", 25, PIN_AIN}, {"1.8V", 28, PIN_PWR},
    {"GND", -1, PIN_GND}, {"AIN4", 29, PIN_AIN},
};

static const i2c_t n2_i2c_support_list[I2C_MAX] = {
    {"I2C-2", "/dev/i2c-2"},
    {"I2C-3", "/dev/i2c-3"},
};

static const i2c_t c4_i2c_support_list[I2C_MAX] = {
    {"I2C-2", "/dev/i2c-2"},
    {"I2C-3", "/dev/i2c-3"},
};

static const pwm_t n2_pwm_support_list[PWM_MAX] = {
    {1, 8, 0}, // Pin #12
    {3, 8 ,1}, // Pin #15
    {23, 4, 0}, // Pin #33
    {24, 4, 1}, // Pin #35
};

static const pwm_t c4_pwm_support_list[PWM_MAX] = {
    {1, 4, 0}, // Pin #12
    {3, 4 ,1}, // Pin #15
    {23, 0, 0}, // Pin #33
    {24, 0, 1}, // Pin #35
};

static const uart_t n2_uart_support_list[UART_MAX] = {
    {"UART-1", "/dev/ttyS1"}, // #8, #10
};

static const uart_t c4_uart_support_list[UART_MAX] = {
    {"UART-1", "/dev/ttyS1"}, // #8, #10
};

PinManager::PinManager(){
    char boardName[PROPERTY_VALUE_MAX];

    property_get(BOARD_PROPERTY, boardName, NULL);
    board = boardName;

    ALOGD("Board is %s", board.c_str());
}

void PinManager::init() {
    if (board == "odroidn2") {
        pinList = (pin_t*)n2_pin_support_list;
        i2cList = (i2c_t*)n2_i2c_support_list;
        pwmList = (pwm_t*)n2_pwm_support_list;
        uartList = (uart_t*)n2_uart_support_list;
    } else if (board == "odroidc4") {
        pinList = (pin_t*)c4_pin_support_list;
        i2cList = (i2c_t*)c4_i2c_support_list;
        pwmList = (pwm_t*)c4_pwm_support_list;
        uartList = (uart_t*)c4_uart_support_list;
    } else {
        ALOGD("Board is not initialized");
        return;
    }

    wiringPiSetup();
    initPwm();
}

std::vector<pin_t> PinManager::getPinList() {
    std::vector<pin_t> list;
    if (!pinList) {
        ALOGD("Board is not initialized");
        return list;
    }

    for (int i=0; i<PIN_MAX; i++)
        list.push_back(pinList[i]);
    return list;
}

std::vector<std::string> PinManager::getPinNameList() {
    std::vector<std::string> list;
    if (!pinList) {
        ALOGD("Board is not initialized");
        return list;
    }

    for (int i=0; i<PIN_MAX; i++)
        list.push_back(pinList[i].name);
    return list;
}

std::vector<std::string> PinManager::getListOf(int mode) {
    std::vector<std::string> list;
    if (!pinList) {
        ALOGD("Board is not initialized");
        return list;
    }

    switch (mode) {
        case PIN_GPIO:
            for (int i=0; i<PIN_MAX; i++) {
                if (pinList[i].availableModes & PIN_GPIO) {
                    int alt = getAlt(pinList[i].pin);
                    if (alt < 2)
                        list.push_back(pinList[i].name);
                }
            }
            break;
        case PIN_PWM:
            for (int i=0; i<PIN_MAX; i++) {
                if (pinList[i].availableModes & PIN_PWM) {
                    int alt = getAlt(pinList[i].pin);

                    if (alt < 2) {
                        list.push_back(pinList[i].name);
                    }
                    if ((pinList[i].pin < 4) && (alt == 2))
                        list.push_back(pinList[i].name);
                    else if ((pinList[i].pin > 22) && (alt == 5))
                        list.push_back(pinList[i].name);
                }
            }
            break;
        case PIN_I2C:
            for (int i=0; i<I2C_MAX; i++)
                list.push_back(i2cList[i].name);
            break;
    }
    return list;
}

bool PinManager::getValue(int idx) {
    return (digitalRead(pinList[idx].pin) == HIGH);
}

void PinManager::setDirection(int idx, direction_t direction) {
    int pin = pinList[idx].pin;
    switch (direction) {
        case DIRECTION_IN:
            pinMode(pin, INPUT);
            break;
        case DIRECTION_OUT_INITIALLY_HIGH:
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
            break;
        case DIRECTION_OUT_INITIALLY_LOW:
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
            break;
    }
}

void PinManager::setValue(int idx, bool value) {
    digitalWrite(pinList[idx].pin, value?HIGH:LOW);
}

void PinManager::setActiveType(int idx, int activeType) {
    int pin = pinList[idx].pin;
    switch (activeType) {
        case ACTIVE_LOW:
            pullUpDnControl(pin, PUD_UP);
            break;
        case ACTIVE_HIGH:
            pullUpDnControl(pin, PUD_DOWN);
            break;
    }
}

void PinManager::setEdgeTriggerType(int idx, int edgeTriggerType) {
    int pin = pinList[idx].pin;
    switch (edgeTriggerType) {
        case EDGE_NONE:
            return;
            break;
        case EDGE_RISING:
            triggerType[pin] = INT_EDGE_RISING;
            break;
        case EDGE_FALLING:
            triggerType[pin] = INT_EDGE_FALLING;
            break;
        case EDGE_BOTH:
            triggerType[pin] = INT_EDGE_BOTH;
            break;
    }
}

void PinManager::registerCallback(int idx, function_t callback) {
    int pin = pinList[idx].pin;
    wiringPiISR(pin, triggerType[pin], callback);
}

void PinManager::unregisterCallback(int idx) {
    wiringPiISRCancel(pinList[idx].pin);
}

#define PWM_RANGE_MAX 65535

//TODO: reduce pwm array size to fit the pwm number.

void PinManager::initPwm() {
    for (int i=0; i<PWM_MAX; i++) {
        auto pin = pwmList[i];
        initPwmState(pin.index, pin.chip, pin.line);
    }
}

void PinManager::initPwmState(int idx, uint8_t chip, uint8_t node) {
    const auto state = new pwmState();
    // init chip & node info
    state->chip = chip;
    state->node = node;

    // init configuration sysfs path.
    std::ostringstream pwmRoot;

    pwmRoot << "/sys/class/pwm/pwmchip" << std::to_string(state->chip)
        << "/pwm" << std::to_string(state->node) <<"/";
	std::string pwmRootStr = pwmRoot.str();

    state->periodPath =  pwmRootStr + "period";
    state->dutyCyclePath = pwmRootStr + "duty_cycle";
    state->enablePath = pwmRootStr + "enable";

    pwm.insert(std::make_pair(idx, state));
}

inline void PinManager::writeSysfsTo(const std::string path, const std::string value) {
    std::ofstream file(path);
    file << value;
    file.close();
}

void PinManager::openPwm(int idx) {
    auto pin = pinList[idx].pin;
    const auto state = pwm.find(pin)->second;
    std::ostringstream openPath;

    ALOGD("openPwm(chip:%d-node:%d)", state->chip, state->node);

    openPath << "/sys/class/pwm/pwmchip" << std::to_string(state->chip) << "/";

    writeSysfsTo(openPath.str() + "export", std::to_string(state->node));

    state->unexportPath = openPath.str() + "unexport";
    openPath << "pwm" << std::to_string(state->node) << "/";
    writeSysfsTo(openPath.str() + "polarity", "normal");
}

void PinManager::closePwm(int idx) {
    auto pin = pinList[idx].pin;
    const auto state = pwm.find(pin)->second;
    state->period = 0;
    state->cycle_rate = 0;

    ALOGD("closePwm(chip:%d-node:%d)", state->chip, state->node);

    writeSysfsTo(state->enablePath, "0");
    writeSysfsTo(state->dutyCyclePath, "0");
    writeSysfsTo(state->periodPath, "0");
    writeSysfsTo(state->unexportPath, std::to_string(state->node));
}

bool PinManager::setPwmEnable(int idx, bool enabled) {
    auto pin = pinList[idx].pin;
    const auto state = pwm.find(pin)->second;

    writeSysfsTo(state->enablePath, (enabled?"1":"0"));
    return true;
}

bool PinManager::setPwmDutyCycle(int idx, double cycle_rate) {
    auto pin = pinList[idx].pin;
    const auto state = pwm.find(pin)->second;
    unsigned int duty_cycle = (state->period / 100) * cycle_rate;

    writeSysfsTo(state->dutyCyclePath, std::to_string(duty_cycle));

    state->cycle_rate = cycle_rate;

    return true;
}

#define HERTZTONANOSECOND 1000000000
bool PinManager::setPwmFrequency(int idx, double frequency_hz) {
    auto pin = pinList[idx].pin;
    const auto state = pwm.find(pin)->second;

    state->period = HERTZTONANOSECOND / frequency_hz;

    writeSysfsTo(state->periodPath, std::to_string(state->period));

    if (state->cycle_rate != 0) {
        setPwmDutyCycle(idx, state->cycle_rate);
    }

    return true;
}

#include <unistd.h>
#include <linux/i2c-dev.h>

void PinManager::openI2c(int nameIdx, uint32_t address, int idx) {
    int i2cFd = open(i2cList[nameIdx].path.c_str(), O_RDWR);
    if (i2cFd < 0) {
        ALOGD("oepn i2c is failed!");
        return;
    }
    if (ioctl(i2cFd, I2C_SLAVE, address) < 0) {
        ALOGD("Failed to acquire bus access and/or talk to salve.\n");
        return;
    }
    i2c.insert(std::make_pair(idx, i2cFd));
}

void PinManager::closeI2c(int idx) {
    const auto i2cfd = i2c.find(idx)->second;
    close(i2cfd);
    i2c.erase(idx);
}

std::vector<uint8_t> PinManager::readRegBufferI2c(int idx, uint32_t reg, int length) {
    const auto i2cfd = i2c.find(idx)->second;

    uint8_t *buffer = new uint8_t[length];
    write(i2cfd, &reg, 1);
    read(i2cfd, buffer, length);
    std::vector<uint8_t> result(buffer, buffer + length);
    delete[] buffer;

    return result;
}

Result PinManager::writeRegBufferI2c(int idx, uint32_t reg, std::vector<uint8_t> buffer, int length) {
    const auto i2cfd = i2c.find(idx)->second;
    uint8_t *msg = new uint8_t[length+1];

    msg[0] = reg;
    std::copy(buffer.begin(), buffer.end(), msg+1);

    write(i2cfd, msg, length+1);
    delete[] msg;

    return Result::OK;
}

std::unique_ptr<Uart> PinManager::getUart() {
    auto uart = std::make_unique<Uart>(uartList);
    return uart;
}
