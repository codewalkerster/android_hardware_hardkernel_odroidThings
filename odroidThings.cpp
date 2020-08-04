/*
 *    Copyright (c) 2019 Sangchul Go <luke.go@hardkernel.com>
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

#include <hardware/hardware.h>
#include <hardware/odroidThings.h>

#include <cutils/log.h>
#include <utils/Mutex.h>

#include <string>
#include <vector>
#include <memory>
#include "PinManager.h"
#include "Uart.h"

using hardware::hardkernel::odroidthings::things_device_t;
using hardware::hardkernel::odroidthings::things_module_t;

static android::Mutex thingsLock;
static std::unique_ptr<PinManager> gPinManager;
static std::unique_ptr<Uart> gUart;
static std::unique_ptr<Spi> gSpi;

static void things_init() {
    gPinManager = std::make_unique<PinManager>();
    if (gPinManager->init() < 0) {
        ALOGE("Odroid thing is not supported");
        return;
    }

    gUart = gPinManager->getUart();
    gSpi = gPinManager->getSpi();
}

static const std::vector<pin_t> things_getPinList() {
    return gPinManager->getPinList();
}

static const std::vector<std::string> things_getPinNameList() {
    return gPinManager->getPinNameList();
}

static const std::vector<std::string> things_getListOf(int mode) {
    std::vector<std::string> emptyList;
    switch (mode) {
        case PIN_UART:
            if(gUart)
                return gUart->getList();
            break;
        case PIN_SPI:
            if(gSpi)
                return gSpi->getList();
            break;
    }
    return gPinManager->getListOf(mode);
}

static bool things_getValue(int pin) {
    return gPinManager->getValue(pin);
}

static void things_setDirection(int pin, direction_t direction) {
    return gPinManager->setDirection(pin, direction);
}

static void things_setValue(int pin, bool value) {
    return gPinManager->setValue(pin, value);
}

static void things_setActiveType(int pin, int activeType) {
    gPinManager->setActiveType(pin, activeType);
}

static void things_setEdgeTriggerType(int pin, int edgeTriggerType) {
    gPinManager->setEdgeTriggerType(pin, edgeTriggerType);
}

static void things_registerCallback(int pin, function_t callback) {
    gPinManager->registerCallback(pin, callback);
}

static void things_unregisterCallback(int pin) {
    gPinManager->unregisterCallback(pin);
}

static int things_close(struct hw_device_t *dev) {
    android::Mutex::Autolock lock(thingsLock);

    things_device_t *device = (things_device_t*)dev;
    if (device) {
        free(device);
        device = NULL;
    }
    return 0;
}

static void things_pwm_open(int pin) {
    gPinManager->openPwm(pin);
}

static void things_pwm_close(int pin) {
    gPinManager->closePwm(pin);
}

static bool things_pwm_setEnable(int pin, bool enabled) {
    return gPinManager->setPwmEnable(pin, enabled);
}

static bool things_pwm_setDutyCycle(int pin, double cycle_rate) {
    return gPinManager->setPwmDutyCycle(pin, cycle_rate);
}

static bool things_pwm_setFrequency(int pin, double frequency_hz) {
    return gPinManager->setPwmFrequency(pin, frequency_hz);
}

static void things_i2c_open(int nameIdx, uint32_t address, int idx) {
    gPinManager->openI2c(nameIdx, address, idx);
}

static void things_i2c_close(int idx) {
    gPinManager->closeI2c(idx);
}

static const std::vector<uint8_t> things_i2c_readRegBuffer(int idx, uint32_t reg, int length) {
    return gPinManager->readRegBufferI2c(idx, reg, length);
}

static Result things_i2c_writeRegBuffer(int idx, uint32_t reg, std::vector<uint8_t> buffer, int length) {
    return gPinManager->writeRegBufferI2c(idx, reg, buffer, length);
}

static void things_uart_open(int idx) {
    gUart->open(idx);
}

static void things_uart_close(int idx) {
    gUart->close(idx);
}

static bool things_uart_flush(int idx, int direction) {
    return gUart->flush(idx, direction);
}

static bool things_uart_sendBreak(int idx, int duration) {
    return gUart->sendBreak(idx, duration);
}

static bool things_uart_setBaudrate(int idx, int baudrate) {
    return gUart->setBaudrate(idx, baudrate);
}

static bool things_uart_setDataSize(int idx, int size) {
    return gUart->setDataSize(idx, size);
}

static bool things_uart_setHardwareFlowControl(int idx, int mode) {
    return gUart->setHardwareFlowControl(idx, mode);
}

static bool things_uart_setParity(int idx, int mode) {
    return gUart->setParity(idx, mode);
}

static bool things_uart_setStopBits(int idx, int bits) {
    return gUart->setStopBits(idx, bits);
}

static const std::vector<uint8_t> things_uart_read(int idx, int length) {
    return gUart->read(idx, length);
}

static ssize_t things_uart_write(int idx, std::vector<uint8_t> buffer, int length) {
    return gUart->write(idx, buffer, length);
}

static void things_uart_registerCallback(int idx, function_t callback) {
    //gUart->registerCallback(idx, callback);
}

static void things_uart_unregisterCallback(int idx) {
    //gUart->unregisterCallback(idx);
}

static void things_spi_open(int idx) {
    gSpi->open(idx);
}

static void things_spi_close(int idx) {
    gSpi->close(idx);
}

static bool things_spi_setBitJustification(int idx, uint8_t bit) {
    int ret = gSpi->setBitJustification(idx, bit);
    if (ret < 0) {
        ALOGE("Failed to set bit justification(%d)", ret);
        return false;
    }
    return true;
}

static bool things_spi_setBitsPerWord(int idx, uint8_t bit) {
    int ret = gSpi->setBitsPerWord(idx, bit);

    if (ret < 0) {
        ALOGE("Failed to set bit per word(%d)", ret);
        return false;
    }
    return true;
}

static bool things_spi_setMode(int idx, uint8_t mode) {
    int ret = gSpi->setMode(idx, mode);

    if (ret < 0) {
        ALOGE("Failed to set mode(%d)", ret);
        return false;
    }
    return true;
}

static bool things_spi_setCsChange(int idx, bool cs) {
    int ret = gSpi->setCsChange(idx, cs);

    if (ret < 0) {
        ALOGE("Failed to change cs(%d)", ret);
        return false;
    }
    return true;
}

static bool things_spi_setDelay(int idx, uint16_t delay) {
    int ret = gSpi->setDelay(idx, delay);

    if (ret < 0) {
        ALOGE("Failed to set delay");
        return false;
    }
    return true;
}

static bool things_spi_setFrequency(int idx, uint32_t frequency) {
    int ret = gSpi->setFrequency(idx, frequency);

    if (ret < 0) {
        ALOGE("Failed to set frequency(%x)", ret);
        return false;
    }
    return true;
}

static const std::vector<uint8_t> things_spi_transfer(int idx, std::vector<uint8_t> txBuffer, int length) {
    uint8_t * txArray = &txBuffer[0];
    std::vector<uint8_t> rxBuffer(length);
    uint8_t * rxArray = &rxBuffer[0];

    int ret = gSpi->transfer(idx, txArray, rxArray, length);

    if (ret < 0) {
        ALOGE("transfer is failed(%d)", ret);
        return rxBuffer;
    }

    return rxBuffer;
}

static bool things_spi_write(int idx, std::vector<uint8_t> txBuffer, int length) {

    uint8_t *txArray = &txBuffer[0];

    int ret = gSpi->write(idx, txArray, length);

    if (ret < 0) {
        ALOGE("write is failed(%d)", ret);
        return false;
    }

    return true;
}

static const std::vector<uint8_t> things_spi_read(int idx, int length) {
    std::vector<uint8_t> rxBuffer(length);

    int ret = gSpi->read(idx, &rxBuffer[0], length);

    if (ret < 0) {
        ALOGE("read spi is failed(%d)", ret);
        return rxBuffer;
    }

    return rxBuffer;
}

static int things_open(const hw_module_t *module, const char __unused *id,
        struct hw_device_t **device) {
    android::Mutex::Autolock lock(thingsLock);

    things_device_t *dev = (things_device_t*)malloc(sizeof(things_device_t));

    ALOGD("things HAL open");
    if (!dev) {
        ALOGE("no memory for the odroid things device");
        return -ENOMEM;
    }

    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = const_cast<hw_module_t*>(module);
    dev->common.close = things_close;

    dev->common_ops.getPinNameList = things_getPinNameList;
    dev->common_ops.getListOf = things_getListOf;

    dev->gpio_ops.getValue = things_getValue;
    dev->gpio_ops.setDirection = things_setDirection;
    dev->gpio_ops.setValue = things_setValue;
    dev->gpio_ops.setActiveType = things_setActiveType;
    dev->gpio_ops.setEdgeTriggerType = things_setEdgeTriggerType;
    dev->gpio_ops.registerCallback = things_registerCallback;
    dev->gpio_ops.unregisterCallback = things_unregisterCallback;

    dev->pwm_ops.open = things_pwm_open;
    dev->pwm_ops.close = things_pwm_close;
    dev->pwm_ops.setEnable = things_pwm_setEnable;
    dev->pwm_ops.setDutyCycle = things_pwm_setDutyCycle;
    dev->pwm_ops.setFrequency = things_pwm_setFrequency;

    dev->i2c_ops.open = things_i2c_open;
    dev->i2c_ops.close = things_i2c_close;
    dev->i2c_ops.readRegBuffer = things_i2c_readRegBuffer;
    dev->i2c_ops.writeRegBuffer = things_i2c_writeRegBuffer;

    // Modem Control feature will not be supported.
    dev->uart_ops.open = things_uart_open;
    dev->uart_ops.close = things_uart_close;
    dev->uart_ops.flush = things_uart_flush;
    dev->uart_ops.sendBreak = things_uart_sendBreak;
    dev->uart_ops.setBaudrate = things_uart_setBaudrate;
    dev->uart_ops.setDataSize = things_uart_setDataSize;
    dev->uart_ops.setHardwareFlowControl = things_uart_setHardwareFlowControl;
    dev->uart_ops.setParity = things_uart_setParity;
    dev->uart_ops.setStopBits = things_uart_setStopBits;
    dev->uart_ops.read = things_uart_read;
    dev->uart_ops.write = things_uart_write;

    // The callback feature is not supported yet.
    dev->uart_ops.registerCallback = things_uart_registerCallback;
    dev->uart_ops.unregisterCallback = things_uart_unregisterCallback;

    dev->spi_ops.open = things_spi_open;
    dev->spi_ops.close = things_spi_close;
    dev->spi_ops.setBitJustification = things_spi_setBitJustification;
    dev->spi_ops.setBitsPerWord = things_spi_setBitsPerWord;
    dev->spi_ops.setMode = things_spi_setMode;
    dev->spi_ops.setCsChange = things_spi_setCsChange;
    dev->spi_ops.setDelay = things_spi_setDelay;
    dev->spi_ops.setFrequency = things_spi_setFrequency;
    dev->spi_ops.transfer = things_spi_transfer;
    dev->spi_ops.write = things_spi_write;
    dev->spi_ops.read = things_spi_read;

    *device = &dev->common;

    return 0;
}

static struct hw_module_methods_t things_module_methods = {
    .open = things_open,
};

things_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = ODROID_THINGS_MODULE_API_VERSION_1_0,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = ODROID_THINGS_HARDWARE_MODULE_ID,
        .name = " Odroid things module",
        .author = "Hardkernel",
        .methods = &things_module_methods,
        .reserved ={0},
    },
    .init = things_init,
    .getPinList = things_getPinList,
};
