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

#include "OdroidC4.h"
#include "OdroidM1.h"
#include "OdroidM1S.h"
#include "OdroidN2.h"
#include "PinManager.h"
#include <cutils/log.h>
#include <cutils/properties.h>
#include <hardware/odroidthings-base.h>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <set>

PinManager::PinManager(){
    char boardName[PROPERTY_VALUE_MAX];
    std::string name;
    isUnknown= false;

    property_get(BOARD_PROPERTY, boardName, NULL);
    name = boardName;

    if (!name.compare(0, strlen("odroidn2"), "odroidn2"))
        board = std::make_shared<OdroidN2>();
    else if (!name.compare(0, strlen("odroidc4"), "odroidc4"))
        board = std::make_shared<OdroidC4>();
    else if (!name.compare(0, strlen("odroidm1s"), "odroidm1s"))
        board = std::make_shared<OdroidM1S>();
    else if (!name.compare(0, strlen("odroidm1"), "odroidm1"))
        board = std::make_shared<OdroidM1>();
    else {
        board = std::make_shared<Board>(name);
        isUnknown= true;
        ALOGD("Board is not initialized");
        return;
    }

    ALOGD("Board is %s", name.c_str());
}

int PinManager::init() {
    if (isUnknownBoard())
        return -1;

    if (wiringPiSetup()) {
        ALOGD("Board is not initialized");
        return -1;
    }

    // This init sequence order must be fixed.
    initI2cList();
    initPwmList();
    initUartList();
    initSpiList();
    initGpioList();

    return 0;
}

bool PinManager::isUnknownBoard() {
    if (isUnknown)
        return true;

    std::ifstream modelName ("/proc/device-tree/model");
    if (modelName.fail())
        return true;

    std::string name;
    while(!modelName.eof()) {
        modelName >> name;
        if (name.find("ODROID") != std::string::npos)
            break;
    }
    modelName.close();

    std::size_t pos = name.find("-");
    name = name.substr(pos + 1);
    name.pop_back();

    for(const auto& model: models)
        if (model.find(name) != std::string::npos)
            return false;

    return true;
}

std::vector<pin_t> PinManager::getPinList() {
    return board->getPinList();
}

std::vector<std::string> PinManager::getPinNameList() {
    return board->getPinNameList();
}

void PinManager::initI2cList() {
    auto i2cAllList = board->getI2cList();
    for (auto i2c = i2cAllList.begin(); i2c != i2cAllList.end(); i2c++) {
        if (access(i2c->path.c_str(), F_OK) == 0)
            i2cList.push_back(*i2c);
    }
}

void PinManager::initPwmList() {
    auto list = board->getPwmList();
    for (auto pin = list.begin(); pin != list.end(); pin++) {
        if (access(pin->path.c_str(), F_OK) == 0)
            pwmList.push_back(*pin);
    }
}

void PinManager::initUartList() {
    auto uartAllList = board->getUartList();
    for (auto uart = uartAllList.begin(); uart != uartAllList.end(); uart++) {
        if (access(uart->path.c_str(), F_OK) == 0)
            uartList.push_back(*uart);
    }
}

void PinManager::initSpiList() {
    auto spiAllList = board->getSpiList();
    for (auto spi = spiAllList.begin(); spi != spiAllList.end(); spi++) {
        if (access(spi->path.c_str(), F_OK) == 0) {
            spiList.push_back(*spi);
        }
    }
}

void PinManager::initGpioList() {
    std::set<int> usedPinList;

    for (auto i2c = i2cList.begin(); i2c != i2cList.end(); i2c++) {
        usedPinList.insert(i2c->sda);
        usedPinList.insert(i2c->scl);
    }

    for (auto pwm = pwmList.begin(); pwm != pwmList.end(); pwm++) {
        usedPinList.insert(pwm->index);
    }

    for (auto uart = uartList.begin(); uart != uartList.end(); uart++) {
        usedPinList.insert(uart->rx);
        usedPinList.insert(uart->tx);
    }

    for (auto spi = spiList.begin(); spi != spiList.end(); spi++) {
        usedPinList.insert(spi->sclk);
        usedPinList.insert(spi->mosi);
        usedPinList.insert(spi->miso);
        usedPinList.insert(spi->cs);
    }

    auto list = board->getPinList();

    for (auto pin = list.begin(); pin != list.end(); pin++) {
        auto search = usedPinList.find(pin->pin);
        if (search == usedPinList.end())
            gpioList.push_back(*pin);
    }
}

std::unique_ptr<Gpio> PinManager::getGpio() {
    auto gpio = std::make_unique<Gpio>(gpioList);
    return gpio;
}

std::unique_ptr<Pwm> PinManager::getPwm() {
    auto pwm = std::make_unique<Pwm>(board);
    return pwm;
}

std::unique_ptr<I2c> PinManager::getI2c() {
    if (i2cList.size()) {
        auto i2c = std::make_unique<I2c>(i2cList);
        return i2c;
    } else
        return NULL;
}

std::unique_ptr<Uart> PinManager::getUart() {
    if (uartList.size()) {
        auto uart = std::make_unique<Uart>(uartList);
        return uart;
    } else
        return NULL;
}

std::unique_ptr<Spi> PinManager::getSpi() {
    if (spiList.size()) {
        std::unique_ptr<Spi> spi = std::make_unique<Spi>(spiList);
        return spi;
    }
    return NULL;
}
