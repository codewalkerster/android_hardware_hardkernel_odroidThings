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
#include "OdroidN2.h"
#include "PinManager.h"
#include <cutils/log.h>
#include <cutils/properties.h>
#include <hardware/odroidthings-base.h>
#include <fstream>
#include <sstream>
#include <unistd.h>

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

std::unique_ptr<Gpio> PinManager::getGpio() {
    auto gpio = std::make_unique<Gpio>(board->getPinList());
    return gpio;
}

std::unique_ptr<Pwm> PinManager::getPwm() {
    auto pwm = std::make_unique<Pwm>(board);
    return pwm;
}

std::unique_ptr<I2c> PinManager::getI2c() {
    auto i2cList = board->getI2cList();
    std::vector<i2c_t> list;
    for (auto i2c = i2cList.begin(); i2c != i2cList.end(); i2c++) {
        if (access(i2c->path.c_str(), F_OK) == 0)
            list.push_back(*i2c);
    }

    if (list.size()) {
        auto i2c = std::make_unique<I2c>(list);
        return i2c;
    } else
        return NULL;
}

std::unique_ptr<Uart> PinManager::getUart() {
    auto uartList = board->getUartList();
    std::vector<uart_t> list;
    for (auto uart = uartList.begin(); uart != uartList.end(); uart++) {
        if (access(uart->path.c_str(), F_OK) == 0)
            list.push_back(*uart);
    }

    if (list.size()) {
        auto uart = std::make_unique<Uart>(list);
        return uart;
    } else
        return NULL;
}

std::unique_ptr<Spi> PinManager::getSpi() {
    auto spiList = board->getSpiList();
    if (access(spiList[0].path.c_str(), F_OK) == 0) {
        std::unique_ptr<Spi> spi = std::make_unique<Spi>(spiList);
        return spi;
    }
    return NULL;
}
