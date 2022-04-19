/*
 *    Copyright (c) 2022 Sangchul Go <luke.go@hardkernel.com>
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

#include "Board.h"

Board::Board() {
}

Board::Board(std::string name) {
    this->name = name;
}

int Board::getPin(int idx) {
    return pinList[idx].pin;
}

std::vector<pin_t> Board::getPinList() {
    return pinList;
}

std::vector<std::string> Board::getPinNameList() {
    std::vector<std::string> list;

    for (auto pin = pinList.begin(); pin != pinList.end(); pin++)
        list.push_back(pin->name);

    return list;
}

std::vector<i2c_t> Board::getI2cList() {
    return i2cList;
}

std::vector<pwm_t> Board::getPwmList() {
    return pwmList;
}

std::vector<uart_t> Board::getUartList() {
    return uartList;
}

std::vector<spi_t> Board::getSpiList() {
    return spiList;
}
