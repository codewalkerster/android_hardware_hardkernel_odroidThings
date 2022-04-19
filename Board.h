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

#ifndef BOARD_H
#define BOARD_H
#include <hardware/odroidThings.h>
#include <vector>

using hardware::hardkernel::odroidthings::pin_t;
using hardware::hardkernel::odroidthings::i2c_t;
using hardware::hardkernel::odroidthings::pwm_t;
using hardware::hardkernel::odroidthings::uart_t;
using hardware::hardkernel::odroidthings::spi_t;

class Board {
    protected:
        Board();
        std::string name;
        // pin array number is based on physical pin number
        std::vector<pin_t> pinList;
        std::vector<i2c_t> i2cList;
        std::vector<pwm_t> pwmList;
        std::vector<uart_t> uartList;
        std::vector<spi_t> spiList;

    public:
        Board(std::string);
        int getPin(int);
        std::vector<pin_t> getPinList();
        std::vector<std::string> getPinNameList();
        std::vector<i2c_t> getI2cList();
        std::vector<pwm_t> getPwmList();
        std::vector<uart_t> getUartList();
        std::vector<spi_t> getSpiList();
};

#endif
