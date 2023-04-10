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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#include <hardware/odroidThings.h>
#include <wiringPi/wiringPi.h>
#include <vector>
#include <string>

#include "Board.h"
#include "Gpio.h"
#include "Pwm.h"
#include "I2c.h"
#include "Spi.h"
#include "Uart.h"

#define BOARD_PROPERTY "ro.product.device"

using hardware::hardkernel::odroidthings::pin_t;
class PinManager {
    private:
        std::shared_ptr<Board> board;
        bool isUnknown;

        std::vector<std::string> models = {
            "C1/C1+",
            "C2",
            "XU3/XU4",
            "N1",
            "N2/N2Plus/N2L",
            "C4",
            "HC4",
            "M1",
        };

        std::vector<i2c_t> i2cList;
        std::vector<uart_t> uartList;
        std::vector<pwm_t> pwmList;
        std::vector<spi_t> spiList;
        std::vector<pin_t> gpioList;

        // helper function
        bool isUnknownBoard();

        void initI2cList();
        void initPwmList();
        void initUartList();
        void initSpiList();
        void initGpioList();
    public:
        PinManager();
        int init();
        std::vector<pin_t> getPinList();

        // common
        std::vector<std::string> getPinNameList();

        std::unique_ptr<Gpio> getGpio();
        std::unique_ptr<Pwm> getPwm();
        std::unique_ptr<I2c> getI2c();
        std::unique_ptr<Spi> getSpi();
        std::unique_ptr<Uart> getUart();
};

#endif /* PIN_MANAGER_H */
