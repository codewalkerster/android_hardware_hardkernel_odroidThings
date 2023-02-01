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
#include <map>
#include <fstream>
#include <string>

#include "Board.h"
#include "Gpio.h"
#include "I2c.h"
#include "Spi.h"
#include "Uart.h"

#define BOARD_PROPERTY "ro.product.device"

using hardware::hardkernel::odroidthings::pin_t;

struct pwmState{
    unsigned int period;
    double cycle_rate;

    uint8_t chip;
    uint8_t node;
    std::string periodPath;
    std::string dutyCyclePath;
    std::string enablePath;
    std::string unexportPath;
};

class PinManager {
    private:
        std::shared_ptr<Board> board;
        std::map<int, std::shared_ptr<pwmState>> pwm;
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

        int initPwm();

        // helper function
        void initPwmState(int idx, uint8_t chip, uint8_t node);
        void writeSysfsTo(const std::string path, const std::string value);
        bool isUnknownBoard();

    public:
        PinManager();
        int init();
        std::vector<pin_t> getPinList();

        // common
        std::vector<std::string> getPinNameList();
        std::vector<std::string> getListOf(int);

        // gpio
        std::unique_ptr<Gpio> getGpio();

        // pwm
        void openPwm(int);
        void closePwm(int);
        bool setPwmEnable(int, bool);
        bool setPwmDutyCycle(int, double);
        bool setPwmFrequency(int, double);

        // i2c
        std::unique_ptr<I2c> getI2c();

        // spi
        std::unique_ptr<Spi> getSpi();

        // uart
        std::unique_ptr<Uart> getUart();
};

#endif /* PIN_MANAGER_H */
