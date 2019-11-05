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

#define BOARD_PROPERTY "ro.product.device"

using hardware::hardkernel::odroidthings::pin_t;
using hardware::hardkernel::odroidthings::function_t;

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
        std::string board;
        pin_t *pinList;
        int triggerType[PIN_MAX] = {INT_EDGE_SETUP,};
        std::map<int, pwmState *> pwm;

        void initPwm();

        // helper function
        void initPwmState(int idx, uint8_t chip, uint8_t node);
        void writeSysfsTo(std::string path, std::string value);

        enum ActiveType {
            ACTIVE_LOW,
            ACTIVE_HIGH,
        };
        enum EdgeTrigger {
            EDGE_NONE,
            EDGE_RISING,
            EDGE_FALLING,
            EDGE_BOTH,
        };
    public:
        PinManager();
        void init();
        std::vector<pin_t> getPinList();

        // common
        std::vector<std::string> getPinNameList();
        std::vector<std::string> getListOf(int);

        // gpio
        bool getValue(int);
        void setDirection(int, direction_t);
        void setValue(int, bool);
        void setActiveType(int, int);
        void setEdgeTriggerType(int, int);
        void registerCallback(int, function_t);
        void unregisterCallback(int);

        // pwm
        void openPwm(int);
        void closePwm(int);
        bool setPwmEnable(int, bool);
        bool setPwmDutyCycle(int, double);
        bool setPwmFrequency(int, double);
};

#endif /* PIN_MANAGER_H */
