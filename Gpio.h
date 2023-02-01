/*
 *    Copyright (c) 2023 Sangchul Go <luke.go@hardkernel.com>
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

#ifndef GPIO_H_
#define GPIO_H_

#include "Board.h"
#include <hardware/odroidThings.h>
#include <wiringPi/wiringPi.h>
#include <map>
#include <vector>

using hardware::hardkernel::odroidthings::function_t;
using boardPtr = std::shared_ptr<Board>;

struct gpioContext {
    int pin;
    int triggerType = INT_EDGE_SETUP;
    gpioContext(int pin): pin(pin){};
};

using gpioCtxPtr = std::shared_ptr<gpioContext>;

class Gpio {
    private:
        std::map<int, gpioCtxPtr> gpio;
        boardPtr board;

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

        inline gpioCtxPtr getCtx(int);

    public:
        Gpio(boardPtr);
        std::vector<std::string> getList();
        void open(int);
        void close(int);
        bool getValue(int);
        void setDirection(int, direction_t);
        void setValue(int, bool);
        void setActiveType(int, int);
        void setEdgeTriggerType(int, int);
        void registerCallback(int, function_t);
        void unregisterCallback(int);
};
#endif /* GPIO_H_ */
