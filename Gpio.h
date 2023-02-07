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
#include "GpioCallback.h"
#include <hardware/odroidThings.h>
#include <map>
#include <vector>

using cbPtr = std::shared_ptr<GpioCallback>;

struct gpioContext {
    int pin;
    cbPtr cb;
    gpioContext(int pin): pin(pin), cb(nullptr) {};
};

using boardPtr = std::shared_ptr<Board>;
using gpioCtxPtr = std::shared_ptr<gpioContext>;
using hardware::hardkernel::odroidthings::function_t;

class Gpio {
    private:
        std::map<int, gpioCtxPtr> gpio;
        boardPtr board;

        enum ActiveType {
            ACTIVE_LOW,
            ACTIVE_HIGH,
        };
        inline gpioCtxPtr getCtx(int);
        inline cbPtr getCb(int);

    public:
        Gpio(boardPtr board): board(board) {};
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
