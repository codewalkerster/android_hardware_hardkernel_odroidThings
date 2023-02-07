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

#ifndef GPIO_CALLBACK_H_
#define GPIO_CALLBACK_H_

#include <thread>
#include <hardware/odroidThings.h>

using hardware::hardkernel::odroidthings::function_t;

class GpioCallback {
    private:
        pthread_t callbackThread;
        pthread_mutex_t mutex;
        function_t callback;

        int fd;
        int epfd;
        std::string pinNum;
        int triggerType;

        enum EdgeTrigger {
            EDGE_NONE,
            EDGE_RISING,
            EDGE_FALLING,
            EDGE_BOTH,
        };

        void setupPin();
        void clearPendingIrq();
        void loop();
        int initEpoll();
        void setPriority();
        int wait(int);

    public:
        GpioCallback (int pin):
            callback(NULL), epfd(0),
            pinNum(std::to_string(pin)), triggerType(EDGE_NONE) {};
        ~GpioCallback();

        void registerCb(function_t);
        void unregisterCb();
        void setTriggerType(int);
};

#endif /* GPIO_CALLBACK_H_ */
