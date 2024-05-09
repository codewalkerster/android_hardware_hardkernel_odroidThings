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

#ifndef UART_CALLBACK_H_
#define UART_CALLBACK_H_

#include "ringbuffer.h"
#include <hardware/odroidThings.h>
#include <pthread.h>
#include <termios.h>
#include <vector>

#define UART_CALLBACK_SIZE_PROPERTY "persist.android.things.uart.callback.size"
#define DEFAULT_BUFFER_SIZE 256

using hardware::hardkernel::odroidthings::function_t;

class UartCallback {
    private:
        int fd;
        int epfd;
        struct termios option;

        pthread_t callbackThread;
        pthread_mutex_t mutex;
        function_t callback;

        ring_buffer_t *readBuffer;
        size_t callbackBufferSize;

        void initRingBuffer();
        void destroyRingBuffer();
        void setNonBlocking(bool);
        void setMinInTimeout();

        int initEpoll();
        int wait();
        void doCallback(int, char*);
        void loop();
    public:
        UartCallback(int, struct termios);
        ~UartCallback();
        void registerCb(function_t callback);
        void unregisterCb();

        bool isCb();
        std::vector<uint8_t> read(const int length);
};

#endif /* UART_CALLBACK_H_ */
