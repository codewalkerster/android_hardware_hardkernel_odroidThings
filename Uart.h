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

#ifndef UART_H_
#define UART_H_

#include <hardware/odroidThings.h>
#include <map>
#include <termios.h>
#include <unistd.h>
#include <vector>

using hardware::hardkernel::odroidthings::uart_t;

struct uartState {
    int fd;
    struct termios option;

    uint8_t hwFlowControl;
    uint8_t parity;
    uint8_t controlLine;
};

class Uart {
    private:
        std::vector<uart_t> uartList;
        std::map<int, uartState> uart;
        Uart();
        uint32_t getBaudrate(const int baudrate);

    public:
        Uart(std::vector<uart_t> list);
        std::vector<std::string> getList();
        void open(const int index);
        void close(const int index);
        bool flush(const int index, const int direction);
        bool sendBreak(const int index, const int duration);
        bool setBaudrate(const int index, const int baudrate);
        bool setDataSize(const int index, const int size);
        bool setHardwareFlowControl(const int index, const int mode);
        bool setParity(const int index, const int mode);
        bool setStopBits(const int index, const int bits);
        std::vector<uint8_t> read(const int index, const int length);
        ssize_t write(const int index, const std::vector<uint8_t> data, const int length);

        /*
         * The modem control feature is not used, No implimented.
         */
        bool clearModemControl(const int index, const int mode);
        bool setModemControl(const int index, const int mode);
};
#endif /* UART_H_ */
