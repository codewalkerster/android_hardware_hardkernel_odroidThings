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

#include "Uart.h"
#include <cutils/log.h>
#include <cerrno>

Uart::Uart() {
}

Uart::Uart(uart_t *list) {
    uartList = list;
}

std::vector<std::string> Uart::getList() {
    std::vector<std::string> list;
    for (int i=0; i<UART_MAX; i++)
        list.push_back(uartList[i].name);
    return list;
}

void Uart::open(const int index) {
    int fd = ::open(uartList[index].path.c_str(), O_RDWR | O_NDELAY | O_NOCTTY);

    if (fd < 0) {
        ALOGE("open uart failed");
        return;
    }

    uartState state;
    state.fd = fd;
    tcgetattr(fd, &(state.option));

    state.option.c_lflag &= ~ECHO;
    state.option.c_iflag &= ~ICRNL;
    tcsetattr(fd, TCSAFLUSH, &(state.option));

    uart.insert(std::make_pair(index, state));
}

void Uart::close(const int index) {
    auto state= uart.find(index)->second;

    state.option.c_lflag &= ECHO;
    state.option.c_iflag &= ICRNL;
    tcsetattr(state.fd, TCSAFLUSH, &(state.option));

    ::close(state.fd);
    uart.erase(index);
}

bool Uart::flush(const int index, const int direction) {
    const auto state = uart.find(index)->second;
    int flush = 0;
    switch (direction) {
        case 0: // FLUSH_IN
            flush = TCIFLUSH;
            break;
        case 1: // FLUSH_OUT
            flush = TCOFLUSH;
            break;
        case 2: // FLUSH_INOUT
            flush = TCIOFLUSH;
            break;
    }
    if (tcflush(state.fd, flush) == -1) {
        switch (errno) {
            case EBADF:
                ALOGE("fd is invalid");
                break;
            case ENOTTY:
                ALOGE("fd is not a tty");
                break;
            case EINVAL:
                ALOGE("fd is not supported");
                break;
            case EIO:
                ALOGE("io error");
                break;
            default:
                ALOGE("unknown error");
        }
        return false;
    }
    return true;
}

bool Uart::sendBreak(const int index,const int duration) {
    const auto state = uart.find(index)->second;

    if (tcsendbreak(state.fd, duration) == -1) {
        switch (errno) {
            case EBADF:
                ALOGE("fd is invalid");
                break;
            case ENOTTY:
                ALOGE("fd is not terminal");
                break;
            default:
                ALOGE("unknown error");
        }
        return false;
    }
    return true;
}

bool Uart::setBaudrate(const int index, const int baudrate) {
    auto iterator = uart.find(index);
    uint32_t baud = getBaudrate(baudrate);

    cfsetospeed(&(iterator->second.option), baud);
    cfsetispeed(&(iterator->second.option), baud);
    tcsetattr(iterator->second.fd, TCSANOW, &(iterator->second.option));
    return true;
}

uint32_t Uart::getBaudrate(int baudrate) {
    uint32_t baud;
    switch (baudrate) {
        case 1200:
            baud = B1200;
            break;
        case 2400:
            baud = B2400;
            break;
        case 4800:
            baud = B4800;
            break;
        case 9600:
            baud = B9600;
            break;
        case 19200:
            baud = B19200;
            break;
        case 38400:
            baud = B38400;
            break;
        case 57600:
            baud = B57600;
            break;
        case 115200:
            baud = B115200;
            break;
        case 230400:
            baud = B230400;
            break;
        case 460800:
            baud = B460800;
            break;
        case 500000:
            baud = B500000;
            break;
        case 576000:
            baud = B576000;
            break;
        case 921600:
            baud = B921600;
            break;
        case 1000000:
            baud = B1000000;
            break;
        case 1152000:
            baud = B1152000;
            break;
        case 1500000:
            baud = B1500000;
            break;
        case 2000000:
            baud = B2000000;
            break;
        case 2500000:
            baud = B2500000;
            break;
        case 3000000:
            baud = B3000000;
            break;
        case 3500000:
            baud = B3500000;
            break;
        case 4000000:
            baud = B4000000;
            break;
    }
    return baud;
}

bool Uart::setDataSize(const int index, const int size) {
    auto iterator = uart.find(index);
    iterator->second.option.c_cflag &= ~CSIZE;
    switch (size) {
        case 5:
            iterator->second.option.c_cflag |= CS5;
            break;
        case 6:
            iterator->second.option.c_cflag |= CS6;
            break;
        case 7:
            iterator->second.option.c_cflag |= CS7;
            break;
        case 8:
            iterator->second.option.c_cflag |= CS8;
            break;
        default:
            ALOGE("Invalid data size");
            return false;
    }
    tcsetattr(iterator->second.fd, TCSANOW, &(iterator->second.option));
    return true;
}

bool Uart::setHardwareFlowControl(const int index, const int mode) {
    auto iterator = uart.find(index);
    switch (mode) {
        case 0: // HW_FLOW_CONTROL_NONE
            iterator->second.option.c_cflag &= ~CRTSCTS;
            break;
        case 1: // HW_FLOW_CONTROL_AUTO_RTSCTS
            iterator->second.option.c_cflag |= CRTSCTS;
            break;
    }
    tcsetattr(iterator->second.fd, TCSANOW, &(iterator->second.option));
    return true;
}

bool Uart::setParity(const int index, const int mode) {
    auto iterator = uart.find(index);
    switch (mode) {
        case 0: // PARITY_ NONE
            iterator->second.option.c_cflag &= ~PARENB;
            break;
        case 1: // PARITY_ EVEN
            iterator->second.option.c_cflag |= PARENB;
            iterator->second.option.c_cflag &= ~(PARODD|CMSPAR);
            break;
        case 2: // PARITY_ODD
            iterator->second.option.c_cflag |= (PARENB | PARODD);
            iterator->second.option.c_cflag &= ~CMSPAR;
            break;
        case 3: // PARITY_MARK
            iterator->second.option.c_cflag |= (PARENB | PARODD | CMSPAR);
            break;
        case 4: // PARITY_SPACE
            iterator->second.option.c_cflag |= (PARENB | CMSPAR);
            iterator->second.option.c_cflag &= ~PARODD;
            break;
        default:
            ALOGE("parti is invalid");
            return false;
    }
    tcsetattr(iterator->second.fd, TCSANOW, &(iterator->second.option));
    return true;
}

bool Uart::setStopBits(const int index, const int bits) {
    auto iterator = uart.find(index);
    switch (bits) {
        case 1: // 1 stop bits
            iterator->second.option.c_cflag &= ~CSTOPB;
            break;
        case 2: // 2 stop bits
            iterator->second.option.c_cflag |= CSTOPB;
            break;
        default:
            ALOGE("Invalid stop bits");
            return false;
    }
    tcsetattr(iterator->second.fd, TCSANOW, &(iterator->second.option));
    return true;
}

std::vector<uint8_t> Uart::read(const int index, const int length) {
    const auto state = uart.find(index)->second;

    uint8_t *buffer = new uint8_t[length];
    auto ret = ::read(state.fd, buffer, length);
    if (ret < 0) {
        delete[] buffer;
        std::vector<uint8_t> empty(0);
        return empty;
    }
    std::vector<uint8_t> result(buffer, buffer + ret);
    delete[] buffer;

    return result;
}

ssize_t Uart::write(const int index, const std::vector<uint8_t> data, const int length) {
    const auto state = uart.find(index)->second;
    uint8_t *buffer = new uint8_t[length];

    std::copy(data.begin(), data.end(), buffer);

    ssize_t result = ::write(state.fd, buffer, length);
    delete[] buffer;

    return result;
}
