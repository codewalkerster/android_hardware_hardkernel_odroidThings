/*
 *    Copyright (c) 2020 - 2023 Sangchul Go <luke.go@hardkernel.com>
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
#include <cerrno>
#include <cutils/log.h>
#include <fcntl.h>
#include <stdlib.h>

Uart::Uart() {
}

Uart::Uart(std::vector<uart_t> list) {
    uartList = list;
}

std::vector<std::string> Uart::getList() {
    std::vector<std::string> list;
    for (size_t i=0; i < uartList.size(); i++)
        list.push_back(uartList[i].name);
    return list;
}

inline uartCtxPtr Uart::getCtx(int idx) {
    return uart[idx];
}

inline cbptr Uart::getCb(int idx) {
    auto ctx = getCtx(idx);
    if (ctx->cb == nullptr)
        ctx->cb = std::make_shared<UartCallback>(ctx->fd, ctx->option);
    return ctx->cb;
}

void Uart::open(const int index) {
    int fd = ::open(uartList[index].path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        ALOGE("open uart failed");
        return;
    }

    {
        const auto ctx = std::make_shared<uartContext>();
        ctx->init(fd);
        uart.insert(std::make_pair(index, ctx));
    }
}

void uartContext::init(int fd) {
        this->fd = fd;
        tcgetattr(fd, &option);
        backup_option = option;

        option.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
        option.c_oflag &= ~OPOST;
        option.c_lflag &= ~(ECHO |ECHONL | ICANON | ISIG | IEXTEN);
        tcsetattr(fd, TCSANOW, &option);
        cb = nullptr;
}

void Uart::close(const int index) {
    ALOGD("[%d] %s", __LINE__, __func__);
    auto ctx= getCtx(index);

    tcsetattr(ctx->fd, TCSANOW, &(ctx->backup_option));

    ::close(ctx->fd);
    uart.erase(index);
}

bool Uart::flush(const int index, const int direction) {
    const auto ctx = getCtx(index);
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
    if (tcflush(ctx->fd, flush) == -1) {
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
    const auto ctx = getCtx(index);

    if (tcsendbreak(ctx->fd, duration) == -1) {
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
    auto ctx= getCtx(index);
    uint32_t baud = getBaudrate(baudrate);

    cfsetospeed(&(ctx->option), baud);
    cfsetispeed(&(ctx->option), baud);
    tcsetattr(ctx->fd, TCSANOW, &(ctx->option));
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
    auto ctx = getCtx(index);
    ctx->option.c_cflag &= ~CSIZE;
    switch (size) {
        case 5:
            ctx->option.c_cflag |= CS5;
            break;
        case 6:
            ctx->option.c_cflag |= CS6;
            break;
        case 7:
            ctx->option.c_cflag |= CS7;
            break;
        case 8:
            ctx->option.c_cflag |= CS8;
            break;
        default:
            ALOGE("Invalid data size");
            return false;
    }
    tcsetattr(ctx->fd, TCSANOW, &(ctx->option));
    return true;
}

bool Uart::setHardwareFlowControl(const int index, const int mode) {
    auto ctx = getCtx(index);
    switch (mode) {
        case 0: // HW_FLOW_CONTROL_NONE
            ctx->option.c_cflag &= ~CRTSCTS;
            break;
        case 1: // HW_FLOW_CONTROL_AUTO_RTSCTS
            ctx->option.c_cflag |= CRTSCTS;
            break;
    }
    tcsetattr(ctx->fd, TCSANOW, &(ctx->option));
    return true;
}

bool Uart::setParity(const int index, const int mode) {
    auto ctx= getCtx(index);
    switch (mode) {
        case 0: // PARITY_ NONE
            ctx->option.c_cflag &= ~PARENB;
            break;
        case 1: // PARITY_ EVEN
            ctx->option.c_cflag |= PARENB;
            ctx->option.c_cflag &= ~(PARODD|CMSPAR);
            break;
        case 2: // PARITY_ODD
            ctx->option.c_cflag |= (PARENB | PARODD);
            ctx->option.c_cflag &= ~CMSPAR;
            break;
        case 3: // PARITY_MARK
            ctx->option.c_cflag |= (PARENB | PARODD | CMSPAR);
            break;
        case 4: // PARITY_SPACE
            ctx->option.c_cflag |= (PARENB | CMSPAR);
            ctx->option.c_cflag &= ~PARODD;
            break;
        default:
            ALOGE("parti is invalid");
            return false;
    }
    tcsetattr(ctx->fd, TCSANOW, &(ctx->option));
    return true;
}

bool Uart::setStopBits(const int index, const int bits) {
    auto ctx = getCtx(index);
    switch (bits) {
        case 1: // 1 stop bits
            ctx->option.c_cflag &= ~CSTOPB;
            break;
        case 2: // 2 stop bits
            ctx->option.c_cflag |= CSTOPB;
            break;
        default:
            ALOGE("Invalid stop bits");
            return false;
    }
    tcsetattr(ctx->fd, TCSANOW, &(ctx->option));
    return true;
}

std::vector<uint8_t> Uart::read(const int index, const int length) {
    const auto ctx = getCtx(index);
    auto cb = getCb(index);

    if (cb->isCb()) {
        return cb->read(length);
    } else {
        auto buffer = std::make_unique<uint8_t[]>(length);

        auto ret = ::read(ctx->fd, buffer.get(), length);
        if (ret < 0) {
            std::vector<uint8_t> empty(0);
            return empty;
        }
        std::vector<uint8_t> result(buffer.get(), buffer.get() + ret);

        return result;
    }
}

ssize_t Uart::write(const int index, const std::vector<uint8_t> data, const int length) {
    const auto ctx = getCtx(index);
    auto buffer = std::make_unique<uint8_t[]>(length);

    std::copy(data.begin(), data.end(), buffer.get());

    ssize_t result = ::write(ctx->fd, buffer.get(), length);

    return result;
}

void Uart::registerCallback(const int index, function_t callback) {
    auto cb = getCb(index);
    const auto ctx = getCtx(index);
    cb->registerCb(callback);
}

void Uart::unregisterCallback(const int index) {
    const auto ctx = getCtx(index);
    auto cb = getCb(index);

    cb->unregisterCb();
    tcsetattr(ctx->fd, TCSANOW, &(ctx->option));
}
