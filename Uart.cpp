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
#include <cerrno>
#include <cutils/log.h>
#include <cutils/properties.h>
#include <thread>
#include <fcntl.h>
#include <sys/epoll.h>
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
    return uart.find(idx)->second;
}

void Uart::open(const int index) {
    int fd = ::open(uartList[index].path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        ALOGE("open uart failed");
        return;
    }

    {
        char bufferSize[128];
        const auto ctx = std::make_shared<uartContext>();

        property_get(UART_CALLBACK_SIZE_PROPERTY,
                bufferSize, DEFAULT_BUFFER_SIZE);
        ctx->callbackBufferSize = atoi(bufferSize);
        if (ctx->callbackBufferSize == 0)
            ctx->callbackBufferSize = atoi(DEFAULT_BUFFER_SIZE);
        ctx->callback = NULL;
        ctx->fd = fd;
        tcgetattr(fd, &(ctx->option));
        ctx->backup_option = ctx->option;

        ctx->option.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
        ctx->option.c_oflag &= ~OPOST;
        ctx->option.c_lflag &= ~(ECHO |ECHONL | ICANON | ISIG | IEXTEN);
        tcsetattr(fd, TCSANOW, &(ctx->option));

        ctx->readBuffer = NULL;

        pthread_mutex_init(&ctx->mutex, NULL);

        uart.insert(std::make_pair(index, ctx));
    }
}

void Uart::close(const int index) {
    ALOGD("[%d] %s", __LINE__, __func__);
    auto ctx= getCtx(index);

    tcsetattr(ctx->fd, TCSANOW, &(ctx->backup_option));

    pthread_mutex_destroy(&ctx->mutex);

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
    char *buffer = new char[length]{};

    if (ctx->readBuffer == NULL) {
    auto ret = ::read(ctx->fd, buffer, length);
    if (ret < 0) {
        delete[] buffer;
        std::vector<uint8_t> empty(0);
        return empty;
    }
    std::vector<uint8_t> result(buffer, buffer + ret);
    delete[] buffer;

    return result;
    } else {
        auto ret = ring_buffer_dequeue_arr(ctx->readBuffer, buffer, (ring_buffer_size_t)length);
    if (ret < 0) {
        delete[] buffer;
        std::vector<uint8_t> empty(0);
        return empty;
    }
        std::vector<uint8_t> result (buffer, buffer + ret);
        delete[] buffer;

        return result;
    }
}

ssize_t Uart::write(const int index, const std::vector<uint8_t> data, const int length) {
    const auto ctx = getCtx(index);
    uint8_t *buffer = new uint8_t[length];

    std::copy(data.begin(), data.end(), buffer);

    ssize_t result = ::write(ctx->fd, buffer, length);
    delete[] buffer;

    return result;
}

void Uart::registerCallback(const int index, function_t callback) {
    const auto ctx = getCtx(index);

    if (ctx->callback == NULL) {
        pthread_mutex_lock(&(ctx->mutex));
        ctx->callback = callback;
        ctx->backup_callback_option = ctx->option;

        ctx->readBuffer = new ring_buffer_t;
        char *buffer = new char[ctx->callbackBufferSize]{};
        ring_buffer_init(ctx->readBuffer, buffer, ctx->callbackBufferSize);

        pthread_mutex_unlock(&(ctx->mutex));

        int flags = fcntl(ctx->fd, F_GETFL, 0);
        flags |= O_NONBLOCK;
        fcntl(ctx->fd, F_SETFL, flags);

        std::thread callbackThread = std::thread(&Uart::callbackRun, this, index);
        ctx->callbackThread = callbackThread.native_handle();
        callbackThread.detach();
    }
}

void Uart::unregisterCallback(const int index) {
    const auto ctx = getCtx(index);
    int ret = pthread_kill(ctx->callbackThread, 0);

    pthread_mutex_lock(&(ctx->mutex));
    ALOGD("unregister callback - %d", ret);
    ctx->callback = NULL;
    ctx->option = ctx->backup_callback_option;

    delete[] ctx->readBuffer->buffer;
    delete ctx->readBuffer;
    ctx->readBuffer = NULL;

    pthread_mutex_unlock(&(ctx->mutex));

    int flags = fcntl(ctx->fd, F_GETFL, 0);
    flags &= ~O_NONBLOCK;
    fcntl(ctx->fd, F_SETFL, flags);

    tcsetattr(ctx->fd, TCSANOW, &(ctx->option));
}

#define EPOLL_MAX_CONN 2
#define MAX_EVENTS 128

void Uart::callbackRun(const int index) {
    const auto ctx = getCtx(index);
    struct epoll_event ev;
    struct epoll_event events[MAX_EVENTS];
    int epfd;
    char *buffer = new char[ctx->callbackBufferSize];

    int timeout = 1000;

    ctx->option.c_cc[VMIN] = 1;
    ctx->option.c_cc[VTIME] = 0;

    tcflush(ctx->fd, TCIOFLUSH);
    tcsetattr(ctx->fd, TCSANOW, &(ctx->option));

    epfd = epoll_create(EPOLL_MAX_CONN);
    ev.events = EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLHUP;
    ev.data.fd = ctx->fd;
    int ret = epoll_ctl(epfd, EPOLL_CTL_ADD, ctx->fd, &ev);

    if (ret < 0) {
        ALOGE("err epoll_ctl %i", errno);
        delete[] buffer;
        return;
    }

    while (1) {
        int event_count;
        event_count = epoll_wait(epfd, events, MAX_EVENTS, timeout);

        if (ctx->callback == NULL) {
            ALOGE("call back is null!");
            delete[] buffer;
            return;
        }

        if (event_count < 0) {
            ALOGE("err epoll_count %d", event_count);
            delete[] buffer;
            return;
        }
        pthread_mutex_lock(&(ctx->mutex));
        for (int i=0; i < event_count; i++) {
            auto length = ::read(ctx->fd, buffer, ctx->callbackBufferSize);
            if (length < 0) {
                ALOGE("read Error : %s", strerror(errno));
            } else {
                ring_buffer_queue_arr(ctx->readBuffer, buffer, length);
                ctx->callback();
            }
        }
        pthread_mutex_unlock(&(ctx->mutex));
    }
    delete[] buffer;
}
