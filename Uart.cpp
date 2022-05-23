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
#include <thread>
#include <fcntl.h>
#include <sys/epoll.h>

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

void Uart::open(const int index) {
    int fd = ::open(uartList[index].path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        ALOGE("open uart failed");
        return;
    }

    {
        const auto state = std::make_shared<uartState>();

        state->callback = NULL;
        state->fd = fd;
        tcgetattr(fd, &(state->option));
        state->backup_option = state->option;

        state->option.c_lflag &= ~ECHO;
        state->option.c_iflag &= ~ICRNL;
        tcsetattr(fd, TCSANOW, &(state->option));

        pthread_mutex_init(&state->mutex, NULL);

        uart.insert(std::make_pair(index, state));
    }
}

void Uart::close(const int index) {
    ALOGE("[%d] %s", __LINE__, __func__);
    auto state= uart.find(index)->second;

    tcsetattr(state->fd, TCSANOW, &(state->backup_option));

    pthread_mutex_destroy(&state->mutex);

    ::close(state->fd);
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
    if (tcflush(state->fd, flush) == -1) {
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

    if (tcsendbreak(state->fd, duration) == -1) {
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
    auto state= uart.find(index)->second;
    uint32_t baud = getBaudrate(baudrate);

    cfsetospeed(&(state->option), baud);
    cfsetispeed(&(state->option), baud);
    tcsetattr(state->fd, TCSANOW, &(state->option));
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
    auto state = uart.find(index)->second;
    state->option.c_cflag &= ~CSIZE;
    switch (size) {
        case 5:
            state->option.c_cflag |= CS5;
            break;
        case 6:
            state->option.c_cflag |= CS6;
            break;
        case 7:
            state->option.c_cflag |= CS7;
            break;
        case 8:
            state->option.c_cflag |= CS8;
            break;
        default:
            ALOGE("Invalid data size");
            return false;
    }
    tcsetattr(state->fd, TCSANOW, &(state->option));
    return true;
}

bool Uart::setHardwareFlowControl(const int index, const int mode) {
    auto state = uart.find(index)->second;
    switch (mode) {
        case 0: // HW_FLOW_CONTROL_NONE
            state->option.c_cflag &= ~CRTSCTS;
            break;
        case 1: // HW_FLOW_CONTROL_AUTO_RTSCTS
            state->option.c_cflag |= CRTSCTS;
            break;
    }
    tcsetattr(state->fd, TCSANOW, &(state->option));
    return true;
}

bool Uart::setParity(const int index, const int mode) {
    auto state= uart.find(index)->second;
    switch (mode) {
        case 0: // PARITY_ NONE
            state->option.c_cflag &= ~PARENB;
            break;
        case 1: // PARITY_ EVEN
            state->option.c_cflag |= PARENB;
            state->option.c_cflag &= ~(PARODD|CMSPAR);
            break;
        case 2: // PARITY_ODD
            state->option.c_cflag |= (PARENB | PARODD);
            state->option.c_cflag &= ~CMSPAR;
            break;
        case 3: // PARITY_MARK
            state->option.c_cflag |= (PARENB | PARODD | CMSPAR);
            break;
        case 4: // PARITY_SPACE
            state->option.c_cflag |= (PARENB | CMSPAR);
            state->option.c_cflag &= ~PARODD;
            break;
        default:
            ALOGE("parti is invalid");
            return false;
    }
    tcsetattr(state->fd, TCSANOW, &(state->option));
    return true;
}

bool Uart::setStopBits(const int index, const int bits) {
    auto state = uart.find(index)->second;
    switch (bits) {
        case 1: // 1 stop bits
            state->option.c_cflag &= ~CSTOPB;
            break;
        case 2: // 2 stop bits
            state->option.c_cflag |= CSTOPB;
            break;
        default:
            ALOGE("Invalid stop bits");
            return false;
    }
    tcsetattr(state->fd, TCSANOW, &(state->option));
    return true;
}

std::vector<uint8_t> Uart::read(const int index, const int length) {
    const auto state = uart.find(index)->second;

    uint8_t *buffer = new uint8_t[length];
    auto ret = ::read(state->fd, buffer, length);
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

    ssize_t result = ::write(state->fd, buffer, length);
    delete[] buffer;

    return result;
}

void Uart::registerCallback(const int index, function_t callback) {
    const auto state = uart.find(index)->second;

    if (state->callback == NULL) {
        pthread_mutex_lock(&(state->mutex));
        state->callback = callback;
        pthread_mutex_unlock(&(state->mutex));

        std::thread callbackThread = std::thread(&Uart::callbackRun, this, index);
        state->callbackThread = callbackThread.native_handle();
        callbackThread.detach();
    }
}

void Uart::unregisterCallback(const int index) {
    const auto state = uart.find(index)->second;
    int ret = pthread_kill(state->callbackThread, 0);

    pthread_mutex_lock(&(state->mutex));
    ALOGD("unregister callback - %d", ret);
    state->callback = NULL;
    pthread_mutex_unlock(&(state->mutex));

    int flags = fcntl(state->fd, F_GETFL, 0);
    flags &= ~O_NONBLOCK;
    fcntl(state->fd, F_SETFL, &flags);

    state->option = state->backup_callback_option;
    tcsetattr(state->fd, TCSANOW, &(state->option));
}

#define EPOLL_MAX_CONN 2
#define MAX_EVENTS 128

void Uart::callbackRun(const int index) {
    const auto state = uart.find(index)->second;
    struct epoll_event ev;
    struct epoll_event events[MAX_EVENTS];
    int epfd;

    int timeout = 1000;

    int flags = fcntl(state->fd, F_GETFL, 0);
    flags |= O_NONBLOCK;
    fcntl(state->fd, F_SETFL, &flags);

    state->option.c_cc[VMIN] = 1;
    state->option.c_cc[VTIME] = 0;
    state->option.c_iflag = IGNBRK;

    tcflush(state->fd, TCIOFLUSH);
    tcsetattr(state->fd, TCSANOW, &(state->option));

    epfd = epoll_create(EPOLL_MAX_CONN);
    ev.events = EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLHUP;
    ev.data.fd = state->fd;
    int ret = epoll_ctl(epfd, EPOLL_CTL_ADD, state->fd, &ev);

    if (ret < 0) {
        ALOGE("err epoll_ctl %i", errno);
        return;
    }

    while (1) {
        int event_count;
        event_count = epoll_wait(epfd, events, MAX_EVENTS, timeout);
        ALOGD("event_count -  %d", event_count);

        ALOGD("callback - %ld", (long)state->callback);
        pthread_mutex_lock(&(state->mutex));
        if (state->callback == NULL) {
            ALOGE("call back is null!");
            pthread_mutex_unlock(&(state->mutex));
            return;
        }
        pthread_mutex_unlock(&(state->mutex));

        if (event_count < 0) {
            ALOGE("err epoll_count %d", event_count);
            return;
        }

        for (int i=0; i < event_count; i++) {
            if (events[i].data.fd == state->fd) {
                state->callback();
            } else {
                ALOGE("this is not fd callback");
            }
        }
    }
}
