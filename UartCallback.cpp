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

#include "UartCallback.h"
#include <cutils/log.h>
#include <cutils/properties.h>
#include <sys/epoll.h>
#include <thread>

UartCallback::UartCallback(int fd, struct termios cbOption) {
    this->fd = fd;
    option = cbOption;
    callback = NULL;
    readBuffer = NULL;

    pthread_mutex_init(&mutex, NULL);

    callbackBufferSize = property_get_int32(UART_CALLBACK_SIZE_PROPERTY,
            DEFAULT_BUFFER_SIZE);
    if (callbackBufferSize == 0)
        callbackBufferSize = DEFAULT_BUFFER_SIZE;
}

UartCallback::~UartCallback() {
    pthread_mutex_destroy(&mutex);
}

bool UartCallback::isCb() {
    return readBuffer != NULL;
}

std::vector<uint8_t> UartCallback::read(const int length) {
    auto buffer = std::make_unique<char[]>(length);

    pthread_mutex_lock(&mutex);
    auto ret = ring_buffer_dequeue_arr(readBuffer, buffer.get(), (ring_buffer_size_t)length);
    pthread_mutex_unlock(&mutex);
    if (ret < 0) {
        std::vector<uint8_t> empty(0);
        return empty;
    }
    std::vector<uint8_t> result (buffer.get(), buffer.get() + ret);

    return result;
}

void UartCallback::initRingBuffer() {
    readBuffer = new ring_buffer_t;
    char *buffer = new char[callbackBufferSize]{};
    ring_buffer_init(readBuffer, buffer, callbackBufferSize);
}

void UartCallback::destroyRingBuffer() {
    delete[] readBuffer->buffer;
    delete readBuffer;
    readBuffer = NULL;
}

inline void UartCallback::setNonBlocking(bool on) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (on)
        flags |= O_NONBLOCK;
    else 
        flags &= ~O_NONBLOCK;
    fcntl(fd, F_SETFL, flags);
}

void UartCallback::registerCb(function_t cb) {
    if (callback == NULL) {
        pthread_mutex_lock(&mutex);
        callback = cb;

        initRingBuffer();
        setNonBlocking(true);

        std::thread cbThread = std::thread(&UartCallback::loop, this);
        callbackThread = cbThread.native_handle();
        cbThread.detach();
        pthread_mutex_unlock(&mutex);
    }
}

void UartCallback::unregisterCb() {
    int ret = pthread_kill(callbackThread, 0);

    pthread_mutex_lock(&mutex);
    ALOGD("unregister callback - %d", ret);
    epoll_ctl(epfd, EPOLL_CTL_DEL, fd, NULL);
    callback = NULL;

    destroyRingBuffer();
    setNonBlocking(false);
    close(epfd);
    pthread_mutex_unlock(&mutex);
}

void UartCallback::setMinInTimeout() {
    option.c_cc[VMIN] = 1;
    option.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &option);
}

#define EPOLL_MAX_CONN 2

int UartCallback::initEpoll() {
    struct epoll_event ev;

    epfd = epoll_create(EPOLL_MAX_CONN);
    ev.events = EPOLLIN | EPOLLPRI;
    ev.data.fd = fd;
    return epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev);
}

#define MAX_EVENTS 128

int UartCallback::wait() {
    struct epoll_event events[MAX_EVENTS];
    int ret;
    ret = epoll_wait(epfd, events, MAX_EVENTS, -1);

    if (callback == NULL) {
        ALOGE("call back is null!");
        return -1;
    }

    if (ret < 0)
        ALOGE("err epoll_count %d", ret);

    return ret;
}

void UartCallback::doCallback(int count, char * buffer) {
    for (int i=0; i < count; i++) {
        auto length = ::read(fd, buffer, callbackBufferSize);
        if (length < 0) {
            if (errno != EAGAIN)
                ALOGE("read Error : %s", strerror(errno));
            else
                ALOGE("Retry read!");
        } else {
            ring_buffer_queue_arr(readBuffer, buffer, length);
            callback();
        }
    }
}

void UartCallback::loop() {
    auto buffer = std::make_unique<char[]>(callbackBufferSize);
    int event_count;

    setMinInTimeout();
    if (initEpoll() < 0) {
        ALOGE("err epoll init %i", errno);
        return;
    }

    while (1) {
        if ((event_count = wait()) >= 0) {
            pthread_mutex_lock(&mutex);
            doCallback(event_count, buffer.get());
            pthread_mutex_unlock(&mutex);
        } else {
            break;
        }
    }
}
