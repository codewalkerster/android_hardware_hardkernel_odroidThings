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

#include "GpioCallback.h"
#include "Helper.h"
#include <cutils/log.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>

using Helper::writeSysfsTo;

GpioCallback::~GpioCallback() {
    if (callback)
        unregisterCb();
    pthread_mutex_destroy(&mutex);
}

void GpioCallback::registerCb(function_t cb) {
    if (callback)
        unregisterCb();

    if (callback == NULL) {
        setupPin();
        clearPendingIrq();

        pthread_mutex_init(&mutex, NULL);

        pthread_mutex_lock (&mutex);
        callback = cb;
        std::thread cbThread = std::thread(&GpioCallback::loop, this);
        callbackThread = cbThread.native_handle();
        cbThread.detach();
        pthread_mutex_unlock (&mutex);
    } 
}

void GpioCallback::unregisterCb() {
    writeSysfsTo("/sys/class/gpio/unexport", pinNum);
    pthread_kill (callbackThread, 0);

    pthread_mutex_lock(&mutex);
    epoll_ctl(epfd, EPOLL_CTL_DEL, fd, NULL);
    ::close(fd);
    fd = -1;
    callback = NULL;
    callbackThread = 0;
    pthread_mutex_unlock(&mutex);
}

void GpioCallback::setTriggerType(int type) {
    triggerType = type;
}

void GpioCallback::setupPin() {
    writeSysfsTo("/sys/class/gpio/export", pinNum);

    writeSysfsTo(
            "/sys/class/gpio/gpio" + pinNum + "/direction",
            "in");

    std::string mode;
    switch (triggerType) {
        case EDGE_FALLING:
            mode = "falling";
            break;
        case EDGE_RISING:
            mode = "rising";
            break;
        case EDGE_BOTH:
            mode = "both";
            break;
        default:
            mode = "none";
            break;
    }
    writeSysfsTo(
            "/sys/class/gpio/gpio" + pinNum + "/edge",
            mode);

    fd = ::open(
            ("/sys/class/gpio/gpio" + pinNum + "/value").c_str(),
            O_RDWR);
}

void GpioCallback::clearPendingIrq() {
    int count;
    char c;

    // Clear pending interrupts
    ioctl (fd, FIONREAD, &count);
    for (int i = 0; i<count; i++)
        read (fd, &c, 1);
}

#define EPOLL_MAX_CONN 1

int GpioCallback::initEpoll() {
    struct epoll_event ev;

    epfd = epoll_create(EPOLL_MAX_CONN);
    ev.events = EPOLLPRI | EPOLLERR;
    ev.data.fd = fd;
    epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev);

    return epfd;
}

#define MAX_EVENTS 2

int GpioCallback::wait(int mS) {
    struct epoll_event events[MAX_EVENTS];
    char c;
    int event_count;

    if (!epfd)
        initEpoll();

    if (fd < 0)
        return -2;

    event_count = epoll_wait(epfd, events, MAX_EVENTS, mS);

    if (event_count < 0) {
        ALOGE("err epoll_count %d", event_count);
        return -1;
    }

    for (int i = 0; i < event_count; i++) {
        lseek(fd, 0, SEEK_SET);
        if (read(fd, &c, 1) < 0)
            ALOGE("Unable to read from fd");
    }

    return event_count;
}

void GpioCallback::setPriority() {
    struct sched_param sched;

    memset (&sched, 0, sizeof(sched));

    sched.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler (0, SCHED_FIFO, &sched);
}

void GpioCallback::loop() {
    int timeout = 1000;

    setPriority();

    while (1) {
        if (wait(timeout) > 0) {
            pthread_mutex_lock(&mutex);
            if (callback == NULL) {
                ALOGE("call back is null!");
                pthread_mutex_unlock(&mutex);
                return;
            }

            callback();
            pthread_mutex_unlock(&mutex);
        } else {
            if (callback == NULL)
                return;
        }
    }
}
