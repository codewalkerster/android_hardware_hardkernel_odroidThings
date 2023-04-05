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

#include "Gpio.h"
#include <wiringPi/wiringPi.h>

Gpio::Gpio(std::vector<pin_t> list) {
    for(unsigned long i = 0; i<list.size(); i++)
        if (list[i].availableModes & PIN_GPIO)
            gpioList.insert(std::make_pair(i, list[i]));
}

std::vector<std::string> Gpio::getList() {
    std::vector<std::string> list;

    for (auto pin = gpioList.begin(); pin != gpioList.end(); pin++) {
        int alt = getAlt(pin->second.pin);
        if (alt < 2)
            list.push_back(pin->second.name);
    }

    return list;
}

inline gpioCtxPtr Gpio::getCtx(int idx) {
    return gpio[idx];
}

inline cbPtr Gpio::getCb(int idx) {
    auto ctx = getCtx(idx);
    if (ctx->cb == nullptr)
        ctx->cb = std::make_shared<GpioCallback>(wpiPinToGpio(ctx->pin));
    return ctx->cb;
}

inline int Gpio::getPin(int idx) {
    return gpio[idx]->pin;
}

void Gpio::open(int idx) {
    const auto ctx = std::make_shared<gpioContext>(idx);

    ctx->pin = gpioList[idx].pin;
    pthread_mutex_init(&ctx->mutex, NULL);
    gpio.insert(std::make_pair(idx, ctx));
}

void Gpio::close(int idx) {
    const auto ctx = getCtx(idx);
    pthread_mutex_destroy(&ctx->mutex);
    gpio.erase(idx);
}

bool Gpio::getValue(int idx) {
    const auto ctx = getCtx(idx);

    pthread_mutex_lock(&ctx->mutex);
    auto pin = getPin(idx);
    bool value = (digitalRead(pin) == HIGH);
    pthread_mutex_unlock(&ctx->mutex);
    return value;
}

void Gpio::setDirection(int idx, direction_t direction) {
    const auto ctx = getCtx(idx);

    pthread_mutex_lock(&ctx->mutex);
    auto pin = getPin(idx);
    switch (direction) {
        case DIRECTION_IN:
            pinMode(pin, INPUT);
            break;
        case DIRECTION_OUT_INITIALLY_HIGH:
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
            break;
        case DIRECTION_OUT_INITIALLY_LOW:
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
            break;
    }
    pthread_mutex_unlock(&ctx->mutex);
}

void Gpio::setValue(int idx, bool value) {
    const auto ctx = getCtx(idx);

    pthread_mutex_lock(&ctx->mutex);
    digitalWrite(getPin(idx), value?HIGH:LOW);
    pthread_mutex_unlock(&ctx->mutex);
}

void Gpio::setActiveType(int idx, int activeType) {
    const auto ctx = getCtx(idx);

    pthread_mutex_lock(&ctx->mutex);
    auto pin = getPin(idx);
    switch (activeType) {
        case ACTIVE_LOW:
            pullUpDnControl(pin, PUD_UP);
            break;
        case ACTIVE_HIGH:
            pullUpDnControl(pin, PUD_DOWN);
            break;
    }
    pthread_mutex_unlock(&ctx->mutex);
}

void Gpio::setEdgeTriggerType(int idx, int edgeTriggerType) {
    auto cb = getCb(idx);
    cb->setTriggerType(edgeTriggerType);
}

void Gpio::registerCallback(int idx, function_t callback) {
    auto cb = getCb(idx);
    cb->registerCb(callback);
}

void Gpio::unregisterCallback(int idx) {
    auto cb = getCb(idx);
    cb->unregisterCb();
}
