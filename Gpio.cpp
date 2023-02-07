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

std::vector<std::string> Gpio::getList() {
    std::vector<std::string> list;
    auto pinList = board->getPinList();

    for (auto pin = pinList.begin(); pin != pinList.end(); pin++) {
        if (pin->availableModes & PIN_GPIO) {
            int alt = getAlt(pin->pin);
            if (alt < 2)
                list.push_back(pin->name);
        }
    }

    return list;
}

inline gpioCtxPtr Gpio::getCtx(int idx) {
    return gpio.find(idx)->second;
}

inline cbPtr Gpio::getCb(int idx) {
    auto ctx = getCtx(idx);
    if (ctx->cb == nullptr)
        ctx->cb = std::make_shared<GpioCallback>(wpiPinToGpio(ctx->pin));
    return ctx->cb;
}

void Gpio::open(int idx) {
    const auto ctx = std::make_shared<gpioContext>(idx);

    ctx->pin = board->getPin(idx);
    gpio.insert(std::make_pair(idx, ctx));
}

void Gpio::close(int idx) {
    const auto ctx = getCtx(idx);
    gpio.erase(idx);
}

bool Gpio::getValue(int idx) {
    return (digitalRead(getCtx(idx)->pin) == HIGH);
}

void Gpio::setDirection(int idx, direction_t direction) {
    int pin = getCtx(idx)->pin;
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
}

void Gpio::setValue(int idx, bool value) {
    digitalWrite(getCtx(idx)->pin, value?HIGH:LOW);
}

void Gpio::setActiveType(int idx, int activeType) {
    int pin = getCtx(idx)->pin;
    switch (activeType) {
        case ACTIVE_LOW:
            pullUpDnControl(pin, PUD_UP);
            break;
        case ACTIVE_HIGH:
            pullUpDnControl(pin, PUD_DOWN);
            break;
    }
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
