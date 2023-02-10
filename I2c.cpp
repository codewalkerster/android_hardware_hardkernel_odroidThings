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

#include "I2c.h"
#include <cutils/log.h>
#include <cutils/properties.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sstream>
#include <unistd.h>

I2c::I2c() {
}

I2c::I2c(std::vector<i2c_t> list) {
    i2cList = list;
}

inline i2cCtxPtr I2c::getCtx(int idx) {
    return i2c[idx];
}

inline void I2c::getProperty(const i2cCtxPtr& ctx) {
    std::ostringstream propertyName;
    propertyName << "persist.android.things.i2c.reg.size."
        << ctx->busIdx << "." << ctx->deviceAddress;

    ctx->regBufSize = property_get_int32(propertyName.str().c_str(), 0);
}

inline uint8_t I2c::getRegBufSize(const i2cCtxPtr& ctx, uint32_t regAddress) {
    uint8_t regBufSize = 1;
    if (ctx->regBufSize) {
        regBufSize = ctx->regBufSize;
    } else {
        if (regAddress & I2C_REG_TWO_BYTE)
            regBufSize = 2;
    }

    return regBufSize;
}

inline uint8_t *I2c::getRegBuffer(const uint8_t size, uint32_t regAddress) {
    uint8_t *regBuf = new uint8_t[size];

    for (int i = size; i > 0; i--) {
        regBuf[i-1] = (uint8_t)(regAddress & 0xFF);
        regAddress >>= 8;
    }

    return regBuf;
}

std::vector<std::string> I2c::getList() {
    std::vector<std::string> list;
    for (size_t i=0; i < i2cList.size(); i++)
        list.push_back(i2cList[i].name);
    return list;
}

void I2c::open(const int busIdx, const uint32_t address, const int idx) {
    int fd = ::open(i2cList[busIdx].path.c_str(), O_RDWR);
    if (fd < 0) {
        ALOGD("open i2c is failed!");
        return;
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        ALOGD("Failed to acquire bus access and/or talk to slave.");
        return;
    }

    const auto ctx = std::make_shared<i2cContext>();

    ctx->fd = fd;
    ctx->busIdx = busIdx;
    ctx->deviceAddress = address;

    getProperty(ctx);

    i2c.insert(std::make_pair(idx, ctx));
}

void I2c::close(const int idx) {
    const auto ctx = getCtx(idx);
    ::close(ctx->fd);
    i2c.erase(idx);
}

const std::vector<uint8_t> I2c::read(const int idx, const int length) {
    const auto ctx = getCtx(idx);

    uint8_t *buffer = new uint8_t[length];

    ::read(ctx->fd, buffer, length);

    std::vector<uint8_t> result(buffer, buffer+length);
    delete[] buffer;

    return result;
}

const std::vector<uint8_t> I2c::readRegBuffer(const int idx,
        const uint32_t regAddress, const int length) {
    const auto ctx = getCtx(idx);

    uint8_t *buffer = new uint8_t[length];

    int regSize = getRegBufSize(ctx, regAddress);
    uint8_t *regBuf = getRegBuffer(regSize, regAddress);

    ::write(ctx->fd, regBuf, regSize);
    ::read(ctx->fd, buffer, length);

    std::vector<uint8_t> result(buffer, buffer+length);
    delete[] buffer;
    delete[] regBuf;

    return result;
}

uint16_t I2c::readRegWord(const int idx, const uint32_t regAddress) {
    const auto ctx = getCtx(idx);

    uint8_t buffer[2];

    int regSize = getRegBufSize(ctx, regAddress);
    uint8_t *regBuf = getRegBuffer(regSize, regAddress);

    ::write(ctx->fd, regBuf, regSize);
    ::read(ctx->fd, buffer, 2);

    delete[] regBuf;

    return ((uint16_t)(buffer[1] << 8) + buffer[0]);
}

uint8_t I2c::readRegByte(const int idx, const uint32_t regAddress) {
    const auto ctx = getCtx(idx);

    uint8_t buffer;

    int regSize = getRegBufSize(ctx, regAddress);
    uint8_t *regBuf = getRegBuffer(regSize, regAddress);

    ::write(ctx->fd, regBuf, regSize);
    ::read(ctx->fd, &buffer, 1);

    delete[] regBuf;

    return buffer;
}

Result I2c::write(const int idx, std::vector<uint8_t> transferData,
        const int length) {
    const auto ctx = getCtx(idx);

    uint8_t *buffer = new uint8_t[length];

    std::copy(transferData.begin(), transferData.end(), buffer);

    ::write(ctx->fd, buffer, length);

    delete[] buffer;

    return Result::OK;
}

Result I2c::writeRegBuffer(const int idx, const uint32_t regAddress,
        std::vector<uint8_t> transferData, const int length) {
    const auto ctx = getCtx(idx);

    int regSize = getRegBufSize(ctx, regAddress);
    uint8_t *regBuf = getRegBuffer(regSize, regAddress);

    uint8_t *buffer = new uint8_t[length + regSize];

    for (int i = 0; i < regSize; i++)
        buffer[i] = regBuf[i];

    std::copy(transferData.begin(), transferData.end(), buffer+regSize);

    ::write(ctx->fd, buffer, length + regSize);

    delete[] buffer;

    return Result::OK;
}

Result I2c::writeRegWord(const int idx, const uint32_t regAddress,
        uint16_t transferData) {
    const auto ctx = getCtx(idx);

    int regSize = getRegBufSize(ctx, regAddress);
    uint8_t *regBuf = getRegBuffer(regSize, regAddress);

    uint8_t *buffer = new uint8_t[sizeof(uint16_t) + regSize];

    int i;
    for (i = 0; i < regSize; i++)
        buffer[i] = regBuf[i];

    buffer[i] = (uint8_t)(transferData & 0xFF);
    transferData >>= 8;
    i++;
    buffer[i] = (uint8_t)(transferData & 0xFF);

    ::write(ctx->fd, buffer, sizeof(uint16_t) + regSize);

    delete[] buffer;

    return Result::OK;
}

Result I2c::writeRegByte(const int idx, const uint32_t regAddress,
        uint8_t transferData) {
    const auto ctx = getCtx(idx);

    int regSize = getRegBufSize(ctx, regAddress);
    uint8_t *regBuf = getRegBuffer(regSize, regAddress);

    uint8_t *buffer = new uint8_t[sizeof(uint8_t) + regSize];

    int i;
    for (i = 0; i < regSize; i++)
        buffer[i] = regBuf[i];
    buffer[i] = transferData;

    ::write(ctx->fd, buffer, sizeof(uint8_t) + regSize);

    delete[] buffer;

    return Result::OK;
}
