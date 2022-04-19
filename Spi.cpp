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

#include "Spi.h"
#include <cutils/log.h>
#include <errno.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <mutex>

Spi::Spi() {
}

Spi::Spi(std::vector<spi_t> list) {
    spiList = list;
}

std::vector<std::string> Spi::getList() {
    std::vector<std::string> list;
    for(auto spi = spiList.begin(); spi != spiList.end(); spi++)
        list.push_back(spi->name);
    return list;
}

void Spi::open(const int index) {
    int fd = ::open(spiList[index].path.c_str(), O_RDWR);
    if (fd < 0) {
        ALOGE("open spi failed");
        return;
    }

    spiState state;
    state.fd = fd;

    //Set default value
    state.mode = 0;
    state.frequency = 1000000;
    state.bits = 8;
    state.csChange = 0;
    state.delay = 0;

    int ret = 0;

    ret = ioctl(state.fd, SPI_IOC_WR_MODE, &state.mode);
    ret = ioctl(state.fd, SPI_IOC_RD_MODE, &state.mode);

    ret = ioctl(state.fd, SPI_IOC_WR_BITS_PER_WORD, &state.bits);
    ret = ioctl(state.fd, SPI_IOC_RD_BITS_PER_WORD, &state.bits);

    ret = ioctl(state.fd, SPI_IOC_WR_MAX_SPEED_HZ, &state.frequency);
    ret = ioctl(state.fd, SPI_IOC_RD_MAX_SPEED_HZ, &state.frequency);

    spi.insert(std::make_pair(index, state));
}

void Spi::close(const int index) {
    const auto state = spi.find(index)->second;
    ::close(state.fd);
    spi.erase(index);
}

int Spi::setBitJustification(const int index, const uint8_t justification) {
    auto iterator = spi.find(index);

    switch (justification) {
        case 0: // MSB_FIRST
            iterator->second.mode &= ~SPI_LSB_FIRST;
            break;
        case 1: // LSB_FIRST
            iterator->second.mode |= SPI_LSB_FIRST;
            break;
    }

    return applyMode(index);
}

int Spi::setBitsPerWord(const int index, const uint8_t bits) {
    auto iterator = spi.find(index);
    iterator->second.bits = bits;
    int ret = 0;
    int fd = iterator->second.fd;

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1) {
        ALOGE("can't set bits per word");
        return ret;
    }

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
        ALOGE("can't get bits per word");

    return ret;
}

int Spi::setMode(const int index, const uint8_t mode) {
    auto iterator = spi.find(index);

    iterator->second.mode &= ~SPI_MODE_3;

    switch (mode) {
        case 0: // MODE_0
            iterator->second.mode |= SPI_MODE_0;
            break;
        case 1: // MODE_1
            iterator->second.mode |= SPI_MODE_1;
            break;
        case 2: // MODE_2
            iterator->second.mode |= SPI_MODE_2;
            break;
        case 3: // MODE_3
            iterator->second.mode |= SPI_MODE_3;
            break;
    }

    return applyMode(index);
}

int Spi::setCsChange(const int index, const bool csChange) {
    auto iterator = spi.find(index);

    iterator->second.csChange = csChange ? 1 : 0;

    if (csChange) {
        iterator->second.mode |= SPI_CS_HIGH;
    } else {
        iterator->second.mode &= ~SPI_CS_HIGH;
    }

    return applyMode(index);
}

int Spi::setDelay(const int index, const uint16_t delay) {
    auto iterator = spi.find(index);

    iterator->second.delay = delay;

    return 0;
}

int Spi::setFrequency(const int index, const uint32_t frequency) {
    auto iterator = spi.find(index);
    int fd = iterator->second.fd;
    iterator->second.frequency = frequency;
    int ret = 0;

    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &frequency);
    if (ret == -1) {
        ALOGE("Failed to set spi max speed");
        return ret;
    }

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &frequency);
    if (ret == -1) {
        ALOGE("Failed to get spi max speed");
    }

    return 0;
}

inline int Spi::applyMode(const int index) {
    auto state = spi.find(index)->second;
    int ret = 0;

    ret = ioctl(state.fd, SPI_IOC_WR_MODE, &state.mode);
    if (ret == -1) {
        ALOGE("Failed to set spi write mode");
        return ret;
    }

    ret = ioctl(state.fd, SPI_IOC_RD_MODE, &state.mode);
    if (ret == -1)
        ALOGE("Failed to get spi read mode");

    return ret;
}

#define SPI_LENGTH_MAX 512

int Spi::read(const int index, const uint8_t *rx, const int length) {
    const auto state = spi.find(index)->second;

    int left_length = length;
    int start_index = 0;

    spi_ioc_transfer trans;
    memset(&trans, 0, sizeof(trans));

    do {
        trans.rx_buf = (unsigned long)&rx[start_index];
        trans.len = (left_length > SPI_LENGTH_MAX)?
            SPI_LENGTH_MAX: left_length;
        trans.delay_usecs = state.delay;
        trans.speed_hz = state.frequency;
        trans.bits_per_word = state.bits;
        trans.cs_change = state.csChange;

        left_length -= SPI_LENGTH_MAX;
        start_index += SPI_LENGTH_MAX;

        int ret = ioctl(state.fd, SPI_IOC_MESSAGE(1), &trans);

        if (ret < 0) {
            std::string errMessage(strerror(errno));
            ALOGE("Failed to do read on SPI BUS %d, ret: (%d) Error message: %s",
                    state.fd, ret, errMessage.c_str());

            if (errno == 22) {
                ALOGE("length - %d, delay - %d, speed- %d, bits- %d, cs_change - %x",
                        length, state.delay, state.frequency, state.bits, state.csChange);
            }

            return ret;
        }
    } while (left_length > 0);

    return errno;
}

int Spi::write(const int index, const uint8_t *tx, const int length) {
    const auto state = spi.find(index)->second;

    int left_length = length;
    int start_index = 0;

    spi_ioc_transfer trans;
    memset(&trans, 0, sizeof(trans));

    do {
        trans.tx_buf = (unsigned long)&tx[start_index];
        trans.len = (left_length > SPI_LENGTH_MAX)?
            SPI_LENGTH_MAX: left_length;
        trans.delay_usecs = state.delay;
        trans.speed_hz = state.frequency;
        trans.bits_per_word = state.bits;
        trans.cs_change = state.csChange;

        left_length -= SPI_LENGTH_MAX;
        start_index += SPI_LENGTH_MAX;

        int ret = ioctl(state.fd, SPI_IOC_MESSAGE(1), &trans);

        if (ret < 0) {
            std::string errMessage(strerror(errno));
            ALOGE("Failed to do write on SPI BUS %d, ret: (%d) Error message: %s",
                    state.fd, ret, errMessage.c_str());

            if (errno == 22) {
                ALOGE("length - %d, delay - %d, speed- %d, bits- %d, cs_change - %x",
                        length, state.delay, state.frequency, state.bits, state.csChange);
            }

            return ret;
        }
    } while (left_length > 0);

    return errno;
}

int Spi::transfer(const int index, const uint8_t *tx, const uint8_t *rx, const int length) {
    const auto state = spi.find(index)->second;

    int left_length = length;
    int start_index = 0;

    spi_ioc_transfer trans;
    memset(&trans, 0, sizeof(trans));

    do {
        trans.tx_buf = (unsigned long)&tx[start_index];
        trans.rx_buf = (unsigned long)&rx[start_index];
        trans.len = (left_length > SPI_LENGTH_MAX)?
            SPI_LENGTH_MAX: left_length;
        trans.delay_usecs = state.delay;
        trans.speed_hz = state.frequency;
        trans.bits_per_word = state.bits;
        trans.cs_change = state.csChange;

        left_length -= SPI_LENGTH_MAX;
        start_index += SPI_LENGTH_MAX;

        int ret = ioctl(state.fd, SPI_IOC_MESSAGE(1), &trans);

        if (ret < 0) {
            std::string errMessage(strerror(errno));
            ALOGE("Failed to do transfer on SPI BUS %d, ret: (%d) Error message: %s",
                    state.fd, ret, errMessage.c_str());

            if (errno == 22) {
                ALOGE("length - %d, delay - %d, speed- %d, bits- %d, cs_change - %x",
                        length, state.delay, state.frequency, state.bits, state.csChange);
            }

            return ret;
        }
    } while (left_length > 0);

    return errno;
}
