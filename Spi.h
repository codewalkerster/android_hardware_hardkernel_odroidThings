/*
 *    Copyright (c) 2019 Sangchul Go <luke.go@hardkernel.com>
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

#ifndef SPI_H_
#define SPI_H_

#include <hardware/odroidThings.h>
#include <map>
#include <unistd.h>
#include <vector>

using hardware::hardkernel::odroidthings::spi_t;

struct spiState {
    int fd;

    uint8_t mode;
    uint32_t frequency;
    uint8_t bits;
    uint8_t csChange;
    uint16_t delay;
};

class Spi {
    private:
        std::vector<spi_t> spiList;
        std::map<int, spiState> spi;
        Spi();
        int applyMode(const int index);

    public:
        Spi(std::vector<spi_t> list);
        std::vector<std::string> getList();
        void open(const int index);
        void close(const int index);

        int setBitJustification(const int index, const uint8_t justification);
        int setBitsPerWord(const int index, const uint8_t bits);
        int setMode(const int index, const uint8_t mode);
        int setCsChange(const int index, const bool csChange);
        int setDelay(const int index, const uint16_t delay);
        int setFrequency(const int index, const uint32_t frequency);

        int read(const int index, const uint8_t *rx, const int length);
        int write(const int index, const uint8_t *tx, const int length);
        int transfer(const int index, const uint8_t *tx, const uint8_t *rx, const int length);
};
#endif /* SPI_H_ */
