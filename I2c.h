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

#ifndef I2C_H_
#define I2C_H_

#include <hardware/odroidThings.h>
#include <map>
#include <vector>

using hardware::hardkernel::odroidthings::i2c_t;

struct i2cState {
    int fd;
    int busIdx;
    uint32_t deviceAddress;
    uint8_t regBufSize;
};

using i2cStatePtr = std::shared_ptr<i2cState>;

#define I2C_REG_TWO_BYTE 0x10000000

class I2c {
    private:
        std::vector<i2c_t> i2cList;
        std::map<int, i2cStatePtr> i2c;
        I2c();

        inline void getProperty(const i2cStatePtr&);
        inline uint8_t getRegBufSize(const i2cStatePtr&, uint32_t);
        inline uint8_t *getRegBuffer(const uint8_t, uint32_t);
    public:
        I2c(std::vector<i2c_t> list);
        std::vector<std::string> getList();
        void open(const int, const uint32_t, const int);
        void close(const int);
        const std::vector<uint8_t> read(const int, const int);
        const std::vector<uint8_t> readRegBuffer(const int,
                const uint32_t, const int);
        uint16_t readRegWord(const int, const uint32_t);
        uint8_t readRegByte(const int, const uint32_t);
        Result write(const int, std::vector<uint8_t>, const int);
        Result writeRegBuffer(const int,
                const uint32_t, std::vector<uint8_t>, const int);
        Result writeRegWord(const int, const uint32_t, uint16_t);
        Result writeRegByte(const int, const uint32_t, uint8_t);
};
#endif /* I2C_H_ */
