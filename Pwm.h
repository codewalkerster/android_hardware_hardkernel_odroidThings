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

#ifndef PWM_H_
#define PWM_H_

#include "Board.h"
#include <map>

#define HERTZTONANOSECOND 1000000000

struct pwmContext {
    unsigned int period;
    double cycle_rate;

    uint8_t chip;
    uint8_t node;
    std::string periodPath;
    std::string dutyCyclePath;
    std::string enablePath;
    std::string unexportPath;
};

using boardPtr = std::shared_ptr<Board>;
using pwmCtxPtr = std::shared_ptr<pwmContext>;

class Pwm {
    private:
        boardPtr board;
        std::map<int, pwmCtxPtr> pwm;

        void initContext(int, uint8_t, uint8_t);
        inline pwmCtxPtr getCtx(int);

    public:
        Pwm(boardPtr);
        std::vector<std::string> getList();
        void open(int);
        void close(int);
        bool setEnable(int, bool);
        bool setDutyCycle(int, double);
        bool setFrequency(int, double);
};
#endif /* PWM_H_ */
