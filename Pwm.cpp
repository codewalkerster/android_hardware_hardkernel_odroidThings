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

#include "Pwm.h"
#include "Helper.h"
#include <cutils/log.h>
#include <fstream>
#include <sstream>
#include <dirent.h>

using Helper::writeSysfsTo;

Pwm::Pwm(boardPtr board) {
    auto list = board->getPwmList();
    for (auto pin = list.begin(); pin != list.end(); pin++)
        initContext(pin->index, pin->path, pin->line);

    auto pinList = board->getPinList();
    for (unsigned long i=0; i<pinList.size(); i++)
        if (pinList[i].availableModes & PIN_PWM)
            if (pwm.count(pinList[i].pin) > 0)
                pwmList.insert(std::make_pair(i, pinList[i]));
}

std::string Pwm::getRootPath(std::string path) {
    std::ostringstream pwmRoot;
    pwmRoot << path << "/pwm";
    struct dirent *root;
    DIR *dir = opendir(pwmRoot.str().c_str());
    root = readdir(dir);
    pwmRoot << "/" << root->d_name;
    closedir(dir);

    return pwmRoot.str();
}

void Pwm::initContext(int idx, std::string rootPath, uint8_t node) {
    const auto ctx = std::make_shared<pwmContext>();

    if (access(rootPath.c_str(), F_OK) == 0) {
        std::ostringstream pwmRoot;
        ctx->rootPath = getRootPath(rootPath);

        pwmRoot << ctx->rootPath;
        pwmRoot << "/pwm" << std::to_string(node) <<"/";
        std::string pwmRootStr = pwmRoot.str();

        ctx->node = node;

        ctx->periodPath =  pwmRootStr + "period";
        ctx->dutyCyclePath = pwmRootStr + "duty_cycle";
        ctx->enablePath = pwmRootStr + "enable";

        pwm.insert(std::make_pair(idx, ctx));
    }
}

inline pwmCtxPtr Pwm::getCtx(int idx) {
    return pwm [pwmList[idx].pin];
}

std::vector<std::string> Pwm::getList() {
    static std::vector<std::string> list;

    if (list.size())
        return list;

    for (auto it = pwmList.begin(); it != pwmList.end(); ++it)
        list.push_back (it->second.name);

    return list;
}

void Pwm::open(int idx) {
    const auto ctx = getCtx(idx);
    std::ostringstream openPath;

    openPath << ctx->rootPath << "/";

    writeSysfsTo(openPath.str() + "export", std::to_string(ctx->node));

    ctx->unexportPath = openPath.str() + "unexport";
    openPath << "pwm" << std::to_string(ctx->node) << "/";
    writeSysfsTo(openPath.str() + "polarity", "normal");
}

void Pwm::close(int idx) {
    const auto ctx = getCtx(idx);
    ctx->period = 0;
    ctx->cycle_rate = 0;

    writeSysfsTo(ctx->enablePath, "0");
    writeSysfsTo(ctx->dutyCyclePath, "0");
    writeSysfsTo(ctx->periodPath, "0");
    writeSysfsTo(ctx->unexportPath, std::to_string(ctx->node));
}

bool Pwm::setEnable(int idx, bool enabled) {
    const auto ctx = getCtx(idx);

    writeSysfsTo(ctx->enablePath, (enabled?"1":"0"));
    return true;
}

bool Pwm::setDutyCycle(int idx, double cycle_rate) {
    const auto ctx = getCtx(idx);
    unsigned int duty_cycle = (ctx->period / 100) * cycle_rate;

    writeSysfsTo(ctx->dutyCyclePath, std::to_string(duty_cycle));

    ctx->cycle_rate = cycle_rate;

    return true;
}

bool Pwm::setFrequency(int idx, double frequency_hz) {
    const auto ctx = getCtx(idx);

    ctx->period = HERTZTONANOSECOND / frequency_hz;

    writeSysfsTo(ctx->periodPath, std::to_string(ctx->period));

    if (ctx->cycle_rate != 0) {
        setDutyCycle(idx, ctx->cycle_rate);
    }

    return true;
}
