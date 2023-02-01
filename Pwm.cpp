#include "Pwm.h"
#include <cutils/log.h>
#include <fstream>
#include <sstream>

Pwm::Pwm(boardPtr board): board(board) {
    auto list = board->getPwmList();
    for (auto pin = list.begin(); pin != list.end(); pin++)
        initContext(pin->index, pin->chip, pin->line);
}

void Pwm::initContext(int idx, uint8_t chip, uint8_t node) {
    const auto ctx = std::make_shared<pwmContext>();
    // init chip & node info
    ctx->chip = chip;
    ctx->node = node;

    // init configuration sysfs path.
    std::ostringstream pwmRoot;

    pwmRoot << "/sys/class/pwm/pwmchip" << std::to_string(ctx->chip);
    if (access(pwmRoot.str().c_str(), F_OK) == 0) {
        pwmRoot << "/pwm" << std::to_string(ctx->node) <<"/";
        std::string pwmRootStr = pwmRoot.str();

        ctx->periodPath =  pwmRootStr + "period";
        ctx->dutyCyclePath = pwmRootStr + "duty_cycle";
        ctx->enablePath = pwmRootStr + "enable";

        pwm.insert(std::make_pair(idx, ctx));
    }
}

inline void Pwm::writeSysfsTo(const std::string path, const std::string value) {
    std::ofstream file(path);
    file << value;
    file.close();
}

inline pwmCtxPtr Pwm::getCtx(int idx) {
    auto pin = board->getPin(idx);
    return pwm.find(pin)->second;
}

std::vector<std::string> Pwm::getList() {
    std::vector<std::string> list;
    auto pinList = board->getPinList();

    for (auto pin = pinList.begin(); pin != pinList.end(); pin++) {
        if (pin->availableModes & PIN_PWM) {
            if (pwm.count(pin->pin) > 0) {
                list.push_back(pin->name);
            }
        }
    }

    return list;
}

void Pwm::open(int idx) {
    const auto ctx = getCtx(idx);
    std::ostringstream openPath;

    ALOGD("openPwm(chip:%d-node:%d)", ctx->chip, ctx->node);

    openPath << "/sys/class/pwm/pwmchip" << std::to_string(ctx->chip) << "/";

    writeSysfsTo(openPath.str() + "export", std::to_string(ctx->node));

    ctx->unexportPath = openPath.str() + "unexport";
    openPath << "pwm" << std::to_string(ctx->node) << "/";
    writeSysfsTo(openPath.str() + "polarity", "normal");
}

void Pwm::close(int idx) {
    const auto ctx = getCtx(idx);
    ctx->period = 0;
    ctx->cycle_rate = 0;

    ALOGD("closePwm(chip:%d-node:%d)", ctx->chip, ctx->node);

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
