#include "Pwm.h"
#include <cutils/log.h>
#include <fstream>
#include <sstream>

Pwm::Pwm(std::shared_ptr<Board> board): board(board) {
    auto list = board->getPwmList();
    for (auto pin = list.begin(); pin != list.end(); pin++)
        initState(pin->index, pin->chip, pin->line);
}

void Pwm::initState(int idx, uint8_t chip, uint8_t node) {
    const auto state = std::make_shared<pwmState>();
    // init chip & node info
    state->chip = chip;
    state->node = node;

    // init configuration sysfs path.
    std::ostringstream pwmRoot;

    pwmRoot << "/sys/class/pwm/pwmchip" << std::to_string(state->chip);
    if (access(pwmRoot.str().c_str(), F_OK) == 0) {
        pwmRoot << "/pwm" << std::to_string(state->node) <<"/";
        std::string pwmRootStr = pwmRoot.str();

        state->periodPath =  pwmRootStr + "period";
        state->dutyCyclePath = pwmRootStr + "duty_cycle";
        state->enablePath = pwmRootStr + "enable";

        pwm.insert(std::make_pair(idx, state));
    }
}
inline void Pwm::writeSysfsTo(const std::string path, const std::string value) {
    std::ofstream file(path);
    file << value;
    file.close();
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
    auto pin = board->getPin(idx);
    const auto state = pwm.find(pin)->second;
    std::ostringstream openPath;

    ALOGD("openPwm(chip:%d-node:%d)", state->chip, state->node);

    openPath << "/sys/class/pwm/pwmchip" << std::to_string(state->chip) << "/";

    writeSysfsTo(openPath.str() + "export", std::to_string(state->node));

    state->unexportPath = openPath.str() + "unexport";
    openPath << "pwm" << std::to_string(state->node) << "/";
    writeSysfsTo(openPath.str() + "polarity", "normal");
}

void Pwm::close(int idx) {
    auto pin = board->getPin(idx);
    const auto state = pwm.find(pin)->second;
    state->period = 0;
    state->cycle_rate = 0;

    ALOGD("closePwm(chip:%d-node:%d)", state->chip, state->node);

    writeSysfsTo(state->enablePath, "0");
    writeSysfsTo(state->dutyCyclePath, "0");
    writeSysfsTo(state->periodPath, "0");
    writeSysfsTo(state->unexportPath, std::to_string(state->node));
}

bool Pwm::setEnable(int idx, bool enabled) {
    auto pin = board->getPin(idx);
    const auto state = pwm.find(pin)->second;

    writeSysfsTo(state->enablePath, (enabled?"1":"0"));
    return true;
}

bool Pwm::setDutyCycle(int idx, double cycle_rate) {
    auto pin = board->getPin(idx);
    const auto state = pwm.find(pin)->second;
    unsigned int duty_cycle = (state->period / 100) * cycle_rate;

    writeSysfsTo(state->dutyCyclePath, std::to_string(duty_cycle));

    state->cycle_rate = cycle_rate;

    return true;
}

bool Pwm::setFrequency(int idx, double frequency_hz) {
    auto pin = board->getPin(idx);
    const auto state = pwm.find(pin)->second;

    state->period = HERTZTONANOSECOND / frequency_hz;

    writeSysfsTo(state->periodPath, std::to_string(state->period));

    if (state->cycle_rate != 0) {
        setDutyCycle(idx, state->cycle_rate);
    }

    return true;
}
