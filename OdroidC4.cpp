#include "OdroidC4.h"

OdroidC4::OdroidC4() {
    pinList = {
        {"", -1, 0},
        {"3.3V", -1, PIN_PWR}, {"5V", -1, PIN_PWR},
        {"3", 8, PIN_GPIO|PIN_I2C}, {"5V", -1, PIN_PWR},
        {"5", 9, PIN_GPIO|PIN_I2C}, {"GND", -1, PIN_GND},
        {"7", 7, PIN_GPIO}, {"8", 15, PIN_GPIO|PIN_UART},
        {"GND", -1, PIN_GND}, {"10", 16, PIN_GPIO|PIN_UART},
        {"11", 0, PIN_GPIO}, {"12", 1, PIN_GPIO|PIN_PWM},
        {"13", 2, PIN_GPIO}, {"GND", -1, PIN_GND},
        {"15", 3, PIN_GPIO|PIN_PWM|PIN_UART}, {"16", 4, PIN_GPIO},
        {"3.3V", -1, PIN_PWR}, {"18", 5, PIN_GPIO},
        {"19", 12, PIN_GPIO|PIN_SPI}, {"GND", -1, PIN_GND},
        {"21", 13, PIN_GPIO|PIN_SPI}, {"22", 6, PIN_GPIO},
        {"23", 14, PIN_GPIO|PIN_SPI}, {"24", 10, PIN_GPIO|PIN_SPI},
        {"25", -1, PIN_GND}, {"26", 11, PIN_GPIO|PIN_UART},
        {"27", 30, PIN_GPIO|PIN_I2C}, {"28", 31, PIN_GPIO|PIN_I2C},
        {"29", 21, PIN_GPIO}, {"GND", -1, PIN_GND},
        {"31", 22, PIN_GPIO}, {"32", 26, PIN_GPIO|PIN_UART},
        {"33", 23, PIN_GPIO|PIN_PWM|PIN_UART}, {"GND", -1, PIN_GND},
        {"35", 24, PIN_GPIO|PIN_PWM}, {"36", 27, PIN_GPIO},
        {"AIN3", 25, PIN_AIN}, {"1.8V", 28, PIN_PWR},
    };

    i2cList = {
        {"I2C-1", "/dev/i2c-0", "3", "5"},
        {"I2C-2", "/dev/i2c-1", "27", "28"},
    };

    pwmList = {
        {1, 4, 0}, // Pin #12
        {3, 4 ,1}, // Pin #15
        {23, 0, 0}, // Pin #33
        {24, 0, 1}, // Pin #35
    };

    uartList = {
        {"UART-1", "/dev/ttyS1", "10", "8"},
        {"UART-2", "/dev/ttyS2", "15", "35"},
        {"UART-3", "/dev/ttyS3", "26", "32"},
        {"UART-4", "/dev/ttyS4", "2", "6"},
    };

    spiList = {
        {"SPI0.0", "/dev/spidev0.0"},
    };

    Board("odroidc4");
}
