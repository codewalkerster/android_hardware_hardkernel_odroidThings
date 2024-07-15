#include "OdroidM2.h"

OdroidM2::OdroidM2() {
    pinList = {
        {"", -1, 0},
        {"3.3V", -1, PIN_PWR}, {"5V", -1, PIN_PWR},
        {"3", 8, PIN_GPIO|PIN_I2C}, {"5V", -1, PIN_PWR},
        {"5", 9, PIN_GPIO|PIN_I2C}, {"GND", -1, PIN_GND},
        {"7", 7, PIN_GPIO|PIN_PWM}, {"8", 15, PIN_GPIO|PIN_UART},
        {"GND", -1, PIN_GND}, {"10", 16, PIN_GPIO|PIN_UART},
        {"11", 0, PIN_GPIO|PIN_UART}, {"12", 1, PIN_GPIO|PIN_PWM},
        {"13", 2, PIN_GPIO|PIN_UART}, {"GND", -1, PIN_GND},
        {"15", 3, PIN_GPIO|PIN_PWM}, {"16", 4, PIN_GPIO|PIN_UART},
        {"3.3V", -1, PIN_PWR}, {"18", 5, PIN_GPIO|PIN_UART},
        {"19", 12, PIN_GPIO|PIN_SPI}, {"GND", -1, PIN_GND},
        {"21", 13, PIN_GPIO|PIN_SPI}, {"22", 6, PIN_GPIO|PIN_I2C},
        {"23", 14, PIN_GPIO|PIN_SPI}, {"24", 10, PIN_GPIO|PIN_SPI|PIN_I2C},
        {"25", -1, PIN_GND}, {"26", 11, PIN_GPIO},
        {"27", 30, PIN_GPIO|PIN_I2C|PIN_UART}, {"28", 31, PIN_GPIO|PIN_I2C|PIN_UART},
        {"29", 21, PIN_GPIO|PIN_UART}, {"GND", -1, PIN_GND},
        {"31", 22, PIN_GPIO|PIN_UART}, {"32", 26, PIN_GPIO},
        {"33", 23, PIN_GPIO|PIN_PWM}, {"GND", -1, PIN_GND},
        {"35", 24, PIN_GPIO}, {"36", 27, PIN_GPIO},
        {"AIN3", 25, PIN_AIN}, {"1.8V", 28, PIN_PWR},
    };

    i2cList = {
        {"I2C-1", "/dev/i2c-0", 8, 9}, // Pin #3, #5
        {"I2C-2", "/dev/i2c-1", 30, 31}, // Pin #27, #28
        {"I2C-3", "/dev/i2c-2", 6, 10}, // Pin #22, #24
    };

    pwmList = {
        { // Pin #7
            .index = 7,
            .path = "/sys/devices/platform/febd0030.pwm",
            .line = 0
        },
        { // Pin #12
            .index = 1,
            .path = "/sys/devices/platform/fd8b0030.pwm",
            .line = 0
        },
        { // Pin #15
            .index = 3,
            .path =  "/sys/devices/platform/febf0030.pwm",
            .line = 0
        },
        { // Pin #33
            .index = 23,
            .path = "/sys/devices/platform/febe0000.pwm",
            .line = 0
        },
    };

    uartList = {
        {"UART-1", "/dev/ttyS0", 16, 15}, // Pin #10, #8
        {"UART-2", "/dev/ttyS1", 0, 2}, // Pin #11, #13
        {"UART-3", "/dev/ttyS2", 4, 5}, // Pin #16, #18
        {"UART-4", "/dev/ttyS3", 31, 30}, // Pin #28, #27
        {"UART-5", "/dev/ttyS4", 21, 22}, // Pin #29, #31
        {"UART-USB-0", "/dev/ttyUSB0", -1, -1},
        {"UART-USB-1", "/dev/ttyUSB1", -1, -1},
        {"UART-ACM-0", "/dev/ttyACM0", -1, -1},
        {"UART-ACM-1", "/dev/ttyACM1", -1, -1},
    };

    spiList = { // Pin #19, #21, #23
        {"SPI0.0", "/dev/spidev0.0", 14, 12, 13, 10}, // Pin #24
    };

    Board("odroidm2");
}
