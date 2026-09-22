#pragma once

#include <cstdint>
#include "api/Common.h"

#ifdef __cplusplus
namespace arduino {
    using ::PinStatus;
    using ::PinMode;

    constexpr PinStatus LOW = ::LOW;
    constexpr PinStatus HIGH = ::HIGH;
    constexpr PinStatus CHANGE = ::CHANGE;
    constexpr PinStatus FALLING = ::FALLING;
    constexpr PinStatus RISING = ::RISING;

    constexpr PinMode INPUT = ::INPUT;
    constexpr PinMode OUTPUT = ::OUTPUT;
    constexpr PinMode INPUT_PULLUP = ::INPUT_PULLUP;
    constexpr PinMode INPUT_PULLDOWN = ::INPUT_PULLDOWN;
    constexpr PinMode OUTPUT_OPENDRAIN = ::OUTPUT_OPENDRAIN;
}
#endif

// TOAD_H7 Board Pin Numbering (matching variant_TOAD_H7.h)
// Port A (0-15)
#define PA0 0
#define PA1 1
#define PA2 2
#define PA3 3
#define PA4 4
#define PA5 5
#define PA6 6
#define PA7 7
#define PA8 8
#define PA9 9
#define PA10 10
#define PA11 11
#define PA12 12
#define PA13 13
#define PA14 14
#define PA15 15

// Port B (16-31)
#define PB0 16
#define PB1 17
#define PB2 18
#define PB3 19
#define PB4 20
#define PB5 21
#define PB6 22
#define PB7 23
#define PB8 24
#define PB9 25
#define PB10 26
#define PB11 27
#define PB12 28
#define PB13 29
#define PB14 30
#define PB15 31

// Port C (32-45)
#define PC0 32
#define PC1 33
#define PC4 34
#define PC5 35
#define PC6 36
#define PC7 37
#define PC8 38
#define PC9 39
#define PC10 40
#define PC11 41
#define PC12 42
#define PC13 43
#define PC14 44
#define PC15 45

// Port D (46-61)
#define PD0 46
#define PD1 47
#define PD2 48
#define PD3 49
#define PD4 50
#define PD5 51
#define PD6 52
#define PD7 53
#define PD8 54
#define PD9 55
#define PD10 56
#define PD11 57
#define PD12 58
#define PD13 59
#define PD14 60
#define PD15 61

// Port E (62-77)
#define PE0 62
#define PE1 63
#define PE2 64
#define PE3 65
#define PE4 66
#define PE5 67
#define PE6 68
#define PE7 69
#define PE8 70
#define PE9 71
#define PE10 72
#define PE11 73
#define PE12 74
#define PE13 75
#define PE14 76
#define PE15 77

// Port F (78-93)
#define PF0 78
#define PF1 79
#define PF2 80
#define PF3 81
#define PF4 82
#define PF5 83
#define PF6 84
#define PF7 85
#define PF8 86
#define PF9 87
#define PF10 88
#define PF11 89
#define PF12 90
#define PF13 91
#define PF14 92
#define PF15 93

// Port G (94-109)
#define PG0 94
#define PG1 95
#define PG2 96
#define PG3 97
#define PG4 98
#define PG5 99
#define PG6 100
#define PG7 101
#define PG8 102
#define PG9 103
#define PG10 104
#define PG11 105
#define PG12 106
#define PG13 107
#define PG14 108
#define PG15 109

// Port H (110-125)
#define PH0 110
#define PH1 111
#define PH2 112
#define PH3 113
#define PH4 114
#define PH5 115
#define PH6 116
#define PH7 117
#define PH8 118
#define PH9 119
#define PH10 120
#define PH11 121
#define PH12 122
#define PH13 123
#define PH14 124
#define PH15 125

// Port I (126-138)
#define PI0 126
#define PI1 127
#define PI2 128
#define PI3 129
#define PI4 130
#define PI5 131
#define PI6 132
#define PI7 133
#define PI8 134
#define PI9 135
#define PI10 136
#define PI11 137
#define PI15 138

#define NUM_DIGITAL_PINS 141

// Special not connected pin
#define NC 0xFFFFFFFF
