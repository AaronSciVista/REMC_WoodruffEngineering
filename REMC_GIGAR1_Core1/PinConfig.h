// PinConfig.h
#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// Digital Inputs
#define PIN_ACTUATE      D7
#define PIN_ARM          D3
#define PIN_MSW_POS_A    D32
#define PIN_MSW_POS_B    D34

// Digital Outputs
#define PIN_EM_ACT       D29  // EM Toggle
#define PIN_READY        D4
#define PIN_LIN_ACT_A    D27  // Engage
#define PIN_LIN_ACT_B    D25  // Disengage 
#define PIN_MSW_A_OUT    D5   
#define PIN_MSW_B_OUT    D6   

// Analog Inputs (scaled to 3.3 V externally)
#define PIN_SWITCH_CURRENT       A3
#define PIN_SWITCH_VOLTAGE       A6
#define PIN_TEMP_1               A2
#define PIN_OUTPUT_VOLTAGE_A     A4
#define PIN_OUTPUT_VOLTAGE_B     A5

// Analog Output Pins (PWM) ***NOT USED***
// #define PIN_VOLTAGE_A_OUT        D9
// #define PIN_VOLTAGE_B_OUT        D10
// #define PIN_VOLTAGE_SWITCH_OUT   D11
// #define PIN_CURRENT_SWITCH_OUT   D12

#endif // PIN_CONFIG_H