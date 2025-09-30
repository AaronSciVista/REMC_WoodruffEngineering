// PinConfig.h
#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// Fast GPIO helpers for STM32H747 (Arduino Giga)
#include "stm32h7xx.h"

// Digital Inputs
#define PIN_ACTUATE      D7
#define PIN_ARM          D3
#define PIN_MSW_POS_A    D51
#define PIN_MSW_POS_B    D53

// Digital Outputs
#define PIN_EM_ACT       D29  // EM Toggle
#define PIN_READY        D4
#define PIN_LIN_ACT_A    D27  // Engage
#define PIN_LIN_ACT_B    D25  // Disengage 
#define PIN_MSW_A_OUT    D5   
#define PIN_MSW_B_OUT    D6   

// Analog Inputs (scaled to 3.3 V externally)
#define PIN_SWITCH_CURRENT       A3
#define PIN_SWITCH_VOLTAGE       A6
#define PIN_TEMP_1               A2
#define PIN_OUTPUT_VOLTAGE_A     A4
#define PIN_OUTPUT_VOLTAGE_B     A5

// Map Arduino pins to STM32 ports/bits
#define MSW_A_GPIO   GPIOE
#define MSW_A_BIT    5u      // D51 → PE5
#define MSW_B_GPIO   GPIOG
#define MSW_B_BIT    7u      // D53 → PG7

// Read ACTIVE-LOW switch states, directly from the ports:
static inline bool mswA_low_fast() {
  return (MSW_A_GPIO->IDR & (1u << MSW_A_BIT)) == 0;
}
static inline bool mswB_low_fast() {
  return (MSW_B_GPIO->IDR & (1u << MSW_B_BIT)) == 0;
}

// Batch read (most deterministic): 1 read per port, then mask
static inline void msw_read_both_fast(bool &a_low, bool &b_low) {
  uint32_t e = MSW_A_GPIO->IDR;  // snapshot Port E (PE0..PE15)
  uint32_t g = MSW_B_GPIO->IDR;  // snapshot Port G (PG0..PG15)
  a_low = (e & (1u << MSW_A_BIT)) == 0;
  b_low = (g & (1u << MSW_B_BIT)) == 0;
}

#endif // PIN_CONFIG_H
