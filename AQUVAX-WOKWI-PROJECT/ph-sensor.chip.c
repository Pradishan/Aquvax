// pH Sensor Chip implementation in C
// Filename: ph-sensor.chip.c

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// Define ADC parameters
#define VOLTAGE_REF 3.3
#define ADC_RESOLUTION 4095

// Chip pin definitions
enum {
  PIN_VCC,
  PIN_GND,
  PIN_DATA,
  PIN_COUNT
};

// Chip state
typedef struct {
  double ph_value;
  uint32_t adc_output;
} chip_state_t;

// Global state
static chip_state_t state = {
  .ph_value = 7.0,
  .adc_output = 0
};

// Update the ADC output based on pH value
static void update_adc_output() {
  // pH to voltage conversion (pH 7 = 2.5V, each pH unit = ~0.17V change)
  double voltage = 3.5 - (state.ph_value * 0.17);
  
  // Voltage to ADC value
  state.adc_output = (uint32_t)round((voltage / VOLTAGE_REF) * ADC_RESOLUTION);
  
  // Clamp to valid range
  if (state.adc_output > ADC_RESOLUTION) {
    state.adc_output = ADC_RESOLUTION;
  }
}

// Called when the chip is created
void chip_init() {
  printf("pH Sensor initialized with pH = %.1f\n", state.ph_value);
  update_adc_output();
}

// Called when a control value changes (pH slider)
bool chip_set_control(const char *name, double value) {
  if (strcmp(name, "ph-value") == 0) {
    // Clamp pH to valid range
    if (value < 0) value = 0;
    if (value > 14) value = 14;
    
    state.ph_value = value;
    update_adc_output();
    
    printf("pH changed to %.1f, ADC output: %u\n", state.ph_value, state.adc_output);
    return true;
  }
  return false;
}

// Called to get the value of a pin
uint32_t chip_pin_value(uint8_t pin) {
  if (pin == PIN_DATA) {
    return state.adc_output;
  }
  return 0;
}

// Called when a digital pin value changes (not used for this sensor)
void chip_on_pin_change(uint8_t pin, uint32_t value) {
  // This sensor doesn't have input pins that affect its behavior
}

// Define the pin mode (analog or digital)
uint8_t chip_pin_mode(uint8_t pin) {
  if (pin == PIN_DATA) {
    return 2; // 2 = analog output
  }
  return 0; // 0 = digital
}