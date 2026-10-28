#include "samd51_sensing.h"
#include "../../../drivers/hardware_specific/samd/samd_mcu.h"

#if defined(_SAMD51_) || defined(_SAME51_)

#include "../../hardware_api.h"

#define _SAMD51_ADC_VOLTAGE 3.3f
#define _SAMD51_ADC_RESOLUTION 4095.0f
#define _SAMD51_VSENSE_DIVIDER (29.04f / 2.04f)

#define SENSING_PIN_IA 0
#define SENSING_PIN_IB 1
#define SENSING_PIN_VA 3
#define SENSING_PIN_VB 4
#define SENSING_PIN_VC 5
#define SENSING_PIN_VM 6
#define SENSING_PIN_COUNT 7

static int sensing_pins[SENSING_PIN_COUNT] = { _NC, _NC, _NC, _NC, _NC, _NC, _NC };
static volatile uint16_t sensing_raw[SENSING_PIN_COUNT] = { 0, 0, 0, 0, 0, 0, 0 };
static volatile float sensing_filtered[SENSING_PIN_COUNT] = { 0, 0, 0, 0, 0, 0, 0 };
static volatile uint64_t sensing_timestamp_us[SENSING_PIN_COUNT] = { 0, 0, 0, 0, 0, 0, 0 };
static volatile uint8_t sensing_count = 0;
static volatile uint8_t sensing_current_count = 0;
static volatile uint8_t sensing_voltage_count = 0;
static volatile uint8_t sensing_vm_count = 0;
static volatile bool sensing_initialized = false;
static uint32_t sensing_last_micros = 0;
static uint64_t sensing_micros64 = 0;

static const uint8_t sensing_adc0_pins[] = { SENSING_PIN_VA, SENSING_PIN_VB, SENSING_PIN_VC, SENSING_PIN_VM };
static const uint8_t sensing_adc1_pins[] = { SENSING_PIN_IA, SENSING_PIN_IB };
static volatile uint8_t sensing_adc0_index = 0;
static volatile uint8_t sensing_adc1_index = 0;
static volatile bool sensing_skip_first_adc0 = true;
static volatile bool sensing_skip_first_adc1 = true;

static uint64_t sensingNowUs() {
  uint32_t now = (uint32_t)micros();
  sensing_micros64 += (uint32_t)(now - sensing_last_micros);
  sensing_last_micros = now;
  return sensing_micros64;
}

static uint8_t sensingADC1Channel(int pin) {
  if (pin == A2) return 0; // PB08, ADC1 AIN0
  if (pin == A3) return 1; // PB09, ADC1 AIN1
  return 0;
}

static void sensingSetMux(Adc* adc, uint8_t pin_index, bool adc1) {
  while (adc->SYNCBUSY.bit.INPUTCTRL);
  adc->INPUTCTRL.bit.MUXPOS = adc1
    ? sensingADC1Channel(sensing_pins[pin_index])
    : g_APinDescription[sensing_pins[pin_index]].ulADCChannelNumber;
}

static void sensingServiceADC(Adc* adc, const uint8_t* pin_map, uint8_t pin_count,
                              volatile uint8_t* index, volatile bool* skip_first, bool adc1) {
  if (!adc->INTFLAG.bit.RESRDY) return;
  uint16_t result = adc->RESULT.reg;
  adc->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  if (*skip_first) {
    *skip_first = false;
    return;
  }
  if (pin_count == 0) return;

  uint64_t sample_us = sensingNowUs();
  if (*index >= pin_count) *index = 0;
  uint8_t pin_index = pin_map[*index];
  sensing_raw[pin_index] = result;
  if (sensing_timestamp_us[pin_index] == 0) {
    sensing_filtered[pin_index] = result;
  } else {
    sensing_filtered[pin_index] += 0.02f * ((float)result - sensing_filtered[pin_index]);
  }
  sensing_timestamp_us[pin_index] = sample_us;
  (*index)++;
  if (*index >= pin_count) *index = 0;
  sensingSetMux(adc, pin_map[*index], adc1);
}

void _serviceSAMD51Sensing() {
  sensingServiceADC(ADC0, sensing_adc0_pins, sizeof(sensing_adc0_pins),
                    &sensing_adc0_index, &sensing_skip_first_adc0, false);
  sensingServiceADC(ADC1, sensing_adc1_pins, sizeof(sensing_adc1_pins),
                    &sensing_adc1_index, &sensing_skip_first_adc1, true);
}

extern "C" void ADC0_0_Handler(void) {
  _serviceSAMD51Sensing();
}

void* _configureADCLowSide(const void* driver_params, const int pinA, const int pinB, const int pinC) {
  _UNUSED(driver_params);

  if (_isset(sensing_pins[0]) || _isset(sensing_pins[1]) || _isset(sensing_pins[2])) {
    SIMPLEFOC_DEBUG("SAMD51-CUR: ERR: Pins already in use for current sensing!");
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
  }

  int pins[3] = { pinA, pinB, pinC };
  for (uint8_t i = 0; i < 3; i++) {
    if (_isset(pins[i])) {
      pinMode(pins[i], INPUT);
      pinPeripheral(pins[i], PIO_ANALOG);
    }
    sensing_pins[i] = pins[i];
  }
  sensing_current_count = _isset(pinC) ? 3 : 2;
  sensing_count = sensing_current_count + sensing_voltage_count + sensing_vm_count;

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = _SAMD51_ADC_VOLTAGE / _SAMD51_ADC_RESOLUTION,
  };
  return params;
}

float _readADCVoltageLowSide(const int pin, const void* cs_params) {
  _serviceSAMD51Sensing();
  float countsToVolts = ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
  for (uint8_t i = 0; i < 3; i++) {
    if (sensing_pins[i] == pin) return sensing_raw[i] * countsToVolts;
  }
  return 0.0f;
}

void _configureSAMD51VoltageSense(int pinA, int pinB, int pinC) {
  int pins[3] = { pinA, pinB, pinC };
  for (uint8_t i = 0; i < 3; i++) {
    if (_isset(pins[i])) {
      pinMode(pins[i], INPUT);
      pinPeripheral(pins[i], PIO_ANALOG);
    }
    sensing_pins[3 + i] = pins[i];
  }
  sensing_voltage_count = 3;
  sensing_count = sensing_current_count + sensing_voltage_count + sensing_vm_count;
}

void _configureSAMD51VMSense(int pin) {
  if (_isset(pin)) {
    pinMode(pin, INPUT);
    pinPeripheral(pin, PIO_ANALOG);
  }
  sensing_pins[SENSING_PIN_VM] = pin;
  sensing_vm_count = 1;
  sensing_count = sensing_current_count + sensing_voltage_count + sensing_vm_count;
}

float _readSAMD51VoltageSense(const int pin) {
  _serviceSAMD51Sensing();
  for (uint8_t i = 3; i < 6; i++) {
    if (sensing_pins[i] == pin) return sensing_filtered[i] * (_SAMD51_ADC_VOLTAGE / _SAMD51_ADC_RESOLUTION) * _SAMD51_VSENSE_DIVIDER;
  }
  return 0.0f;
}

float _readSAMD51VMSense(const int pin) {
  _serviceSAMD51Sensing();
  if (sensing_pins[SENSING_PIN_VM] == pin) return sensing_filtered[SENSING_PIN_VM] * (_SAMD51_ADC_VOLTAGE / _SAMD51_ADC_RESOLUTION) * _SAMD51_VSENSE_DIVIDER;
  return 0.0f;
}

uint64_t _getSAMD51SensingTimestamp(const int pin) {
  for (uint8_t i = 0; i < SENSING_PIN_COUNT; i++) {
    if (sensing_pins[i] == pin) return sensing_timestamp_us[i];
  }
  return 0;
}

static void sensingConfigureADC(Adc* adc, uint8_t pin_index, bool adc1) {
  adc->CTRLA.bit.ENABLE = 0;
  while (adc->SYNCBUSY.bit.ENABLE);

  adc->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV8 | ADC_CTRLA_ONDEMAND;
  adc->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;
  adc->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT;
  adc->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0x10);
  adc->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;
  while (adc->SYNCBUSY.bit.CTRLB);
  while (adc->SYNCBUSY.bit.REFCTRL);
  while (adc->SYNCBUSY.bit.SAMPCTRL);
  while (adc->SYNCBUSY.bit.AVGCTRL);

  uint8_t channel = adc1
    ? sensingADC1Channel(sensing_pins[pin_index])
    : g_APinDescription[sensing_pins[pin_index]].ulADCChannelNumber;
  while (adc->SYNCBUSY.bit.INPUTCTRL);
  adc->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS(channel) | ADC_INPUTCTRL_MUXNEG_GND;
  while (adc->SYNCBUSY.bit.INPUTCTRL);

  adc->EVCTRL.reg = ADC_EVCTRL_STARTEI;
}

void* _driverSyncLowSide(void* driver_params, void* cs_params) {
  if (sensing_count == 0) return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;

  uint8_t tcc_num = 0;
  const SAMDHardwareDriverParams* par = (const SAMDHardwareDriverParams*)driver_params;
  if (par != nullptr && par->tccPinConfigurations[0] != nullptr) {
    tcc_num = par->tccPinConfigurations[0]->tcc.tccn;
  }

  Tcc* tcc = TCC0;
  uint8_t evgen = EVSYS_ID_GEN_TCC0_OVF;
  if (tcc_num == 1) {
    tcc = TCC1;
    evgen = EVSYS_ID_GEN_TCC1_OVF;
  } else if (tcc_num == 2) {
    tcc = TCC2;
    evgen = EVSYS_ID_GEN_TCC2_OVF;
  }

  MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1;
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2;

  GCLK->PCHCTRL[TCC2_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

  sensingConfigureADC(ADC0, SENSING_PIN_VA, false);
  sensingConfigureADC(ADC1, SENSING_PIN_IA, true);
  ADC0->INTENSET.reg = ADC_INTENSET_RESRDY;

  EVSYS->Channel[0].CHANNEL.reg = EVSYS_CHANNEL_EVGEN(evgen) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS;
  EVSYS->USER[EVSYS_ID_USER_ADC1_START].reg = EVSYS_USER_CHANNEL(1);
  EVSYS->Channel[1].CHANNEL.reg = EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TCC2_OVF) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS;
  EVSYS->USER[EVSYS_ID_USER_ADC0_START].reg = EVSYS_USER_CHANNEL(2);

  tcc->CTRLA.bit.ENABLE = 0;
  while (tcc->SYNCBUSY.bit.ENABLE);
  tcc->EVCTRL.reg |= TCC_EVCTRL_OVFEO;
  tcc->CTRLA.bit.ENABLE = 1;
  while (tcc->SYNCBUSY.bit.ENABLE);

  TCC2->CTRLA.bit.ENABLE = 0;
  while (TCC2->SYNCBUSY.bit.ENABLE);
  TCC2->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1;
  TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NFRQ;
  while (TCC2->SYNCBUSY.bit.WAVE);
  TCC2->PER.reg = 2499;
  while (TCC2->SYNCBUSY.bit.PER);
  TCC2->EVCTRL.reg = TCC_EVCTRL_OVFEO;
  TCC2->CTRLA.bit.ENABLE = 1;
  while (TCC2->SYNCBUSY.bit.ENABLE);

  sensing_adc0_index = 0;
  sensing_adc1_index = 0;
  sensing_skip_first_adc0 = true;
  sensing_skip_first_adc1 = true;
  ADC0->CTRLA.bit.ENABLE = 1;
  while (ADC0->SYNCBUSY.bit.ENABLE);
  ADC1->CTRLA.bit.ENABLE = 1;
  while (ADC1->SYNCBUSY.bit.ENABLE);

  sensing_initialized = true;
  return cs_params;
}

#endif
