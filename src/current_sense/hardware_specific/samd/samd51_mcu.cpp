#include "samd51_sensing.h"
#include "../../../drivers/hardware_specific/samd/samd_mcu.h"

#if defined(_SAMD51_) || defined(_SAME51_)

#include "../../hardware_api.h"

#define _SAMD51_ADC_VOLTAGE 3.3f
#define _SAMD51_ADC_RESOLUTION 4095.0f

static int sensing_pins[6] = { _NC, _NC, _NC, _NC, _NC, _NC };
static volatile uint16_t sensing_raw[6] = { 0, 0, 0, 0, 0, 0 };
static volatile uint8_t sensing_count = 0;
static volatile uint8_t sensing_index = 0;
static volatile bool sensing_initialized = false;
static volatile bool sensing_skip_first = true;

static void sensingSetMux(uint8_t index) {
  while (ADC0->SYNCBUSY.bit.INPUTCTRL);
  ADC0->INPUTCTRL.bit.MUXPOS = g_APinDescription[sensing_pins[index]].ulADCChannelNumber;
}

extern "C" void ADC0_0_Handler(void) {
  if (!ADC0->INTFLAG.bit.RESRDY) return;
  uint16_t result = ADC0->RESULT.reg;
  ADC0->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  if (sensing_skip_first) {
    sensing_skip_first = false;
    return;
  }
  if (sensing_count == 0) return;

  if (sensing_index >= sensing_count) sensing_index = 0;
  sensing_raw[sensing_index] = result;
  sensing_index++;
  if (sensing_index >= sensing_count) sensing_index = 0;
  sensingSetMux(sensing_index);
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
  sensing_count = 3;

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = _SAMD51_ADC_VOLTAGE / _SAMD51_ADC_RESOLUTION,
  };
  return params;
}

float _readADCVoltageLowSide(const int pin, const void* cs_params) {
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
  sensing_count = 6;
}

float _readSAMD51VoltageSense(const int pin) {
  for (uint8_t i = 3; i < 6; i++) {
    if (sensing_pins[i] == pin) return sensing_raw[i] * (_SAMD51_ADC_VOLTAGE / _SAMD51_ADC_RESOLUTION);
  }
  return 0.0f;
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
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC0;

  ADC0->CTRLA.bit.ENABLE = 0;
  while (ADC0->SYNCBUSY.bit.ENABLE);

  ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV8;
  ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;
  ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT;
  ADC0->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0x10);
  ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;
  while (ADC0->SYNCBUSY.bit.CTRLB);
  while (ADC0->SYNCBUSY.bit.REFCTRL);
  while (ADC0->SYNCBUSY.bit.SAMPCTRL);
  while (ADC0->SYNCBUSY.bit.AVGCTRL);

  while (ADC0->SYNCBUSY.bit.INPUTCTRL);
  ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS(g_APinDescription[sensing_pins[0]].ulADCChannelNumber)
                      | ADC_INPUTCTRL_MUXNEG_GND;
  while (ADC0->SYNCBUSY.bit.INPUTCTRL);

  ADC0->EVCTRL.reg = ADC_EVCTRL_STARTEI;
  ADC0->INTENSET.reg = ADC_INTENSET_RESRDY;

  EVSYS->Channel[0].CHANNEL.reg = EVSYS_CHANNEL_EVGEN(evgen) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS;
  EVSYS->USER[EVSYS_ID_USER_ADC0_START].reg = EVSYS_USER_CHANNEL(1);

  tcc->CTRLA.bit.ENABLE = 0;
  while (tcc->SYNCBUSY.bit.ENABLE);
  tcc->EVCTRL.reg |= TCC_EVCTRL_OVFEO;
  tcc->CTRLA.bit.ENABLE = 1;
  while (tcc->SYNCBUSY.bit.ENABLE);

  sensing_index = 0;
  sensing_skip_first = true;
  ADC0->CTRLA.bit.ENABLE = 1;
  while (ADC0->SYNCBUSY.bit.ENABLE);

  NVIC_ClearPendingIRQ(ADC0_0_IRQn);
  NVIC_SetPriority(ADC0_0_IRQn, 1);
  NVIC_EnableIRQ(ADC0_0_IRQn);

  sensing_initialized = true;
  return cs_params;
}

#endif
