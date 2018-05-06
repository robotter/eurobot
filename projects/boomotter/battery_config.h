
/// ADC used for battery
#define BATTERY_ADC   ADCA
/// Pin used by ADC to read voltage value
#define BATTERY_ADC_PORTPIN PORTPIN(B,2)
/// ADC mux configuration
#define BATTERY_ADC_MUX   ADC_CH_MUXPOS_PIN10_gc

/// Linear transformation of the raw ADC value
///@{
#define BATTERY_VOLTAGE_COEF  6.61
#define BATTERY_VOLTAGE_OFFSET   3495
///@}

