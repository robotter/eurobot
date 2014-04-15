#ifndef TCS3772X__
#define TCS3772X__

#include <i2c/i2c.h>
#include "tcs3772x_defs.h"

typedef struct{
  uint8_t I2cAddress;
  i2cm_t *I2c;
  uint16_t IntegrationTime_ms;
  uint8_t ProximityPulseNb;
}tcs3772x_t;

typedef struct{
  uint16_t clear_value;
  uint16_t red_value;
  uint16_t green_value;
  uint16_t blue_value;
 }tcs3772x_ColorResult_t;

/** @brief initialize the i2c port and the sensor
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @param i2c I2C peripheral may be i2cC i2cD, ...
 * @param model : must be one of the following models :37721, 37723, 37725 or 37727
 * @retval -2  error : bad model
 * @retval -1  error : i2c error
 * @retval 0   init ok
 */
int8_t tcs3772x_init(tcs3772x_t *Sensor, i2cm_t *i2c, uint16_t model);


/** @brief set integration time for RGBC 
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @param integration_time_ms integration time (in ms)
 * @retval -2 out of range IntegrationTime_ms value (must be between 2 and 614 ms)
 * @retval -1  error : i2c error
 * @retval 0   function ok
 * integration_time_ms is stored in structure and sent to the device
 */
int8_t tcs3772x_RGBCSetIntegrationTime_ms(tcs3772x_t *Sensor, uint16_t IntegrationTime_ms);


/** @brief set number of pulses sent during proximity detection state of fsm 
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @param proximity_pulse_nb number of pulse sent (muste be between 1 and 255) 
 * @retval -2  error : ppulse equal to 0 (illegal)
 * @retval -1  error : i2c error
 * @retval 0   function ok
 * proximity_pulse_nb is stored in structure and sent to the device
 */
int8_t tcs3772x_ProximitySetPulseNb(tcs3772x_t *Sensor, uint8_t proximityPulseNb);

/** @brief enable the sensor, wakes it from power off
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @retval -1  error : i2c error
 * @retval 0   function ok
 */
int8_t tcs3772x_Enable(tcs3772x_t *Sensor);

/** @brief disable the sensor, configure it in low power mode
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @retval -1  error : i2c error
 * @retval 0   function ok
 */
int8_t tcs3772x_Disable(tcs3772x_t *Sensor);

/** @brief enable the proximity detection 
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @retval -1  error : i2c error
 * @retval 0   function ok
 */
int8_t tcs3772x_ProximityEnableDetection(tcs3772x_t *Sensor);

/** @brief disable the proximity detection 
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @retval -1  error : i2c error
 * @retval 0   function ok
 */
int8_t tcs3772x_ProximityDisableDetection(tcs3772x_t *Sensor);

/** @brief enable the proximity detection interrupt
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @retval -1  error : i2c error
 * @retval 0   function ok
 */
int8_t tcs3772x_ProximityEnableInterrupt(tcs3772x_t *Sensor);

/** @brief disable the proximity detection interrupt
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @retval -1  error : i2c error
 * @retval 0   function ok
 */
int8_t tcs3772x_ProximityDisableInterrupt(tcs3772x_t *Sensor);

/** @brief Clear the pending proximity detection interrupt
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @retval -1  error : i2c error
 * @retval 0   function ok
 */
int8_t tcs3772x_ProximityClearInterrupt(tcs3772x_t *Sensor);

/** @brief return the result of a color conversion cycle
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @param result pointer to a tcs3772x_ColorResult_t strucutre that will contain the 
 * result of the color conversion cycle
 * @retval -1  error : i2c error
 * @retval 0   function ok
 * @remark stop proximity detection function, perform a color conversion and restart proximity detection function
 */
int8_t tcs3772x_RGBCGetValue(tcs3772x_t *Sensor, tcs3772x_ColorResult_t *result);

/** @brief set the threshold that will trigger a proximity interrupt. 
 * @brief set the number of consecutive proximity mesure out of range before triggering an interrupt 
 * @param sensor  pointer to the sensor tcs3772x_t structure
 * @param LowThreshold threshold, value bellow will rise interrupt
 * @param HighThreshold threshold, value above will rise interrupt
 * @param ConsecutiveMeasOutRangeThreshold number of consecutives mesaures that trigger a proximity interrupt. MUST BE between 1 and 15 (included)
 * @retval -2 out of range ConsecutiveMeasOutRangeThreshold value (must be between 1 and 15))
 * @retval -1  error : i2c error
 * @retval 0   function ok
 */
int8_t tcs3772x_SetProximityInterruptThreshold(tcs3772x_t *Sensor, uint16_t LowThreshold, uint16_t HighThreshold, uint8_t ConsecutiveMeasOutRangeThreshold);


#endif //TCS3772X__
