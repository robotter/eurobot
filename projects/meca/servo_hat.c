/*
 *  Copyright RobOtter (2016)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <avr/io.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <avarix.h>
#include <avarix/portpin.h>
#include <i2c/i2c.h>
#include <timer/uptime.h>
#include "servo_hat.h"
#include "config.h"

#define SH_CHANNEL_NB 16
#define SH_I2C_ADDRESS 0b01000000

i2cm_t *const servo_hat_i2c = i2cE;
typedef struct
{ 
    uint16_t position_percent;
    uint16_t min_pwm;
    uint16_t max_pwm;
}servo_hat_channel_t;



static servo_hat_channel_t sh_channels[SH_CHANNEL_NB];

void servo_hat_init(void)
{
    const uint8_t mode1_reg = 0b00100001;
    const uint8_t mode1_reg_osc_off = 0b00110001;
    const uint8_t mode2_reg = 0b00000100; // open drain
    const uint8_t prescaler_reg = 122; // 50Hz for analog servo
    uint8_t buf[2] = {0x00, mode1_reg_osc_off};
    i2cm_send(servo_hat_i2c, SH_I2C_ADDRESS, buf, sizeof(buf)/sizeof(buf[0]));
    buf[0] = 0x01; // mode 2 register
    buf[1] = mode2_reg;
    i2cm_send(servo_hat_i2c, SH_I2C_ADDRESS, buf, sizeof(buf)/sizeof(buf[0]));

    buf[0] = 0xFE; //prescaler address
    buf[1] = prescaler_reg;
    i2cm_send(servo_hat_i2c, SH_I2C_ADDRESS, buf, sizeof(buf)/sizeof(buf[0]));

    buf[0] = 0x00; //enable oscillator once prescaler set
    buf[1] = mode1_reg;
    i2cm_send(servo_hat_i2c, SH_I2C_ADDRESS, buf, sizeof(buf)/sizeof(buf[0]));
}

void servo_hat_configure_channel(uint8_t channel, uint16_t min_pwm, uint16_t max_pwm)
{
    if (channel < SH_CHANNEL_NB)
    {
	sh_channels[channel].min_pwm = min_pwm;
	sh_channels[channel].max_pwm = max_pwm;
    }
}

void servo_hat_set_pwm(uint8_t channel, uint16_t position_percent)
{
    uint8_t reg_addr = 0x06 + 4* channel;

    const uint16_t start_pwm = 0x0000;
    uint16_t end_pwm = (uint16_t)((uint32_t)position_percent * (uint32_t)(sh_channels[channel].max_pwm - sh_channels[channel].min_pwm))/100;
    end_pwm += sh_channels[channel].min_pwm;

    end_pwm &= 0x0fff;

    uint8_t buf[5];
    buf[0] = reg_addr;
    buf[1] = (uint8_t)start_pwm;
    buf[2] = (uint8_t)(start_pwm>>8);
    buf[3] = (uint8_t)end_pwm;
    buf[4] = (uint8_t)(end_pwm>>8);

    i2cm_send(servo_hat_i2c, SH_I2C_ADDRESS, buf, sizeof(buf)/sizeof(buf[0]));
}




