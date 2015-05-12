#include <avr/io.h>
#include "servo.h"
#include "motor.h"
#include "bumper.h"

#define BACKWARD_AUTOSET_MOTOR_SPEED  -80
#define FORWARD_MOTOR_SPEED           170


typedef enum {
  FSM_WAIT_TRAVEL_BEGINNING,
  FSM_WAIT_TRAVEL_END,
  FSM_WAIT_GALIPEUR_UNLOCK,
  FSM_BACKWARD_AUTOSET,
  FSM_GO_FORWARD,
  FSM_UNLOAD_CARPET,
  FSM_WAIT_CARPET_ROLLED_OUT,
  FSM_GO_BACKWARD,
  FSM_END
} pmi_tfm_t;



void pmi_fsm_manager(void)
{
  static uint8_t wait_galipeur_unlock_cnt = 0;
  static pmi_tfm_t fsm = FSM_WAIT_TRAVEL_BEGINNING;
  static uint8_t servo_move_cnt = 0;
  static uint8_t servo_move_wait_cnt = 0;
  static uint8_t wait_carpet_roll_down_cnt = 0;
  static uint8_t go_backward_cnt = 0;

  bumper_manage();

  switch (fsm)
  {
    case FSM_WAIT_TRAVEL_BEGINNING:
      // set servo to center position
      servo_set((SERVO_POS_MIN + SERVO_POS_MAX) /2);
      if (is_hook_lifted_up())
      {
        // pmi lifted up => let's travel !!!
        fsm = FSM_WAIT_TRAVEL_END;
      }
      break;

    case FSM_WAIT_TRAVEL_END:
      if (is_hook_lifted_up() == 0)
      {
        // end of travel => need to go to work :/
        fsm = FSM_WAIT_GALIPEUR_UNLOCK;
      }
      break;

    // before woring, go to sleep some time to let galipeur go away
    case FSM_WAIT_GALIPEUR_UNLOCK:
      wait_galipeur_unlock_cnt++;
      if (wait_galipeur_unlock_cnt > 10)
      {
        // time to go to work, first autoset
        fsm = FSM_BACKWARD_AUTOSET;
      }
      break;

    case FSM_BACKWARD_AUTOSET:
      motor_set_pwm(BACKWARD_AUTOSET_MOTOR_SPEED);
      if (is_rear_bumper_pushed())
      {
        fsm = FSM_GO_FORWARD;
      }
      break;

    case FSM_GO_FORWARD:
      motor_set_pwm(FORWARD_MOTOR_SPEED);
      if (is_front_bumper_pushed())
      {
        motor_set_pwm(0);
        fsm = FSM_UNLOAD_CARPET;
      }
      break;

    case FSM_UNLOAD_CARPET:
      if (servo_move_cnt % 2 == 0)
      {
        servo_set(SERVO_POS_MIN);
      }
      else
      {
        servo_set(SERVO_POS_MAX);
      }

      if (servo_move_wait_cnt > 50)
      {
        servo_move_cnt++;
        servo_move_wait_cnt = 0;
      }
      else
      {
        servo_move_wait_cnt++;
      }

      if (servo_move_cnt > 6)
      {
        servo_set((SERVO_POS_MIN + SERVO_POS_MAX) /2);
        fsm = FSM_WAIT_CARPET_ROLLED_OUT;
      }
      break;

    case FSM_WAIT_CARPET_ROLLED_OUT:
      wait_carpet_roll_down_cnt++;
      if (wait_carpet_roll_down_cnt > 50)
      {
        fsm = FSM_GO_BACKWARD;
      }
      break;

    case FSM_GO_BACKWARD:
      motor_set_pwm(BACKWARD_AUTOSET_MOTOR_SPEED);
      go_backward_cnt++;
      if (go_backward_cnt > 50)
      {
        motor_set_pwm(0);
        fsm = FSM_END;
      }
      break;

    case FSM_END:
      // job done, nothing to do
      break;
  }
}



int main(void)
{
  servo_init();
  motor_init();
  bumper_init();

  while (1)
  {
    for (uint32_t it=0; it<2000; it++) ;

    pmi_fsm_manager();
  }
  while (1)
  {
    PORTB |= _BV(3);

    for (uint32_t it=0; it<25000; it++) ;

    servo_set(SERVO_POS_MIN);
    motor_set_pwm(-150);
    PORTB = 0;

    for (uint32_t it=0; it<25000; it++) ;
    servo_set(SERVO_POS_MAX);
    motor_set_pwm(-180);
  }
}

