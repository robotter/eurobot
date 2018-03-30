
#define SE_AX12_CYLINDER_ID 5

#define SE_CYLINDER_MIN 44
#define SE_CYLINDER_MAX 1023

typedef enum{
  CYLINDER_INIT,
  CYLINDER_IDLE,
  CYLINDER_BALLEATER_PRE_TAKE,
  CYLINDER_BALLEATER_TAKE,
  CYLINDER_BALLEATER_POST_TAKE,
  CYLINDER_FIND_EMPTY,
  CYLINDER_FIND_EMPTY_ORDER_MOVING,
  CYLINDER_FIND_EMPTY_MOVING,
  CYLINDER_EATBALL,
  CYLINDER_TAKEBALLS,
  CYLINDER_IS_FULL,
  CYLINDER_THROWBALLS,
  CYLINDER_TRASHBALLS,
}cylinder_state_t;

typedef struct {
  cylinder_state_t state;
  uint8_t balls_loaded;

  uint8_t position;

  uint32_t moving_ts;
  uint32_t drop_ts;
}cylinder_t;


extern cylinder_t cylinder;


void cylinder_update(void);
void cylinder_init(void);
void cylinder_shutdown(void);
