
#define SE_AX12_CYLINDER_ID 5

#define SE_CYLINDER_MIN 44
#define SE_CYLINDER_MAX 1023

typedef enum{
  CYLINDER_INIT,
  CYLINDER_TAKEBALLS,
  CYLINDER_EATBALL,
  CYLINDER_IS_FULL,
  CYLINDER_THROWBALLS,
  CYLINDER_TRASHBALLS,
}cylinder_state_t;

typedef struct {
  cylinder_state_t state;
  uint8_t balls_loaded;
}cylinder_t;


extern cylinder_t cylinder;


void cylinder_update(void);
void cylinder_init(void);
void cylinder_shutdown(void);
