/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

#define K_VEL 27925268.03190926
#define K_ANG 0.000174532925199

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

typedef struct k_odo {
    float k_left;
    float k_right;
} k_odo_t;

void init_parameter(void);
void update_parameter(void);
void init_pid_control(void);
void update_pid_l(void);
void update_pid_r(void);
void InitPid1(void); /* PID data structure: PIDstruct for PID 1 (Motor left) */
void InitPid2(void); /* PID data structure: PIDstruct for PID 2 (Motor right) */
int Velocity(void);
int MotorPIDL(void); /* Esecution velocity PID for left motor */
int MotorPIDR(void); /* Esecution velocity PID for right motor */
void adc_motors_current(void); /* Mean valure for current measure motors */