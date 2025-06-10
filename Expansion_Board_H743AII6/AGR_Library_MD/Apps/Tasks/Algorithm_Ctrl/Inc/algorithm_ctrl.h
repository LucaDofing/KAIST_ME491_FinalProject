#ifndef ALGORITHM_CTRL_INC_ALGORITHM_CTRL_H_
#define ALGORITHM_CTRL_INC_ALGORITHM_CTRL_H_

#include "module.h"

#include <stdbool.h>
#include <math.h>

#include "ioif_tim_common.h"
#include "error_dictionary.h"

#include "msg_hdlr.h"
#include "exppack_ctrl.h"
#include "data_object_common.h"
#include "ioif_adc_common.h"
#include "task_mngr.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define DT 0.001
#define GEAR_RATIO 18.75
#define MOTOR_TORQUE_CONSTANT 0.085


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _ControlMode_t {
    DEFAULT_CONTRL_MODE,       // 0: 기본 보조 모드
    POSITION_CTRL = 1,         // 1: 위치 제어 모드
    GRAVITY_COMPENSATION,      // 2: 중력 보상 모드
    IMPEDANCE_CTRL,            // 3: 임피던스 제어 모드
    TORQUE_CTRL,               // 4: 토크 제어 모드
    STEP_CURRENT_CTRL,         // 5: 스텝 전류 모드 ← 추가
    USER_DEFINED_CTRL,         // 6: 사용자 정의 모드 ← 추가
    CONTROL_MODE_NUM           // 7: 전체 모드 개수
} ControlMode;

typedef enum _MOTOR_t {
	RH_MOTOR = 0,
	LH_MOTOR = 1,
} MOTOR;

typedef struct _RobotData_t {
	float u_totalInput;		// control input

	float thighTheta_act;	// Thigh degree (received [MD->MiniCM->EXTboard])
	float position_act;		// Actual position (received [MD->MiniCM->EXTboard])
	float position_ref;		// Reference position

	float accX;			// (received [MD->MiniCM->EXTboard])
	float accY;			// (received [MD->MiniCM->EXTboard])
	float gyrZ;

	float e;
	float e_prev;
	float e_dot;
} RobotData_t;

typedef struct _ImpedanceCtrl {
	/* Parameters */
	float epsilon, Kp, Kd, lambda;

	float gap_epsilon;
	float gap_Kp;
	float gap_Kd;
	float gap_lambda;

	float e;  // error
	float ef; // output of error function

	float ef_f;
	float ef_diff;

	float duration;

	float control_input;

	uint16_t i; // 1ms counter
	uint8_t ON; // flag
} ImpedanceCtrl;

typedef struct _GravComp {
	float grav_comp_torque;
	float grav_gain;
	float f_grav_comp_torque;
	float grav_alpha;

	float control_input;  // control input
} GravComp;

typedef struct _StepCurr {

	float control_input;  // control input
} StepCurr;

typedef struct _UserDefinedCtrl {

	float control_input;  // control input
} UserDefinedCtrl;

/* PID Controller Structure needed for EMG controller */
typedef struct _PIDObject {
    float Kp;                   // Proportional gain
    float Ki;                   // Integral gain
    float Kd;                   // Derivative gain
    float integralMax;          // Maximum integral value
    float integralMin;          // Minimum integral value
    float outputMax;            // Maximum output value
    float outputMin;            // Minimum output value
    float integral;             // Integral term
    float error;                // Current error
    float errorPrev;            // Previous error
    float target;               // Target value
    float control_input;        // Control input
} PIDObject;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskObj_t algorithmCtrlTask;
extern RobotData_t robotDataObj_RH;
extern RobotData_t robotDataObj_LH;
extern uint8_t motionMap_selection;
extern uint8_t startPvector_decoding;
extern ControlMode controlMode;
// extern float EMG_Rawsignal;
extern float EMG_R1_Rawsignal;
extern float EMG_L1_Rawsignal;
extern float free_var1;
extern float free_var2;
extern float free_var3;
extern float free_var4;
extern float free_var5;
extern uint8_t CM_connect_signal;
extern uint8_t CM_disconnect_signal;
extern uint8_t ackSignal;
extern float f_vector_input_RH;
extern float f_vector_input_LH;
extern float assist_level;

/* Additional variables needed for EMG controller */
extern GravComp gravCompDataObj_RH;
extern GravComp gravCompDataObj_LH;
extern ImpedanceCtrl impedanceCtrl_RH;
extern ImpedanceCtrl impedanceCtrl_LH;
extern StepCurr StepCurr_RH;
extern StepCurr StepCurr_LH;
extern UserDefinedCtrl UserDefinedCtrl_RH;
extern UserDefinedCtrl UserDefinedCtrl_LH;
extern PIDObject posCtrl_RH;
extern PIDObject posCtrl_LH;
extern StudentsData_t studentsDataObj_RH;
extern StudentsData_t studentsDataObj_LH;
extern Extpack_Data_t extpackDataObj;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitAlgorithmCtrl(void);
void RunAlgorithmCtrl(void* params);

/* Include the EMG controller header */
#include "robot_emg_controller.h"

#endif /* ALGORITHM_CTRL_INC_ALGORITHM_CTRL_H_ */