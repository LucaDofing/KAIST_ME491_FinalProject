#ifndef ALGORITHM_CTRL_H
#define ALGORITHM_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Constants */
#define ALGORITHM_CTRL_PERIOD   0.01f  // 10ms control period
#define DEG2RAD                 0.01745329252f
#define RAD2DEG                 57.29577951f

/* State Definitions */
#define STATE_OFF               0
#define STATE_STANDBY           1
#define STATE_ENABLE            2
#define STATE_ERROR             3

/* Control Mode Definitions */
#define NO_CONTROL              0
#define POSITION_CTRL           1
#define GRAVITY_COMPENSATION    2
#define IMPEDANCE_CTRL          3
#define STEP_CURRENT            4
#define F_VECTOR_CTRL           5
#define USER_DEFINED_CTRL       6

/* Motor Definitions */
#define RH_MOTOR                0
#define LH_MOTOR                1

/* Task Object */
typedef struct {
    uint8_t state;
    uint8_t prevState;
    uint32_t loopCnt;
    uint32_t loopPrevCnt;
    float timeElap;
    float timePrevElap;
} TaskObj_t;

/* Robot Data Structure */
typedef struct {
    float thighTheta_act;       // Actual thigh angle (degrees)
    float thighOmega_act;       // Actual thigh angular velocity (degrees/s)
    float thighAlpha_act;       // Actual thigh angular acceleration (degrees/s^2)
    float thighTorque_act;      // Actual thigh torque (Nm)
    float thighCurrent_act;     // Actual thigh motor current (A)
    float control_input;        // Control input to the motor
} RobotData_t;

/* Student Data Structure */
typedef struct {
    float thigh_theta;          // Thigh angle (degrees)
    float thigh_omega;          // Thigh angular velocity (degrees/s)
    float thigh_alpha;          // Thigh angular acceleration (degrees/s^2)
    float thigh_torque;         // Thigh torque (Nm)
    float thigh_current;        // Thigh motor current (A)
} StudentsData_t;

/* Extension Pack Data Structure */
typedef struct {
    struct {
        float emg_R1_MA;        // Right EMG channel 1 moving average
        float emg_L1_MA;        // Left EMG channel 1 moving average
    } emg_data;
    
    struct {
        uint16_t fsr1_R1;       // Right FSR 1
        uint16_t fsr2_R2;       // Right FSR 2
        uint16_t fsr1_L1;       // Left FSR 1
        uint16_t fsr2_L2;       // Left FSR 2
    } fsr_data;
} Extpack_Data_t;

/* PID Controller Structure */
typedef struct {
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

/* Gravity Compensation Structure */
typedef struct {
    float grav_comp_torque;     // Gravity compensation torque
    float grav_gain;            // Gravity gain
    float f_grav_comp_torque;   // Filtered gravity compensation torque
    float grav_alpha;           // Gravity filter coefficient
    float control_input;        // Control input
} GravComp;

/* Impedance Controller Structure */
typedef struct {
    float epsilon;              // Epsilon value for error function
    float Kp;                   // Proportional gain
    float Kd;                   // Derivative gain
    float lambda;               // Filter coefficient
    
    float gap_epsilon;          // Epsilon gap
    float gap_Kp;               // Proportional gain gap
    float gap_Kd;               // Derivative gain gap
    float gap_lambda;           // Filter coefficient gap
    
    float e;                    // Error
    float ef;                   // Error function
    float ef_f;                 // Filtered error function
    float ef_diff;              // Error function derivative
    
    float duration;             // Duration
    
    float control_input;        // Control input
    
    uint32_t i;                 // Counter
    uint8_t ON;                 // ON flag
} ImpedanceCtrl;

/* Step Current Structure */
typedef struct {
    float control_input;        // Control input
} StepCurr;

/* User Defined Controller Structure */
typedef struct {
    float control_input;        // Control input
} UserDefinedCtrl;

/* P-Vector Decoder Structure */
typedef struct {
    uint32_t motionCnt;         // Motion counter
    uint32_t motionIdx;         // Motion index
} P_Vector_Decoder;

/* F-Vector Decoder Structure */
typedef struct {
    uint32_t motionCnt;         // Motion counter
    uint32_t motionIdx;         // Motion index
} F_Vector_Decoder;

/* Motion Map File Info Structure */
typedef struct {
    uint8_t dummy;              // Dummy variable
} MotionMapFileInfo;

/* Global Variables */
extern TaskObj_t algorithmCtrlTask;
extern uint8_t CM_connect_signal;
extern uint8_t CM_disconnect_signal;
extern uint8_t motionMap_selection;
extern uint8_t startPvector_decoding;
extern uint8_t controlMode;
extern float EMG_Rawsignal;
extern float EMG_R1_Rawsignal;
extern float EMG_L1_Rawsignal;
extern uint16_t fsr1_R1;
extern uint16_t fsr2_R2;
extern uint16_t fsr1_L1;
extern uint16_t fsr2_L2;
extern float free_var1;
extern float free_var2;
extern float free_var3;
extern float free_var4;
extern float free_var5;
extern RobotData_t robotDataObj_RH;
extern RobotData_t robotDataObj_LH;
extern GravComp gravCompDataObj_RH;
extern GravComp gravCompDataObj_LH;
extern ImpedanceCtrl impedanceCtrl_RH;
extern ImpedanceCtrl impedanceCtrl_LH;
extern StepCurr StepCurr_RH;
extern StepCurr StepCurr_LH;
extern UserDefinedCtrl UserDefinedCtrl_RH;
extern UserDefinedCtrl UserDefinedCtrl_LH;
extern bool isFirstPos_RH;
extern bool isFirstPos_LH;
extern bool isFirstImp_RH;
extern bool isFirstImp_LH;
extern bool pVectorTrig_RH;
extern bool pVectorTrig_LH;
extern bool fVectorTrig_RH;
extern bool fVectorTrig_LH;
extern uint8_t MotionMap_ID_RH;
extern uint8_t MotionMap_ID_LH;
extern float RightHipFlexionTorque;
extern float RightHipExtensionTorque;
extern float LeftHipFlexionTorque;
extern float LeftHipExtensionTorque;
extern uint8_t ackSignal;
extern float f_vector_input_RH;
extern float f_vector_input_LH;
extern float assist_level;
extern StudentsData_t studentsDataObj_RH;
extern StudentsData_t studentsDataObj_LH;
extern Extpack_Data_t extpackDataObj;
extern PIDObject posCtrl_RH;
extern PIDObject posCtrl_LH;

/* Function Prototypes */
void InitAlgorithmCtrl(void);
void RunAlgorithmCtrl(void* params);

#endif /* ALGORITHM_CTRL_H */
