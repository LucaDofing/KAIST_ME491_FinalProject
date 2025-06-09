#include "algorithm_ctrl.h"

/* ------------------- Default Variables ------------------ */
TaskObj_t algorithmCtrlTask;

uint8_t CM_connect_signal = 0;
uint8_t CM_disconnect_signal = 0;

uint8_t motionMap_selection = 99;
uint8_t startPvector_decoding = 0;

ControlMode controlMode = DEFAULT_CONTRL_MODE;

float EMG_Rawsignal = 0.0f;
float EMG_R1_Rawsignal = 0.0f;
float EMG_L1_Rawsignal = 0.0f;

uint16_t fsr1_R1 = 0;
uint16_t fsr2_R2 = 0;
uint16_t fsr1_L1 = 0;
uint16_t fsr2_L2 = 0;

float free_var1 = 0.0f;
float free_var2 = 0.0f;
float free_var3 = 0.0f;
float free_var4 = 0.0f;
float free_var5 = 0.0f;

RobotData_t robotDataObj_RH;
RobotData_t robotDataObj_LH;
GravComp gravCompDataObj_RH;
GravComp gravCompDataObj_LH;
ImpedanceCtrl impedanceCtrl_RH;
ImpedanceCtrl impedanceCtrl_LH;
StepCurr StepCurr_RH;
StepCurr StepCurr_LH;
UserDefinedCtrl UserDefinedCtrl_RH;
UserDefinedCtrl UserDefinedCtrl_LH;

/* For Code Time Check */
static uint32_t STUDENTcodeStartTick = 0;
static uint32_t STUDENTcodeEndTick = 0;
static uint32_t algorithmCtrlLoopCnt;
static float algorithmCtrlTimeElap;

bool isFirstPos_RH = true;
bool isFirstPos_LH = true;
bool isFirstImp_RH = true;
bool isFirstImp_LH = true;

bool pVectorTrig_RH = false;
bool pVectorTrig_LH = false;
bool fVectorTrig_RH = false;
bool fVectorTrig_LH = false;
uint8_t MotionMap_ID_RH = 0;
uint8_t MotionMap_ID_LH = 0;

float RightHipFlexionTorque = 0.0f;
float RightHipExtensionTorque = 0.0f;
float LeftHipFlexionTorque = 0.0f;
float LeftHipExtensionTorque = 0.0f;

static uint32_t lastUpdateCnt = 0;
static uint8_t currentAmp = 0;
static bool done = false;

uint8_t ackSignal = 0;

float f_vector_input_RH = 0.0f;
float f_vector_input_LH = 0.0f;

float assist_level = 1.0f;
/* -------------------------------------------------------- */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

// Updata Data
static void UpdateRobotData(RobotData_t* robotDataObj, StudentsData_t* StudentsDataObj);
static void UpdateExtensionBoardData(Extpack_Data_t* ExtPackDataObj);

// PIF Vector Trigger
static void PvectorTrigger(P_Vector_Decoder* pvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft);
static void InitFvectorMaxTorque(F_Vector_Decoder* fvectorObj, uint8_t isLeft);
static void FvectorTrigger(F_Vector_Decoder* fvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft);

// Position Controller
static void InitPositionControl(PIDObject* posCtrl);
static void InitPosCtrlHoming(PIDObject* posCtrl, bool* triggerSignal, RobotData_t* robotDataObj, P_Vector_Decoder* pvectorObj);
static void PositionCtrl_Sample(RobotData_t* robotDataObj, PIDObject* posCtrl);

// Gravity Compensator
static void InitGravityCompensation(GravComp* gravComp);
static void GravityCompensation_Sample(GravComp* gravComp, RobotData_t* robotDataObj);

// Impedance Controller
static void InitImpedanceSetting(ImpedanceCtrl* impedanceCtrl);
static void ImpedanceControl_Sample(ImpedanceCtrl* impedanceCtrl, RobotData_t* robotDataObj, PIDObject* posCtrl, bool* isFirstImp);
static void error_filter2(ImpedanceCtrl *impedanceCtrl);

// Input Saturation
static void ControlInputSaturation(RobotData_t *robotDataObj, PIDObject *posCtrl, GravComp *gravComp, ImpedanceCtrl *impedanceCtrl, StepCurr *StepCurr, UserDefinedCtrl *UserDefinedCtrl, float f_vector_input);
/* -------------------------------------------------------- */

/*---------- (1) START of STUDENT CODE (Declare the functions to use) -----------*/

// EMG Filtering parameters
#define EMG_BUFFER_SIZE 20
#define FILTER_ORDER 4
float emg_R1_buffer[EMG_BUFFER_SIZE] = {0};
float emg_L1_buffer[EMG_BUFFER_SIZE] = {0};
float emg_R1_filtered = 0.0f;
float emg_L1_filtered = 0.0f;
float emg_R1_envelope = 0.0f;
float emg_L1_envelope = 0.0f;
int buffer_index = 0;

// Butterworth filter coefficients (4th order bandpass 20-450Hz)
// These coefficients would normally be calculated based on the exact sampling rate
// For demonstration purposes, using pre-calculated values
const float bandpass_b[5] = {0.0971f, 0.0f, -0.1942f, 0.0f, 0.0971f};
const float bandpass_a[5] = {1.0f, -2.4389f, 2.2948f, -0.9738f, 0.1576f};

// Low-pass filter coefficients for envelope detection (6Hz cutoff)
const float lowpass_b[5] = {0.0007f, 0.0029f, 0.0044f, 0.0029f, 0.0007f};
const float lowpass_a[5] = {1.0f, -3.0904f, 3.8008f, -2.0939f, 0.4326f};

// Filter state variables
float emg_R1_bp_states[8] = {0}; // Bandpass states
float emg_L1_bp_states[8] = {0};
float emg_R1_lp_states[8] = {0}; // Lowpass states
float emg_L1_lp_states[8] = {0};

// EMG threshold for activation detection
float emg_activation_threshold = 0.04f;

// Assist torque gain - scales the EMG signal to appropriate torque
float emg_torque_gain = 1.5f;

// Controller state
typedef enum {
    STATE_STANCE,
    STATE_SWING
} GaitPhaseState;

GaitPhaseState right_leg_state = STATE_STANCE;
GaitPhaseState left_leg_state = STATE_STANCE;

// Gait phase detection using both EMG and motion
bool detect_swing_initiation(float hip_angle, float hip_vel, float emg_envelope);

// EMG signal processing functions
float update_bandpass_filter(float new_sample, float* states, const float* b, const float* a);
float update_lowpass_filter(float new_sample, float* states, const float* b, const float* a);
float process_emg_sample(float raw_emg, float* bp_states, float* lp_states);

/*---------- (1) END of STUDENT CODE (Declare the functions to use) -------------*/

DOP_COMMON_SDO_CB(algorithmCtrlTask)

void InitAlgorithmCtrl(void)
{
    InitTask(&algorithmCtrlTask);

	InitFvectorMaxTorque(&fvectorObj_RH, RH_MOTOR);
	InitFvectorMaxTorque(&fvectorObj_LH, LH_MOTOR);

	/* State Definition */
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,		StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_ID_STUDENTS);

	// PDO
	/* For PDO setting */

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_ID_STUDENTS)

	/* Timer Callback Allocation */
	if (IOIF_StartTimIT(IOIF_TIM3) > 0) {
		//TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM3, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunAlgorithmCtrl, NULL);
}

void RunAlgorithmCtrl(void* params)
{
	/* Loop Start Time Check */
	STUDENTcodeStartTick = DWT->CYCCNT;

	ackSignal = !ackSignal; // 0과 1 토글

	/* Run Device */
	RunTask(&algorithmCtrlTask);

	/* Elapsed Time Check */
	STUDENTcodeEndTick = DWT->CYCCNT;
	if (STUDENTcodeEndTick < STUDENTcodeStartTick) {
		algorithmCtrlTimeElap = ((4294967295 - STUDENTcodeStartTick) + STUDENTcodeEndTick) / 480;	// in microsecond (Roll-over)
	}
	else {
		algorithmCtrlTimeElap = (DWT->CYCCNT - STUDENTcodeStartTick) / 480;							// in microsecond
	}
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{
	StateTransition(&algorithmCtrlTask.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Run(void)
{
	StateTransition(&algorithmCtrlTask.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent(void)
{
	EntRoutines(&algorithmCtrlTask.routine);

	InitPositionControl(&posCtrl_RH);
	InitPositionControl(&posCtrl_LH);

	InitGravityCompensation(&gravCompDataObj_RH);
	InitGravityCompensation(&gravCompDataObj_LH);

	InitImpedanceSetting(&impedanceCtrl_RH);
	InitImpedanceSetting(&impedanceCtrl_LH);

	algorithmCtrlLoopCnt = 0;
}

static void StateEnable_Run(void)
{
	if (CM_connect_signal == 1) {
		Send_ExtensionBoardEnable();
		CM_connect_signal = 0;
	}

	RunRoutines(&algorithmCtrlTask.routine);

	/*---------------------------- Data gathering (DO NOT CHANGE THIS) ----------------------------*/
	UpdateRobotData(&robotDataObj_RH, &StudentsDataObj_RH);
	UpdateRobotData(&robotDataObj_LH, &StudentsDataObj_LH);
	UpdateExtensionBoardData(&ExtPackDataObj);
	/*---------------------------------------------------------------------------------------------*/

	/*------------------------- Control Sample Code ------------------------*/
	if ((SUIT_State_curr >= 3 && SUIT_State_curr <= 17) || (SUIT_State_curr >= 33 && SUIT_State_curr <= 45) ||
		(SUIT_State_curr >= 66 && SUIT_State_curr <= 75)) {
		if (controlMode == POSITION_CTRL) {	// 1 - Position Control
			// Init Position For Safety
			if (isFirstPos_RH == true) {
				InitPosCtrlHoming(&posCtrl_RH, &pVectorTrig_RH, &robotDataObj_RH, &pvectorObj_RH);
				isFirstPos_RH = false;
			} else {
				PvectorTrigger(&pvectorObj_RH, &MotionMap_File, &robotDataObj_RH, &pVectorTrig_RH, &MotionMap_ID_RH, RH_MOTOR);
				PositionCtrl_Sample(&robotDataObj_RH, &posCtrl_RH);
			}
			if (isFirstPos_LH == true) {
				InitPosCtrlHoming(&posCtrl_LH, &pVectorTrig_LH, &robotDataObj_LH, &pvectorObj_LH);
				isFirstPos_LH = false;
			} else {
				PvectorTrigger(&pvectorObj_LH, &MotionMap_File, &robotDataObj_LH, &pVectorTrig_LH, &MotionMap_ID_LH, LH_MOTOR);
				PositionCtrl_Sample(&robotDataObj_LH, &posCtrl_LH);
			}
			
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		} else if (controlMode == GRAVITY_COMPENSATION) {	// 2 - Gravity Compensation
			GravityCompensation_Sample(&gravCompDataObj_RH, &robotDataObj_RH);
			GravityCompensation_Sample(&gravCompDataObj_LH, &robotDataObj_LH);
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		} else if (controlMode == IMPEDANCE_CTRL) {   // 3 - Impedance Control
			ImpedanceControl_Sample(&impedanceCtrl_RH, &robotDataObj_RH, &posCtrl_RH, &isFirstImp_RH);
			ImpedanceControl_Sample(&impedanceCtrl_LH, &robotDataObj_LH, &posCtrl_LH, &isFirstImp_LH);
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
		} else if (controlMode == TORQUE_CTRL) {   // 4 - Torque Control
			FvectorTrigger(&fvectorObj_RH, &MotionMap_File, &robotDataObj_RH, &fVectorTrig_RH, &MotionMap_ID_RH, RH_MOTOR);
			FvectorTrigger(&fvectorObj_LH, &MotionMap_File, &robotDataObj_LH, &fVectorTrig_LH, &MotionMap_ID_LH, LH_MOTOR);
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
		} else if (controlMode == STEP_CURRENT_CTRL) {   // 5 - Step Current Control
		    // 매 루프에서 전류 입력 지정 (초기화 방지)
		    posCtrl_RH.control_input = 0.0f;
		    posCtrl_LH.control_input = 0.0f;
		    gravCompDataObj_RH.control_input = 0.0f;
		    gravCompDataObj_LH.control_input = 0.0f;
		    impedanceCtrl_RH.control_input = 0.0f;
		    impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
		    UserDefinedCtrl_RH.control_input = 0.0f;
		    UserDefinedCtrl_LH.control_input = 0.0f;

		    StepCurr_RH.control_input = (float)currentAmp;

		    if (!done && (algorithmCtrlLoopCnt - lastUpdateCnt) >= 3000)
		    {
		        if (currentAmp < 4) {
		            currentAmp++;
		            lastUpdateCnt = algorithmCtrlLoopCnt;
		        }
		        else {
		            currentAmp = 0;     // 3초 유지 후 0A로
		            done = true;        // 이후에는 다시 증가하지 않음
		            lastUpdateCnt = algorithmCtrlLoopCnt;
		        }
		    }

		} else if (controlMode == USER_DEFINED_CTRL) {   // 6 - User Defined Control

		} else {
			// default : SUIT H10 Assist Mode
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		}
	}
	/*---------------------- END of Control Sample Code ---------------------*/

	/*--------------------- (2) START of STUDENT CODE (Write your code in this section) --------------------*/

    // Set the control mode to USER_DEFINED_CTRL for our EMG-based controller
    controlMode = USER_DEFINED_CTRL;

    // 1. FIRST STEP: Process EMG signals
    // This must happen before any control decisions are made
    emg_R1_filtered = process_emg_sample(EMG_R1_Rawsignal, emg_R1_bp_states, emg_R1_lp_states);
    emg_L1_filtered = process_emg_sample(EMG_L1_Rawsignal, emg_L1_bp_states, emg_L1_lp_states);

    // Store filtered values for debugging
    free_var1 = emg_R1_filtered;
    free_var2 = emg_L1_filtered;

    // 2. SECOND STEP: Calculate hip velocity and store previous values
    // We need to calculate velocity before making control decisions
    float hip_vel_RH = 0.0f;
    float hip_vel_LH = 0.0f;
    
    // Use static variables to maintain previous values across function calls
    static float prev_angle_RH = 0.0f;
    static float prev_angle_LH = 0.0f;
    static bool first_call = true;
    
    if (first_call) {
        // Initialize on first call
        prev_angle_RH = robotDataObj_RH.thighTheta_act;
        prev_angle_LH = robotDataObj_LH.thighTheta_act;
        first_call = false;
    } else {
        // Calculate velocity (deg/s) - multiply by 100 assuming 100Hz control frequency
        hip_vel_RH = (robotDataObj_RH.thighTheta_act - prev_angle_RH) * 100.0f;
        hip_vel_LH = (robotDataObj_LH.thighTheta_act - prev_angle_LH) * 100.0f;
        
        // Update previous values for next iteration
        prev_angle_RH = robotDataObj_RH.thighTheta_act;
        prev_angle_LH = robotDataObj_LH.thighTheta_act;
    }
    
    // Store velocity for debugging
    free_var3 = hip_vel_RH;
    free_var4 = hip_vel_LH;

    // 3. THIRD STEP: Detect gait phases using EMG and motion data
    // Simplified gait phase detection - directly use EMG and velocity
    bool right_swing_phase = (emg_R1_filtered > emg_activation_threshold) && (hip_vel_RH > 0.5f);
    bool left_swing_phase = (emg_L1_filtered > emg_activation_threshold) && (hip_vel_LH > 0.5f);
    
    // Store phase detection result for debugging
    free_var5 = (float)right_swing_phase;

    // 4. FOURTH STEP: Apply torque based on detected gait phase
    if (right_swing_phase) {
        // Flexion assistance during swing (positive torque for hip flexion)
        UserDefinedCtrl_RH.control_input = emg_R1_filtered * emg_torque_gain;
    } else {
        // No assistance during stance
        UserDefinedCtrl_RH.control_input = 0.0f;
    }
    
    if (left_swing_phase) {
        // Flexion assistance during swing
        UserDefinedCtrl_LH.control_input = emg_L1_filtered * emg_torque_gain;
    } else {
        // No assistance during stance
        UserDefinedCtrl_LH.control_input = 0.0f;
    }

    // Reset other control inputs
    posCtrl_RH.control_input = 0.0f;
    posCtrl_LH.control_input = 0.0f;
    gravCompDataObj_RH.control_input = 0.0f;
    gravCompDataObj_LH.control_input = 0.0f;
    impedanceCtrl_RH.control_input = 0.0f;
    impedanceCtrl_LH.control_input = 0.0f;
    f_vector_input_RH = 0.0f;
    f_vector_input_LH = 0.0f;
    StepCurr_RH.control_input = 0.0f;
    StepCurr_LH.control_input = 0.0f;

	/*------------------------------------ (2) END of STUDENT CODE-----------------------------------------*/
	// Control Input Saturation (Do not Delete)
	ControlInputSaturation(&robotDataObj_RH, &posCtrl_RH, &gravCompDataObj_RH, &impedanceCtrl_RH, &StepCurr_RH , &UserDefinedCtrl_RH, f_vector_input_RH);
	ControlInputSaturation(&robotDataObj_LH, &posCtrl_LH, &gravCompDataObj_LH, &impedanceCtrl_LH, &StepCurr_LH , &UserDefinedCtrl_LH, f_vector_input_LH);
	algorithmCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
    ExtRoutines(&algorithmCtrlTask.routine);
}

static void StateError_Run(void)
{

}

static void UpdateRobotData(RobotData_t* robotDataObj, StudentsData_t* StudentsDataObj)
{
	robotDataObj->thighTheta_act = StudentsDataObj->thighThetaAct;
	robotDataObj->position_act = StudentsDataObj->positionAct;
	robotDataObj->accX = StudentsDataObj->accX;
	robotDataObj->accY = StudentsDataObj->accY;
	robotDataObj->gyrZ = StudentsDataObj->gyrZ;
}

static void UpdateExtensionBoardData(Extpack_Data_t* ExtPackDataObj)
{
	// EMG_Rawsignal = ExtPackDataObj->emg_data.emg_R2_rawSign[0] / 2048.0f;
	EMG_R1_Rawsignal = ExtPackDataObj->emg_data.emg_R1_rawSign[0] / 2048.0f;
	EMG_L1_Rawsignal = ExtPackDataObj->emg_data.emg_L1_rawSign[0] / 2048.0f;

	fsr1_R1 = ExtPackDataObj->fsr_data.fsr_R1_raw;
	fsr2_R2 = ExtPackDataObj->fsr_data.fsr_R2_raw;
	fsr1_L1 = ExtPackDataObj->fsr_data.fsr_L1_raw;
	fsr2_L2 = ExtPackDataObj->fsr_data.fsr_L2_raw;

	free_var1 = robotDataObj_RH.accX;
	free_var2 = robotDataObj_RH.accY;
	free_var3 = robotDataObj_RH.accX;
	// free_var4 = robotDataObj_RH.accY;
	// free_var5 = robotDataObj_RH.accX;
}

static void PvectorTrigger(P_Vector_Decoder* pvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft)
{
	if (*triggerSignal == true) {
		*triggerSignal = false;			// Reset

		pvectorObj->yd_f = robotDataObj->position_act * M_PI / 180.0f;

		if (*MM_ID >= 0 && *MM_ID < 40) {
			*pvectorObj = MotionMap_File->MS[*MM_ID].MD[isLeft].p_vector_decoder;		// Assign
			// *MM_ID = 99;		// Reset
		}
	}
}

static void InitFvectorMaxTorque(F_Vector_Decoder* fvectorObj, uint8_t isLeft)
{
	memset(fvectorObj, 0, sizeof(*fvectorObj));

	// Max 8~10Nm이나 매우 위험할 수 있으므로 Test시에는 2~3Nm로 실험 하세요
	// default 8Nm
	// 로봇 설정 최대치 10Nm
	if (isLeft) {
		LeftHipFlexionTorque = 2.0f;	// 3Nm, saturation
		LeftHipExtensionTorque = 2.0f;	// 3Nm
	} else {
		RightHipFlexionTorque = 2.0f;	// 3Nm
		RightHipExtensionTorque = 2.0f;	// 3Nm
	}
}

static void FvectorTrigger(F_Vector_Decoder* fvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft)
{
	if (*triggerSignal == true) {
		*triggerSignal = false;			// Reset

		if (*MM_ID >= 0 && *MM_ID < 40) {
			*fvectorObj = MotionMap_File->MS[*MM_ID].MD[isLeft].f_vector_decoder;		// Assign
			// *MM_ID = 99;		// Reset
		}

		// global variable ID에 따른 Torque Max assign
		float t_tauMax = 0.0f;
		for (int i = 0; i < F_VECTOR_BUFF_SIZE; i++) {
			if (fvectorObj->f_buffer[i].globalVariableID == 0x01) {
				t_tauMax = RightHipFlexionTorque;
			}
			if (fvectorObj->f_buffer[i].globalVariableID == 0x02) {
				t_tauMax = RightHipExtensionTorque;
			}
			if (fvectorObj->f_buffer[i].globalVariableID == 0x03) {
				t_tauMax = LeftHipFlexionTorque;
			}
			if (fvectorObj->f_buffer[i].globalVariableID == 0x04) {
				t_tauMax = LeftHipExtensionTorque;
			}

			// Torque -> Current Input Conversion
			fvectorObj->f_buffer[i].tau_max = (t_tauMax / GEAR_RATIO / MOTOR_TORQUE_CONSTANT) * (fvectorObj->f_buffer[i].coefficient * 0.01);

			// F vector Reset Logic
			// if (fvectorObj->f_buffer[i].mode_idx == 255) {
			// 	if (isLeft) f_vector_input_LH = 0.0f;
			// 	else f_vector_input_RH = 0.0f;
			// 	fvectorObj->f_buffer[i].mode_idx = 0;
			// 	fvectorObj->f_buffer[i].tau_max = 0;
			// 	fvectorObj->f_buffer[i].delay = 0;
			// 	fvectorObj->f_buffer[i].u = 0;
			// 	fvectorObj->f_buffer[i].u_old1 = 0;
			// 	fvectorObj->f_buffer[i].u_old2 = 0;
			// 	fvectorObj->f_buffer[i].tau = 0;
			// 	fvectorObj->f_buffer[i].tau_old1 = 0;
			// 	fvectorObj->f_buffer[i].tau_old2 = 0;
			// 	fvectorObj->f_buffer[i].t_end = 0;
			// 	fvectorObj->f_buffer[i].time_stamp = 0;
			// 	fvectorObj->f_buffer[i].is_full = 0;
			// }
		}
	}
}

static void InitPositionControl(PIDObject* posCtrl)
{
	memset(posCtrl, 0, sizeof(*posCtrl));

	posCtrl->Kp = 1.5;
	posCtrl->Kd = 0.2;
}

static void InitPosCtrlHoming(PIDObject* posCtrl, bool* triggerSignal, RobotData_t* robotDataObj, P_Vector_Decoder* pvectorObj)
{
	pvectorObj->yd_f = robotDataObj->position_act * M_PI / 180.0f;
	pvectorObj->N = 1;
	pvectorObj->p_buffer->yd = 0;
	pvectorObj->p_buffer->s0 = 60;
	pvectorObj->p_buffer->sd = 60;
	pvectorObj->p_buffer->L = 3000;
}

static void PositionCtrl_Sample(RobotData_t* robotDataObj, PIDObject* posCtrl)
{
	posCtrl->err = (posCtrl->ref) - (robotDataObj->position_act * M_PI / 180);
	posCtrl->err_diff = (posCtrl->err - posCtrl->err_prev) / DT;
	posCtrl->err_prev =  posCtrl->err;

	posCtrl->control_input = posCtrl->Kp * posCtrl->err + posCtrl->Kd * posCtrl->err_diff;
}

static void InitGravityCompensation(GravComp* gravComp)
{
	memset(gravComp, 0, sizeof(*gravComp));

	gravComp->grav_gain = 0.5;
	gravComp->grav_alpha = 0.99;
}

static void GravityCompensation_Sample(GravComp* gravComp, RobotData_t* robotDataObj)
{
	if (robotDataObj->thighTheta_act > 0) 
	{
		gravComp->grav_comp_torque =  gravComp->grav_gain * (sin((robotDataObj->thighTheta_act) * M_PI / 180));
		gravComp->f_grav_comp_torque = gravComp->grav_alpha * gravComp->f_grav_comp_torque + (1 - gravComp->grav_alpha) * gravComp->grav_comp_torque;
	}
	else
	{
		gravComp->grav_comp_torque = 0;
		gravComp->f_grav_comp_torque = gravComp->grav_alpha * gravComp->f_grav_comp_torque + (1 - gravComp->grav_alpha) * gravComp->grav_comp_torque;
	}

	gravComp->control_input = gravComp->f_grav_comp_torque;
}

static void InitImpedanceSetting(ImpedanceCtrl* impedanceCtrl)
{
	memset(impedanceCtrl, 0, sizeof(*impedanceCtrl));
	
	impedanceCtrl->epsilon = 5.0f * M_PI / 180.0f;
	impedanceCtrl->Kp = 1.5f;
	impedanceCtrl->Kd = 0.2f;
	impedanceCtrl->lambda = 1.0f;
	impedanceCtrl->duration = 300.0f;
}

static void ImpedanceControl_Sample(ImpedanceCtrl* impedanceCtrl, RobotData_t* robotDataObj, PIDObject* posCtrl, bool* isFirstImp)
{
	float t_epsilon = 0.0f;
	float t_Kp = 0.0f;
	float t_Kd = 0.0f;
	float t_lambda = 0.0f;

	if (*isFirstImp == true && impedanceCtrl->ON == 0) {
		posCtrl->ref = robotDataObj->position_act * M_PI / 180.0f;
		t_epsilon = impedanceCtrl->epsilon; // unit: rad   (0.001745329252 = 0.1 * pi/180)
		t_Kp      = impedanceCtrl->Kp;
		t_Kd      = impedanceCtrl->Kd;
		t_lambda  = impedanceCtrl->lambda;

		if (impedanceCtrl->duration > 0) {
			float invT      = 1/impedanceCtrl->duration;

			impedanceCtrl->gap_epsilon = (t_epsilon - impedanceCtrl->epsilon) * invT;
			impedanceCtrl->gap_Kp      = (t_Kp      - impedanceCtrl->Kp)      * invT;
			impedanceCtrl->gap_Kd      = (t_Kd      - impedanceCtrl->Kd)      * invT;
			impedanceCtrl->gap_lambda  = (t_lambda  - impedanceCtrl->lambda)  * invT;
		}

		impedanceCtrl->i  = 0; // initialize 1ms counter
		impedanceCtrl->ON = 1;
		*isFirstImp = false;
	}

	if (impedanceCtrl->ON == 1) {
		if (impedanceCtrl->duration == 0) {
			impedanceCtrl->epsilon = t_epsilon;
			impedanceCtrl->Kp      = t_Kp;
			impedanceCtrl->Kd      = t_Kd;
			impedanceCtrl->lambda  = t_lambda;
		} else {
			impedanceCtrl->epsilon = impedanceCtrl->epsilon + impedanceCtrl->gap_epsilon;
			impedanceCtrl->Kp      = impedanceCtrl->Kp      + impedanceCtrl->gap_Kp;
			impedanceCtrl->Kd      = impedanceCtrl->Kd      + impedanceCtrl->gap_Kd;
			impedanceCtrl->lambda  = impedanceCtrl->lambda  + impedanceCtrl->gap_lambda;
			impedanceCtrl->i++;
		}

		if (impedanceCtrl->i >= impedanceCtrl->duration)	{
			impedanceCtrl->ON = 0;
			impedanceCtrl->i = 0;
		}
	}

	/* Impedance Controller */
	impedanceCtrl->e = posCtrl->ref - (robotDataObj->position_act * M_PI / 180.0f);

	error_filter2(impedanceCtrl);

	float t_ef_diff = 0.0;

	if (((impedanceCtrl->ef > 0) & (impedanceCtrl->ef_diff > 0)) | ((impedanceCtrl->ef <= 0) & (impedanceCtrl->ef_diff <= 0))) {
		t_ef_diff = +impedanceCtrl->ef_diff;
	} else {
		t_ef_diff = -impedanceCtrl->ef_diff;
	}

	impedanceCtrl->control_input = impedanceCtrl->Kp * impedanceCtrl->ef + impedanceCtrl->Kd * t_ef_diff;
}

static void error_filter2(ImpedanceCtrl *impedanceCtrl)
{
	// f2(e,t) = lambda * e + (1 - lambda)*sign(e)*max(|e| - epsilon, 0)
	float t_abs_e = 0.0;
	float t_sign_e = 0.0;
	float t_max = 0.0;
	float t_diff = 0.0;
	float y_ef = 0.0;

	/* Calculate 'sign(e) & |e|' */
	if (impedanceCtrl->e > 0)	{t_abs_e = +impedanceCtrl->e; t_sign_e = +1; }
	else						{t_abs_e = -impedanceCtrl->e; t_sign_e = -1; }

	/* Calculate 'max(|e| - epsilon, 0)' */
	t_diff = t_abs_e - impedanceCtrl->epsilon;
	if (t_diff > 0) {t_max = t_diff;}
	else            {t_max = 0;}

	y_ef = (impedanceCtrl->lambda * impedanceCtrl->e) + (1 - impedanceCtrl->lambda) * t_sign_e * t_max;

	impedanceCtrl->ef_diff = (y_ef - impedanceCtrl->ef) * 0.001;

	impedanceCtrl->ef = y_ef;
}

static void ControlInputSaturation(RobotData_t *robotDataObj, PIDObject *posCtrl, GravComp *gravComp, ImpedanceCtrl *impedanceCtrl, StepCurr *StepCurr, UserDefinedCtrl *UserDefinedCtrl, float f_vector_input)
{
	// Saturate control input to ±9 using fminf and fmaxf
	robotDataObj->u_totalInput = fminf(fmaxf((posCtrl->control_input + /* control input of PID position controller */
								gravComp->control_input + /* control input of gravity compensation */
								impedanceCtrl->control_input + /* control input of impedance controller */
								StepCurr->control_input + /* control input of step current input controller */
								UserDefinedCtrl->control_input + /* control input of user defined controller */
								f_vector_input) * assist_level, -9.0f), 9.0f); /* control input of F-vector decoded */
								// assist_level 0 ~ 1.0 (0~100%, 0.1, 10%단위)
}

/*----------- (3) START of STUDENT CODE (Define the functions to use) ------------*/

// Implementation of bandpass filter for EMG signal
float update_bandpass_filter(float new_sample, float* states, const float* b, const float* a) {
    // Direct Form II Transposed implementation - efficient for real-time processing
    float result = b[0] * new_sample + states[0];
    
    // Update states - shift values in the state array
    for (int i = 0; i < FILTER_ORDER-1; i++) {
        states[i] = b[i+1] * new_sample - a[i+1] * result + states[i+1];
    }
    states[FILTER_ORDER-1] = b[FILTER_ORDER] * new_sample - a[FILTER_ORDER] * result;
    
    return result;
}

// Implementation of lowpass filter for envelope detection
float update_lowpass_filter(float new_sample, float* states, const float* b, const float* a) {
    // Direct Form II Transposed implementation - efficient for real-time processing
    float result = b[0] * new_sample + states[0];
    
    // Update states - shift values in the state array
    for (int i = 0; i < FILTER_ORDER-1; i++) {
        states[i] = b[i+1] * new_sample - a[i+1] * result + states[i+1];
    }
    states[FILTER_ORDER-1] = b[FILTER_ORDER] * new_sample - a[FILTER_ORDER] * result;
    
    return result;
}

// Process a single EMG sample through the full processing pipeline
// This function is optimized for real-time processing
float process_emg_sample(float raw_emg, float* bp_states, float* lp_states) {
    // Step 1: Apply bandpass filter to remove noise and DC offset
    // Uses Direct Form II Transposed implementation for efficiency
    float filtered = update_bandpass_filter(raw_emg, bp_states, bandpass_b, bandpass_a);
    
    // Step 2: Rectify the signal (take absolute value)
    float rectified = fabsf(filtered);
    
    // Step 3: Apply lowpass filter to get the envelope
    // Uses Direct Form II Transposed implementation for efficiency
    float envelope = update_lowpass_filter(rectified, lp_states, lowpass_b, lowpass_a);
    
    return envelope;
}

// Detect swing phase initiation based on EMG and hip velocity
// This function is called directly in the main control loop now
bool detect_swing_initiation(float hip_angle, float hip_vel, float emg_envelope) {
    // Swing phase initiation: quad activation plus hip flexion beginning
    // Relaxed conditions for better detection in real data
    if (emg_envelope > emg_activation_threshold && hip_vel > 0.5f) {
        return true;
    }
    return false;
}

/*------------ (3) END of STUDENT CODE (Define the functions to use) -------------*/
