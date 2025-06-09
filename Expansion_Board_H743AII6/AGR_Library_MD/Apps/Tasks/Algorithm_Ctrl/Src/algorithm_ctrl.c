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
	algorithmCtrlTask.state = STATE_OFF;
	algorithmCtrlTask.prevState = STATE_OFF;
	algorithmCtrlTask.loopCnt = 0;
	algorithmCtrlTask.loopPrevCnt = 0;
	algorithmCtrlTask.timeElap = 0.0f;
	algorithmCtrlTask.timePrevElap = 0.0f;

	// Initialize Position Controller
	InitPositionControl(&posCtrl_RH);
	InitPositionControl(&posCtrl_LH);

	// Initialize Gravity Compensation
	InitGravityCompensation(&gravCompDataObj_RH);
	InitGravityCompensation(&gravCompDataObj_LH);

	// Initialize Impedance Controller
	InitImpedanceSetting(&impedanceCtrl_RH);
	InitImpedanceSetting(&impedanceCtrl_LH);
}

void RunAlgorithmCtrl(void* params)
{
	algorithmCtrlTask.loopCnt++;
	algorithmCtrlTask.timeElap = (float)algorithmCtrlTask.loopCnt * ALGORITHM_CTRL_PERIOD;

	switch (algorithmCtrlTask.state) {
	case STATE_OFF:
		StateOff_Run();
		break;

	case STATE_STANDBY:
		StateStandby_Run();
		break;

	case STATE_ENABLE:
		if (algorithmCtrlTask.state != algorithmCtrlTask.prevState) {
			algorithmCtrlTask.prevState = algorithmCtrlTask.state;
			StateEnable_Ent();
		}
		StateEnable_Run();
		break;

	case STATE_ERROR:
		StateError_Run();
		break;

	default:
		break;
	}

	algorithmCtrlTask.loopPrevCnt = algorithmCtrlTask.loopCnt;
	algorithmCtrlTask.timePrevElap = algorithmCtrlTask.timeElap;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{
	// Wait for CM connection
	if (CM_connect_signal == 1) {
		algorithmCtrlTask.state = STATE_STANDBY;
		CM_connect_signal = 0;
	}
}

static void StateStandby_Run(void)
{
	// Wait for CM disconnection
	if (CM_disconnect_signal == 1) {
		algorithmCtrlTask.state = STATE_OFF;
		CM_disconnect_signal = 0;
	}

	// Wait for CM enable signal
	if (controlMode != NO_CONTROL) {
		algorithmCtrlTask.state = STATE_ENABLE;
	}
}

static void StateEnable_Ent(void)
{
	// Set the first flag
	isFirstPos_RH = true;
	isFirstPos_LH = true;
	isFirstImp_RH = true;
	isFirstImp_LH = true;
}

static void StateEnable_Run(void)
{
	// Wait for CM disconnection
	if (CM_disconnect_signal == 1) {
		algorithmCtrlTask.state = STATE_OFF;
		CM_disconnect_signal = 0;
		controlMode = NO_CONTROL;
		StateEnable_Ext();
		return;
	}

	// Wait for CM disable signal
	if (controlMode == NO_CONTROL) {
		algorithmCtrlTask.state = STATE_STANDBY;
		StateEnable_Ext();
		return;
	}

	// Update Robot Data
	UpdateRobotData(&robotDataObj_RH, &studentsDataObj_RH);
	UpdateRobotData(&robotDataObj_LH, &studentsDataObj_LH);

	// Update Extension Board Data
	UpdateExtensionBoardData(&extpackDataObj);

	/*---------------------- START of Control Sample Code ---------------------*/
	{
		if (controlMode == POSITION_CTRL) {	// 1 - Position Control
			PositionCtrl_Sample(&robotDataObj_RH, &posCtrl_RH);
			PositionCtrl_Sample(&robotDataObj_LH, &posCtrl_LH);
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
		} else if (controlMode == STEP_CURRENT) {   // 4 - Step Current
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
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
			isFirstPos_RH = true;
			isFirstPos_LH = true;
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		} else if (controlMode == F_VECTOR_CTRL) {   // 5 - F-Vector Control
			f_vector_input_RH = 0.0f;
			f_vector_input_LH = 0.0f;
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
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		} else if (controlMode == USER_DEFINED_CTRL) {   // 6 - User Defined Control
			// Call the EMG controller update function
			emg_controller_update();
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

	// Control Input Saturation
	ControlInputSaturation(&robotDataObj_RH, &posCtrl_RH, &gravCompDataObj_RH, &impedanceCtrl_RH, &StepCurr_RH, &UserDefinedCtrl_RH, f_vector_input_RH);
	ControlInputSaturation(&robotDataObj_LH, &posCtrl_LH, &gravCompDataObj_LH, &impedanceCtrl_LH, &StepCurr_LH, &UserDefinedCtrl_LH, f_vector_input_LH);
}

static void StateEnable_Ext(void)
{
	// Reset control input
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
	UserDefinedCtrl_RH.control_input = 0.0f;
	UserDefinedCtrl_LH.control_input = 0.0f;
}

static void StateError_Run(void)
{
	// Wait for CM disconnection
	if (CM_disconnect_signal == 1) {
		algorithmCtrlTask.state = STATE_OFF;
		CM_disconnect_signal = 0;
	}
}

static void UpdateRobotData(RobotData_t* robotDataObj, StudentsData_t* StudentsDataObj)
{
	// Update robot data from student data
	robotDataObj->thighTheta_act = StudentsDataObj->thigh_theta;
	robotDataObj->thighOmega_act = StudentsDataObj->thigh_omega;
	robotDataObj->thighAlpha_act = StudentsDataObj->thigh_alpha;
	robotDataObj->thighTorque_act = StudentsDataObj->thigh_torque;
	robotDataObj->thighCurrent_act = StudentsDataObj->thigh_current;
}

static void UpdateExtensionBoardData(Extpack_Data_t* ExtPackDataObj)
{
	// Update EMG data
	EMG_R1_Rawsignal = ExtPackDataObj->emg_data.emg_R1_MA;
	EMG_L1_Rawsignal = ExtPackDataObj->emg_data.emg_L1_MA;
	
	// Update FSR data
	fsr1_R1 = ExtPackDataObj->fsr_data.fsr1_R1;
	fsr2_R2 = ExtPackDataObj->fsr_data.fsr2_R2;
	fsr1_L1 = ExtPackDataObj->fsr_data.fsr1_L1;
	fsr2_L2 = ExtPackDataObj->fsr_data.fsr2_L2;
}

static void PvectorTrigger(P_Vector_Decoder* pvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft)
{
	if (*triggerSignal == true) {
		*triggerSignal = false;			// Reset

		pvectorObj->yd_f = robotDataObj->thighTheta_act * M_PI / 180.0f;

		if (*MM_ID >= 0 && *MM_ID < 40) {
			*pvectorObj = MotionMap_File->MS[*MM_ID].MD[isLeft].p_vector_decoder;		// Assign
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
		}
	}
}

static void InitPositionControl(PIDObject* posCtrl)
{
	// Initialize position control parameters
	posCtrl->Kp = 0.0f;
	posCtrl->Ki = 0.0f;
	posCtrl->Kd = 0.0f;
	posCtrl->integralMax = 0.0f;
	posCtrl->integralMin = 0.0f;
	posCtrl->outputMax = 0.0f;
	posCtrl->outputMin = 0.0f;
	posCtrl->integral = 0.0f;
	posCtrl->error = 0.0f;
	posCtrl->errorPrev = 0.0f;
	posCtrl->target = 0.0f;
	posCtrl->control_input = 0.0f;
}

static void InitPosCtrlHoming(PIDObject* posCtrl, bool* triggerSignal, RobotData_t* robotDataObj, P_Vector_Decoder* pvectorObj)
{
	// Initialize position control homing
	posCtrl->target = robotDataObj->thighTheta_act;
	*triggerSignal = false;
	pvectorObj->motionCnt = 0;
	pvectorObj->motionIdx = 0;
}

static void PositionCtrl_Sample(RobotData_t* robotDataObj, PIDObject* posCtrl)
{
	// Position control sample
	posCtrl->error = posCtrl->target - robotDataObj->thighTheta_act;
	posCtrl->integral += posCtrl->error * ALGORITHM_CTRL_PERIOD;
	
	// Anti-windup
	if (posCtrl->integral > posCtrl->integralMax) {
		posCtrl->integral = posCtrl->integralMax;
	} else if (posCtrl->integral < posCtrl->integralMin) {
		posCtrl->integral = posCtrl->integralMin;
	}
	
	// PID control
	posCtrl->control_input = posCtrl->Kp * posCtrl->error + posCtrl->Ki * posCtrl->integral + posCtrl->Kd * (posCtrl->error - posCtrl->errorPrev) / ALGORITHM_CTRL_PERIOD;
	
	// Saturation
	if (posCtrl->control_input > posCtrl->outputMax) {
		posCtrl->control_input = posCtrl->outputMax;
	} else if (posCtrl->control_input < posCtrl->outputMin) {
		posCtrl->control_input = posCtrl->outputMin;
	}
	
	// Update previous error
	posCtrl->errorPrev = posCtrl->error;
}

static void InitGravityCompensation(GravComp* gravComp)
{
	// Initialize gravity compensation parameters
	gravComp->grav_comp_torque = 0.0f;
	gravComp->grav_gain = 0.0f;
	gravComp->f_grav_comp_torque = 0.0f;
	gravComp->grav_alpha = 0.0f;
	gravComp->control_input = 0.0f;
}

static void GravityCompensation_Sample(GravComp* gravComp, RobotData_t* robotDataObj)
{
	// Gravity compensation sample
	gravComp->grav_comp_torque = gravComp->grav_gain * sin(robotDataObj->thighTheta_act * DEG2RAD);
	gravComp->f_grav_comp_torque = gravComp->grav_alpha * gravComp->f_grav_comp_torque + (1.0f - gravComp->grav_alpha) * gravComp->grav_comp_torque;
	gravComp->control_input = gravComp->f_grav_comp_torque;
}

static void InitImpedanceSetting(ImpedanceCtrl* impedanceCtrl)
{
	// Initialize impedance control parameters
	impedanceCtrl->epsilon = 0.0f;
	impedanceCtrl->Kp = 0.0f;
	impedanceCtrl->Kd = 0.0f;
	impedanceCtrl->lambda = 0.0f;
	
	impedanceCtrl->gap_epsilon = 0.0f;
	impedanceCtrl->gap_Kp = 0.0f;
	impedanceCtrl->gap_Kd = 0.0f;
	impedanceCtrl->gap_lambda = 0.0f;
	
	impedanceCtrl->e = 0.0f;
	impedanceCtrl->ef = 0.0f;
	impedanceCtrl->ef_f = 0.0f;
	impedanceCtrl->ef_diff = 0.0f;
	
	impedanceCtrl->duration = 0.0f;
	
	impedanceCtrl->control_input = 0.0f;
	
	impedanceCtrl->i = 0;
	impedanceCtrl->ON = 0;
}

static void ImpedanceControl_Sample(ImpedanceCtrl* impedanceCtrl, RobotData_t* robotDataObj, PIDObject* posCtrl, bool* isFirstImp)
{
	// Impedance control sample
	if (*isFirstImp) {
		posCtrl->target = robotDataObj->thighTheta_act;
		*isFirstImp = false;
	}
	
	impedanceCtrl->e = posCtrl->target - robotDataObj->thighTheta_act;
	
	// Error function
	if (impedanceCtrl->e > impedanceCtrl->epsilon) {
		impedanceCtrl->ef = impedanceCtrl->e - impedanceCtrl->epsilon;
	} else if (impedanceCtrl->e < -impedanceCtrl->epsilon) {
		impedanceCtrl->ef = impedanceCtrl->e + impedanceCtrl->epsilon;
	} else {
		impedanceCtrl->ef = 0.0f;
	}
	
	// Filter error function
	error_filter2(impedanceCtrl);
	
	// Impedance control
	impedanceCtrl->control_input = impedanceCtrl->Kp * impedanceCtrl->ef_f - impedanceCtrl->Kd * robotDataObj->thighOmega_act;
}

static void error_filter2(ImpedanceCtrl *impedanceCtrl)
{
	// Error filter
	impedanceCtrl->ef_f = impedanceCtrl->lambda * impedanceCtrl->ef_f + (1.0f - impedanceCtrl->lambda) * impedanceCtrl->ef;
}

static void ControlInputSaturation(RobotData_t *robotDataObj, PIDObject *posCtrl, GravComp *gravComp, ImpedanceCtrl *impedanceCtrl, StepCurr *StepCurr, UserDefinedCtrl *UserDefinedCtrl, float f_vector_input)
{
	// Control input saturation
	float totalInput = posCtrl->control_input + gravComp->control_input + impedanceCtrl->control_input + f_vector_input + StepCurr->control_input + UserDefinedCtrl->control_input;
	
	// Saturation
	if (totalInput > 15.0f) {
		totalInput = 15.0f;
	} else if (totalInput < -15.0f) {
		totalInput = -15.0f;
	}
	
	// Set control input
	robotDataObj->control_input = totalInput;
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
