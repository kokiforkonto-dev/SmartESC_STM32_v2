#include "stdlib.h"
#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "cmsis_os.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "digital_output.h"
#include "state_machine.h"
#include "pwm_common.h"
#include "VescCommand.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_pwr.h"
#include "product.h"
#include "music.h"
#include "current_sense.h"
#include "VescToSTM.h"

#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)

#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)

FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t * oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
RDivider_Handle_t *pBusSensorM1;

NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
CURR_Handle_t *pMainCurrentSensor;

PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
FW_Handle_t *pFW[NBR_OF_MOTORS];
MTPA_Handle_t *pMaxTorquePerAmpere[2] = {MC_NULL,MC_NULL};
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

uint8_t bMCBootCompleted = 0;

#if MUSIC_ENABLE
MUSIC_PARAM bldc_music;
#endif

void TSK_MediumFrequencyTaskM1(void);
void FOC_Clear(uint8_t bMotor);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);
void UI_Scheduler(void);

__weak void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS],MCT_Handle_t* pMCTList[NBR_OF_MOTORS] )
{
  STM_Init(&STM[M1]);

  bMCBootCompleted = 0;
  pCLM[M1] = &CircleLimitationM1;
  pFW[M1] = &FW_M1;

  pwmcHandle[M1] = &PWM_Handle_M1._Super;
  R3_2_Init(&PWM_Handle_M1);

  startTimers();

  PID_HandleInit(&PIDSpeedHandle_M1);
  pPIDSpeed[M1] = &PIDSpeedHandle_M1;

  pSTC[M1] = &SpeednTorqCtrlM1;
  HALL_Init (&HALL_M1);

  STC_Init(pSTC[M1],pPIDSpeed[M1], &HALL_M1._Super);

  PID_HandleInit(&PIDIqHandle_M1);
  PID_HandleInit(&PIDIdHandle_M1);
  pPIDIq[M1] = &PIDIqHandle_M1;
  pPIDId[M1] = &PIDIdHandle_M1;

  pBusSensorM1 = &RealBusVoltageSensorParamsM1;
  RVBS_Init(pBusSensorM1);

  pMPM[M1] = &PQD_MotorPowMeasM1;
  pMPM[M1]->pVBS = &(pBusSensorM1->_Super);
  pMPM[M1]->pFOCVars = &FOCVars[M1];

  NTC_Init(&TempSensorParamsM1);
  pTemperatureSensor[M1] = &TempSensorParamsM1;

  CURR_Init(&CurrentSensorParams);
  pMainCurrentSensor = &CurrentSensorParams;

  pREMNG[M1] = &RampExtMngrHFParamsM1;
  REMNG_Init(pREMNG[M1]);

  PID_HandleInit(&PIDFluxWeakeningHandle_M1);
  FW_Init(pFW[M1],pPIDSpeed[M1],&PIDFluxWeakeningHandle_M1);

  FOC_Clear(M1);
  FOCVars[M1].max_i_batt = 100;
  FOCVars[M1].min_i_batt = -100;
  FOCVars[M1].bDriveInput = EXTERNAL;
  FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
  FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;
  oMCInterface[M1] = & Mci[M1];
  MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1] );
  MCI_ExecSpeedRamp(oMCInterface[M1],
  STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0);
  pMCIList[M1] = oMCInterface[M1];
  MCT[M1].pPIDSpeed = pPIDSpeed[M1];
  MCT[M1].pPIDIq = pPIDIq[M1];
  MCT[M1].pPIDId = pPIDId[M1];
  MCT[M1].pPIDFluxWeakening = &PIDFluxWeakeningHandle_M1;
  MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
  MCT[M1].pRevupCtrl = MC_NULL;
  MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &HALL_M1;
  MCT[M1].pSpeedSensorAux = MC_NULL;
  MCT[M1].pSpeedSensorVirtual = MC_NULL;
  MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
  MCT[M1].pStateMachine = &STM[M1];
  MCT[M1].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M1];
  MCT[M1].pMainCurrentSensor = (CURR_Handle_t *) pMainCurrentSensor;
  MCT[M1].pBusVoltageSensor = &(pBusSensorM1->_Super);
  MCT[M1].pBrakeDigitalOutput = MC_NULL;
  MCT[M1].pNTCRelay = MC_NULL;
  MCT[M1].pMPM =  (MotorPowMeas_Handle_t*)pMPM[M1];
  MCT[M1].pFW = pFW[M1];
  MCT[M1].pFF = MC_NULL;

  MCT[M1].pPosCtrl = MC_NULL;

  MCT[M1].pSCC = MC_NULL;
  MCT[M1].pOTT = MC_NULL;
  pMCTList[M1] = &MCT[M1];

#if MUSIC_ENABLE
  int32_t status = music_init(&bldc_music);
  if(status){
  	set_music_command(Music_OFF, &bldc_music);
  }
#endif

  bMCBootCompleted = 1;
}

__weak void MC_RunMotorControlTasks(void)
{
  if ( bMCBootCompleted ) {
    MC_Scheduler();
  }
}

__weak void MC_Scheduler(void)
{
  if (bMCBootCompleted == 1)
  {
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();
      hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
    }
    if(hBootCapDelayCounterM1 > 0u)
    {
      hBootCapDelayCounterM1--;
    }
    if(hStopPermanencyCounterM1 > 0u)
    {
      hStopPermanencyCounterM1--;
    }
  }
}

__weak void TSK_MediumFrequencyTaskM1(void)
{
  State_t StateM1;
  int16_t wAux = 0;

  bool IsSpeedReliable = HALL_CalcAvrgMecSpeedUnit( &HALL_M1, &wAux );
  PQD_CalcElMotorPower( pMPM[M1] );

  StateM1 = STM_GetState( &STM[M1] );

  switch ( StateM1 )
  {
  case IDLE_START:
    R3_2_TurnOnLowSides( pwmcHandle[M1] );
    TSK_SetChargeBootCapDelayM1( CHARGE_BOOT_CAP_TICKS );
    STM_NextState( &STM[M1], CHARGE_BOOT_CAP );
    break;

  case CHARGE_BOOT_CAP:
    if ( TSK_ChargeBootCapDelayHasElapsedM1() )
    {
      PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_START );
      STM_NextState(&STM[M1],OFFSET_CALIB);
    }
    break;

  case OFFSET_CALIB:
    if ( PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_EXEC ) )
    {
      STM_NextState( &STM[M1], CLEAR );
    }
    break;

  case CLEAR:
    HALL_Clear( &HALL_M1 );

    if ( STM_NextState( &STM[M1], START ) == true )
    {
      FOC_Clear( M1 );
      R3_2_SwitchOnPWM( pwmcHandle[M1] );
    }
    break;

  case START:
    {
        STM_NextState( &STM[M1], START_RUN );
    }
    break;

  case START_RUN:
    {
      FOC_InitAdditionalMethods(M1);
      FOC_CalcCurrRef( M1 );
      STM_NextState( &STM[M1], RUN );
    }
    STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1] );
    MCI_ExecBufferedCommands( oMCInterface[M1] );

    break;

  case RUN:
    MCI_ExecBufferedCommands( oMCInterface[M1] );
    FOC_CalcCurrRef( M1 );

    if( !IsSpeedReliable )
    {
      STM_FaultProcessing( &STM[M1], MC_SPEED_FDBK, 0 );
    }
    break;

  case ANY_STOP:
    R3_2_SwitchOffPWM( pwmcHandle[M1] );
    FOC_Clear( M1 );
    MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M1] );
    TSK_SetStopPermanencyTimeM1( STOPPERMANENCY_TICKS );
    STM_NextState( &STM[M1], STOP );
    break;

  case STOP:
    if ( TSK_StopPermanencyTimeHasElapsedM1() )
    {
      STM_NextState( &STM[M1], STOP_IDLE );
    }
    break;

  case STOP_IDLE:
    STM_NextState( &STM[M1], IDLE );
    break;

  default:
    break;
  }
}

__weak void FOC_Clear(uint8_t bMotor)
{
  ab_t NULL_ab = {(int16_t)0, (int16_t)0};
  qd_t NULL_qd = {(int16_t)0, (int16_t)0};
  alphabeta_t NULL_alphabeta = {(int16_t)0, (int16_t)0};

  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
  FOCVars[bMotor].Iqdref = NULL_qd;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
  PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  if (pFW[bMotor])
  {
    FW_Clear(pFW[bMotor]);
  }
}
__weak void FOC_InitAdditionalMethods(uint8_t bMotor)
{
}

__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
  qd_t IqdTmp;

  if(FOCVars[bMotor].bDriveInput == INTERNAL)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    FOCVars[bMotor].Iqdref.q = FOCVars[bMotor].hTeref;

    int32_t batt_i = (int32_t)((int32_t)FOCVars[bMotor].Iqdref.q * (int32_t)FW_M1.AvVolt_qd.q) / 32768;
    batt_i = abs(batt_i);
    int16_t q_temp = FOCVars[bMotor].Iqdref.q;

    uint8_t regen = false;
    if(FOCVars[bMotor].Iqdref.q > 0 && FW_M1.AvVolt_qd.q <0){
    	regen = true;
    }else if(FOCVars[bMotor].Iqdref.q < 0 && FW_M1.AvVolt_qd.q > 0){
    	regen = true;
    }
    if(regen){
    	if((-batt_i < FOCVars[bMotor].min_i_batt) ){
    		q_temp = (int32_t)((int32_t)FOCVars[bMotor].min_i_batt * 32768) / (int32_t)FW_M1.AvVolt_qd.q;
    	}
    }else{
    	if((batt_i > FOCVars[bMotor].max_i_batt) ){
    		q_temp = (int32_t)((int32_t)FOCVars[bMotor].max_i_batt * 32768) / (int32_t)FW_M1.AvVolt_qd.q;
    	}
    }

    if(FOCVars[bMotor].Iqdref.q < 0 && q_temp > 0){
    	q_temp = -q_temp;
    }

    FOCVars[bMotor].Iqdref.q = VescToSTM_Iq_lim_hook(q_temp);

    if (pFW[bMotor])
    {
       IqdTmp.q = FOCVars[bMotor].Iqdref.q;
       IqdTmp.d = FOCVars[bMotor].UserIdref;
       FOCVars[bMotor].Iqdref = FW_CalcCurrRef(pFW[bMotor],IqdTmp);
    }

  }else{
	  FOCVars[bMotor].Iqdref.q =  VescToSTM_Iq_lim_hook(FOCVars[bMotor].Iqdref.q);
  }
}

__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
__weak uint8_t TSK_HighFrequencyTask(void)
{
	uint32_t cycles = *DWT_CYCCNT;

	uint8_t bMotorNbr = 0;
	uint16_t hFOCreturn;

	HALL_CalcElAngle (&HALL_M1);

	hFOCreturn = FOC_CurrControllerM1();

	if(samples.state == SAMP_START){
		switch(samples.mode){
		case DEBUG_SAMPLING_NOW:
			samples.state = SAMP_SAMPLING;
			break;
		case DEBUG_SAMPLING_START:
			if(FOCVars[M1].Iqdref.q){
				samples.state = SAMP_SAMPLING;
			}
			break;
		default:
			break;
		}
	}

	if(samples.state == SAMP_SAMPLING && samples.n_samp){
		samples.dec_state++;
		if(samples.dec_state == samples.dec){
			samples.dec_state = 0;
			samples.m_curr0_samples[samples.index] = (uint16_t)FOCVars[M1].Iab.a >>4;
			samples.m_curr1_samples[samples.index] = (uint16_t)FOCVars[M1].Iab.b >>4;
			samples.m_curr0_samples[samples.index] |= (((uint16_t)HALL_M1._Super.hElAngle<<4) & 0xF000);
			samples.m_curr1_samples[samples.index] |= (uint16_t)HALL_M1._Super.hElAngle & 0xF000;
#if SCOPE_UVW == 1
			samples.m_v0_samples[samples.index] = (uint16_t)PWM_Handle_M1._Super.CntPhA >>3;
			samples.m_v1_samples[samples.index] = (uint16_t)PWM_Handle_M1._Super.CntPhB >>3;
			samples.m_v2_samples[samples.index] = (uint16_t)PWM_Handle_M1._Super.CntPhC >>3;
#endif

			samples.index++;
			if(samples.index == samples.n_samp){
				samples.dec_state = 0;
				samples.index = 0;
				samples.state = SAMP_FINISHED;
			}
		}
	}

	if(hFOCreturn == MC_FOC_DURATION){
		STM_FaultProcessing(&STM[M1], MC_FOC_DURATION, 0);
	}

	FOCVars[M1].cycles_last = *DWT_CYCCNT - cycles;
	if(FOCVars[M1].cycles_last > FOCVars[M1].cycles_max){
		FOCVars[M1].cycles_max = FOCVars[M1].cycles_last;
	}

	return bMotorNbr;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
#define AVG_SAMPLES 512

inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;

  int16_t hElAngle;
  uint16_t hCodeError;
  SpeednPosFdbk_Handle_t *speedHandle;

#if MUSIC_ENABLE
  music_update(&bldc_music);
#endif

  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
#if MUSIC_ENABLE
  Vqd.q += bldc_music.music_signal;
#endif

  Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
  if(abs(Vqd.q) < FOCVars[M1].min_duty) Vqd.q = 0;
  if(abs(Vqd.d) < FOCVars[M1].min_duty) Vqd.d = 0;

  Vqd = Circle_Limitation(pCLM[M1], Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;
  FW_DataProcess(pFW[M1], Vqd, Iqd);
  return(hCodeError);
}

__weak void TSK_SafetyTask(void)
{
	if (bMCBootCompleted == 1)
	{
		TSK_SafetyTask_PWMOFF(M1);
		RCM_ExecUserConv ();
	}
}

uint8_t last_pwm_state=false;

__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  uint16_t CodeReturn = MC_NO_ERROR;
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};

  CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(pTemperatureSensor[bMotor]);
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);
  CodeReturn |= CURR_CalcMainCurrent(pMainCurrentSensor);

  if(bMotor == M1)
  {
	  uint16_t voltage_fault = RVBS_CalcAvVbus(pBusSensorM1);
	  if(voltage_fault==MC_UNDER_VOLT){
		  CodeReturn |=  errMask[bMotor] & voltage_fault;

		  VescToSTM_pwm_stop();
		  last_pwm_state = false;
	  }else if (voltage_fault==MC_OVER_VOLT){
		  CodeReturn |=  errMask[bMotor] & voltage_fault;

		  VescToSTM_pwm_stop();
		  last_pwm_state = false;
	  }else if (last_pwm_state==false){
		  if(CodeReturn == MC_NO_ERROR){
			  VescToSTM_pwm_start();
			  last_pwm_state=true;
		  }
	  }
  }

  STM_FaultProcessing(&STM[bMotor], CodeReturn, ~CodeReturn);
  switch (STM_GetState(&STM[bMotor]))
  {
  case FAULT_NOW:
	VescToSTM_pwm_stop();
    FOC_Clear(bMotor);
    MPM_Clear((MotorPowMeas_Handle_t*)pMPM[bMotor]);
    break;
  case FAULT_OVER:
	VescToSTM_pwm_stop();
    break;
  default:
    break;
  }
}

__weak MCI_Handle_t * GetMCI(uint8_t bMotor)
{
  MCI_Handle_t * retVal = MC_NULL;
  if (bMotor < NBR_OF_MOTORS)
  {
    retVal = oMCInterface[bMotor];
  }
  return retVal;
}

__weak MCT_Handle_t* GetMCT(uint8_t bMotor)
{
  MCT_Handle_t* retVal = MC_NULL;
  if (bMotor < NBR_OF_MOTORS)
  {
    retVal = &MCT[bMotor];
  }
  return retVal;
}

__weak void TSK_HardwareFaultTask(void)
{
  R3_2_SwitchOffPWM(pwmcHandle[M1]);
  STM_FaultProcessing(&STM[M1], MC_SW_ERROR, 0);
}

void startMediumFrequencyTask(void const * argument)
{
  for(;;)
  {
    vTaskDelay(1);
    MC_RunMotorControlTasks();
  }
}

void StartSafetyTask(void const * argument)
{
	vTaskDelay(200);
  for(;;)
  {
    vTaskDelay(1);
    TSK_SafetyTask();
    task_PWR(NULL);
  }
}

__weak void mc_lock_pins (void)
{
LL_GPIO_LockPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin);
LL_GPIO_LockPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin);
LL_GPIO_LockPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin);
LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
LL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
LL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
LL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
#ifdef M365
LL_GPIO_LockPin(M1_TEMPERATURE_GPIO_Port, M1_TEMPERATURE_Pin);
#endif
LL_GPIO_LockPin(M1_CURR_AMPL_W_GPIO_Port, M1_CURR_AMPL_W_Pin);
// LL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
LL_GPIO_LockPin(M1_CURR_AMPL_U_GPIO_Port, M1_CURR_AMPL_U_Pin);
LL_GPIO_LockPin(M1_CURR_AMPL_V_GPIO_Port, M1_CURR_AMPL_V_Pin);
#ifdef G30P
LL_GPIO_LockPin(MAIN_CURR_AMPL_GPIO_Port, MAIN_CURR_AMPL_Pin);
#endif
}
