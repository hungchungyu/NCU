//pma
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2807x_device.h"     // DSP2807x Headerfile Include File
#include "F2807x_Examples.h"   // DSP2807x Examples Include File
#include "math.h"
#include "SPEED_PI.h"
#include "CURRENT_PI.h"
#include "SVPWM.h"
#include "encoder.h"
#include "reference_model.h"             //0610++
#include "ADC.h"                         //0610++
#include "three_phase_PI.h"        //三相獨立PI 0815++
#include "speed_encoder.h"


#define REFERENCE_VDAC     0 ////++++
#define REFERENCE_VREF     1 ////+++++


void ConfigureADC(void);//
void ConfigureEPWM(void);//adc 中斷的epwm

void SetupADCEpwm1(Uint16 channel);
void SetupADCEpwm2(Uint16 channel);//+++
void SetupADCEpwm3(Uint16 channel);//+++


interrupt void adca1_isr(void);


void InitSpi (void);                                           //(SPI)
void Gpio_Init();                                              //(SPI)
void spi_xmit();                                               //(SPI)


#define RESULTS_BUFFER_SIZE 256
Uint16 AdcbResults[RESULTS_BUFFER_SIZE];
Uint16 AdcaResults[RESULTS_BUFFER_SIZE];
Uint16 resultsIndex;
Uint16 bufferFull;


extern  int16 SPI_CH1,SPI_CH2,SPI_CH3,SPI_CH4;


interrupt void epwm2_current_timer_isr(void);//中斷:先 (內迴路)
interrupt void epwm6_current_timer_isr(void);//中斷:後 (外迴路)


void repInit(void);
void InitEPwm9_10_11(void);
void designepwm(void);
void clarkeconver(float Acurrent,float Bcurrent,float Ccurrent);
void parkconver(float apha,float beta,float theta);
void inparkconver(float ind,float inq,float theta);
void inclarkeconver(float inapha,float inbeta);
void dac();
void delay_loop();
double BSCRLFNN(double wrm_rad_cmd, double wrm_rad);
double CureFitting(float iq_cmd);



//fpwm=10kHz  Tpwm=1/fpwm=2*TBPRD*TBLCK  TBPRD=6000
struct clarkes
{
	float apha,beta;
}clarke_varible={0,0};

struct parks
{
	float id;
	float iq;
}park_varible={0,0};

struct inclarkes
{
	float vref1;
	float vref2;
	float vref3;
}inclarke_varible={0,0,0};

struct inparks
{
	float inapha;
	float inbeta;
}inpark_varible={0,0};

/*******dq軸電流*******/
float id_cmd=0;
float iq_cmd=0;
float error_id=0;
float error_iq=0;
/*******uvw電流*******/
//////////////////////////////////////////////三項獨立電流控制/////////////////////////////////////////////////////
float iu_cmd = 0;
float iv_cmd = 0;
float iw_cmd = 0;
float error_iu=0;
float error_iv=0;
float error_iw=0;
float Uu_cmd=0;
float Uv_cmd=0;
float Uw_cmd=0;
//////////////////////////////////////////////三項獨立電流控制/////////////////////////////////////////////////////
/*******dq軸電壓*******/
float Ud_cmd=0;
float Uq_cmd=0;
float Ud_cmd_limit=0;
/*******dq軸電壓*******/

/*******電器參數*******/

/*******電器參數*******/

float theta=0;
float theta_PI=0;
float Wm_ref=0;
float wm_error=0;

int reset=1;//重新初始
//========================Sin Wave
double  Sin_Angle = 0;
int  Angle_Cnt = 0;
//========================Square Wave
double  Square_Angle = 0;	
//========================BSCRLFNN
double wrm_rad_cmd_dot = 0;
double wrm_rad_cmd_old = 0;
double wrm_rad_dot = 0;
double wrm_rad_old = 0;
double error_1 = 0;
double error_1_dot = 0;
double error_2 = 0;
double lambda_1 = 0;

double C1 = 1;

double mean[6] = { 1,1,1,1.1,1,1 };
double sigma[6] = { 1,1,1,1,1,1 };
double xin1, xin2;
//float Wjl[9];
double wl[9] = { 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 };
double wk[9] = { 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 };


double yout2[6];
double yout3[9];
double yout4;

double pre_yout3[9] = { 1,1,1,1,1,1,1,1,1 };
double iq_cmd = 0;
double phi_het = 0;
double dot_phi_het;
double gama = 18;
double phi_old;
double fresh[6] = {0};

double n_w = 0.01;
double n_mean = 0.175;
double n_sigma = 0.017;
double n_wl = 0.22;
double n_MP = 0.95;

double L1;
double L2;
double L3;
double L4;
double L5;
double L6;
double L7;
double L8;
double xin1_2;
double xin1_3;
double xin2_2;
double xin2_3;
double w_MP[9] = { 1,1,1,1,1,1,1,1,1 };
double yout_L[9];
double yout_c_delta_l[9];
int count3, count6, count9;
double xin_mean[6];
double sigma_square[6];
double rfnngain_1;
//========================
Uint16 sdata1;  // send A channel data
Uint16 sdata2;  // send B channel data
Uint16 sdata3;  // send C channel data
Uint16 sdata4;  // send D channel data
Uint16 rdata;  // received data

float spi_1,spi_2,spi_3,spi_4;   //決定spi傳哪些資訊

int spi_offset1=0;
//float spi_mult1=0.1045;

int spi_offset2=2115;
//float spi_mult2=0.1045;

int spi_offset3=2105;
//float spi_mult3=0.1045;

int spi_offset4=2112;
//float spi_mult4=0.1045;



void main(void)
{


	InitSysCtrl();

	EALLOW;
	ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0  ;        //時脈經過(EPWMCLKDIV)初始設定會除2，設0不除頻
	EDIS;

	InitEPwm9Gpio();
	InitEPwm10Gpio();
	InitEPwm11Gpio();

	InitEPwm3Gpio();     //用GPIO4 看AD/DA中斷
	InitEPwm8Gpio();     //用GPIO14/15看中斷


	Gpio_Init();                         //0516++
	InitSpi();                           //0512++

	DINT;
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;

	InitPieVectTable();

	Encoder_Init();                      //theta calculate


    EALLOW;  // This is needed to write to EALLOW protected registers

	PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1 (ADDA)
	PieVectTable.EPWM2_INT = &epwm2_current_timer_isr;
	PieVectTable.EPWM6_INT = &epwm6_current_timer_isr;

    EDIS;    // This is needed to disable write to EALLOW protected registers


	InitEPwm9_10_11();
	designepwm();//interrupt

	//Configure the ADC and power it up
	ConfigureADC();

	//Configure the ePWM
	ConfigureEPWM();

	//Setup the ADC for ePWM triggered conversions on channel 0
	SetupADCEpwm1(2);//B2                                         電流迴授給這3個，不用開腳位
	SetupADCEpwm2(2);//A2
	SetupADCEpwm3(3);//B3


	IER |= M_INT3;//Enable CPU interrupts3//M_INT3為線

	IER |= M_INT1; //Enable group 1 interrupts//M_INT1為線

	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

	for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
	{
		AdcbResults[resultsIndex] = 0;
		AdcaResults[resultsIndex] = 0;//+++
	}
	resultsIndex = 0;
	bufferFull = 0;
	//中斷:先挑2組(INTx1~INTx12),再選線路(1~16)
	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;//Enable ePWM2_INT使能電流回路中斷//優先//INTx2為組//PIEIER3為線
	PieCtrlRegs.PIEIER3.bit.INTx6 = 1;//Enable ePWM6_INT使能位置(速度)回路中斷
	//enable PIE interrupt
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;//Enable adca1_INT (ADDA)中斷

	current_PI_varible.Uo_d=0;
	current_PI_varible.Uo_q=0;
	current_PI_varible.sumd=0;
	current_PI_varible.sumq=0;

	spi_1=0;
	spi_2=0;
	spi_3=0;
	spi_4=0;
//////////////////////////////////////////////三項獨立電流控制/////////////////////////////////////////////////////
	u_current_PI_varible.sumu = 0;
	u_current_PI_varible.Uo_u = 0;
	Uo_u_last = 0;

	v_current_PI_varible.sumv = 0;
	v_current_PI_varible.Uo_v = 0;
	Uo_v_last = 0;

	w_current_PI_varible.sumw = 0;
	w_current_PI_varible.Uo_w = 0;
	Uo_w_last = 0;

	u_current_PI_varible.kp_u =10;
	u_current_PI_varible.ki_u =1.4;

	v_current_PI_varible.kp_v =10;
	v_current_PI_varible.ki_v =1.4;

	w_current_PI_varible.kp_w =10;
	w_current_PI_varible.ki_w =1.4;
//////////////////////////////////////////////三項獨立電流控制/////////////////////////////////////////////////////
//	current_PI_varible.kp_q=0.5;
//	current_PI_varible.ki_q=0;

//	current_PI_varible.kp_d=0;
//	current_PI_varible.ki_d=0;
	//低速調整KP=0.001，KI=0.0005 id=1
//	speed_PI_varible.PI_kp=0.0028;//0.0028
//	speed_PI_varible.PI_ki=0.00005;//0.00005


	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;//sync ePWM
	EALLOW;
	DacaRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;//改VREF(3V)
	DacbRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;//改VREF(3V)
	DaccRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;//改VREF(3V)
	            //Enable D A C output
	DacaRegs.DACOUTEN.bit.DACOUTEN = 1;//output enable
	DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
	DaccRegs.DACOUTEN.bit.DACOUTEN = 1;
	EDIS;

	//take conversions indefinitely in loop
	do
	{
		//start ePWM
		EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
		EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

		//wait while ePWM causes ADC conversions, which then cause interrupts,
		//which fill the results buffer, eventually setting the bufferFull
		//flag
		while(!bufferFull);
		bufferFull = 0; //clear the buffer full flag
		EPwm1Regs.ETSEL.bit.SOCAEN = 0;  //disable SOCA
		EPwm1Regs.TBCTL.bit.CTRMODE = 3; //freeze counter
		asm("   ESTOP0");
	}while(1);
}
interrupt void epwm6_current_timer_isr(void)//epwm6中斷開始 speed 是1ms
{
/*
	EALLOW;

	GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;
	GpioCtrlRegs.GPBPUD.bit.GPIO41 =1;
	GpioCtrlRegs.GPBMUX1.bit.GPIO41 =0;

	GpioDataRegs.GPBSET.bit.GPIO41 = 1;
	int count;
	for(count=0;count<100;count++);
	GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1;

	EDIS;
*/
	Encoder_Speed_Cal ();
//======================================================速度命令==============================================
	if (reset == 1)       //當reset=1 時不給速度命令  且數值初始化  dsp pwm同臂為同high//晶體端為同low
	{
		repInit();
	}
	else if (reset == 0)       //當reset=0 時開始給速度命令  一切正常動作
	{

		if(Angle_Cnt > 5000)
			Angle_Cnt = 0;
		else
			Angle_Cnt++;

		Sin_Angle = 0.072/*進一次中斷要加0.072度*/*0.01745/*轉rad*/*Angle_Cnt;
		
		//Sin Wave
		Wm_ref = 1000 + 100*Sin(Sin_Angle);	//每5秒1個Sin波0，
		//Square Wave
		if(Angle_Cnt < = 1250)
			Wm_ref = 1000 + 100*Sin(Sin_Angle);
		else if(Angle_Cnt > 1250 && Angle_Cnt<=2500)
			Wm_ref = 1000 + 100;
		else if(Angle_Cnt > 2500 && Angle_Cnt<=3750)
			Wm_ref = 1000 + 100*Sin(Sin_Angle) + 100;
		else if(Angle_Cnt > 3750 && Angle_Cnt<=5000)
			Wm_ref = 1000;

		if (Wm_ref > 1100)                 
			Wm_ref = 1100;
		else if (Wm_ref < -1100)       
			Wm_ref = -1100;	            

		//wm_error = (Wm_ref - Encoder_Var.Speed_Rpm);
		//speed_PI(wm_error);

		BSCRLFNN(Wm_ref,Encoder_Var.Speed_Rpm);

	}

	EPwm6Regs.ETCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

interrupt void epwm2_current_timer_isr(void)         //current loop
{
	EALLOW;

	GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;
	GpioCtrlRegs.GPBPUD.bit.GPIO41 =1;
	GpioCtrlRegs.GPBMUX1.bit.GPIO41 =0;

	GpioDataRegs.GPBSET.bit.GPIO41 = 1;
	int count;
	for(count=0;count<100;count++);
	GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1;

	EDIS;
	Encoder_Cal();
//	theta_PI=(Encoder_Var.ElecTheta)*0.01745329251;             // 真實角度回授      *0.0174533將encoder回授角度單位(度)轉換成(徑度) *2pi/360=0.0174533
	theta=(Encoder_Var.ElecTheta)*0.0174533;
	//=======================三相電流回授=================================================================
	JiSuan();

	clarkeconver(ADC_varible.I_A, ADC_varible.I_B, ADC_varible.I_C);//電流回授給座標轉換

//	clarkeconver(inclarke_varible.vref1,inclarke_varible.vref2,inclarke_varible.vref3);
	if (reset == 1)                    //當reset=1  數值初始化  pwm同臂一high一low
	{
		repInit();
	}
	else if(reset == 0)             //當reset=0 時給電流命令        開器內迴路pi         一切正常動作
	{
		EALLOW;
		GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;    // Disable pull-up on GPIO16 (EPWM9A)
		GpioCtrlRegs.GPAPUD.bit.GPIO17 = 1;    // Disable pull-up on GPIO17 (EPWM9B)
		GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 1;   // Configure GPIO16 as EPWM9A
		GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 1;   // Configure GPIO17 as EPWM9B
		GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;   // Configure GPIO16 as EPWM9A
		GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;   // Configure GPIO17 as EPWM9B
		GpioDataRegs.GPASET.bit.GPIO16 = 1;   //Set High initially 設定初始狀態為不導通
		GpioDataRegs.GPASET.bit.GPIO17 = 1;   //Set High initially 設定初始狀態為不導通

		GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;    // Disable pull-up on GPIO18 (EPWM10A)
		GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;    // Disable pull-up on GPIO19 (EPWM10B)
		GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 1;   // Configure GPIO18 as EPWM10A
		GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 1;   // Configure GPIO19 as EPWM10B
		GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;   // Configure GPIO18 as EPWM10A
		GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;   // Configure GPIO19 as EPWM10B
		GpioDataRegs.GPASET.bit.GPIO18 = 1;   //Set High initially 設定初始狀態為不導通
		GpioDataRegs.GPASET.bit.GPIO19 = 1;   //Set High initially 設定初始狀態為不導通

		GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO20 (EPWM11A)
		GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO21 (EPWM11B)
		GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EPWM11A
		GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as tEPWM11B
		GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EPWM11A
		GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as EPWM11B
		GpioDataRegs.GPASET.bit.GPIO20 = 1;   //Set High initially 設定初始狀態為不導通
		GpioDataRegs.GPASET.bit.GPIO21 = 1;   //Set High initially 設定初始狀態為不導通
		EDIS;

        error_iq = iq_cmd-park_varible.iq;

		error_id = CureFitting(iq_cmd)-park_varible.id;
		

//////////////////////////////////////////////三項獨立電流控制/////////////////////////////////////////////////////
		iu_cmd = 1;
		iv_cmd = -0.5;
		iw_cmd = -0.5;

		error_iu = iu_cmd-ADC_varible.I_A;
		error_iv = iv_cmd-ADC_varible.I_B;
		error_iw = iw_cmd-ADC_varible.I_C;

		u_current_PI(error_iu);
		v_current_PI(error_iv);
		w_current_PI(error_iw);

		Uu_cmd = u_current_PI_varible.Uo_u;
		Uv_cmd = v_current_PI_varible.Uo_v;
		Uw_cmd = w_current_PI_varible.Uo_w;

		if(Uu_cmd>40)
		{
			Uu_cmd = 40; //DC SOURCE =80V
		}
		else if(Uu_cmd<-40)
		{
			Uu_cmd = -40;
		}

		if(Uv_cmd>40)
		{
			Uv_cmd = 40; //DC SOURCE =80V
		}
		else if(Uv_cmd<-40)
		{
			Uv_cmd = -40;
		}

		if(Uw_cmd>40)
		{
			Uw_cmd = 40; //DC SOURCE =80V
		}
		else if(Uw_cmd<-40)
		{
			Uw_cmd = -40;
		}

		spwm(Uu_cmd,Uv_cmd,Uw_cmd);
//////////////////////////////////////////////三項獨立電流控制/////////////////////////////////////////////////////
//		error_id = 0;
//		error_iq = 0;
//		error_iq = speed_PI_varible.speed_iq_ref - park_varible.iq;

//		current_PI(error_id,error_iq);

//		Ud_cmd = current_PI_varible.Uo_d;
//		Uq_cmd = current_PI_varible.Uo_q;

//		Ud_cmd = 0;
//		Uq_cmd = -30;

//		inparkconver(Ud_cmd,Uq_cmd,theta);
//		inparkconver(current_PI_varible.Uo_d,current_PI_varible.Uo_q,theta);

	}
	dac();

	EPwm2Regs.ETCLR.bit.INT =1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
	return;
}

void InitEPwm9_10_11(void)
{
	//PWM切10kHz//
	//開關A臂//
	// Setup TBCLK
	EPwm9Regs.TBPRD = 6000;       // 10kHz,pwm-Set timer period(1/10000)/(2*(1/120000000))=TBPRD
	EPwm9Regs.TBPHS.bit.TBPHS = 0x0000; //相位是0      // Phase is 0
	EPwm9Regs.TBCTR = 0x0000;//時間基準計數器(數有幾個三角波的計數位置為零)  // Clear counter

	EPwm9Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; //上下數模式 2
	EPwm9Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading 0

	EPwm9Regs.TBCTL.bit.HSPCLKDIV = 0;// Clock ratio to SYSCLKOUT TBCLK=(120MHz/(2^0*(1)))=120MHz
	EPwm9Regs.TBCTL.bit.CLKDIV = 0;

	// Setup shadow register load on ZERO
	EPwm9Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;//作為CMPA緩衝
	EPwm9Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;//作為CMPB緩衝
	EPwm9Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;//選擇什麼時候載入新的CMPA值(時間基準計數器為零時)
	EPwm9Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;//選擇什麼時候載入新的CMPB值(時間基準計數器為零時)

	// Set Compare values
	EPwm9Regs.CMPA.bit.CMPA =0;//0(100% duty); 6000(0% duty)   // Set compare A value ,pull low or pull high  0809  0

	EPwm9Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // Set PWM1A on Zero,數到set(為所設的6000)都為上數(初始值沒差) 1
	EPwm9Regs.AQCTLA.bit.CAD = AQ_SET;    // Clear PWM1A on event A up count 2

	// Active Low PWMs - Setup Deadband
	EPwm9Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;//EPWMxA、EPWMxB  DB全使能, (ePWMxA)rising-edge and (ePWMxB)falling-edge, DB_FULL_ENABLE=3
	EPwm9Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;//EPWMxA可以相反,low動作互補, DB_ACTV_LOC:1
	EPwm9Regs.DBCTL.bit.IN_MODE =DBA_ALL;//EPWMXxA是下降和上升延遲輸入源 , DBA_ALL:0

	EPwm9Regs.DBRED.all =240;//120是1us
	EPwm9Regs.DBFED.all =240;//360是3us
//=========================================================================================
	//開關B臂//
	EPwm10Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; //上下數模式
	EPwm10Regs.TBPRD = 6000;       // 10kHzpwm-Set timer period
	EPwm10Regs.TBCTL.bit.PHSEN =TB_DISABLE;    // Disable phase loading
	EPwm10Regs.TBPHS.bit.TBPHS =0x0000;       // Phase is 0
	EPwm10Regs.TBCTR = 0x0000;                  // Clear counter
	EPwm10Regs.TBCTL.bit.HSPCLKDIV = 0;// Clock ratio to SYSCLKOUT TBCLK=120MHz
	EPwm10Regs.TBCTL.bit.CLKDIV = 0;

	EPwm10Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm10Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm10Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm10Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set Compare values
	EPwm10Regs.CMPA.bit.CMPA =0;//0;    // Set compare A value  0809  0
	EPwm10Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM1A on Zero
	EPwm10Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM1A on event A, up count

	// Active Low PWMs - Setup Deadband
	EPwm10Regs.DBCTL.bit.OUT_MODE =DB_FULL_ENABLE;//EPWMxA、EPWMxB  DB全使能
	EPwm10Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;//EPWMxB可以相反
	EPwm10Regs.DBCTL.bit.IN_MODE = DBA_ALL;//EPWMXxA是下降和上升延遲輸入源;
	EPwm10Regs.DBRED.all =240;//120是1us
	EPwm10Regs.DBFED.all =240;//360是3us
//=========================================================================================
////////開關C臂/////////////////////////////////////////////////////////
	EPwm11Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; //上下數模式
	EPwm11Regs.TBPRD = 6000;       // 10kHzpwm-Set timer period
	EPwm11Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	EPwm11Regs.TBPHS.bit.TBPHS = 0x0000;       // Phase is 0
	EPwm11Regs.TBCTR = 0x0000;                  // Clear counter
	EPwm11Regs.TBCTL.bit.HSPCLKDIV = 0;// Clock ratio to SYSCLKOUT TBCLK=120MHz
	EPwm11Regs.TBCTL.bit.CLKDIV = 0;

	// Setup shadow register load on ZERO
	EPwm11Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm11Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm11Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm11Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set Compare values
	EPwm11Regs.CMPA.bit.CMPA =0;//0;    // Set compare A value  0809  0

	// Set actions
	EPwm11Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM1A on Zero
	EPwm11Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM1A on event A, up count

	// Active Low PWMs - Setup Deadband
	EPwm11Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;//EPWMxA、EPWMxB  DB全使能
	EPwm11Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;//EPWMxB可以相反
	EPwm11Regs.DBCTL.bit.IN_MODE =0;//EPWMXxA是下降和上升延遲輸入源;
	EPwm11Regs.DBRED.all =240;//120是1us
	EPwm11Regs.DBFED.all =240;//360是3us
}

void designepwm(void)
{
	   EALLOW;
	   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
	   EDIS;
	   ////電流回路epwm中斷初始10kHz,0.1ms
	   EPwm2Regs.CMPA.bit.CMPA =3000;	  // Set compare A value
	   ////電流回路epwm中斷初始10kHz,0.1ms
	   EPwm2Regs.TBPRD = 6000;              // Set period for ePWM1
	   ////電流回路epwm中斷初始20kHz,0.05ms
//	   EPwm2Regs.TBPRD = 3000;
	   EPwm2Regs.TBCTL.bit.CTRMODE = 2;		  // 上下計數count up and start
	   EPwm2Regs.TBCTL.bit.HSPCLKDIV=0;//0=1
	   EPwm2Regs.TBCTL.bit.CLKDIV=0;//2^0=1
	   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	   EPwm2Regs.ETSEL.bit.INTEN = 1;  // Enable INT
	   EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;// Generate INT on 1st event

	   EALLOW;
	   EPwm2Regs.ETSEL.bit.SOCAEN = 1;
	   EPwm2Regs.ETSEL.bit.SOCASEL = 2;
	   EPwm2Regs.ETPS.bit.SOCAPRD = 1;
	   EDIS;
	   /////位置(速度)迴路epwm中斷初始1kHz,1ms

	   EPwm6Regs.CMPA.bit.CMPA =30000;//1562//Set compare A value, 沒有超過2^16=65535就不用除頻//
	   EPwm6Regs.TBPRD =60000;//1562//1875              // Set period for ePWM1
	   EPwm6Regs.TBCTL.bit.CTRMODE = 2;		  // count up and start
	   EPwm6Regs.TBCTL.bit.HSPCLKDIV=0;//5
	   EPwm6Regs.TBCTL.bit.CLKDIV=0;//2
	   EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	   EPwm6Regs.ETSEL.bit.INTEN = 1;  // Enable INT
	   EPwm6Regs.ETPS.bit.INTPRD = ET_1ST;//ET_1ST;

	   EALLOW;
	   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;         // Start all the timers synced,p.37
	   EDIS;

}
void clarkeconver(float Acurrent,float Bcurrent,float Ccurrent)
{
	clarke_varible.apha=(Acurrent*0.6667)-(Bcurrent*0.3333)-(Ccurrent*0.33333);//2/3=0.6667
	clarke_varible.beta=(Bcurrent*0.5773)-(Ccurrent*0.5773);//sqrt(3)/3=0.5773
	parkconver(clarke_varible.apha,clarke_varible.beta,theta);

}
void parkconver(float apha,float beta,float theta)
{
	park_varible.id=(apha*cos(theta))+(beta*sin(theta));
	park_varible.iq=(beta*cos(theta))-(apha*sin(theta));

}

void inparkconver(float ind,float inq,float theta)
{
	inpark_varible.inapha=ind*cos(theta)-inq*sin(theta);
	inpark_varible.inbeta=ind*sin(theta)+inq*cos(theta);
	inclarkeconver(inpark_varible.inapha,inpark_varible.inbeta);
}
void inclarkeconver(float inapha,float inbeta)
{
/*
	inclarke_varible.vref1=inbeta;                        //svpwm
	inclarke_varible.vref2=-(0.5*inbeta)+(0.866*inapha);
	inclarke_varible.vref3=-(0.5*inbeta)-(0.866*inapha);

	leo_svpwm(inclarke_varible.vref1,inclarke_varible.vref2,inclarke_varible.vref3);//svpwm
*/
	inclarke_varible.vref1=inapha;
	inclarke_varible.vref2=-(0.5*inapha)+(0.866*inbeta);//sqrt(3)/2=0.866
	inclarke_varible.vref3=-(0.5*inapha)-(0.866*inbeta);
	spwm(inclarke_varible.vref1,inclarke_varible.vref2,inclarke_varible.vref3);
//	zerot_svpwm(inclarke_varible.vref1,inclarke_varible.vref2,inclarke_varible.vref3);

}

void repInit(void)
{
	id_cmd=0;    //電流閉迴路 id命令
	iq_cmd=0;    //電流閉迴路 iq命令

	iu_cmd = 0;
	iv_cmd = 0;
	iw_cmd = 0;

	speed_PI_varible.speed_iq_ref = 0;

	id_error_last=0;
	Uo_d_last=0;
	current_PI_varible.sumd=0;
	current_PI_varible.Uo_d=0;

	iq_error_last=0;
	current_PI_varible.Uo_q=0;
	Uo_q_last=0;
	current_PI_varible.sumq=0;
/////////////////////////////////////////////////////////////////////////////
	error_iu = 0;
	error_iv = 0;
	error_iw = 0;

	u_current_PI_varible.sumu = 0;
	u_current_PI_varible.Uo_u = 0;
	Uo_u_last = 0;

	v_current_PI_varible.sumv = 0;
	v_current_PI_varible.Uo_v = 0;
	Uo_v_last = 0;

	w_current_PI_varible.sumw = 0;
	w_current_PI_varible.Uo_w = 0;
	Uo_w_last = 0;

	Uu_cmd = 0;
	Uv_cmd = 0;
	Uw_cmd = 0;
	/////////////////////////////////////////////////////////////////////////////

	Ud_cmd=0;
	Uq_cmd=0;

	inpark_varible.inapha=0;
	inpark_varible.inbeta=0;

	inclarke_varible.vref1=0;
	inclarke_varible.vref2=0;
	inclarke_varible.vref3=0;

	zsvpwm_varible.ta = 0;
	zsvpwm_varible.tb = 0;
	zsvpwm_varible.tc = 0;
	zsvpwm_varible.tmax = 0;
	zsvpwm_varible.tmin = 0;
	zsvpwm_varible.teff = 0;
	zsvpwm_varible.to = 0;

	inclarke_varible.vref1=0;
	inclarke_varible.vref2=0;
	inclarke_varible.vref3=0;

	EPwm9Regs.CMPA.bit.CMPA = 0;
	EPwm10Regs.CMPA.bit.CMPA = 0;
	EPwm11Regs.CMPA.bit.CMPA = 0;

	ADC_varible.AD_V[0]=0;
	ADC_varible.AD_V[1]=0;
	ADC_varible.AD_V[2]=0;

	ADC_varible.I_A=0;
	ADC_varible.I_B=0;
	ADC_varible.I_C=0;

	clarke_varible.apha=0;
	clarke_varible.beta=0;

	park_varible.id=0;
	park_varible.iq=0;

	error_id=0;
	error_iq=0;

	Ud_cmd_limit=0;

	speed_PI_varible.speed_sum = 0;
	speed_PI_varible.speed_iq_ref_last = 0;
	Encoder_Var.Speed_Rpm = 0;
	Wm_ref=500;
	wm_error=0;

	Encoder_Var.x_old = 0;
	Encoder_Var.tmp = 0;
	Encoder_Var.last_tmp = 0;
	speed_PI_varible.Wm_error_last = 0;



	EALLOW;

	GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;    // Disable pull-up on GPIO16 (EPWM9A)
	GpioCtrlRegs.GPAPUD.bit.GPIO17 = 1;    // Disable pull-up on GPIO17 (EPWM9B)
	GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0;   // Configure GPIO16 as GPIO notEPWM9A
	GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 0;   // Configure GPIO17 as GPIO notEPWM9B
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;   // Configure GPIO16 as GPIO notEPWM9A
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;   // Configure GPIO17 as GPIO notEPWM9B
	GpioDataRegs.GPASET.bit.GPIO16 = 0;   //Set High initially 設定初始狀態為不導通
	GpioDataRegs.GPASET.bit.GPIO17 = 0;   //Set High initially 設定初始狀態為不導通

	GpioCtrlRegs.GPADIR.bit.GPIO16 = 1; 	//output
	GpioDataRegs.GPADAT.bit.GPIO16 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO17 = 1; 	//output
	GpioDataRegs.GPADAT.bit.GPIO17 = 0;

	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;    // Disable pull-up on GPIO18 (EPWM10A)
	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;    // Disable pull-up on GPIO19 (EPWM10B)
	GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 0;   // Configure GPIO18 as GPIO notEPWM10A
	GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 0;   // Configure GPIO19 as GPIO notEPWM10B
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;   // Configure GPIO18 as GPIO notEPWM10A
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;   // Configure GPIO19 as GPIO notEPWM10B
	GpioDataRegs.GPASET.bit.GPIO18 = 0;   //Set High initially 設定初始狀態為不導通
	GpioDataRegs.GPASET.bit.GPIO19 = 0;   //Set High initially 設定初始狀態為不導通

	GpioCtrlRegs.GPADIR.bit.GPIO18 = 1; 	//output
	GpioDataRegs.GPADAT.bit.GPIO18 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO19 = 1; 	//output
	GpioDataRegs.GPADAT.bit.GPIO19 = 0;

	GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO20 (EPWM11A)
	GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO21 (EPWM11B)
	GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 0;   // Configure GPIO20 as GPIO not EPWM11A
	GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 0;   // Configure GPIO21 as GPIO notEPWM11B
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;   // Configure GPIO20 as GPIO notEPWM11A
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;   // Configure GPIO21 as GPIO notEPWM11B
	GpioDataRegs.GPASET.bit.GPIO20 = 0;   //Set High initially 設定初始狀態為不導通
	GpioDataRegs.GPASET.bit.GPIO21 = 0;   //Set High initially 設定初始狀態為不導通


	GpioCtrlRegs.GPADIR.bit.GPIO20 = 1; 	//output
	GpioDataRegs.GPADAT.bit.GPIO20 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO21 = 1; 	//output
	GpioDataRegs.GPADAT.bit.GPIO21 = 0;


	EDIS;
}

//Write ADC configurations and power up the ADC for both ADC A and ADC B
void ConfigureADC(void)
{
	EALLOW;

	AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4++++
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	//Set pulse positions to late
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//power up the ADC
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;//+++
	//delay for 1ms to allow ADC time to power up
	DELAY_US(1000);

	EDIS;
}

void ConfigureEPWM(void)
{
	EALLOW;
	// Assumes ePWM clock is already enabled
	EPwm1Regs.ETSEL.bit.SOCAEN	= 0;	        // Disable SOC on A group
	EPwm1Regs.ETSEL.bit.SOCASEL	= 4;	        // Select SOC on up-count
	EPwm1Regs.ETPS.bit.SOCAPRD = 1;		        // Generate pulse on 1st event
	EPwm1Regs.CMPA.bit.CMPA = 0x0800;          // Set compare A value to 2048 counts

	EPwm1Regs.TBPRD = 0x1000;			        // Set period to 4096 counts
	EPwm1Regs.TBCTL.bit.CTRMODE = 3;            // freeze counter
	EDIS;
}

void SetupADCEpwm1(Uint16 channel)
{
	Uint16 acqps;

	//determine minimum acquisition window (in SYSCLKS) based on resolution
	if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION){
		acqps = 14; //75ns
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}
	//Select the channels to convert and end of conversion flag
	EALLOW;
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles

	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
	EDIS;
}

void SetupADCEpwm2(Uint16 channel)
{
	Uint16 acqps;

	//determine minimum acquisition window (in SYSCLKS) based on resolution
	if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
		acqps = 14; //75ns
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}

	//Select the channels to convert and end of conversion flag
	EALLOW;
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
	EDIS;

}

void SetupADCEpwm3(Uint16 channel)//+++
{
	Uint16 acqps;

	//determine minimum acquisition window (in SYSCLKS) based on resolution
	if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION){
		acqps = 14; //75ns
		//acqps = 28;
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}

	//Select the channels to convert and end of conversion flag
	EALLOW;
	AdcbRegs.ADCSOC1CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
	AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
	AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
	EDIS;

}

interrupt void adca1_isr(void)
{

	//GpioDataRegs.GPATOGGLE.bit.GPIO4 = 1 ;       //檢測中斷頻率

	AdcbResults[resultsIndex++] = AdcbResultRegs.ADCRESULT0;
	AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT0;
	AdcbResults[resultsIndex++] = AdcbResultRegs.ADCRESULT1;

	if(RESULTS_BUFFER_SIZE <= resultsIndex)
	{
		resultsIndex = 0;
		bufferFull = 1;
	}
/*
	DacbRegs.DACVALS.bit.DACVALS = park_varible.iq*51.2+2048;              //iq回授  2048/40=51.2  除40做正規化
	DacaRegs.DACVALS.bit.DACVALS =wm_error1*0.4917+2048;                //2048/4165=0.4917 4165 normalize
	DacbRegs.DACVALS.bit.DACVALS = Encoder_Var.Speed_Rpm*0.8192+2048+35;    //encoder得到的速度    2048/2500=0.8192    除2500做正規化         35為offset補償
*/
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag+++s
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

//=====================================SPI==========================================================
void InitSpi (void)
{

	SpiaRegs.SPICCR.all =0x0000; //0x000F;	     // Reset on, rising edge, 16-bit char bits
	SpiaRegs.SPICTL.all =0x0006;    		     // Enable master mode, normal phase,
	                                                 // enable talk, and SPI int disabled.
	// Initialize SPI FIFO registers
	SpiaRegs.SPIFFTX.all=0xE040;                 //SPIFFENA = 1
	SpiaRegs.SPIFFRX.all=0x204F;                 //Receive FIFO has 4 words.  RXFFIENA = 1
	SpiaRegs.SPIFFCT.all=0x0;

	//LSPCLK = 30 MHz
	SpiaRegs.SPIBRR.all =2;                           //SPI Baud Rate = 7.5 Mbps
	SpiaRegs.SPIPRI.bit.FREE = 1;                 // Set so breakpoints don't disturb xmission
	//SpiaRegs.SPIPRI.all = 0x0010;
	SpiaRegs.SPICCR.all = 0x00DF;               // can't 0x009F(problem)	     // Relinquish SPI from Reset

}
//==================================================SPI============================================================

void Gpio_Init()
{
	EALLOW;
	GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3;	// Configure GPIO58 as SPISIMOA
	GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;    // Configure GPIO58 as SPISIMOA
	GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;		// 1=OUTput,  0=INput

	GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 3;	// Configure GPIO59 as SPISOMIA
	GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;    // Configure GPIO59 as SPISOMIA
	GpioCtrlRegs.GPBDIR.bit.GPIO59 = 0;		// 1=OUTput,  0=INput

	GpioCtrlRegs.GPBGMUX2.bit.GPIO60 =3 ;	// Configure GPIO60 as SPICLKA
	GpioCtrlRegs.GPBMUX2.bit.GPIO60 =3 ;    // Configure GPIO60 as SPICLKA
	GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;		// 1=OUTput,  0=INput

	GpioCtrlRegs.GPCGMUX2.bit.GPIO90 =0 ;	// Configure GPIO90 as GPIO90(CS1)
	GpioCtrlRegs.GPCMUX2.bit.GPIO90 =0 ;    // Configure GPIO90 as GPIO90(CS1)
	GpioCtrlRegs.GPCDIR.bit.GPIO90 = 1;		// 1=OUTput,  0=INput

	GpioCtrlRegs.GPCGMUX2.bit.GPIO91 =0 ;	// Configure GPIO91 as GPIO91(CS2)
	GpioCtrlRegs.GPCMUX2.bit.GPIO91 =0 ;    // Configure GPIO91 as GPIO91(CS2)
	GpioCtrlRegs.GPCDIR.bit.GPIO91 = 1;		// 1=OUTput,  0=INput

	GpioCtrlRegs.GPCGMUX2.bit.GPIO92 =0 ;	// Configure GPIO92 as GPIO92(LDAC)
	GpioCtrlRegs.GPCMUX2.bit.GPIO92 =0 ;    // Configure GPIO92 as GPIO92(LDAC)
	GpioCtrlRegs.GPCDIR.bit.GPIO92 = 1;		// 1=OUTput,  0=INput


//	GpioCtrlRegs.GPBMUX1.bit.GPIO41 =0;
//	GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;


	GpioCtrlRegs.GPBMUX1.bit.GPIO42 =0;
	GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;


	GpioCtrlRegs.GPBMUX1.bit.GPIO43 =0;
	GpioCtrlRegs.GPBDIR.bit.GPIO43 = 0;


	//-----SPI GPIO initial value -----

	GpioDataRegs.GPCDAT.bit.GPIO90 = 1; //(CS1)
	GpioDataRegs.GPCDAT.bit.GPIO91 = 1; //(CS2)
	GpioDataRegs.GPCDAT.bit.GPIO92 = 1; //(LDAC)

	EDIS;
}

void dac()
{
	//================================決定spi傳哪些資訊==================================

	//0~4095 => -10~10。 1V=>spi*204.8+2048


//	spi_1 = iu_cmd*204.8;
//	spi_2 = ADC_varible.I_A*204.8;

	spi_1 = iw_cmd*204.8;
	spi_2 = ADC_varible.I_C*204.8;;
	spi_3 = iv_cmd*204.8;
	spi_4 = ADC_varible.I_B*204.8;

/*
	spi_1 = Uu_cmd*20.48;
	spi_2 = Uv_cmd*20.48;
	spi_3 = Uw_cmd*20.48;
*/
	/*
	spi_1 = EQep3Regs.QPOSCNT*2;//2000/10000=0.2
	spi_2 = inclarke_varible.vref1*20.48;//2000/400=5;
	spi_3 = inclarke_varible.vref2*20.48;
	spi_4 = inclarke_varible.vref3*20.48;
*/

	/*
	spi_1 = EQep3Regs.QPOSCNT*2;
	spi_2 = inpark_varible.inapha*20.48;
	spi_3 = inpark_varible.inbeta*20.48;
*/
	/*
	spi_1 = ADC_varible.I_A*204.8;

	spi_2 = ADC_varible.I_B*204.8;

	spi_3 = ADC_varible.I_C*204.8;

	spi_4 = EQep3Regs.QPOSCNT*0.2;
*/
	/*
	spi_1 = iq_cmd*204.8;
	spi_2 = park_varible.iq*204.8;
	spi_3 = current_PI_varible.Uo_q*1.24;
	spi_4 = current_PI_varible.Uo_q*1.24;
*/
	/*
	spi_1 = iq_cmd*30;
	spi_2 = iq_cmd*30;
	spi_3 = iq_cmd*30;
	spi_4 = iq_cmd*30;
*/
	/*
	spi_1 = inclarke_varible.vref1*204.8;
	spi_2 = inclarke_varible.vref2*204.8;
	spi_3 = inclarke_varible.vref3*204.8;
	spi_4 = Uq_cmd*204.8;
*/
	/*
	spi_1 = current_PI_varible.Uo_d*204.8;
	spi_2 = current_PI_varible.Uo_q*204.8;
	spi_3 = inpark_varible.inapha*204.8;
	spi_4 = inpark_varible.inbeta*204.8;
*/
	/*
	spi_1 = EPwm9Regs.CMPA.bit.CMPA*0.33;
	spi_2 = EPwm10Regs.CMPA.bit.CMPA*0.33;
	spi_3 = EPwm11Regs.CMPA.bit.CMPA*0.33;
*/
/*
	spi_1 = EQep3Regs.QPOSCNT*2.7;
	spi_2 = Encoder_Var.ElecTheta*5.5;
*/

	/*
	spi_1 = clarke_varible.apha*204.8;
	spi_2 = clarke_varible.beta*204.8;
*/
	/*
	spi_1 = Wm_ref*0.4096;
	spi_2 = Encoder_Var.Speed_Rpm*0.4096;
	spi_3 = park_varible.id*204.8;
	spi_4 = park_varible.iq*204.8;
*/




	//=======================( CH1 - Q24 )=====================
    //GpioDataRegs.GPCDAT.bit.GPIO90 = 0;   //Start transmit Data==>CS1=low
  	GpioDataRegs.GPCCLEAR.bit.GPIO90 = 1;   //Start transmit Data==>CS1=low

  	// +28672為使用DACa


//	sdata1=28672+(spi_1*2048+2048)+spi_offset1;//28672= 0111 0000 0000 0000 (data只能用後面的12bit去傳0~4095)
	sdata1=28672+spi_1+2090; //2160對應到0V，滿是4095
//  sdata1=28672+(spi_1);

	spi_xmit(sdata1);

	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }	 // Wait until data is received

  	//GpioDataRegs.GPCDAT.bit.GPIO90 = 1;   //transmit over==>CS1=high
	GpioDataRegs.GPCSET.bit.GPIO90 = 1;   //transmit over==>CS1=high

  	//GpioDataRegs.GPCDAT.bit.GPIO92 = 0;   //transmit over==> LDAC=low to high
	GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1;   //transmit over==> LDAC=low to high

	delay_loop();

  	//GpioDataRegs.GPCDAT.bit.GPIO92 = 1;
	GpioDataRegs.GPCSET.bit.GPIO92 = 1;
  	  // Check against sent data
	rdata = SpiaRegs.SPIRXBUF;

	//========================( CH2 - Q24 )====================

  	//	GpioDataRegs.GPCDAT.bit.GPIO90 = 0;   //Start transmit Data==>CS1=low
	GpioDataRegs.GPCCLEAR.bit.GPIO90 = 1;   //Start transmit Data==>CS1=low


//	sdata2= 61440+(spi_2*2048+2048)+spi_offset2;         //offset -7
	sdata2 = 61440+(spi_2)+2080;//2130 對應到0V

	spi_xmit(sdata2);

	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }	 // Wait until data is received

	//GpioDataRegs.GPCDAT.bit.GPIO90 = 1;   //transmit over==>CS1=high
	GpioDataRegs.GPCSET.bit.GPIO90 = 1;   //transmit over==>CS1=high

	 // GpioDataRegs.GPCDAT.bit.GPIO92 = 0;   //transmit over==> LDAC=low to high
	GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1;   //transmit over==> LDAC=low to high

	delay_loop();

	 // GpioDataRegs.GPCDAT.bit.GPIO92 = 1;
	GpioDataRegs.GPCSET.bit.GPIO92 = 1;
	  	  		  	  // Check against sent data
	rdata = SpiaRegs.SPIRXBUF;

	//========================( CH3 - Q24 )====================

	// 	GpioDataRegs.GPCDAT.bit.GPIO91 = 0;   //Start transmit Data==>CS1=low
	GpioDataRegs.GPCCLEAR.bit.GPIO91 = 1;   //Start transmit Data==>CS2=low

//	sdata3=28672+(spi_3*2048+2048)+spi_offset3;
	sdata3=28672+(spi_3)+2080;//2120 對應到0V
	spi_xmit(sdata3);

	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }	 // Wait until data is received

	GpioDataRegs.GPCSET.bit.GPIO91 = 1;   //transmit over==>CS2=high

	GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1;   //transmit over==> LDAC=low to high

	delay_loop();

	GpioDataRegs.GPCSET.bit.GPIO92 = 1;

	rdata = SpiaRegs.SPIRXBUF;


	//========================( CH4 - Q24 )====================
  	//GpioDataRegs.GPCDAT.bit.GPIO91 = 0;   //Start transmit Data==>CS1=low
	GpioDataRegs.GPCCLEAR.bit.GPIO91 = 1;   //Start transmit Data==>CS2=low

//	sdata4=61440+(spi_4*2048+2048);           //offset -2
	sdata4=61440+(spi_4)+2080;
	spi_xmit(sdata4);

	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }	 // Wait until data is received

	// GpioDataRegs.GPCDAT.bit.GPIO91 = 1;   //transmit over==>CS1=high
	GpioDataRegs.GPCSET.bit.GPIO91 = 1;   //transmit over==>CS2=high

	//GpioDataRegs.GPCDAT.bit.GPIO92 = 0;   //transmit over==> LDAC=low to high
	GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1;   //transmit over==> LDAC=low to high

	delay_loop();

	//GpioDataRegs.GPCDAT.bit.GPIO92 = 1;
	GpioDataRegs.GPCSET.bit.GPIO92 = 1;
	  	  				  		  // Check against sent data
	  rdata = SpiaRegs.SPIRXBUF;
}

void delay_loop()
{
    int a;
    for (a = 0; a < 2; a++) {}
}

void spi_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF=a;
}

double BSCRLFNN(double wrm_rad_cmd, double wrm_rad)
{
	wrm_rad_cmd_dot = (wrm_rad_cmd - wrm_rad_cmd_old) / DELTA_T1;//轉速命令的微分 = (轉速命令-上一次的轉速命令)/delta_T1
	wrm_rad_cmd_old = wrm_rad_cmd;//再讓上一次的轉速命令迴傳

	wrm_rad_dot = (wrm_rad - wrm_rad_old) / DELTA_T1;//轉速的微分 = (轉速-上一次轉速)/delta_T1
	wrm_rad_old = wrm_rad;//再讓上一次的轉速迴傳

	error_1 = wrm_rad_error;//e1=轉速命令-轉速  (6)
	error_1_dot = wrm_rad_cmd_dot - wrm_rad_dot;//e1微分=轉速命令微分-轉速微分  (7)

	lambda_1 = -C1 * error_1 - wrm_rad_cmd_dot;//  ??1 =-c1e1-轉速命令微分  (8)

	error_2 = wrm_rad_dot+ lambda_1;

	//===================================layer1 input layer================================================

	xin1 = error_1/1000 ;
	xin2 = error_2/100000 ;


	//===================================layer2 membership layer================================================
	for (count3 = 0; count3 < 3; count3++)//calculate xin1-mean & xin2-mean
	{
		xin_mean[count3] = xin1 - mean[count3];
		xin_mean[count3 + 3] = xin2 - mean[count3 + 3];
	}
	for (count6 = 0; count6 < 6; count6++)
	{
		sigma_square[count6] = sigma[count6] * sigma[count6];//calculate sigma square
		yout2[count6] = (exp(-((xin_mean[count6] * xin_mean[count6]) / sigma_square[count6])));

	}

	//===================================layer3 Legendre layer================================================

	xin1_2 = xin1 * xin1;
	xin1_3 = xin1_2 * xin1;
	L1 = xin1;
	L2 = 1.5 * xin1_2 - 0.5;
	L3 = 2.5 * xin1_3 - 1.5;
	L4 = 4.375 * xin1_3 - 1.125 * xin1_2 - 2.25;

	xin2_2 = xin2 * xin2;
	xin2_3 = xin2_2 * xin2;
	L5 = xin2;
	L6 = 1.5 * xin2_2 - 0.5;
	L7 = 2.5 * xin2_3 - 1.5;
	L8 = 4.375 * xin2_3 - 1.125 * xin2_2 - 2.25;

	yout_L[0] = w_MP[0] + L1 * w_MP[1] + L2 * w_MP[2] + L3 * w_MP[3] + L4 * w_MP[4] + L5 * w_MP[5] + L6 * w_MP[6] + L7 * w_MP[7] + L8 * w_MP[8];
	yout_L[1] = w_MP[0] + L1 * w_MP[1] + L2 * w_MP[2] + L3 * w_MP[3] + L4 * w_MP[4] + L5 * w_MP[5] + L6 * w_MP[6] + L7 * w_MP[7] + L8 * w_MP[8];
	yout_L[2] = w_MP[0] + L1 * w_MP[1] + L2 * w_MP[2] + L3 * w_MP[3] + L4 * w_MP[4] + L5 * w_MP[5] + L6 * w_MP[6] + L7 * w_MP[7] + L8 * w_MP[8];
	yout_L[3] = w_MP[0] + L1 * w_MP[1] + L2 * w_MP[2] + L3 * w_MP[3] + L4 * w_MP[4] + L5 * w_MP[5] + L6 * w_MP[6] + L7 * w_MP[7] + L8 * w_MP[8];
	yout_L[4] = w_MP[0] + L1 * w_MP[1] + L2 * w_MP[2] + L3 * w_MP[3] + L4 * w_MP[4] + L5 * w_MP[5] + L6 * w_MP[6] + L7 * w_MP[7] + L8 * w_MP[8];
	yout_L[5] = w_MP[0] + L1 * w_MP[1] + L2 * w_MP[2] + L3 * w_MP[3] + L4 * w_MP[4] + L5 * w_MP[5] + L6 * w_MP[6] + L7 * w_MP[7] + L8 * w_MP[8];
	yout_L[6] = w_MP[0] + L1 * w_MP[1] + L2 * w_MP[2] + L3 * w_MP[3] + L4 * w_MP[4] + L5 * w_MP[5] + L6 * w_MP[6] + L7 * w_MP[7] + L8 * w_MP[8];
	yout_L[7] = w_MP[0] + L1 * w_MP[1] + L2 * w_MP[2] + L3 * w_MP[3] + L4 * w_MP[4] + L5 * w_MP[5] + L6 * w_MP[6] + L7 * w_MP[7] + L8 * w_MP[8];
	//===================================layer4 rule and recurrent layer================================================



	yout3[0] = yout2[0] * yout2[3] * pre_yout3[0] * wl[0];
	yout3[1] = yout2[0] * yout2[4] * pre_yout3[1] * wl[1];
	yout3[2] = yout2[0] * yout2[5] * pre_yout3[2] * wl[2];
	yout3[3] = yout2[1] * yout2[3] * pre_yout3[3] * wl[3];
	yout3[4] = yout2[1] * yout2[4] * pre_yout3[4] * wl[4];
	yout3[5] = yout2[1] * yout2[5] * pre_yout3[5] * wl[5];
	yout3[6] = yout2[2] * yout2[3] * pre_yout3[6] * wl[6];
	yout3[7] = yout2[2] * yout2[4] * pre_yout3[7] * wl[7];
	yout3[8] = yout2[2] * yout2[5] * pre_yout3[8] * wl[8];

	//===================================layer5 consequent layer================================================
	yout_c_delta_l[0] = yout3[0] * yout_L[0];
	yout_c_delta_l[1] = yout3[1] * yout_L[1];
	yout_c_delta_l[2] = yout3[2] * yout_L[2];
	yout_c_delta_l[3] = yout3[3] * yout_L[3];
	yout_c_delta_l[4] = yout3[4] * yout_L[4];
	yout_c_delta_l[5] = yout3[5] * yout_L[5];
	yout_c_delta_l[6] = yout3[6] * yout_L[6];
	yout_c_delta_l[7] = yout3[7] * yout_L[7];
	yout_c_delta_l[8] = yout3[8] * yout_L[8];

	//===================================layer6 output layer================================================
	yout4 = yout_c_delta_l[0] * wk[0] + yout_c_delta_l[1] * wk[1] + yout_c_delta_l[2] * wk[2] + yout_c_delta_l[3] * wk[3]
		+ yout_c_delta_l[4] * wk[4] + yout_c_delta_l[5] * wk[5] + yout_c_delta_l[6] * wk[6] + yout_c_delta_l[7] * wk[7] + yout_c_delta_l[8] * wk[8];
	iq_cmd = yout4 + phi_het;//(43)

//===================================learning algorithms================================================
	dot_phi_het = -gama * error_2;//(55)
	phi_old = phi_het;
	phi_het = phi_old + (dot_phi_het)*0.00025;

	for (count9 = 0; count9 < 9; count9++) pre_yout3[count9] = yout3[count9];

	for (count9 = 0; count9 < 9; count9++) wk[count9] = wk[count9] + (n_w * error_2 * yout_c_delta_l[count9]) * 0.001;//(49)
	for (count6 = 0; count6 < 6; count6++) fresh[count6] = error_2 * wk[count6] * yout_c_delta_l[count6]*0.001;

	for (count6 = 0; count6 < 6; count6++)
	{
		w_MP[count6] = w_MP[count6] - (n_MP * fresh[count6]);//*ts1k*1000;// revise to 1
		wl[count6] = wl[count6] - (n_wl * fresh[count6]);//*ts1k*1000;
		mean[count6] = mean[count6] - (n_mean * fresh[count6]);//*ts1k*1000;//-1 0 1 -1 0 1
		sigma[count6] = sigma[count6] - (n_sigma * fresh[count6]);//*ts1k*1000;
	}


	/*	delta_layer4 = learning_a2 * xin1 + learning_a2 * xin2;
		for (count9 = 0; count9 < 9; count9++) nwk[count9] = wk[count9] + (-eta_1 * delta_layer4 * yout3[count9]);
		for (count9 = 0; count9 < 9; count9++) delta_layer3[count9] = delta_layer4 * wk[count9];

		wl[0] = wl[0] + (-eta_2 * delta_layer3[0] * pre_yout3[0] * yout2[0] * yout2[3]);
		wl[1] = wl[1] + (-eta_2 * delta_layer3[1] * pre_yout3[1] * yout2[0] * yout2[4]);
		wl[2] = wl[2] + (-eta_2 * delta_layer3[2] * pre_yout3[2] * yout2[0] * yout2[5]);
		wl[3] = wl[3] + (-eta_2 * delta_layer3[3] * pre_yout3[3] * yout2[1] * yout2[3]);
		wl[4] = wl[4] + (-eta_2 * delta_layer3[4] * pre_yout3[4] * yout2[1] * yout2[4]);
		wl[5] = wl[5] + (-eta_2 * delta_layer3[5] * pre_yout3[5] * yout2[1] * yout2[5]);
		wl[6] = wl[6] + (-eta_2 * delta_layer3[6] * pre_yout3[6] * yout2[2] * yout2[3]);
		wl[7] = wl[7] + (-eta_2 * delta_layer3[7] * pre_yout3[7] * yout2[2] * yout2[4]);
		wl[8] = wl[8] + (-eta_2 * delta_layer3[8] * pre_yout3[8] * yout2[2] * yout2[5]);

		yout_c_delta_l[0] = delta_layer4 * yout3[0];
		yout_c_delta_l[1] = delta_layer4 * yout3[1];
		yout_c_delta_l[2] = delta_layer4 * yout3[2];
		yout_c_delta_l[3] = delta_layer4 * yout3[3];
		yout_c_delta_l[4] = delta_layer4 * yout3[4];
		yout_c_delta_l[5] = delta_layer4 * yout3[5];
		yout_c_delta_l[6] = delta_layer4 * yout3[6];
		yout_c_delta_l[7] = delta_layer4 * yout3[7];
		yout_c_delta_l[8] = delta_layer4 * yout3[8];

		for (count9 = 0; count9 < 9; count9++)	w_L[count9] = w_L[count9] + (-eta_L * yout_c_delta_l[count9] * yout_L[count9]);

		delta_layer2[0] = delta_layer3[0] * yout3[0] + delta_layer3[1] * yout3[1] + delta_layer3[2] * yout3[2];
		delta_layer2[1] = delta_layer3[3] * yout3[3] + delta_layer3[4] * yout3[4] + delta_layer3[5] * yout3[5];
		delta_layer2[2] = delta_layer3[6] * yout3[6] + delta_layer3[7] * yout3[7] + delta_layer3[8] * yout3[8];
		delta_layer2[3] = delta_layer3[0] * yout3[0] + delta_layer3[3] * yout3[3] + delta_layer3[6] * yout3[6];
		delta_layer2[4] = delta_layer3[1] * yout3[1] + delta_layer3[4] * yout3[4] + delta_layer3[7] * yout3[7];
		delta_layer2[5] = delta_layer3[2] * yout3[2] + delta_layer3[5] * yout3[5] + delta_layer3[8] * yout3[8];
		for (count9 = 0; count9 < 9; count9++) wk[count9] = nwk[count9];
		for (count9 = 0; count9 < 9; count9++)
		{
			if (wk[count9] >= 0.999) wk[count9] = 0.999;
			if (wl[count9] >= 0.999) wl[count9] = 0.999;
			if (wk[count9] <= -0.999) wk[count9] = -0.999;
			if (wl[count9] <= -0.999) wl[count9] = -0.999;
			if (w_L[count9] <= -0.999) wk[count9] = -0.999;
			if (wl[count9] <= -0.999) wl[count9] = -0.999;
		}
		*/
	for (count9 = 0; count9 < 9; count9++)
	{
		if (pre_yout3[count9] < 0.05) pre_yout3[count9] = 0.05;
		else pre_yout3[count9] = pre_yout3[count9];
	}
	return iq_cmd;
	//===================================================================================


}

double CureFitting(float iq_cmd)
{
    id_cmd = 11.7436 - 10.373 * iq_cmd + 3.4223 * iq_cmd * iq_cmd -0.3381 * iq_cmd * iq_cmd * iq_cmd;
    return -id_cmd;
}