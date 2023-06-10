
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode

#include <stdint.h>
#include "PWM.h"
#include "tm4c123gh6pm.h"
#include "PLL.h"
#define B  8.7
#define A  61839
#define NUM_SAMPLES 5
#define PERIOD 10000
#define STEP 1000
#define MAX_DUTY 9000

unsigned int x,y,COLOR;
unsigned long H,L,Z;
unsigned char tabledist, eqdist, eqdist1, tabledist1;
const unsigned int adctable[] ={3690, 2602, 2065, 1731, 1580, 1424, 1370, 940, 861, 782, 757, 685, 650};
const unsigned int disttab[] = {10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70};

unsigned char sample=0;
volatile unsigned long ADCvalue, ADCvalue1;

unsigned int too_closeL, too_closeR;

void ADC0_InitSWTriggerSeq3_Ch1(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000010;   // 1) activate clock for Port E
  delay = SYSCTL_RCGC2_R;         //    allow time for clock to stabilize
  GPIO_PORTE_DIR_R &= ~0x04;      // 2) make PE2 input
  GPIO_PORTE_AFSEL_R |= 0x04;     // 3) enable alternate function on PE2
  GPIO_PORTE_DEN_R &= ~0x04;      // 4) disable digital I/O on PE2
  GPIO_PORTE_AMSEL_R |= 0x04;     // 5) enable analog function on PE2
  SYSCTL_RCGC0_R |= 0x00010000;   // 6) activate ADC0 
  delay = SYSCTL_RCGC2_R;         
  SYSCTL_RCGC0_R &= ~0x00000300;  // 7) configure for 125K 
  ADC0_SSPRI_R = 0x0123;          // 8) Sequencer 3 is highest priority
  ADC0_ACTSS_R &= ~0x0008;        // 9) disable sample sequencer 3
  ADC0_EMUX_R &= ~0xF000;         // 10) seq3 is software trigger
  ADC0_SSMUX3_R = (ADC0_SSMUX3_R&0xFFFFFFF0)+1; // 11) channel Ain1 (PE2)
  ADC0_SSCTL3_R = 0x0006;         // 12) no TS0 D0, yes IE0 END0
  ADC0_ACTSS_R |= 0x0008;         // 13) enable sample sequencer 3
}

unsigned long ADC0_InSeq3(void){  
	unsigned long result;
  ADC0_PSSI_R = 0x0008;            // 1) initiate SS3
  while((ADC0_RIS_R&0x08)==0){};   // 2) wait for conversion done
  result = ADC0_SSFIFO3_R&0xFFF;   // 3) read result
  ADC0_ISC_R = 0x0008;             // 4) acknowledge completion
  return result;
}

void ADC1_InitSWTriggerSeq3_Ch9(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000010;   // 1) activate clock for Port E
  delay = SYSCTL_RCGC2_R;         //    allow time for clock to stabilize
  GPIO_PORTE_DIR_R &= ~0x10;      // 2) make PE2 input
  GPIO_PORTE_AFSEL_R |= 0x10;     // 3) enable alternate function on PE2
  GPIO_PORTE_DEN_R &= ~0x10;      // 4) disable digital I/O on PE2
  GPIO_PORTE_AMSEL_R |= 0x10;     // 5) enable analog function on PE2
  SYSCTL_RCGC0_R |= 0x00020000;   // 6) activate ADC0 
  delay = SYSCTL_RCGC2_R;         
  SYSCTL_RCGC0_R &= ~0x00000300;  // 7) configure for 125K 
  ADC1_SSPRI_R = 0x0123;          // 8) Sequencer 3 is highest priority
  ADC1_ACTSS_R &= ~0x0008;        // 9) disable sample sequencer 3
  ADC1_EMUX_R &= ~0x0F00;         // 10) seq3 is software trigger
  ADC1_SSMUX3_R = (ADC0_SSMUX3_R&0xFFFFFFF0)+9; // 11) channel Ain1 (PE2)
  ADC1_SSCTL3_R = 0x0006;         // 12) no TS0 D0, yes IE0 END0
  ADC1_ACTSS_R |= 0x0008;         // 13) enable sample sequencer 3
}

unsigned long ADC1_InSeq9(void){  
	unsigned long result;
  ADC1_PSSI_R = 0x0008;            // 1) initiate SS3
  while((ADC1_RIS_R&0x08)==0){};   // 2) wait for conversion done
  result = ADC1_SSFIFO3_R&0xFFF;   // 3) read result
  ADC1_ISC_R = 0x0008;             // 4) acknowledge completion
  return result;
}





unsigned char eq_calcution(unsigned int ADC_Value){
	unsigned char dist=0;
	dist = (A/ADC_Value) - B;
	return dist;
}

unsigned char tb_estimation(unsigned int ADC_Value){
	unsigned char dist=0; 
	int x1=0, x2=0, y1=0, y2=0, d=0;
	for (int j = 0; j < sizeof(adctable); j++){
		if(ADC_Value >= adctable[j]){
			x1 = adctable[j-1];
			y1 = (j-1) * 5 + 10;
			x2 = adctable[j];
			y2 = j * 5 + 10;
			break;
		}
	}
	dist = y1 + (ADC_Value - x1) * ((y2 - y1) / (x2 - x1)); // derived from distance forumla
	return dist;
}

void Delay(void){unsigned long volatile time;
  time = 727240*100/91;  // 1 sec
  while(time){
		time--;
  }
	for (time=0;time<1000;time=time+10) {
	}
}

void Delayer(void){unsigned long volatile time;
  time = 727240*25/91;  // 1 sec
  while(time){
		time--;
  }
	for (time=0;time<1000;time=time+10) {
	}
}

void SysTick_Init(void){ 
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = 800000 - 1;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
  NVIC_SYS_PRI3_R=(NVIC_SYS_PRI3_R & 0x1FFFFFFF)| 0x40000000;
  NVIC_ST_CTRL_R = 0x07;
	EnableInterrupts();
}

void SysTick_Handler(void){
	sample = 1;
	if(x<(y-3) && COLOR==1){
		GPIO_PORTF_DATA_R = 0x08;
	}
	if(y<(x-3) && COLOR==1){
		GPIO_PORTF_DATA_R = 0x04;
	}
}

void Switch_Init(void){  unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; //  unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x11;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x11;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000F000F; //  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void Motors_Init(void){
  SYSCTL_RCGC2_R |= 0x00000001; // activate clock for port A
  GPIO_PORTA_AMSEL_R &= ~0xF0;      // disable analog functionality on PA5,6
  GPIO_PORTA_PCTL_R &= ~0xFFFF0000; // configure PA5,6 as GPIO
  GPIO_PORTA_DIR_R |= 0xF0;     // make PA5,6 out
  GPIO_PORTA_DR8R_R |= 0xF0;    // enable 8 mA drive on PA5,6
  GPIO_PORTA_AFSEL_R &= ~0xF0;  // disable alt funct on PA5,6
  GPIO_PORTA_DEN_R |= 0xF0;     // enable digital I/O on PA5,6
}

void LED_init(void){
	SYSCTL_RCGC2_R |= 0x00000020;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;
	GPIO_PORTF_AMSEL_R &= ~0x0E;
	GPIO_PORTF_PCTL_R &= ~0x0000FFF0;
	GPIO_PORTF_DIR_R |= 0x0E;
	GPIO_PORTF_AFSEL_R &= ~0x0E;
	GPIO_PORTF_DEN_R |= 0x0E;
}

void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
		if(Z == 0){
			Z = 1;
		}
		else if(Z == 1) {
			Z = 0;
  }
}
	if(GPIO_PORTF_RIS_R&0x02){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x02;  // acknowledge flag4
		if(H == 0){
			H = 1;
		}
		else if(H == 1) {
			H = 0;
  }
}
	Delay(); //delay for debouncing 
}

int main(void){
	DisableInterrupts();
	COLOR = 0;
	H = L = Z = 0;
	ADC0_InitSWTriggerSeq3_Ch1();
	ADC1_InitSWTriggerSeq3_Ch9();
  PWM0A_Init(PERIOD);         // initialize PWM0, PB6
  PWM0B_Init(PERIOD);         // initialize PWM0, PB7
	SysTick_Init();
	PWM0A_Duty(H); //update duty cycle
	PWM0B_Duty(H);
	Motors_Init(); //PA4-PA7
	LED_init(); //PF1-PF3
	Switch_Init(); //on-board buttons
	GPIO_PORTA_DATA_R = 0x30;
	GPIO_PORTF_DATA_R = 0x0A;
	Delay();
	Delay();
	GPIO_PORTF_DATA_R = 0x00;
	PWM0A_Duty(3100); //update duty cycle
	PWM0B_Duty(3000);
	EnableInterrupts();
	while(1){
	COLOR = 1;
	unsigned char count=0;
	while (count<NUM_SAMPLES) {
			while (!sample){} // sample one value every 1/20=0.05 second 
			sample = 0;
			ADCvalue += ADC0_InSeq3();
			ADCvalue1 += ADC1_InSeq9();
			count++;
	  }
		ADCvalue/=NUM_SAMPLES;
		ADCvalue1/=NUM_SAMPLES;
		eqdist = eq_calcution(ADCvalue);
		tabledist = tb_estimation(ADCvalue);
		eqdist1 = eq_calcution(ADCvalue1);
		tabledist1 = tb_estimation(ADCvalue1);
		count = 0;
		x = (eqdist + tabledist)/2;
		y = (eqdist1 + tabledist1)/2;
		if(x <= 20 && x<y){
			COLOR = 0;
			PWM0A_Duty(0);
			PWM0B_Duty(0);
			GPIO_PORTF_DATA_R = 0x02;
			Delay();
			GPIO_PORTF_DATA_R = 0x00;
			GPIO_PORTA_DATA_R = 0xC0;
			PWM0A_Duty(3100); //update duty cycle
			PWM0B_Duty(3000);
			Delay();
			Delay();
			GPIO_PORTA_DATA_R = 0x10;
			Delay();
			PWM0A_Duty(0);
			PWM0B_Duty(0);
			Delay();
			PWM0A_Duty(3100); //update duty cycle
			PWM0B_Duty(3000);
			GPIO_PORTA_DATA_R = 0x30;
	}
		if(y <= 20 && y<x){
			COLOR = 0;
			PWM0A_Duty(0);
			PWM0B_Duty(0);
			GPIO_PORTF_DATA_R = 0x02;
			Delay();
			GPIO_PORTF_DATA_R = 0x00;
			GPIO_PORTA_DATA_R = 0xC0;
			PWM0A_Duty(3100); //update duty cycle
			PWM0B_Duty(3000);
			Delay();
			Delay();
			GPIO_PORTA_DATA_R = 0x20;
			Delay();
			PWM0A_Duty(0);
			PWM0B_Duty(0);
			Delay();
			PWM0A_Duty(3100); //update duty cycle
			PWM0B_Duty(3000);
			GPIO_PORTA_DATA_R = 0x30;
			}
		if(x <= 50 && x > 20 && x<y)
		{
			if(x>29){
			PWM0A_Duty(3300);
			PWM0B_Duty(3000);
			}
			else
			{
//			PWM0A_Duty(3100);
//			PWM0B_Duty(0);
//			Delayer();
			PWM0A_Duty(3100);
			PWM0B_Duty(3000);
			}
		}
		else if(y <= 50 && y > 20 && y<x)
		{
			if(y>30){
			PWM0A_Duty(3000);
			PWM0B_Duty(3200);
			}
			else{
//			PWM0A_Duty(0);
//			PWM0B_Duty(3000);
//			Delayer();
			PWM0A_Duty(3100);
			PWM0B_Duty(3000);
			}
		}
		else
		{
			GPIO_PORTA_DATA_R = 0x30;
			PWM0A_Duty(3100);
			PWM0B_Duty(3000);
		}
		if (x >=70 && y < 31)
		{
			PWM0A_Duty(0);
			PWM0B_Duty(3000);
			Delayer();
			Delayer();
			PWM0A_Duty(3100);
			PWM0B_Duty(3000);
		}
		if (y >=70 && x < 31)
		{
			PWM0A_Duty(3100);
			PWM0B_Duty(0);
			Delayer();
			Delayer();
			PWM0A_Duty(3100);
			PWM0B_Duty(3000);
		}
		if(x >=70 && y >=70 && Z == 0)
		{
			Delay();
			while(Z == 0)
			{
				GPIO_PORTF_DATA_R = 0x06;
				PWM0A_Duty(0);
				PWM0B_Duty(0);
			}
		}
		if(H == 1 || Z == 1)
		{
			while(H == 1)
			{
				PWM0A_Duty(0);
				PWM0B_Duty(0);
				GPIO_PORTF_DATA_R = 0x0E;
			}
			while(Z == 1)
			{
				PWM0A_Duty(0);
				PWM0B_Duty(0);
				GPIO_PORTF_DATA_R = 0x00;
			}
		ADCvalue=0;
		ADCvalue1=0;
	}
}
}

