#include "includes.h"

#include <avr/io.h>
#include <util/delay.h>

#define  TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE
#define  N_TASKS        6

OS_STK       TaskStk[N_TASKS][TASK_STK_SIZE];
OS_EVENT	  *Mbox;
OS_EVENT* mutex;
OS_FLAG_GRP* t_grp;		//task의 실행 순서를 정하는 EventFlag
// 0: LED, 1: 버저, 2: 광, 3: 온도

volatile INT8U	FndNum;
volatile INT8U LedOper;
volatile INT8U TaskNum;
INT8U const myMapTbl[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
void FndTask(void *data);
void FndDisplayTask(void *data);
void LedTask (void *data);
void BuzzerTask(void* data);
void TemperatureTask(void *data);
void LightTask(void* data);
void InitI2C();
int ReadTemperature(void);
ISR(INT4_vect);
ISR(INT5_vect);

int main (void)
{
	INT8U err;
	OSInit();
	OS_ENTER_CRITICAL();
	TCCR0 = 0x07;
	TIMSK = _BV(TOIE0);
	TCNT0 = 256 - (CPU_CLOCK_HZ / OS_TICKS_PER_SEC / 1024);
	DDRE=0xCF;	//SW 입력 모드
	EICRB=0x0A;
	EIMSK=0x30;
	sei();
	//interrupt설정
	OS_EXIT_CRITICAL();

	mutex=OSMutexCreate(1,&err);
	t_grp=OSFlagCreate(myMapTbl[TaskNum],&err);
	OSTaskCreate(FndTask, (void *)0, (void *)&TaskStk[0][TASK_STK_SIZE - 1], 1);
	OSTaskCreate(FndDisplayTask, (void *)0, (void *)&TaskStk[1][TASK_STK_SIZE - 1], 2);
	OSTaskCreate(LedTask, (void *)0, (void *)&TaskStk[2][TASK_STK_SIZE - 1], 3);
	OSTaskCreate(BuzzerTask, (void *)0, (void *)&TaskStk[3][TASK_STK_SIZE - 1], 4);
	OSTaskCreate(TemperatureTask, (void *)0, (void *)&TaskStk[4][TASK_STK_SIZE - 1], 5);
	OSTaskCreate(LightTask, (void *)0, (void *)&TaskStk[5][TASK_STK_SIZE - 1], 6);

	OSStart();

	return 0;
}

/*
SW1: Task Switching용, 한번 누를때마다 LED->버저->광->온도 순으로 Cycle
FND에는 LED, BUZZ, 빛 정도, 온도 표시
*/
ISR(INT4_vect) {
	INT8U err;
	OSMutexPend(mutex,0,&err);
	OSFlagPend(t_grp,myMapTbl[TaskNum],OS_FLAG_WAIT_SET_ALL+OS_FLAG_CONSUME,0,&err);
	TaskNum=(TaskNum+1)&0x03;
	OSFlagPost(t_grp,myMapTbl[TaskNum],OS_FLAG_SET,&err);
	OSMutexPost(mutex);
}

/*
SW2: operation 전환용, 한번 누를때마다 operation바뀜
LED: Left Circular Shift, Right Circular Shift, 깜빡깜빡, 왼쪽오른쪽왼쪽오른쪽,계속켜짐
Buzzer: 도레미파솔라시도~, 도시라솔파미레도~ , 음악 하나선정해서 연주
광: 생각 해봐야함...
온도: 정확도 조절?, 이것도 생각좀...., 일단 FND 온도 표시
*/
ISR(INT5_vect) {
	INT8U err;
	OSMutexPend(mutex,0,&err);
	switch(TaskNum) {
		case 0:		//LED
		LedOper=(LedOper+1)&0x03;
		break;
		case 1:		//BUZZER
		break;
		case 2:		//Light
		break;
		case 3:		//Temp
		break;
	}
	OSMutexPost(mutex);
}


void InitI2C()
{
    PORTD = 3; 						// For Pull-up override value
    SFIOR &= ~(1 << PUD); 			// PUD
    TWSR = 0; 						// TWPS0 = 0, TWPS1 = 0
    TWBR = 32;						// for 100  K Hz bus clock
	TWCR = _BV(TWEA) | _BV(TWEN);	// TWEA = Ack pulse is generated
									// TWEN = TWI 동작을 가능하게 한다
}

int ReadTemperature(void)
{
	int value;

	TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);	// START 전송
	while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

	TWDR = 0x98 + 1; 							//TEMP_I2C_ADDR + 1 > SLA+R 준비, R=1
	TWCR = _BV(TWINT) | _BV(TWEN);				// SLA+R 전송
	while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);	// 1st DATA 준비
	while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

	//온도센서는 16bit 기준으로 값을 가져오므로
	//8비트씩 2번을 받아야 한다.
	value = TWDR << 8;							// 1 byte DATA 수신
	TWCR = _BV(TWINT) | _BV(TWEN); 				// SLA+R 전송
	while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

	value |= TWDR;								// 1 byte DATA 수신
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);	// STOP 전송

	value >>= 8;

	TIMSK = (value >= 33) ? TIMSK | _BV(TOIE2): TIMSK & ~_BV(TOIE2);

	return value;
}

void FndTask (void *data)
{
	INT8U	err;

	data = data;

	while (1) {
		// FndNum pend

		OSTimeDlyHMSM(0, 0, 0, 100);
	}
}

void FndDisplayTask (void *data)
{
	unsigned char FND_DATA[ ]= {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x27, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x80, 0x40, 0x08, 0x00};
	unsigned int num[4];

	data = data;

	DDRC = 0xff;
	DDRG = 0x0f;

	while(1)  {
		OSTimeDlyHMSM(0,0,0,20);
	}
}


void LedOperation(INT8U oper, INT8U* led, INT8U* ledstatus) {
	switch(oper) {
		case 0:		//left circular shift
		PORTA=(*led);
		if((*led)==0x80) {
			(*led)=0x01;
		} else {
			(*led)<<=1;
		}
		break;
		case 1:		//right circular shift
		PORTA=(*led);
		if((*led)==0x01) {
			(*led)=0x80;
		} else {
			(*led)>>=1;
		}
		break;
		case 2:		//깜빡깜빡
		if(*ledstatus) {
			PORTA=0xFF;
			*ledstatus=0;
		} else {
			PORTA=0x00;
			*ledstatus=1;
		}
		PORTA=(*led);
		break;
		case 3:		//왼쪽오른쪽반복
		PORTA=(*led);
		if(*ledstatus) {
			if((*led)==0x80) {
				*ledstatus=0;
			} else {
				(*led)<<=1;
			}
		} else {
			if((*led)==0x01) {
				*ledstatus=1;
			} else {
				(*led)>>=1;
			}
		}
		break;
	}
}

void LedTask (void *data)
{
	INT8U err;
	INT8U led=0x01;
	INT8U status=1;
	DDRA=0xFF;
	data=data;
	while(1) {
		OSFlagPend(t_grp,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);
		LedOperation(LedOper,&led,&status);
		OSTimeDlyHMSM(0,0,0,200);
	}
}

void BuzzerTask(void* data) {
	INT8U err;
	//INT8U delay=17;
	INT8U on=1;
	data=data;

	while(1) {
		OSFlagPend(t_grp,0x02,OS_FLAG_WAIT_SET_ALL,0,&err);
		if(on) {		//일단 타이머 없이 아무소리만 나게 해봄 과연....
			PORTB=0x00;
			on=0;
		} else {
			PORTB=0x10;
			on=1;
		}
		OSTimeDlyHMSM(0,0,0,1);
	}
}

void LightTask(void* data) {
	INT8U err;
	data=data;
	while(1) {
		OSFlagPend(t_grp,0x04,OS_FLAG_WAIT_SET_ALL,0,&err);

		OSTimeDlyHMSM(0,0,0,200);
	}
}

void TemperatureTask (void *data)
{
	int	value;
	INT8U err;
	data = data;
	InitI2C();

	while (1)  {
		OSFlagPend(t_grp,0x08,OS_FLAG_WAIT_SET_ALL,0,&err);
		OS_ENTER_CRITICAL();
		value = ReadTemperature();
		OS_EXIT_CRITICAL();

	// value post

		OSTimeDlyHMSM(0, 0, 0, 100);
	}
}
