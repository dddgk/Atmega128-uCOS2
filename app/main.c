#include "includes.h"

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define  TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE
#define  N_TASKS        9

OS_STK TaskStk[N_TASKS][TASK_STK_SIZE];
OS_EVENT* Mbox;
//FND 메일박스
OS_EVENT* LightMbox;
//광센서 메일박스
OS_EVENT* mutex;
OS_EVENT* MsgQ;
OS_FLAG_GRP* t_grp;		//task의 실행 순서를 정하는 EventFlag
void* MsgQTbl[2];  //Message Queue Table

// 0: LED, 1: 버저, 2: 광, 3: 온도

volatile INT8U LedOper;
volatile INT8U BuzzOper;
volatile INT8U TaskNum;
volatile INT8U on=1;
volatile INT8U IND=0;
volatile INT8U fre=0;
volatile INT8U status=0;
const INT8U myMapTbl[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
const INT8U LED[4]={0,0x38,0x79,0x3F};	//LED 출력위함
const INT8U BUZZ[4]={0x7F,0x3E,0x5B,0x5B};	//BUZZ 출력위함
const INT8U DIGIT[12]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7c,0x07,0x7f,0x67,0x40,0x00};
//0,1,2,3,4,5,6,7,8,9,-,공백
const INT8U rabbit[25]={7,4,4,7,4,0,2,4,2,0,4,7,12,7,12,7,12,7,4,7,2,5,4,2,0};
const INT8U rabbit_dly[25]={4,2,2,2,2,4,4,2,2,2,2,4,3,1,2,2,2,2,4,4,2,2,2,2,4};
const INT8U canon[]={
	19,16,17,19,16,17,19,11,9,11,12,14,16,17,
	16,12,14,16,4,5,7,9,7,5,7,12,11,12,
	9,12,11,9,7,5,7,5,4,5,7,9,11,12,
	9,12,11,12,11,12,11,9,11,12,14,16,17,19,
	19,16,17,19,16,17,19,11,9,11,12,14,16,17,
	16,12,14,16,4,5,7,9,7,5,7,12,11,12,
	9,12,11,9,7,5,7,5,4,5,7,9,11,12,
	9,12,11,12,11,12,11,12,14,12,11,12,9,11,12
};
const INT8U canon_dly[]={
	4,2,2,4,2,2,2,2,2,2,2,2,2,2,4,2,2,4,2,2,2,2,2,2,2,2,2,2,
	4,2,2,4,2,2,2,2,2,2,2,2,2,2,4,2,2,4,2,2,2,2,2,2,2,2,2,2,
	4,2,2,4,2,2,2,2,2,2,2,2,2,2,4,2,2,4,2,2,2,2,2,2,2,2,2,2,
	4,2,2,4,2,2,2,2,2,2,2,2,2,2,4,2,2,4,2,2,2,2,2,2,2,2,2,2,8
};
const INT8U Frequency[25]={17,31,43,55,66,77,87,97,106,114,122,129,137,143,150,156,161,167,172,176,181,185,189,193,196};
//도(0),도#(1),레(2),레#(3),미(4),파(5),파#(6),솔(7),솔#(8),라(9),라#(10),시(11)
//도(12),도#(13),레(14),레#(15),미(16),파(17),파#(18),솔(19),솔#(20),라(21),라#(22),시(23),도(24)
INT8U FNDData[4]={0x3F,0x79,0x38,0};
void FndTask(void *data);
void FndDisplayTask(void *data);
void LedTask (void *data);
void BuzzerTask(void* data);
void TemperatureTask(void *data);
void ReadTemperatureTask(void *data);
void ReadLightTask(void* data);
void LightTask(void* data);
void InitI2C();
ISR(INT4_vect);
ISR(INT5_vect);
ISR(TIMER2_OVF_vect);

struct Mail {
	INT8U data;
	INT8U sel;
};//FND 메일박스위한 구조체

int main (void)
{
	INT8U err;
	OSInit();
	OS_ENTER_CRITICAL();
	TCCR0 = 0x07;
	TIMSK = 0x01;
	TCNT0 = 256 - (CPU_CLOCK_HZ / OS_TICKS_PER_SEC / 1024);
	DDRE=0xCF;	//SW 입력 모드
	EICRB=0x0A;	//SW Falling Edge
	EIMSK=0x30;	//INT4,5 활성화
	sei();
	//interrupt설정
	OS_EXIT_CRITICAL();

	Mbox=OSMboxCreate((void*)0);
	LightMbox=OSMboxCreate((void*)0);
	mutex=OSMutexCreate(1,&err);
	t_grp=OSFlagCreate(myMapTbl[TaskNum],&err);
	MsgQ=OSQCreate(MsgQTbl,2);
	//메일박스, 뮤텍스, 이벤트플래그, 메시지큐 생성
	
	OSTaskCreate(TemperatureTask, (void *)0, (void *)&TaskStk[0][TASK_STK_SIZE - 1], 2);
	OSTaskCreate(ReadTemperatureTask, (void *)0, (void *)&TaskStk[1][TASK_STK_SIZE - 1], 3);
	OSTaskCreate(LightTask, (void *)0, (void *)&TaskStk[2][TASK_STK_SIZE - 1], 4);
	OSTaskCreate(ReadLightTask, (void *)0, (void *)&TaskStk[3][TASK_STK_SIZE - 1], 5);
	OSTaskCreate(LedTask, (void *)0, (void *)&TaskStk[4][TASK_STK_SIZE - 1], 6);
	OSTaskCreate(BuzzerTask, (void *)0, (void *)&TaskStk[5][TASK_STK_SIZE - 1], 7);
	OSTaskCreate(FndDisplayTask, (void *)0, (void *)&TaskStk[6][TASK_STK_SIZE - 1], 8);
	OSTaskCreate(FndTask, (void *)0, (void *)&TaskStk[7][TASK_STK_SIZE - 1], 9);
	//Task 생성

	OSStart();

	return 0;
}
//온도 측정한거 받아서 FNDData 수정하는 Task 우선순위 높음
//온도 측정 Task 우선순위 낮음
//메시지 큐로 2개 데이터 (low, high)보내줌
//받는 Task가 2개 data pend해서 합침
/*
SW1: Task Switching용, 한번 누를때마다 LED->버저->광->온도 순으로 Cycle
FND에는 LED, BUZZ, 빛 충전정도, 온도 표시
*/
ISR(INT4_vect) {
	INT8U err,i;
	PORTA=0x00;	//LED 초기화
	OSMutexPend(mutex,1,&err);
	if(err) {	//mutex 사용중일경우 interrupt 무시
		return;
	}
	OSFlagPend(t_grp,myMapTbl[TaskNum],OS_FLAG_WAIT_SET_ALL+OS_FLAG_CONSUME,0,&err);
	//현재 Task Flag 없애기
	TaskNum=(TaskNum+1)&0x03;	//다른 Task로 전환
	switch(TaskNum) {
		case 0:
		for(i=0;i<4;i++) {		//FND에 LED 표시
			FNDData[i]=LED[3-i];
		}
		break;
		case 1:
		IND=0;
		for(i=0;i<4;i++) {		//FND에 BUZZ 표시
			FNDData[i]=BUZZ[3-i];
		}
		break;
	}
	OSMutexPost(mutex);
	OSFlagPost(t_grp,myMapTbl[TaskNum],OS_FLAG_SET,&err);	//Task Flag SET
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
	OSMutexPend(mutex,1,&err);
	if(err) {	//뮤텍스 사용중일 시 인터럽트 무시
		return;
	}
	switch(TaskNum) {	//Task 전환이 필요한 Task만 수행(LED, BUZZER)
		case 0:		//LED
		LedOper=(LedOper+1)&0x03;
		break;
		case 1:		//BUZZER
		BuzzOper=(BuzzOper+1)&0x01;
		IND=0;
		break;
	}
	OSMutexPost(mutex);
}

ISR(TIMER2_OVF_vect) {
	if(status) {	//음계간의 딜레이 주기위함
		if(on) {
			PORTB=0x00;
			on=0;
		} else {
			PORTB=0x10;
			on=1;
		}
		TCNT2=Frequency[fre];	//음 결정
	}
}

void FndTask (void *data)
{
	data = data;
	INT8U sel=0;
	struct Mail fnddata;
	while (1) {
		fnddata.data=FNDData[sel];	//표시할 수 결정
		fnddata.sel=sel;			//표시할 FND 위치결정
		sel=(sel+1)&0x03;
		OSMboxPost(Mbox,&fnddata);	//메일박스로 전송(FndDisplayTask로)
		OSTimeDlyHMSM(0,0,0,2);	//2ms delay
	}
}

void FndDisplayTask (void *data)
{
	INT8U err;
    data = data;
    DDRC = 0xff;	//FND 출력포트 출력모드
    DDRG = 0x0f;	//FND 선택포트 출력모드
	struct Mail fnddata;
    while(1)  {
		fnddata=*(struct Mail*)OSMboxPend(Mbox,0,&err);	//FndTask로 값받음
		PORTG=myMapTbl[fnddata.sel];	//출력할 위치결정
		PORTC=fnddata.data;				//FND 출력
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
		break;
		case 3:		//왼쪽오른쪽반복
		PORTA=(*led);
		if(*ledstatus) {
			if((*led)==0x80) {
				*ledstatus=0;
				(*led)>>=1;
			} else {
				(*led)<<=1;
			}
		} else {
			if((*led)==0x01) {
				*ledstatus=1;
				(*led)<<=1;
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
	INT8U led=0x01;	//LED 위치 -> oper0,1,3에 사용
	INT8U status=1;	//LED 상태 -> oper2,3에 사용
	DDRA=0xFF;		//LED 출력모드
	data=data;
	while(1) {
		PORTA=0x00;	//LED초기화
		OSFlagPend(t_grp,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);
		LedOperation(LedOper,&led,&status);	//LED 동작모드 결정
		OSTimeDlyHMSM(0,0,0,200);
	}
}

void BuzzerTask(void* data) {	
	INT8U err;
	data=data;
	INT16U dly=0;		//음계 길이 결정
	fre=0;				//음계 결정 변수 초기화
	DDRB=0x10;			//버저 포트 출력모드
	TCCR2 = 0x03;		//timer2 32분주 -> Buzzer 위함
	TIMSK |= 0x40;	 //Timer2 Overflow Interrupt 활성화
	while(1) {
		status=0;
		OSTimeDlyHMSM(0,0,0,50);	//음계간 딜레이
		OSFlagPend(t_grp,0x02,OS_FLAG_WAIT_SET_ALL,0,&err);
		fre=((BuzzOper==0) ? rabbit[IND]:canon[IND]);	//operation에 따라 음계결정
		dly=((BuzzOper==0) ? rabbit_dly[IND]:canon_dly[IND]/2);	//음계 길이 결정
		IND=(IND+1)%((BuzzOper==0) ? 26:113);	//다음음계로 변경
		status=1;		//버저 출력
		OSTimeDlyHMSM(0,0,0,dly*125);	//음계길이만큼 딜레이
	}
}

//빛양을 읽어서 메일박스로 전송하는 TASK
void ReadLightTask(void* data) {	
	INT8U err;
	data=data;
	INT8U low,high;
	INT16U value;
	ADMUX=0x00;	//ADMUX ADC0 사용
	ADCSRA=0x87;	//ADC사용, 프리스케일러 128분주
	while(1) {
		OSFlagPend(t_grp,0x04,OS_FLAG_WAIT_SET_ALL,0,&err);
		ADCSRA|=0x40;	//ADC 변환 시작
		while((ADCSRA & 0x10) != 0x10);	//변환 완료까지 대기
		low=ADCL;
		high=ADCH;
		value=(high<<8) | low;
		//값 받아와서 변환
		OSMboxPost(LightMbox,&value);	//Mailbox로 전송
		OSTimeDlyHMSM(0,0,0,50);
	}
}

//readLightTask로부터 빛의 양을 받아서 배터리 충전하는 Task
void LightTask(void* data) {
	INT8U err,i;
	data=data;
	const INT16U LIGHT=871;	//빛의 세기 기준 결정위함
	INT16U val;
	INT16U Elec=0;	//충전정도 결정 변수
	INT16U tmp;
	INT8U led=0x00;	//LED 관련 변수
	while(1) {
		val=*(INT16U*)OSMboxPend(LightMbox,0,&err);
		if(val>=LIGHT) {
			if(Elec!=1000) {
				Elec+=4;
			}
		} else {
			if(Elec!=0) {
				Elec-=4;
			}
		}
		led=0x00;
		tmp=Elec/125;	//LED 8개이므로 8로 나눠 켜짐정도 결정
		if(Elec) {	//0이면 아예 안켜지고 나머지는 켜짐
			for(i=0;i<=tmp;i++) {
				led|=myMapTbl[i];
			}
		}
		PORTA=led;	//LED on
		i=0;
		tmp=Elec;
		OSMutexPend(mutex,1,&err);
		if(err) {
			continue;
		}
		while(i<4) {	//FND에 값 100.0 단위로 표시
			FNDData[i]=DIGIT[tmp%10];
			if(i==1) {
				FNDData[i]|=0x80;
			}
			i++;
			tmp/=10;
		}
		OSMutexPost(mutex);
	}
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

void TemperatureTask (void *data)
{	
	INT8U err;
	data = data;
	INT8U low,high;
	int value;
	INT8U value_int,value_deci;		//정수부분, 실수부분
	while (1)  {
		low=0;
		high=0;
		low=*(INT8U*)OSQPend(MsgQ,0,&err);
		high=*(INT8U*)OSQPend(MsgQ,0,&err);
		//메시지큐에서 값 받음
		value=high;
		value<<=8;
		value|=low;
		OSMutexPend(mutex,1,&err);
		if(err) {		//뮤텍스 사용중일경우 넘어감
			continue;
		}
		if((value&0x8000) != 0x8000) {	
			FNDData[3]=DIGIT[11];
		} else {
			FNDData[3]=DIGIT[10];	//음수이면 - 표시
			value=(~value)-1;
		}
		value_int=(INT8U)((value&0x7f00)>>8);
		value_deci=(INT8U)(value&0x00ff);
		FNDData[2]=DIGIT[(value_int/10)%10];	
		FNDData[1]=(DIGIT[value_int%10]|0x80);
		//FND에 정수부분표시
		FNDData[0]=DIGIT[((value_deci&0x80)==0x80)*5];
		//FND에 실수부분표시
		OSMutexPost(mutex);
	}
}

void ReadTemperatureTask (void *data)
{
	INT8U err;
	INT8U low,high;
	data = data;
	InitI2C();

	while (1)  {
		OSFlagPend(t_grp,0x08,OS_FLAG_WAIT_SET_ALL,0,&err);
		OS_ENTER_CRITICAL();

		TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);	// START 전송
		while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

		TWDR = 0x98 + 1; 							//TEMP_I2C_ADDR + 1 > SLA+R 준비, R=1
		TWCR = _BV(TWINT) | _BV(TWEN);				// SLA+R 전송
		while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);	// 1st DATA 준비
		while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

		//온도센서는 16bit 기준으로 값을 가져오므로
		//8비트씩 2번을 받아야 한다.
		high = TWDR;								// 1 byte DATA 수신
		TWCR = _BV(TWINT) | _BV(TWEN); 				// SLA+R 전송
		while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

		low = TWDR;								// 1 byte DATA 수신
		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);	// STOP 전송

		TIMSK = ((((int)high<<8)+low) >= 33) ? TIMSK | _BV(TOIE2): TIMSK & ~_BV(TOIE2);

		OS_EXIT_CRITICAL();
		OSQPost(MsgQ,&low);
		OSQPost(MsgQ,&high);		//메시지큐로 low, high 데이터 전송

	// value post

		OSTimeDlyHMSM(0, 0, 0, 500);
	}
}
