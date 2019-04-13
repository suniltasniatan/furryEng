#include <Arduino.h>
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#define STACK_SIZE 100
#define dataPin 7
#define latchPin 8
#define clockPin 13
#define rear 2
#define RIGHT_BACK_FORWARD 3
#define LEFT_FRONT_FORWARD 12
#define RIGHT_FRONT_FORWARD 9
#define LEFT_BACK_FORWARD 4
#define RIGHT_FRONT_REVERSE 5
#define LEFT_FRONT_REVERSE 6
#define RIGHT_BACK_REVERSE 10
#define LEFT_BACK_REVERSE 11
unsigned long threshold_F;
unsigned long threshold2_F;
unsigned long threshold_R;
unsigned long threshold_L;
unsigned long threshold_LT;
unsigned long threshold_RT;
unsigned long Timer;
boolean flag_MOTOR;

int ledOn2[]={
  1,2,4,8,16,32,64,128,0,0,0,0, 0, 0, 0,  0, 0, 0, 0,0,0,0,0,128,64,32,16,8,4,2,1};
int ledOn1[]={
  0,0,0,0, 0, 0, 0,  0,1,2,4,8,16,32,64,128,64,32,16,8,4,2,1,  0, 0, 0, 0,0,0,0,0};


byte leds = 0;
boolean flag;
char blueToothVal;
char lastValue;

SoftwareSerial hc06 = SoftwareSerial(1,0);

xQueueHandle QueueLED;
xQueueHandle QueueMotor;
//xQueueHandle QueueAudio;
TickType_t xTickTime = 1000;

///////LEDS///////////////////////////////////////////////////////////
void taskRearMoving(){
	digitalWrite(rear, HIGH);
	delay(500);
	digitalWrite(rear, LOW);
	delay(500);
}


void taskFrontMoving(){
	leds = 0;
		   digitalWrite(latchPin, LOW);
		   shiftOut(dataPin, clockPin, LSBFIRST, leds);
		   digitalWrite(latchPin, HIGH);
		delay(60);
		for (int i = 0; i < 8; i++)
		{
		  bitSet(leds, i);
		   digitalWrite(latchPin, LOW);
		   shiftOut(dataPin, clockPin, LSBFIRST, leds);
		   digitalWrite(latchPin, HIGH);
		  delay(60);
		}
}



void taskRearStationary(){
	digitalWrite(rear, HIGH);
	delay(250);
	digitalWrite(rear, LOW);
	delay(250);
}

void taskFrontStationary(){
	leds = 0;
	   uint8_t leds = 1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
	   digitalWrite(latchPin, LOW);
	   shiftOut(dataPin, clockPin, LSBFIRST, leds);
	   digitalWrite(latchPin, HIGH);
	delay(500);
	for (int i = 0; i < 8; i++)
	{
	  bitSet(leds, i);
	   uint8_t leds = 1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
	   digitalWrite(latchPin, LOW);
	   shiftOut(dataPin, clockPin, LSBFIRST, leds);
	   digitalWrite(latchPin, HIGH);
	  delay(500);
	}
}

/////MOTOR//////////////////////////////////////////////////
/*
void taskMotorForward(){
	threshold_F = millis() + 200;
	if(millis()<threshold_F){
	      analogWrite(RIGHT_BACK_FORWARD, 255);
	      analogWrite(LEFT_FRONT_FORWARD, 190); //right motor moves forward at full speed
	      analogWrite(LEFT_BACK_FORWARD, 130);
	      analogWrite(RIGHT_FRONT_FORWARD, 255);
	    }
	    if(millis()>threshold_F){
	    if(flag_MOTOR==0){
	    threshold2_F =millis()+500;
	      while(millis()<threshold2_F){
	      analogWrite(RIGHT_BACK_FORWARD, 255);
	      analogWrite(LEFT_FRONT_FORWARD, 180); //right motor moves forward at full speed
	      analogWrite(LEFT_BACK_FORWARD, 0);
	      analogWrite(RIGHT_FRONT_FORWARD, 255);
	      }
	      flag_MOTOR=1;
	    }
	    else if(flag_MOTOR==1){
	    threshold2_F = millis()+400;
	      while(millis()<threshold2_F){
	      analogWrite(RIGHT_BACK_FORWARD, 255);
	      analogWrite(LEFT_FRONT_FORWARD, 0); //right motor moves forward at full speed
	      analogWrite(LEFT_BACK_FORWARD, 190);
	      analogWrite(RIGHT_FRONT_FORWARD, 255);
	      }
	      flag_MOTOR=0;
	    }
	    }

}
*/

void taskMotorForward(){
	threshold_F = millis() + 200;
	if(millis()<threshold_F){
	      analogWrite(RIGHT_BACK_FORWARD, 255);
	      analogWrite(LEFT_FRONT_FORWARD, 190); //right motor moves forward at full speed
	      analogWrite(LEFT_BACK_FORWARD, 130);
	      analogWrite(RIGHT_FRONT_FORWARD, 255);
	    }
	    if(millis()>threshold_F){
	    if(flag_MOTOR==0){
	    threshold2_F =millis()+500;
	      while(millis()<threshold2_F){
	      analogWrite(RIGHT_BACK_FORWARD, 255);
	      analogWrite(LEFT_FRONT_FORWARD, 180); //right motor moves forward at full speed
	      analogWrite(LEFT_BACK_FORWARD, 0);
	      analogWrite(RIGHT_FRONT_FORWARD, 255);
	      }
	      flag_MOTOR=1;
	    }
	    else if(flag_MOTOR==1){
	    threshold2_F = millis()+400;
	      while(millis()<threshold2_F){
	      analogWrite(RIGHT_BACK_FORWARD, 255);
	      analogWrite(LEFT_FRONT_FORWARD, 0); //right motor moves forward at full speed
	      analogWrite(LEFT_BACK_FORWARD, 190);
	      analogWrite(RIGHT_FRONT_FORWARD, 255);
	      }
	      flag_MOTOR=0;
	    }
	    }

}

void taskMotorBackward(){
	analogWrite(RIGHT_FRONT_REVERSE, 255);
	analogWrite(LEFT_FRONT_REVERSE, 255); //right motor moves forward at full speed
	analogWrite(RIGHT_BACK_REVERSE, 255);
	analogWrite(LEFT_BACK_REVERSE, 255);
}

void stop(){
	analogWrite(RIGHT_FRONT_REVERSE, 0);
	analogWrite(LEFT_FRONT_REVERSE, 0); //right motor moves forward at full speed
	analogWrite(RIGHT_BACK_REVERSE, 0);
	analogWrite(LEFT_BACK_REVERSE, 0);
    analogWrite(RIGHT_BACK_FORWARD, 0);
    analogWrite(LEFT_FRONT_FORWARD, 0); //right motor moves forward at full speed
    analogWrite(LEFT_BACK_FORWARD, 0);
    analogWrite(RIGHT_FRONT_FORWARD, 0);
}

void taskMotorRight(){
	threshold_R = millis() + 350;
	if(millis()<threshold_R){
	    analogWrite(LEFT_FRONT_FORWARD, 255);
	    analogWrite(RIGHT_FRONT_REVERSE, 255); //right motor moves forward at full speed
	    analogWrite(RIGHT_BACK_REVERSE, 255);
	    analogWrite(LEFT_BACK_FORWARD, 255);
	} else {
	    analogWrite(LEFT_FRONT_FORWARD, 0);
	    analogWrite(RIGHT_FRONT_REVERSE, 0); //right motor moves forward at full speed
	    analogWrite(RIGHT_BACK_REVERSE, 0);
	    analogWrite(LEFT_BACK_FORWARD, 0);
	    }
}

void taskMotorLeft(){
	threshold_L = millis() + (unsigned long) 370;
	if(millis()<threshold_L){
	    analogWrite(LEFT_FRONT_REVERSE, 255);
	    analogWrite(RIGHT_FRONT_FORWARD, 255); //right motor moves forward at full speed
	    analogWrite(RIGHT_BACK_FORWARD, 255);
	    analogWrite(LEFT_BACK_REVERSE, 255);
	 } else {
	   analogWrite(LEFT_FRONT_REVERSE, 0);
	   analogWrite(RIGHT_FRONT_FORWARD, 0); //right motor moves forward at full speed
	   analogWrite(RIGHT_BACK_FORWARD, 0);
	   analogWrite(LEFT_BACK_REVERSE, 0);
     }

}

void taskMotorLeftTurn(){
	threshold_LT = millis() + 1000;
	if(millis()<threshold_LT){
	    analogWrite(LEFT_FRONT_FORWARD, 0);
	    analogWrite(RIGHT_FRONT_FORWARD, 150); //right motor moves forward at full speed
	    analogWrite(RIGHT_BACK_FORWARD, 150);
	    analogWrite(LEFT_BACK_FORWARD, 0);
	 } else {
	    analogWrite(LEFT_FRONT_FORWARD, 0);
	    analogWrite(RIGHT_FRONT_FORWARD, 0); //right motor moves forward at full speed
	    analogWrite(RIGHT_BACK_FORWARD, 0);
	    analogWrite(LEFT_BACK_FORWARD, 0);
	  }

}

void taskMotorRightTurn(){
	threshold_RT = millis() + 1000;
	 if(millis()<threshold_RT){
	    analogWrite(LEFT_FRONT_FORWARD, 130);
	    analogWrite(RIGHT_FRONT_FORWARD, 0); //right motor moves forward at full speed
	    analogWrite(RIGHT_BACK_FORWARD, 100);
	    analogWrite(LEFT_BACK_FORWARD, 130);
	    } else {
	      analogWrite(LEFT_FRONT_FORWARD, 0);
	    analogWrite(RIGHT_FRONT_FORWARD, 0); //right motor moves forward at full speed
	    analogWrite(RIGHT_BACK_FORWARD, 0);
	    analogWrite(LEFT_BACK_FORWARD, 0);
	    }

}

void taskMotorStationary(){
	analogWrite(LEFT_FRONT_FORWARD, 0);
	analogWrite(RIGHT_FRONT_FORWARD, 0); //right motor moves forward at full speed
	analogWrite(RIGHT_BACK_FORWARD, 0);
	analogWrite(LEFT_BACK_FORWARD, 0);
	analogWrite(LEFT_FRONT_REVERSE, 0);
	analogWrite(RIGHT_FRONT_REVERSE, 0); //right motor moves forward at full speed
    analogWrite(RIGHT_BACK_REVERSE, 0);
    analogWrite(LEFT_BACK_REVERSE, 0);

}

/////AUDIO///////////////////////////////////////////////////
void taskAudioPlay(){

}

void taskAudioStop(){

}
///////////////////////////////////////////////////////////////////

void taskLED_FRONT(void *p){
	char command;
	while(1){
		xQueueReceive(QueueLED, &command, 0);
		switch(command){

		case 'F': case 'B': case 'L': case 'R': case 'l': case 'r':
			taskFrontMoving();
			break;
		case 'S': case 'E':
			taskFrontStationary();
			break;
		}
	}
}

void taskLED_REAR(void *p){
	char command;
	while(1){
		xQueueReceive(QueueLED, &command, 0);
		switch(command){

		case 'F': case 'B': case 'L': case 'R': case 'l': case 'r':
			taskRearMoving();
			break;
		case 'S': case 'E':
			taskRearStationary();
			break;
		}
	}
}


void taskMotor(void *p){
	char command;
	while(1){
		xQueueReceive(QueueMotor, &command, 0);

		switch(command){
		case 'F':
			taskMotorForward();
			taskMotorStationary();
			//stop();
			break;
		case 'B':
			taskMotorBackward();
			stop();
			break;
		case 'R':
			taskMotorRight();
			stop();
			break;
		case 'L':
			taskMotorLeft();
			stop();
			break;
		case 'l':
			taskMotorLeftTurn();
			stop();
			break;
		case 'r':
			taskMotorRightTurn();
			stop();
			break;
		case 'S': case 'E':
			taskMotorStationary();
			stop();
			break;
		default:
			taskMotorStationary();
			stop();
			break;
		}
		stop();
	}
}
/*
void taskAudio(void *p){
	char command;
	while(1){
		xQueueReceive(QueueAudio, &command, 0);
		switch(command){
		case 'F': case 'B': case 'L': case 'R': case 'l': case 'r': case 'S':
			taskAudioPlay();
			break;
		case 'E':
			taskAudioStop();
			break;
		}
	}
}
*/
///////////////////////////////////////////////////////////////////

void taskBluetooth(void *p){
for(;;){
	if(hc06.available()){
		blueToothVal = hc06.read();
	}
	xQueueSendToFront(QueueLED, &blueToothVal, 0);
	xQueueSendToFront(QueueMotor, &blueToothVal, 0);
	//xQueueSend(QueueAudio, &blueToothVal, xTickTime);
}
}

void setup() {
	hc06.begin(9600);
	xTaskCreate(taskBluetooth, "Bluetooth", STACK_SIZE, (void *) 1, 1, NULL);;
	QueueLED = xQueueCreate(STACK_SIZE, sizeof(int));
	QueueMotor = xQueueCreate(STACK_SIZE, sizeof(int));
	//QueueAudio = xQueueCreate(STACK_SIZE, sizeof(int));
	xTaskCreate(taskLED_FRONT, "LED_FRONT", STACK_SIZE, (void * ) NULL, 1, NULL);
	xTaskCreate(taskLED_REAR, "LED_BACK", STACK_SIZE, (void * ) NULL, 1, NULL);
	xTaskCreate(taskMotor, "MOTOR", STACK_SIZE, (void * ) NULL, 1, NULL);
	//xTaskCreate(taskAudio, "AUDIO", STACK_SIZE, (void * ) NULL, 1, NULL);
	pinMode(rear, OUTPUT);
	pinMode(latchPin, OUTPUT);
	pinMode(dataPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	pinMode(LEFT_FRONT_FORWARD, OUTPUT);
	pinMode(RIGHT_BACK_FORWARD, OUTPUT);
	pinMode(RIGHT_FRONT_FORWARD, OUTPUT);
	pinMode(LEFT_BACK_FORWARD, OUTPUT);
	pinMode(RIGHT_FRONT_FORWARD, OUTPUT);
	pinMode(LEFT_FRONT_FORWARD, OUTPUT);
	pinMode(RIGHT_BACK_FORWARD, OUTPUT);
	pinMode(LEFT_BACK_FORWARD, OUTPUT);
	//threshold_F = millis() + 200;
	//threshold_R = millis() + 350;
	//threshold_L = millis() + 370;
	//threshold_LT = millis() + 1000;
	//threshold_RT = millis() + 1000;
	flag_MOTOR = 0;
}

void loop() {
	//xTaskCreate(taskBluetooth, "Bluetooth", STACK_SIZE, (void *) 1, 1, NULL);;
	vTaskStartScheduler();
}
