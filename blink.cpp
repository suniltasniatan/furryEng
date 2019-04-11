#include <Arduino.h>
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#define STACK_SIZE 200
#include <semphr.h>
#include <queue.h>
#define dataPin 7
#define latchPin 8
#define clockPin 13
#define RIGHT_BACK_SPEED 3
#define LEFT_FRONT_SPEED 12
#define RIGHT_FRONT_SPEED 9 //front ones
#define LEFT_BACK_SPEED 4
//#define speaker 
#define Rear 3
//left with pin 13 unused
byte leds = 0;

xQueueHandle QueueLED = xQueueCreate(STACK_SIZE, sizeof(int));
xQueueHandle QueueMotor = xQueueCreate(STACK_SIZE, sizeof(int));
xQueueHandle QueueAudio = xQueueCreate(STACK_SIZE, sizeof(int));
TickType_t xTickTime = 1000;
//////////////////NOTES///////////////////////////////////////////////////
void playNote(char note, int duration){
char names[] = {'c','d','e','f'};
int tones [] = {1915,1700,1519,1432};
int threshold;
int threshold2;
boolean flag;

for (int i=0; i<4;i++){
	if(names[i]==note){
		playTone(tones[i], duration);
	}
}
};

char notesOngoing[]= "degggggggdegggggggdegggggggggf";
int beatsOngoing[]={2,2,1,1,1,1,1,1,1,2,2,1,1,1,1,1,1,1,2,2,1,1,1,1,1,1,1,2,2,1};
int tempo=300;

void playTone(int tone, int duration){
	for(long i=0; i<duration * 1000L; i +=tone*2){
		digitalWrite(speaker, HIGH);
		delayMicroseconds(tone);
	}
}

///////////////////////////////////////////////////////////////////////////
///////////////LEDCOMMANDS/////////////////////////////////////////////////
void taskRearStationary(){
	digitalWrite(Rear, HIGH);
	delay(250);
	digitalWrite(Rear, LOW);
	delay(250);
}

void taskRearMoving(){
	digitalWrite(Rear, HIGH);
	delay(500);
	digitalWrite(Rear, LOW);
	delay(500);
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

void taskFrontMoving(){
	leds = 0;
	   digitalWrite(latchPin, LOW);
	   shiftOut(dataPin, clockPin, LSBFIRST, leds);
	   digitalWrite(latchPin, HIGH);
	delay(500);
	for (int i = 0; i < 8; i++)
	{
	  bitSet(leds, i);
	   digitalWrite(latchPin, LOW);
	   shiftOut(dataPin, clockPin, LSBFIRST, leds);
	   digitalWrite(latchPin, HIGH);
	  delay(500);
	}
}

/*void updateShiftRegister(){
   digitalWrite(latchPin, LOW);
   shiftOut(dataPin, clockPin, LSBFIRST, leds);
   digitalWrite(latchPin, HIGH);
}

void updateShiftRegisterStationary(){
   uint8_t leds = 1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
   digitalWrite(latchPin, LOW);
   shiftOut(dataPin, clockPin, LSBFIRST, leds);
   digitalWrite(latchPin, HIGH);
}
*/
//////////////////////////////////////////////////////////////////////
//////AUDIO COMMANDS//////////////////////////////////////////////////
void taskFinish(){

}

void taskOngoing(){

}
/////////////////////////////////////////////////////////////////////
/////////////MOTOR COMMANDS//////////////////////////////////////////
void taskMotorStationary(){

}

void taskMotorForward(){
	while(1){
	if(millis()<threshold){
          analogWrite(RIGHT_BACK_SPEED, 255);
          analogWrite(LEFT_FRONT_SPEED, 190); //right motor moves forward at full speed
          analogWrite(LEFT_BACK_SPEED, 130); 
          analogWrite(RIGHT_FRONT_SPEED, 255);
        } 
		
    if(millis()>threshold){
    if(flag==0){
      threshold2 =millis()+400;
      while(millis()<threshold2){
      analogWrite(RIGHT_BACK_SPEED, 255);
      analogWrite(LEFT_FRONT_SPEED, 180); //right motor moves forward at full speed
      analogWrite(LEFT_BACK_SPEED, 0); 
      analogWrite(RIGHT_FRONT_SPEED, 255);
      }
      flag=1;
    } 
    else if(flag==1){
      threshold2 = millis()+500;
      while(millis()<threshold2){
      analogWrite(RIGHT_BACK_SPEED, 255);
      analogWrite(LEFT_FRONT_SPEED, 0); //right motor moves forward at full speed
      analogWrite(LEFT_BACK_SPEED, 190); 
      analogWrite(RIGHT_FRONT_SPEED, 255);
      }
      flag=0;
    }
    }
}

}

void taskMotorReverse(){

}

void taskMotorLeft(){

}

void taskMotorRight(){


}

//////////////////////////////////////////////////////////////////////


void taskLED(void *p){
	String command;
	while(1){
		xQueueReceive(QueueLED, &command, xTickTime);
		if((command == "STATIONARY")||(command == "OFF")){
				taskRearStationary();
				taskFrontStationary();
		}
		else{
			taskRearMoving();
			taskFrontMoving();
		}
	}
}

void taskMotor(void *p) {
	String command;
	while(1){
		xQueueReceive(QueueMotor, &command, xTickTime);
		if((command== "STATIONARY")||(command == "OFF")){
			taskMotorStationary();
		}
		else if (command == "FORWARD"){
			taskMotorForward();
		}
		else if (command == "REVERSE"){
			taskMotorReverse();
		}
		else if (command == "LEFT"){
			taskMotorLeft();
		}
		else{
			taskMotorRight();
		}
	}
}


void taskAudio(void *p){
	String command;
	xQueueReceive(QueueAudio, &command, xTickTime);
	if(command == "OFF"){
		taskFinish();
	}
	else{
		taskOngoing();
	}
}

void taskBluetooth(void *p){
	//bluetooth to send the diff queues
	String command;
	if(COMMAND == "OFF"){
		command = "OFF";
		xQueueSend(QueueLED, &command, xTickTime);
		xQueueSend(QueueMotor, &command, xTickTime);
		xQueueSend(QueueAudio, &command, xTickTime);
	}
	if(COMMAND == "STATIONARY"){
		command = "STATIONARY";
		xQueueSend(QueueLED, &command, xTickTime);
		xQueueSend(QueueMotor, &command, xTickTime);
		xQueueSend(QueueAudio, &command, xTickTime);
	}
	else if (COMMAND == "RIGHT"){
		command = "RIGHT";
		xQueueSend(QueueLED, &command, xTickTime);
		xQueueSend(QueueMotor, &command, xTickTime);
		xQueueSend(QueueAudio, &command, xTickTime);
	}
	else if (COMMAND == "REVERSE"){
		command = "REVERSE";
		xQueueSend(QueueLED, &command, xTickTime);
		xQueueSend(QueueMotor, &command, xTickTime);
		xQueueSend(QueueAudio, &command, xTickTime);
	}
	else if (COMMAND == "LEFT"){
		command = "LEFT";
		xQueueSend(QueueLED, &command, xTickTime);
		xQueueSend(QueueMotor, &command, xTickTime);
		xQueueSend(QueueAudio, &command, xTickTime);
	}
	else{
		command = "FORWARD";
		xQueueSend(QueueLED, &command, xTickTime);
		xQueueSend(QueueMotor, &command, xTickTime);
		xQueueSend(QueueAudio, &command, xTickTime);
	}
}
void setup() {
  Serial.begin(115200);
	xTaskCreate(taskBluetooth, "Bluetooth", STACK_SIZE, (void *) 1, 1, NULL);
	pinMode(Rear, OUTPUT);
	pinMode(latchPin, OUTPUT);
	pinMode(dataPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	pinMode(LEFT_FRONT_SPEED, OUTPUT);
        pinMode(RIGHT_BACK_SPEED, OUTPUT);
        pinMode(RIGHT_FRONT_SPEED, OUTPUT);
        pinMode(LEFT_BACK_SPEED, OUTPUT);
        threshold = millis() + 200;

}

void loop() {
	xTaskCreate(taskLED, "LED", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(taskMotor, "Motor", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(taskAudio, "Audio", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(taskBluetooth, "Bluetooth", STACK_SIZE, (void *) 1, 1, NULL);
	vTaskStartScheduler();
}

