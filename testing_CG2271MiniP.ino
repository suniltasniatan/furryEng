#define Rear 3
#define dataPin = 4;
#define latchPin = 5;
#define clockPin = 6;
byte leds = 0;

void changeState(){
  state=!state;
}

//tells the Rear LEDs what to do when the bot is not moving
void stationaryTaskRed(){
  digitalWrite(Rear, HIGH);  
  delay(250);
  digitalWrite(Rear, LOW);
  delay(250);
}

//tells the Front LEDs what to do when the bot is not moving
void stationaryTaskGreen(){
  digitalWrite(Rear, HIGH);  
  delay(250);
  digitalWrite(Rear, LOW);
  delay(250);
}

//tells the Rear LEDs what to do when the bot is moving
void movingTaskRed(){
  digitalWrite(Rear, HIGH);  
  delay(500);
  digitalWrite(Rear, LOW);
  delay(500);
}

//tells the Front LEDs what to do when the bot is moving
void movingTaskGreen(){
  leds = 0;
  updateShiftRegister();
  delay(500);
  for (int i = 0; i < 8; i++)
  {
    bitSet(leds, i);
    updateShiftRegister();
    delay(500);
  }
}

void updateShiftRegister()
{
   digitalWrite(latchPin, LOW);
   shiftOut(dataPin, clockPin, LSBFIRST, leds);
   digitalWrite(latchPin, HIGH);
}


void setup() {
  pinMode(Rear, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);

  //stops moving/starts moving
  attachInterrupt(0,changeState(),RISING); //like an interrupt???
}  
}
void loop() { 
  //not moving flag
  if(state==0){
  stationaryTaskRed(); 
  stationaryTaskGreen(); 
  //moving flag
  else if (state==1){
  movingTaskRed();
  movingTaskGreen();
  }
}
