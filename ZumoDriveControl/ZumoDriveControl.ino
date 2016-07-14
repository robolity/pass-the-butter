
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;

//globals for motor speeds
signed int velL; //motor speed received
signed int velR;
signed int dir = 1;   //direction speed recieved is in
signed int accum; //accumulation factor for adding speed characters
boolean speedIncoming = false; //to indicate receiving left & right speed values
boolean leftSpeedIncoming = false;//to indicate to read in motor speed when true
boolean rightSpeedIncoming = false;
boolean leftSpeedReceived = false;//to indicate ready to change motor speed to new speeds
boolean rightSpeedReceived = false;
unsigned int time = 0;

//globals for encoders:
const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";
char report[80];


void setup() {
  Serial.begin(115200);
}

// Reads in speeds in form [dir][side]###
// eg to move right forward speed 140, recieve via serial FR140 
void processSerial(Stream *serial){
  do
  {
    if (serial->available()) { //process input from the USB
      char  inChar = (char)serial->read();
      switch(inChar){
        case 'V':
          serial->println("ZUMO32U4");
          break;
        case 'S':
          speedIncoming = true;
          break;
        case 'L':
          leftSpeedIncoming = true;
          velL = 0;
          accum = 100;
          time = micros();
          break;
        case 'R':
          rightSpeedIncoming = true;
          velR = 0;
          accum = 100;
          break;
        case 'F':
          dir = 1;
          break;
        case 'B':
          dir = -1;
          break;
        default:
          if (inChar >= '0' && inChar <= '9'){
            if (leftSpeedIncoming){
              velL = velL + int((inChar - '0')) * accum;
              time = micros() - time;
              accum = accum / 10; //decreases accum for next digit
              
              // Update the LCD with process time
              //lcd.clear();
              //lcd.print(time);
              
              
              if (accum < 1){ //apply direction and stop adding to vel_l
                velL = velL * dir; 
                leftSpeedIncoming = false;
                leftSpeedReceived = true;
                //time = micros() - time;
                //lcd.gotoXY(0, 1);
                //lcd.print(time);
              } 
            }
            
            if (rightSpeedIncoming){
              velR = velR + int((inChar - '0')) * accum;
              
              accum = accum / 10; //decreases accum for next digit
              
              if (accum < 1){ //apply direction and stop adding to vel_l
                velR = velR * dir; 
                rightSpeedIncoming = false;
                rightSpeedReceived = true;
                
                speedIncoming = false;
                serial->print("D");
                
                //serial->print("R");
                //serial->println(velR);
              }
            }
          }
          break;
      } 
    }
  } while (speedIncoming);
}

void loop() {
  
  static uint8_t lastDisplayTime;
  static uint8_t displayErrorLeftCountdown = 0;
  static uint8_t displayErrorRightCountdown = 0;
  static uint8_t batteryLevel = 0;

  if ((uint8_t)(millis() - lastDisplayTime) >= 50)
  {
    lastDisplayTime = millis();
    
    //****Serial read of intended motor speeds
    processSerial(&Serial);
    
    // Update motor speeds only if new speeds are fully received for both wheels
    if(leftSpeedReceived && rightSpeedReceived){
      leftSpeedReceived = false;
      rightSpeedReceived = false;
      
      //****Setting motors to intended speeds
      velL = constrain(velL, -400, 400);
      velR = constrain(velR, -400, 400);
      
      motors.setSpeeds(velL, velR);
      
      //****Reading encoder values
      int16_t countsLeft = encoders.getCountsLeft();
      int16_t countsRight = encoders.getCountsRight();
  
      bool errorLeft = encoders.checkErrorLeft();
      bool errorRight = encoders.checkErrorRight();
  
      if(encoders.checkErrorLeft())
      {
        // An error occurred on the left encoder channel.
        // Display it on the LCD for the next 10 iterations and
        // also beep.
        displayErrorLeftCountdown = 10;
        buzzer.playFromProgramSpace(encoderErrorLeft);
      }
  
      if(encoders.checkErrorRight())
      {
        // An error occurred on the left encoder channel.
        // Display it on the LCD for the next 10 iterations and
        // also beep.
        displayErrorRightCountdown = 10;
        buzzer.playFromProgramSpace(encoderErrorRight);
      }
      
      
      
      // Update the LCD with encoder counts and error info.
      lcd.clear();
      lcd.print("L");
      lcd.gotoXY(1, 0);
      lcd.print(velL);//countsLeft);
      lcd.gotoXY(4, 0);
      lcd.print("R:");
      lcd.gotoXY(5, 0);
      lcd.print(velR);//countsRight);
      
      if (displayErrorLeftCountdown)
      {
        // Show an exclamation point on the first line to
        // indicate an error from the left encoder.
        lcd.gotoXY(7, 0);
        lcd.print('!');
        displayErrorLeftCountdown--;
      }
  
      if (displayErrorRightCountdown)
      {
        // Show an exclamation point on the second line to
        // indicate an error from the left encoder.
        lcd.gotoXY(7, 1);
        lcd.print('!');
        displayErrorRightCountdown--;
      }
  
      /**** Send encoder information to the serial monitor also.
      snprintf_P(report, sizeof(report),
          PSTR("%6d %6d %1d %1d"),
          countsLeft, countsRight, errorLeft, errorRight);
      //Serial.println(report);
      */
    }
    
    // check battery level
    batteryLevel = readBatteryMillivolts();
    lcd.gotoXY(0, 1);
    lcd.print("        ");
    lcd.gotoXY(0, 1);
    lcd.print("B:");
    lcd.gotoXY(2, 1);
    lcd.print(batteryLevel);
  
    
  }
}

