
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
char char1 = 0; //MSB of speed
char char2 = 0; //LSB of speed
unsigned int enc_l = 0;
unsigned int enc_r = 0;
boolean leftSpeedIncoming = false;//to indicate to read in motor speed when true
boolean rightSpeedIncoming = false;
boolean leftSpeedReceived = false;//to indicate ready to change motor speed to new speeds
boolean rightSpeedReceived = false;
boolean char1Received = false;
unsigned int time = 0;
unsigned int lastDisplayTime = 0;
unsigned int updateTime = 0;

int16_t countsLeft = 0;
int16_t countsRight = 0;

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
  if (serial->available()) { //process input from the USB
    char  inChar = (char)serial->read();
    switch(inChar){
      case 'V':
        serial->println("butter");
        break;
      case 'E':
        lcd.clear();
        lcd.print(enc_l);
        lcd.gotoXY(0, 1);
        lcd.print(enc_r);
      
        //left encoder value
        enc_l = encoders.getCountsLeft();
        serial->write((enc_l & 0xFF00) >> 8);
        serial->write(enc_l & 0x00FF);
        
        //right encoder value
        enc_r = encoders.getCountsRight();
        serial->write((enc_r & 0xFF00) >> 8);
        serial->write(enc_r & 0x00FF);

        //encoder values received terminater
        serial->print('e');
        break;
      case 'L':
        leftSpeedIncoming = true;
        velL = 0;
        time = micros();
        break;
      case 'R':
        rightSpeedIncoming = true;
        velR = 0;
        break;
      case 'F':
        dir = 1;
        break;
      case 'B':
        dir = -1;
        break;
      default:
        if (leftSpeedIncoming){
          if (char1Received == false){
            char1 = inChar;
            char1Received = true;
          }else{
            char2 = (char)serial->read();
            //lcd.clear();
            
            char2 = inChar;
            velL = char1 << 2 | char2;
            
            //lcd.print(char1);
            //lcd.gotoXY(5, 0);
            //lcd.print(char2);
            //lcd.gotoXY(0, 1);
            //lcd.print(velL);
            
            char1 = 0;
            char2 = 0;
            time = micros() - time;
            if (dir == -1){
              velL = -velL;
            }
            leftSpeedIncoming = false;
            leftSpeedReceived = true;
            char1Received = false;
            //time = micros() - time;    
          }            
        }
        if (rightSpeedIncoming){
          if (char1Received == false){
            char1 = inChar;
            char1Received = true;
          }else{
            char2 = inChar;
            velR = char1 << 2 | char2;
            
            char1 = 0;
            char2 = 0;
            time = micros() - time;
            if (dir == -1){
              velR = -velR;
            }
            rightSpeedIncoming = false;
            rightSpeedReceived = true;
            char1Received = false;
            //time = micros() - time;

            //serial->write(countsLeft);
            //serial->write(countsRight);
          } 
        }
        break;
    } 
  }
}

void loop() {
  

  static uint8_t displayErrorLeftCountdown = 0;
  static uint8_t displayErrorRightCountdown = 0;
  static uint8_t batteryLevel = 0;
  
  updateTime = (uint8_t)(millis() - lastDisplayTime);
  
  if (updateTime >= 10)
  {
    // check process speed
    //lcd.gotoXY(0, 1);
    //lcd.print(updateTime);
    
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
      countsLeft = encoders.getCountsLeft();
      countsRight = encoders.getCountsRight();
  
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
      //lcd.clear();
      //lcd.print("L");
      //lcd.gotoXY(1, 0);
      //lcd.print(velL);//countsLeft);
      //lcd.gotoXY(4, 0);
      //lcd.print("R:");
      //lcd.gotoXY(5, 0);
      //lcd.print(velR);//countsRight);
      
      if (displayErrorLeftCountdown)
      {
        // Show an exclamation point on the first line to
        // indicate an error from the left encoder.
        //lcd.gotoXY(7, 0);
        //lcd.print('!');
        displayErrorLeftCountdown--;
      }
  
      if (displayErrorRightCountdown)
      {
        // Show an exclamation point on the second line to
        // indicate an error from the left encoder.
        //lcd.gotoXY(7, 1);
        //lcd.print('!');
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
    /*batteryLevel = readBatteryMillivolts();
    lcd.gotoXY(0, 1);
    lcd.print("        ");
    lcd.gotoXY(0, 1);
    lcd.print("B:");
    lcd.gotoXY(2, 1);
    lcd.print(batteryLevel);
    */
    
  }
}

