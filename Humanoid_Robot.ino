//This code is written by Technolokar and it is used for making the IoT based Voice controlled Humanoid Robot using the Nodemcu.

#define BLYNK_PRINT Serial
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include<SoftwareSerial.h> 
//Started SoftwareSerial at RX and TX pin of ESP8266/NodeMCU
SoftwareSerial s(D3,D4);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#include <ESP8266WiFi.h> 
#include <BlynkSimpleEsp8266.h>

char auth[] = "7xza6450upBYkzysX33A8D8bObs6EpTm";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "project2G";
char pass[] = "techo123";


//define  for degree

#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50



void setup()
{
  s.begin(115200);
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  
    Blynk.begin(auth, ssid, pass);

  while (Blynk.connect() == false)
  {
    // Wait until connected
  
  }
   pwm.setPWM(0, 0, pulseWidth(0));
  
  pwm.setPWM(1, 0, pulseWidth(0));
  pwm.setPWM(2, 0, pulseWidth(0));
  pwm.setPWM(3, 0, pulseWidth(0));
  pwm.setPWM(4, 0, pulseWidth(0));
  
  pwm.setPWM(5, 0, pulseWidth(180));
  pwm.setPWM(6, 0, pulseWidth(180));
  pwm.setPWM(7, 0, pulseWidth(0));

}


int pulseWidth(int angle)
{
int pulse_wide, analog_value;
pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
Serial.println(analog_value);
return analog_value;
}


BLYNK_WRITE(V0) // To respond to word Hello
{
  int pinvalue=param.asInt();
  if(pinvalue==1)
{
 delay(100);
   pwm.setPWM(0, 0, pulseWidth(180));
   delay(100);
   pwm.setPWM(1, 0, pulseWidth(180));
   delay(1000);
   pwm.setPWM(0, 0, pulseWidth(0));
   delay(1000);
   pwm.setPWM(1, 0, pulseWidth(0));
  delay(1000);
   pwm.setPWM(0, 0, pulseWidth(180));
   delay(100);
   pwm.setPWM(1, 0, pulseWidth(180));
   delay(1000);
     pwm.setPWM(0, 0, pulseWidth(0));
   delay(1000);
   pwm.setPWM(1, 0, pulseWidth(0));
  delay(1000);
  s.write("<a>");
  
  
  Serial.println("hello ");
  int pinvalue=0;
  Blynk.virtualWrite(V0,0);
 } 
  }



BLYNK_WRITE(V1)   //  waiter position on
{
  int pinvalue=param.asInt();
  if(pinvalue==1)
  {
pwm.setPWM(1, 0, pulseWidth(180));
delay(10);
pwm.setPWM(6, 0, pulseWidth(0));
delay(10);
  s.write("<b>");
Serial.println("waiter on");
int pinvalue=0;
Blynk.virtualWrite(V1,0);
}

}

BLYNK_WRITE(V2)  // waiter position off
{
 pwm.setPWM(1, 0, pulseWidth(0));
 delay(10);
pwm.setPWM(6, 0, pulseWidth(180));
delay(10);
 Serial.println("waiter postion off");
}



BLYNK_WRITE(V3)//pick and place right sholder
{
  int state1 = param.asInt();
  pwm.setPWM(0, 0, pulseWidth(state1));
  Serial.println("its working");
}
BLYNK_WRITE(V4)//pick and place right elbow
{
  int state2 = param.asInt();
  pwm.setPWM(1, 0, pulseWidth(state2));
}
BLYNK_WRITE(V5)//pick and place right forearm
{
  int state3 = param.asInt();
  pwm.setPWM(2, 0, pulseWidth(state3));
}
BLYNK_WRITE(V6)//pick and place right wrist
{
  int state4 = param.asInt();
  pwm.setPWM(3, 0, pulseWidth(state4));
}
BLYNK_WRITE(V7)//pick and place left sholder
{
  int state5 = param.asInt();
  pwm.setPWM(4, 0,pulseWidth(state5));
}
BLYNK_WRITE(V8)//pick and place left elbow
{
  int state6 = param.asInt();
  pwm.setPWM(5, 0,pulseWidth(state6));
}
BLYNK_WRITE(V9)//pick and place left forearm
{
  int state7 = param.asInt();
  pwm.setPWM(6, 0,pulseWidth(state7));
}
BLYNK_WRITE(V10)//pick and place left wrist
{
  int state8 = param.asInt();
  Serial.println(state8);
  pwm.setPWM(8, 0,pulseWidth(state8));
}
BLYNK_WRITE(V11)
{
  s.write("<e>"); //not working
}

BLYNK_WRITE(V12)
{
  s.write("<f>");
}

BLYNK_WRITE(V13) // not working
{
  s.write("<g>");
}
BLYNK_WRITE(V14) //
{
  s.write("<h>");
}

BLYNK_WRITE(V15)
{
  s.write("<i>");
}

BLYNK_WRITE(V16)
{
  s.write("<j>");
}

BLYNK_WRITE(V17) //not working
{
  s.write("<k>");
}

BLYNK_WRITE(V18)
{
  s.write("<l>"); //not working
}

BLYNK_WRITE(V19) //not working
{
  s.write("<m>");
}

BLYNK_WRITE(V20)
{
  s.write("<n>");
}
BLYNK_WRITE(V21)
{
  s.write("<o>");
}
BLYNK_WRITE(V22)
{
  s.write("<p>");
}
BLYNK_WRITE(V23)
{
  s.write("<q>");
}
BLYNK_WRITE(V24) //not working
{
  s.write("<r>");
}
BLYNK_WRITE(V25) //not working
{
  s.write("<s>");
}
BLYNK_WRITE(V26) //not working
{
  s.write("<t>");
}
BLYNK_WRITE(V27) //not working
{
  s.write("<u>");
}
void loop()
{
  Blynk.run();
 
}
