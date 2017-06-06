#include <PID_v1.h>

int sensorPin = A0;  // select the input pin for the 10K potentiometer
int sensorValue = 0; // variable to store the value coming from the sensor
int setTemp = 0;     // variable to store temp desired
int SSRHPin = 6;     //Turn on heat (electric or gas)
char *heat;
double currentTemp = 0;

//Define Variables we'll be connecting to for PID
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters for PID
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

void setup()
{
  Serial.begin(115200);
  pinMode(SSRHPin, OUTPUT);
  digitalWrite(SSRHPin, LOW);

  windowStartTime = millis();

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void printTemperature()
{
  currentTemp = 30;
  Serial.print("Temp: ");
  Serial.println(currentTemp);
}

void loop(void)
{
  Serial.println();
  Serial.print("Loop starting. Time: ");
  Serial.println(millis());
  //initialize the variables we're linked to
  Input = (currentTemp);
  Serial.print("Input: ");
  Serial.print(Input);
  Serial.println();
  Setpoint = (setTemp);
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.println();
  myPID.Compute();

  Serial.println();
  Serial.print("Computed.  Output: ");
  Serial.println(Output);

  static boolean bHeatOn = false;

  delay(500);

  sensorValue = analogRead(sensorPin);

  setTemp = sensorValue / 10.24; //Gives us a set temp range between 0 and 99 degrees

  printTemperature();
  if (bHeatOn)
  {
    Serial.println("Heat On");
  }
  else
  {
    Serial.println("Heat Off");
  }

  //Heating PID Relay ON/OFF
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime)
  {
    digitalWrite(SSRHPin, HIGH);
    bHeatOn = true;
    Serial.print("Output: ");
    Serial.print(Output);
    Serial.println();
  }
  else
  {
    digitalWrite(SSRHPin, LOW);
    bHeatOn = false;
  }
  Serial.println("Loop completed.");
  Serial.println();
}