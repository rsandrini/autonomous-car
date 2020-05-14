
//Carrega a biblioteca do sensor ultrassonico
#include <Ultrasonic.h>
#include <Servo.h>

#include <Wire.h>
#include <HMC5883L_Simple.h>
 
//Define os pinos para o trigger e echo
#define pinoTrigger 6
#define pinoEcho 5

#define SERVO 7

Servo s; 
int servoPos; 

#define break_a 9
#define break_b 8

#define motor_a 12
#define motor_power_a 3
#define motor_b 13
#define motor_power_b 11

#define TURNFORCE 35000

#define FIXEDSPEEDMOTOR 50

#define MINDISTANCEFRONT 15
#define MINDISTANCESIDE 8

#define DEBUGMODE 0
 
Ultrasonic ultrasonic(pinoTrigger, pinoEcho);

HMC5883L_Simple Compass;

float distFront = 0;
float distLeft = 0;
float distRight = 0;

void setup()
{
  compassSetup();  
  
  //Setup Channel A
  pinMode(motor_a, OUTPUT); //Initiates Motor Channel A pin
  pinMode(break_a, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(motor_b, OUTPUT); //Initiates Motor Channel B pin
  pinMode(break_b, OUTPUT);  //Initiates Brake Channel B pin

  Serial.begin(9600); 
  
  Wire.begin();
  s.attach(SERVO);
  s.write(0);
}

void compassSetup(){
  Compass.SetDeclination(-19, 49, 'W');  
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
}
void loop()
{
  if (DEBUGMODE){
//    delay(2000); 
//    testServo(DEBUGMODE);
    compassTest();
  }  
  else{
    runAI(true);
  }
//  delay(2000);
}


void compassTest(){
  float heading = Compass.GetHeadingDegrees();
   
  Serial.print("Heading: ");
  Serial.println( heading );   
  delay(1000);
}

void testServo(bool debug)
{
//  for(servoPos = 0; servoPos < 180; servoPos++)
//  {
//    s.write(servoPos);
//    delay(15);
//  }
//  for(servoPos = 180; servoPos >= 0; servoPos--)
//  {
//    s.write(servoPos);
//    delay(15);
//  }
//  Serial.println("servo in " + (String)servoPos);

  for (int i =0; i< 10; i++){
    readFront(debug, true);
    Serial.println("servo in " + (String)servoPos);
    delay(500);
    
    readLeft(debug);
    Serial.println("servo in " + (String)servoPos);
    delay(500);

    readFront(debug, true);
    Serial.println("servo in " + (String)servoPos);
    delay(500);
    
    readRight(debug);
    Serial.println("servo in " + (String)servoPos);
    delay(500);
  }
}

void readLeft(bool debug){
  s.write(180);
//  delay(200);
  distLeft = (readSensors(debug, "left") + distLeft ) / 2;
}

void readRight(bool debug){
  s.write(0);
//  delay(200);
  distRight = (readSensors(debug, "right") + distRight) / 2;
}

void readFront(bool debug, bool useDelay){
  s.write(90);
  if (useDelay)
    delay(200);
  distFront = (readSensors(debug, "front") + distFront ) / 2;
}

float readSensors(bool debug, String sensorName){
    float distance=0;
    int loopMax = 4;
    
    for (int i=1; i<=loopMax; i++){
      delay(10);    
      distance = getSensorDistance(ultrasonic) + distance;      
      if (debug){
        debugDistance(distance / i, "sensor " + sensorName);
      }
    }

  return distance / loopMax;
   
}

void readAllSensors(bool debug){
    readLeft(debug);
    delay(500);
    readRight(debug);
    delay(500);
    readFront(debug, true);
}

void runAI(bool debug){
  readFront(debug, false);
  if (distFront < MINDISTANCEFRONT){
    activeBreak();
    // decide the new direction, but first stop
    //    if (read_sensor_front(debug) < MINDISTANCEFRONT/2){
    //      // emergency break
    //      reverse(FIXEDSPEEDMOTOR);
    //      delay(100);
    //    }
    activeBreak();
    delay(1000);
    readAllSensors(debug);

    if (distLeft < MINDISTANCESIDE && distRight < MINDISTANCESIDE){
      reverse(FIXEDSPEEDMOTOR);
      delay(1000);
      turn();
    }
    else{
      turn();
    }
  }
  else{
    forward(FIXEDSPEEDMOTOR);
  }
}

void turn(){
  if (distLeft > distRight){
      turnLeft(((FIXEDSPEEDMOTOR * 2) / 4) + FIXEDSPEEDMOTOR, false, true);
  }
  else {
      turnRight(((FIXEDSPEEDMOTOR * 2) / 4) + FIXEDSPEEDMOTOR, false, true);
  }
}


void motorTest(){
  forward(FIXEDSPEEDMOTOR);
  delay(500);
  reverse(FIXEDSPEEDMOTOR);
  delay(500);
  turnLeft(FIXEDSPEEDMOTOR, false, false);
  delay(500);
  turnRight(FIXEDSPEEDMOTOR, false, false);
  delay(500);
}

void debugDistance(float value, String sensorName){
    Serial.println("Distancia em cm (" + sensorName +"): " + (String)value);
}

void forward(int speedMotor){
   Serial.println("FORWARD");
   activeMotorA(speedMotor, false);
   activeMotorB(speedMotor, false);
}

void reverse(int speedMotor){
   Serial.println("BACK");
   activeMotorA(speedMotor, true);
   activeMotorB(speedMotor, true);
}

void turnLeft(int speedMotor, bool reverse, bool use4x4){
  Serial.println("LEFT");
  activeMotorA(speedMotor, reverse);
//  activeBreakB();
  if (use4x4)
    activeMotorB(speedMotor, !reverse);
}

void turnRight(int speedMotor, bool reverse, bool use4x4){
  Serial.println("RIGHT");
  activeMotorB(speedMotor, reverse);
//  activeBreakA();
  if (use4x4)
    activeMotorA(speedMotor, !reverse);
}

void activeBreak(){
  activeBreakA();
  activeBreakB();
}


float getSensorDistance(Ultrasonic ultrasonicSensor){
  //Le as informacoes do sensor, em cm e pol
  float cmMsec;
  long microsec = ultrasonicSensor.timing();
  cmMsec = ultrasonicSensor.convert(microsec, Ultrasonic::CM);
  return cmMsec;
}

void activeMotorA(int speedMotor, bool reverse){
    // MOTOR A
    if (reverse)
      digitalWrite(motor_a, LOW); //Establishes forward direction of Channel A
    else
      digitalWrite(motor_a, HIGH); //Establishes forward direction of Channel A
    digitalWrite(break_a, LOW);   //Disengage the Brake for Channel A
    analogWrite(motor_power_a, speedMotor);   //Spins the motor on Channel A at speed
}

void activeMotorB(int speedMotor, bool reverse){
    // MOTOR B
    if (reverse)
      digitalWrite(motor_b, LOW);  //Establishes backward direction of Channel B
    else
      digitalWrite(motor_b, HIGH);  //Establishes backward direction of Channel B
    digitalWrite(break_b, LOW);   //Disengage the Brake for Channel B
    analogWrite(motor_power_b, speedMotor);    //Spins the motor on Channel B at half speed
}

void activeBreakA(){
//  Serial.println("BREAK A");
  digitalWrite(break_a, HIGH);  //Engage the Brake for Channel A
}


void activeBreakB(){
//  Serial.println("BREAK B");
  digitalWrite(break_b, HIGH);  //Engage the Brake for Channel B
}
