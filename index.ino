#include <Servo.h>
Servo catapultServo;
#define SERVO_PIN 8
#define CATAPULT_FIRE_PIN 9
int catapultState = 0;

// ------ RIGHT MOTORS ------
#define RIGHT_CONTROL_PIN1 5 // Connected to right motors
#define RIGHT_CONTROL_PIN2 6 // Connected to right motors
#define RIGHT_ENABLE_PIN 7 // Enable
#define RIGHT_MOTOR_SPEED 255 // 0 - 255
bool rightMotorEnable = true;
bool rightMotorDirection = true;

// ------ LEFT MOTORS ------
#define LEFT_CONTROL_PIN1 3 // Connected to left motors
#define LEFT_CONTROL_PIN2 4 // Connected to left motors
#define LEFT_ENABLE_PIN 2 // Enable
#define LEFT_MOTOR_SPEED 255 // 0 - 255
bool leftMotorEnable = true;
bool leftMotorDirection = true;

// ------ SENSOR ------
#include "NewPing.h"
#define ECHO_PIN 11
#define TRIGGER_PIN 12
#define MAX_DISTANCE 200
float distance;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

#define ON_OFF_PIN 10
#define LED_PIN 13
#define BOTTLES_NUMBER 5

bool onOffState = false;

bool catapultLocked = true;

bool firstLeft = true;

void setup()
{
    Serial.begin(9600);

    pinMode(LEFT_CONTROL_PIN1, OUTPUT);
    pinMode(LEFT_CONTROL_PIN2, OUTPUT);
    pinMode(LEFT_ENABLE_PIN, OUTPUT);

    pinMode(RIGHT_CONTROL_PIN1, OUTPUT);
    pinMode(RIGHT_CONTROL_PIN2, OUTPUT);
    pinMode(RIGHT_ENABLE_PIN, OUTPUT);

    digitalWrite(LEFT_ENABLE_PIN, LOW);
    digitalWrite(RIGHT_ENABLE_PIN, LOW);

    pinMode(LED_PIN, OUTPUT);

    pinMode(ON_OFF_PIN, INPUT);
    pinMode(CATAPULT_FIRE_PIN, INPUT);

    pinMode(SERVO_PIN, OUTPUT);
    catapultServo.attach(SERVO_PIN);
    switchCatapultState();

    stopRobot();
}

void loop()
{

    if(digitalRead(ON_OFF_PIN) == HIGH){
        onOffState = !onOffState;
        if(onOffState == true){
            startRobot();
            // Turn on the LED
            analogWrite(LED_PIN, 255);
        }else{
            stopRobot();
            // Turn off the LED
            analogWrite(LED_PIN, 0);
        }
    }
    
    if(digitalRead(CATAPULT_FIRE_PIN) == HIGH){
        analogWrite(LED_PIN, 128);
        switchCatapultState();
        delay(2000);
    }

    updateDirection();
    updateSpeed();

    updateDistance();

    if(distance > 15.00 || distance == 0.00){
        forward();
    }else{
        for (int i = 0; i < BOTTLES_NUMBER; i++)
        {
            bypass();
        }
        
        stopRobot();
    }   
}

// Updaters
void updateDistance()
{
    distance = sonar.ping_cm();
    Serial.println(distance);
}

void updateSpeed()
{
    analogWrite(LEFT_ENABLE_PIN, leftMotorEnable ? LEFT_MOTOR_SPEED : 0);
    analogWrite(RIGHT_ENABLE_PIN, rightMotorEnable ? RIGHT_MOTOR_SPEED : 0);
}

void updateDirection()
{
    changeDirection(leftMotorDirection, LEFT_CONTROL_PIN1, LEFT_CONTROL_PIN2);
    changeDirection(rightMotorDirection, RIGHT_CONTROL_PIN1, RIGHT_CONTROL_PIN2);
}

// Bypass
void bypass(){
    Serial.println("bypass obstacle");
    analogWrite(LED_PIN, 255);
    turn(firstLeft);
    delay(1000);
    analogWrite(LED_PIN, 0);
    forward();
    delay(1000);
    analogWrite(LED_PIN, 255);
    turn(!firstLeft);
    delay(1000);
    analogWrite(LED_PIN, 0);
    firstLeft = !firstLeft;
}

// Turn
void turn(bool isLeft)
{
    leftMotorDirection = isLeft ? false : true;
    rightMotorDirection = isLeft ? true : false;
    updateDirection();
}

// Move forward
void forward()
{
    leftMotorDirection = true;
    rightMotorDirection = true;
    updateDirection();
}

// Stop
void stopRobot()
{
    leftMotorEnable = false;
    rightMotorEnable = false;
    updateSpeed();
}

// Start
void startRobot()
{
    leftMotorEnable = true;
    rightMotorEnable = true;
    updateSpeed();
}

// Switch direction
void changeDirection(bool value, int pin1, int pin2)
{
    digitalWrite(pin1, value ? HIGH : LOW);
    digitalWrite(pin2, value ? LOW : HIGH);
}

void fire()
{
    // pullCatapult(); => never implemented
    switchCatapultState();
}

void switchCatapultState()
{
    catapultLocked = !catapultLocked;
    catapultServo.write(catapultLocked ? 85 : 65);
}