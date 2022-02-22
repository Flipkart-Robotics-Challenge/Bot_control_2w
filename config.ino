#include"include_utils.h"
#include <analogWrite.h>
#include <Servo_ESP32.h>

/****servo pin****/
const int servoPin = 13;//15;

/****left motor*****/
const byte pin_left_a = 27;//23;//21;   //for encoder pulse A
const byte pin_left_b = 14;//22;//18;   //for encoder pulse B
const byte pin_left_fwd = 5;//32;//17; //for H-bridge: run motor forward
const byte pin_left_bwd = 18;//33;//5 ; //for H-bridge: run motor backward
const byte pin_left_pwm = 26;//4;//0; //for H-bridge: motor speed


/****left_front motocr****/
/*const byte pin_left_front_a = 27;//25;//27;   //for encoder pulse A
const byte pin_left_front_b = 26;//;26;   //for encoder pulse B
const byte pin_left_front_pwm = 33;//19;// 33; //for H-bridge: motor speed*/


/****Right motor*****/
const byte pin_right_a = 19;//13;//12;   //for encoder pulse A
const byte pin_right_b = 21;//12;//14;   //for encoder pulse B
const byte pin_right_fwd = 22;//18;//4; //for H-bridge: run motor forward
const byte pin_right_bwd = 23;//5;//16; //for H-bridge: run motor backward
const byte pin_right_pwm = 25;//2; //for H-bridge: motor speed


/****Right_front motor*****/
/*const byte pin_right_front_a = 35;//14;//35;   //for encoder pulse A
const byte pin_right_front_b = 32;//27;//32;   //for encoder pulse B
const byte pin_right_front_pwm = 25;//21;// 25; //for H-bridge: motor speed*/
const int freq = 5000;
const int PWM_ch_left = 0;
const int PWM_ch_right = 1;
const int resolution = 8;
void Set_Pins()
{
    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(pin_left_a,INPUT_PULLUP);
    pinMode(pin_left_b,INPUT_PULLUP);
    pinMode(pin_left_fwd,OUTPUT);
    pinMode(pin_left_bwd,OUTPUT);
    pinMode(pin_left_pwm,OUTPUT);
    digitalWrite(pin_left_fwd,LOW);
    digitalWrite(pin_left_bwd,LOW);
    ledcSetup(PWM_ch_left, freq, resolution);
    ledcAttachPin(pin_left_pwm, PWM_ch_left);
    //analogWriteResolution(pin_left_pwm, 8);
    

    /*pinMode(pin_left_front_a,INPUT_PULLUP);
    pinMode(pin_left_front_b,INPUT_PULLUP);
    analogWriteResolution(pin_left_front_pwm, 13);*/
    

    pinMode(pin_right_a,INPUT_PULLUP);
    pinMode(pin_right_b,INPUT_PULLUP);
    pinMode(pin_right_fwd,OUTPUT);
    pinMode(pin_right_bwd,OUTPUT);
    pinMode(pin_right_pwm,OUTPUT);
    digitalWrite(pin_right_fwd,LOW);
    digitalWrite(pin_right_bwd,LOW);
    ledcSetup(PWM_ch_right, freq, resolution);
    ledcAttachPin(pin_right_pwm, PWM_ch_right);
    //analogWriteResolution(pin_right_pwm, 8);
    

    /*pinMode(pin_right_front_a,INPUT_PULLUP);
    pinMode(pin_right_front_b,INPUT_PULLUP);
    analogWriteResolution(pin_right_front_pwm, 13);*/


    ledcWrite(PWM_ch_left, 0);
    //analogWrite(pin_left_pwm,0);   //stop motor
    digitalWrite(pin_left_fwd,0);  //stop motor
    digitalWrite(pin_left_bwd,0);  //stop motor
    //analogWrite(pin_left_front_pwm,0);   //stop motor

    ledcWrite(PWM_ch_right, 0);
    //analogWrite(pin_right_pwm,0);   //stop motor
    digitalWrite(pin_right_fwd,0);  //stop motor
    digitalWrite(pin_right_bwd,0);  //stop motor
    //analogWrite(pin_right_front_pwm,0);   //stop motor

    //servo1.attach(servoPin); //servo pin 
    //servo1.write(idel_angle);
    //servo1.attach(servoPin); //servo pin 
    
}

void Attach_Interrupts()
{
    attachInterrupt(digitalPinToInterrupt(pin_left_a), detect_a_left, RISING);
    //attachInterrupt(digitalPinToInterrupt(pin_left_front_a), detect_a_left_front, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_right_a), detect_a_right, RISING);
    //attachInterrupt(digitalPinToInterrupt(pin_right_front_a), detect_a_right_front, RISING);
}

void Deattach_Interrupts()
{
    detachInterrupt(pin_left_a);
    //detachInterrupt(digitalPinToInterrupt(pin_left_front_a), detect_a_left_front, RISING);
    detachInterrupt(pin_right_a);
    //detachInterrupt(digitalPinToInterrupt(pin_right_front_a), detect_a_right_front, RISING);
}
