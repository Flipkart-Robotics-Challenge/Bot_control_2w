#include <analogWrite.h>
#include <Servo_ESP32.h>
#include "include_utils.h"

int l_cnt, r_cnt;

typedef struct
{
  int encoder;
  int m_direction;
  int sv_speed; 
  double pv_speed;
  double set_speed;
  double e_speed;     // error of speed = set_speed - pv_speed
  double e_speed_pre; // last error of speed
  double e_speed_sum; // sum error of speed
  double pwm_pulse;   // this value is 0~255
  double kp;
  double ki;
  double kd;

  int temp_e_speed;
  int temp_set_speed;
  int temp_pv_speed;
  int temp_pwm_pulse;
  int temp_e_speed_sum;
  int temp_e_speed_pre;
  int temp_rpm;
} PID;

/**********left*************/
int encoder_left = 0;
int m_direction_left = 0;
int sv_speed_left = 50; // this value is 0~255
double pv_speed_left = 0;
double set_speed_left = 170;
double e_speed_left = 0;            // error of speed = set_speed - pv_speed
double e_speed_pre_left = 0;        // last error of speed
double e_speed_sum_left = 0;        // sum error of speed
double pwm_pulse_left = 0;          // this value is 0~255
double temp_pwm_pulse_left_fwd = 0; // this value is 0~255
double temp_pwm_pulse_left_bwd = 0; // this value is 0~255
double temp_pwm_pulse_left_lft = 0; // this value is 0~255
double temp_pwm_pulse_left_rht = 0; // this value is 0~255
double kp_left = 0.06;
double ki_left = 0.055;
double kd_left = 0.04;
int timer1_counter_left; // for timer
int i_left = 0;

int temp_e_speed_left = 0;
int temp_set_speed_left = 0;
int temp_pv_speed_left = 0;
int temp_pwm_pulse_left = 0;
int temp_e_speed_sum_left = 0;
int temp_e_speed_pre_left = 0;

int temp_left_rpm = 50;

/**********right*************/
int encoder_right = 0;
int m_direction_right = 0;
int sv_speed_right = 50; // this value is 0~255
double pv_speed_right = 0;
double set_speed_right = 170;
double e_speed_right = 0;            // error of speed = set_speed - pv_speed
double e_speed_pre_right = 0;        // last error of speed
double e_speed_sum_right = 0;        // sum error of speed
double pwm_pulse_right = 0;          // this value is 0~255
double temp_pwm_pulse_right_fwd = 0; // this value is 0~255
double temp_pwm_pulse_right_bwd = 0; // this value is 0~255
double temp_pwm_pulse_right_lft = 0; // this value is 0~255
double temp_pwm_pulse_right_rht = 0; // this value is 0~255
double kp_right = 0.06;
double ki_right = 0.055;
double kd_right = 0.04;
int timer1_counter_right; // for timer
int i_right = 0;


int temp_e_speed_right = 0;
int temp_set_speed_right = 0;
int temp_pv_speed_right = 0;
int temp_pwm_pulse_right = 0;
int temp_e_speed_sum_right = 0;
int temp_e_speed_pre_right = 0;

int temp_right_rpm = 50;

char linear_pwm_left, linear_pwm_right, rotary_pwm_left, rotary_pwm_right;

/******cmd_flag******/
uint8_t fwd, bwd, lft, rht, stp, act;
uint8_t linear_flg, rotary_flg;

/*******servo********/
Servo_ESP32 servo1;
#define idel_angle 40
int angleStep = 2;

int angleMin = 0;
int angleMax = 40;

/****motor enable****/
boolean motor_start = false;
// int speed_offset = 5;

/*****ISR*****/

void detect_a_left()
{
  encoder_left += 1;                          // increasing encoder at new pulse
  m_direction_left = digitalRead(pin_left_b); // read direction of motor
}

void detect_a_right()
{
  encoder_right += 1;                           // increasing encoder at new pulse
  m_direction_right = digitalRead(pin_right_b); // read direction of motor
}



/*******************************PID_computation_with_timer_interrupt************************************************************/
void IRAM_ATTR_onTime() // interrupt service routine - tick every 0.1sec
{
  portENTER_CRITICAL_ISR(&timerMux);
  interrupts++;
  portEXIT_CRITICAL_ISR(&timerMux);

  pv_speed_left = 60.0 * (encoder_left / 700.0) / 0.1; // calculate motor speed, unit is rpm
  encoder_left = 0;

  pv_speed_right = 60.0 * (encoder_right / 700.0) / 0.1; // calculate motor speed, unit is rpm
  encoder_right = 0;

  // print out speed
  if (Serial.available() <= 0)
  {
    // Serial.print("speed");
    Serial.println(pv_speed_left); // Print speed (rpm) value to Visual Studio

    Serial.println(pv_speed_right); // Print speed (rpm) value to Visual Studio

    /*Serial.println(pv_speed_left_front);         //Print speed (rpm) value to Visual Studio

    Serial.println(pv_speed_right_front);         //Print speed (rpm) value to Visual Studio*/
  }

  // PID program
  if (motor_start)
  {
    /*****************left_operation********************/
    e_speed_left = set_speed_left - pv_speed_left;
    pwm_pulse_left = (e_speed_left * kp_left) + 
                     (e_speed_sum_left * ki_left) +
                     ((e_speed_left - e_speed_pre_left) * kd_left);
    e_speed_pre_left = e_speed_left;  // save last (previous) error
    e_speed_sum_left += e_speed_left; // sum of error
    if (e_speed_sum_left > 4000)
      e_speed_sum_left = 4000;
    if (e_speed_sum_left < -4000)
      e_speed_sum_left = -4000;

    /*************Right_operation****************************/

    e_speed_right = set_speed_right - pv_speed_right;
    pwm_pulse_right = (e_speed_right * kp_right) + 
                      (e_speed_sum_right * ki_right) + 
                      ((e_speed_right - e_speed_pre_right) * kd_right);
    e_speed_pre_right = e_speed_right;  // save last (previous) error
    e_speed_sum_right += e_speed_right; // sum of error
    if (e_speed_sum_right > 4000)
      e_speed_sum_right = 4000;
    if (e_speed_sum_right < -4000)
      e_speed_sum_right = -4000;
  }
  else
  {
    /*****************left_operation********************/
    e_speed_left = 0;
    e_speed_pre_left = 0;
    e_speed_sum_left = 0;
    pwm_pulse_left = 0;

    /*************Right_operation****************************/
    e_speed_right = 0;
    e_speed_pre_right = 0;
    e_speed_sum_right = 0;
    pwm_pulse_right = 0;
  }

  /*Serial.print("left :");
  Serial.println(pwm_pulse_left);

  Serial.print("  rightt :");
  Serial.println(pwm_pulse_right);*/

  // update new speed left
  if ((pwm_pulse_left<255 & pwm_pulse_left> 0) && (pwm_pulse_right<255 & pwm_pulse_right> 0))
  {
    if (cmd_received == 1)
      Deattach_Interrupts();

    ledcSetup(PWM_ch_left, freq, resolution);
    ledcAttachPin(pin_left_pwm, PWM_ch_left);
    ledcWrite(PWM_ch_left, pwm_pulse_left);

    /* Serial.print("left PWM : ");
    Serial.println(pwm_pulse_left);
    Serial.println(pv_speed_left);
    */

    ledcSetup(PWM_ch_right, freq, resolution);
    ledcAttachPin(pin_right_pwm, PWM_ch_right);
    ledcWrite(PWM_ch_right, pwm_pulse_right);
    /*
    Serial.print("right PWM : ");
    Serial.println(pwm_pulse_right);
    Serial.println("PWM Written!!");
    Serial.println(pv_speed_right);
    */

    if (cmd_received == 1)
    {
      Attach_Interrupts();
      cmd_received = 0;
    }
  }
  else
  {

    if (pwm_pulse_left > 255 && pwm_pulse_right > 255)
    {
      if (cmd_received == 1)
        Deattach_Interrupts();

      ledcSetup(PWM_ch_left, freq, resolution);
      ledcAttachPin(pin_left_pwm, PWM_ch_left);
      ledcWrite(PWM_ch_left, 255);
      // Serial.println(pv_speed_left);

      ledcSetup(PWM_ch_right, freq, resolution);
      ledcAttachPin(pin_right_pwm, PWM_ch_right);
      ledcWrite(PWM_ch_right, 255);
      // Serial.println(pv_speed_right);

      if (cmd_received == 1)
      {
        Attach_Interrupts();
        cmd_received = 0;
      }
    }
    else
    {
      if (cmd_received == 1)
        Deattach_Interrupts();
    
      ledcSetup(PWM_ch_left, freq, resolution);
      ledcAttachPin(pin_left_pwm, PWM_ch_left);
      ledcWrite(PWM_ch_left, 0);
      // analogWrite(pin_left_pwm,0);
      // Serial.println(pv_speed_left);

      ledcSetup(PWM_ch_right, freq, resolution);
      ledcAttachPin(pin_right_pwm, PWM_ch_right);
      ledcWrite(PWM_ch_right, 0);
      // Serial.println(pv_speed_right);

      if (cmd_received == 1)
      {
        Attach_Interrupts();
        cmd_received = 0;
      }
    }
  endt:
    NULL;
  }
}





/******************************************Command_definitions******************************************/
void forward()
{
  //servo_idel();

  linear_flg = 1;
  rotary_flg = 0;

  set_speed_left = temp_left_rpm;
  set_speed_right = temp_right_rpm;

  digitalWrite(pin_left_fwd, 0); // run motor run forward
  digitalWrite(pin_left_bwd, 1);

  digitalWrite(pin_right_fwd, 0); // run motor run forward
  digitalWrite(pin_right_bwd, 1);

  if (cmd_received == 1)
    Deattach_Interrupts();

  Attach_Interrupts();
  timerAlarmEnable(timer);
}

void backward()
{
  //servo_idel();

  linear_flg = 1;
  rotary_flg = 0;

  set_speed_left = temp_left_rpm;
  set_speed_right = temp_right_rpm;
  /*
    pwm_pulse_left = rotary_pwm_left;
    pwm_pulse_right = rotary_pwm_right; */

  digitalWrite(pin_left_fwd, 1); // run motor run backward
  digitalWrite(pin_left_bwd, 0);

  digitalWrite(pin_right_fwd, 1); // run motor run backward
  digitalWrite(pin_right_bwd, 0);

  if (cmd_received == 1)
    Deattach_Interrupts();

  Attach_Interrupts();
  timerAlarmEnable(timer);
}

void right()
{
  //servo_idel();
  if (cmd_received == 1)
  {
    Deattach_Interrupts();
    timerAlarmDisable(timer);
  }

  linear_flg = 0;
  rotary_flg = 1;

  set_speed_left = 0;
  set_speed_right = 0;

  pwm_pulse_left = 0;  // rotary_pwm_left;
  pwm_pulse_right = 0; // rotary_pwm_right;

  digitalWrite(pin_left_fwd, 0); // run motor run forward
  digitalWrite(pin_left_bwd, 0);

  digitalWrite(pin_right_fwd, 0); // run motor run backward
  digitalWrite(pin_right_bwd, 0);
  delay(10);

  digitalWrite(pin_left_fwd, 1); // run motor run forward
  digitalWrite(pin_left_bwd, 0);

  digitalWrite(pin_right_fwd, 0); // run motor run backward
  digitalWrite(pin_right_bwd, 1);

  if ((cmd_received == 1) && (lft || rht))
  {
    ledcSetup(PWM_ch_left, freq, resolution);
    ledcAttachPin(pin_left_pwm, PWM_ch_left);
    ledcWrite(PWM_ch_left, 255);

    ledcSetup(PWM_ch_right, freq, resolution);
    ledcAttachPin(pin_right_pwm, PWM_ch_right);
    ledcWrite(PWM_ch_right, 255);

    if (cmd_received == 1)
    {
      Attach_Interrupts();
      timerAlarmEnable(timer);
      cmd_received = 0;
    }
  }
  delay(10);
  if (cmd_received != 1)
  {
    if ((l_cnt > 1) && (r_cnt > 1) && (lft || rht))
    {
      set_speed_right = 0;
      digitalWrite(pin_left_fwd, 0); // stop motor
      digitalWrite(pin_left_bwd, 0);

      digitalWrite(pin_right_fwd, 0); // stop motor
      digitalWrite(pin_right_bwd, 0);
    }
  }
}

void left()
{
  //servo_idel();
  if (cmd_received == 1)
  {
    Deattach_Interrupts();
    timerAlarmDisable(timer);
  }

  linear_flg = 0;
  rotary_flg = 1;
  set_speed_left = 0;
  set_speed_right = 0;

  pwm_pulse_left = 0;  // rotary_pwm_left;
  pwm_pulse_right = 0; // rotary_pwm_right;

  digitalWrite(pin_left_fwd, 0); // run motor run forward
  digitalWrite(pin_left_bwd, 0);

  digitalWrite(pin_right_fwd, 0); // run motor run backward
  digitalWrite(pin_right_bwd, 0);
  delay(10);

  digitalWrite(pin_left_fwd, 0); // run motor run backward
  digitalWrite(pin_left_bwd, 1);

  digitalWrite(pin_right_fwd, 1); // run motor run forward
  digitalWrite(pin_right_bwd, 0);

  if ((cmd_received == 1) && (lft || rht))
  {
    ledcSetup(PWM_ch_left, freq, resolution);
    ledcAttachPin(pin_left_pwm, PWM_ch_left);
    ledcWrite(PWM_ch_left, 255);
    

    ledcSetup(PWM_ch_right, freq, resolution);
    ledcAttachPin(pin_right_pwm, PWM_ch_right);
    ledcWrite(PWM_ch_right, 255);

    if (cmd_received == 1)
    {
      Attach_Interrupts();
      timerAlarmEnable(timer);
      cmd_received = 0;
    }
  }
  delay(10);

  if (cmd_received != 1)
  {
    if ((l_cnt > 1) && (r_cnt > 1) && (lft || rht))
    {
      set_speed_right = 0;
      digitalWrite(pin_left_fwd, 0); // stop motor
      digitalWrite(pin_left_bwd, 0);

      digitalWrite(pin_right_fwd, 0); // stop motor
      digitalWrite(pin_right_bwd, 0);
    }
  }
}

void stop()
{
  servo_idel();

  set_speed_left = 0;
  set_speed_right = 0;

  Attach_Interrupts();
  timerAlarmEnable(timer);
}

void disable_motor()
{
  servo_idel();
  Deattach_Interrupts();
  timerAlarmDisable(timer);
  digitalWrite(pin_left_fwd, 0); // run motor run backward
  digitalWrite(pin_left_bwd, 0);

  digitalWrite(pin_right_fwd, 0); // run motor run forward
  digitalWrite(pin_right_bwd, 0);
}

void increase_set_rpm()
{
  if (set_speed_left >= 255 && set_speed_right >= 255)
  {
    temp_left_rpm = set_speed_left = 255;
    temp_right_rpm = set_speed_right = 255;
    return;
  }
  temp_left_rpm = set_speed_left += 5;
  temp_right_rpm = set_speed_right += 5;
}
void decrease_set_rpm()
{
  if (set_speed_left <= 0 && set_speed_right <= 0)
  {
    temp_left_rpm = set_speed_left = 0;
    temp_right_rpm = set_speed_right = 0;
    return;
  }
  temp_left_rpm = set_speed_left -= 5;
  temp_right_rpm = set_speed_right -= 5;
}

void servo_idel()
{
  servo1.attach(servoPin); // servo pin
  servo1.write(idel_angle);
}

void servo_set()
{
  // disable_motor();
  for (int angle = angleMax; angle >= angleMin; angle -= angleStep)
  {
    servo1.write(angle);
    Serial.println(angle);
    delay(50);
  }

  for (int angle = angleMin; angle <= angleMax; angle += angleStep)
  {
    servo1.write(angle);
    Serial.println(angle);
    delay(50);
  }
}

int toggle = 0;
void servo_check()
{
  if (toggle)
  {
    servo1.write(40);
    toggle = 0;
  }

  else
  {
    servo1.write(0);
    toggle = 1;
  }
}

void store_data()
{
  temp_e_speed_left = e_speed_left;
  temp_set_speed_left = set_speed_left;
  temp_pv_speed_left = pv_speed_left;
  temp_pwm_pulse_left = pwm_pulse_left;
  temp_e_speed_left = e_speed_left;
  temp_e_speed_sum_left = e_speed_sum_left;
  temp_e_speed_pre_left = e_speed_pre_left;

  temp_e_speed_right = e_speed_right;
  temp_set_speed_right = set_speed_right;
  temp_pv_speed_right = pv_speed_right;
  temp_pwm_pulse_right = pwm_pulse_right;
  temp_e_speed_right = e_speed_right;
  temp_e_speed_sum_right = e_speed_sum_right;
  temp_e_speed_pre_right = e_speed_pre_right;
}

void get_data()
{
  e_speed_left = temp_e_speed_left;
  set_speed_left = temp_set_speed_left;
  pv_speed_left = temp_pv_speed_left;
  pwm_pulse_left = temp_pwm_pulse_left;
  e_speed_left = temp_e_speed_left;
  e_speed_sum_left = temp_e_speed_sum_left;
  e_speed_pre_left = temp_e_speed_pre_left;

  e_speed_right = temp_e_speed_right;
  set_speed_right = temp_set_speed_right;
  pv_speed_right = temp_pv_speed_right;
  pwm_pulse_right = temp_pwm_pulse_right;
  e_speed_right = temp_e_speed_right;
  e_speed_sum_right = temp_e_speed_sum_right;
  e_speed_pre_right = temp_e_speed_pre_right;
}

void reset_data()
{
  e_speed_left = 0;
  set_speed_left = 0;
  pv_speed_left = 0;
  pwm_pulse_left = 0;
  e_speed_left = 0;
  e_speed_sum_left = 0;
  e_speed_pre_left = 0;

  e_speed_right = 0;
  set_speed_right = 0;
  pv_speed_right = 0;
  pwm_pulse_right = 0;
  e_speed_right = 0;
  e_speed_sum_right = 0;
  e_speed_pre_right = 0;
}



/*******************************************  command  *************************************************/
void command(char *cmd)
{
  switch (*cmd)
  {
  case '0':
    servo_idel();
    if (linear_flg)
    {
      linear_pwm_left = pwm_pulse_left;
      linear_pwm_right = pwm_pulse_right;
      linear_flg = 0;
    }
    else if (rotary_flg)
    {
      rotary_pwm_left = pwm_pulse_left;
      rotary_pwm_right = pwm_pulse_right;
      rotary_flg = 0;
    }

    break;

  case '1':

    if (!bwd && !fwd)
      get_data();
    if (!fwd)
    {
      servo_idel();
      cmd_received = 1;
      forward();
      fwd = 1;
      bwd = lft = rht = stp = act = 0;
      l_cnt++;
    }
    break;

  case '2':
    if (!bwd && !fwd)
      get_data();
    if (!bwd)
    {
      servo_idel();
      cmd_received = 1;
      backward();
      bwd = 1;
      fwd = lft = rht = stp = act = 0;
      l_cnt++;
    }
    break;

  case '3':
    servo_idel();
    if (!rht && !lft && (fwd || bwd))
      store_data();

    rht = 1;
    fwd = bwd = lft = stp = act = 0;

    reset_data();

    cmd_received = 1;
    right();
    r_cnt++;
    break;

  case '4':
    servo_idel();

    if (!rht && !lft && (fwd || bwd))
      store_data();
    lft = 1;
    rht = fwd = bwd = stp = act = 0;

    reset_data();

    cmd_received = 1;
    left();
    r_cnt++;
    break;

  case '5':
    servo1.attach(servoPin); // servo pin
    servo1.write(40);
    increase_set_rpm();
    break;

  case '6':
    servo_idel();
    decrease_set_rpm();
    break;

  case '7':
    servo_idel();
    if (!act)
    {
      act = 1;
      bwd = lft = rht = stp = fwd = 0;
      cmd_received = 1;
      servo_set();
      Acknowledge(servo_done);
      break;
    }
    Acknowledge(ack);
    break;

  case '8':
    cmd_received = 1;
    servo_check();
    break;

  case '9':
    if (fwd || bwd)
      store_data();

    if (!stp)
    {
      servo_idel();
      stp = 1;
      fwd = bwd = lft = rht = act = 0;
      cmd_received = 1;
      disable_motor();
    }
    break;

  default:
    servo_idel();
    cmd_received = 1;
    stop();
    break;
  }
}

/******************************************************************************************************/
