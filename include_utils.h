#include <WiFi.h>
#include <esp_task_wdt.h>

/*****main****/
extern int count_per_rev;
extern int encoder;

/****Interrupts***/
extern volatile int interrupts;
extern int totalInterrupts;

extern hw_timer_t* timer;
extern portMUX_TYPE timerMux;


/****communication****/

#define n_wifi 6
#define servo_done 10
#define ack        11
#define str(code) #code

void Wifi_Init(char** ssid, char** pass);
char* Recieve(WiFiClient* client);
void Transmit(WiFiClient* client,char* data);
void Acknowledge(int code);
extern int cmd_received;
extern int retry;





/****actuator****/

extern const int PWM_ch_left ;
extern const int PWM_ch_right ;
extern const int freq ;
extern const int resolution ;

void servo_idel(void);
void detect_a_left(void);
void detect_a_right(void);
void IRAM_ATTR_onTime(void);
void command(char* cmd);
void stop(void);
void left(void);
void right(void);
void backward(void);
void forward(void);
void disable_motor(void);
extern boolean motor_start;

/****config****/
void Set_Pins(void);
void Attach_Interrupts(void);
void Deattach_Interrupts(void);

//#define LED_BUILTIN 2

/**********left & right*************/
extern int encoder_left;
extern int m_direction_left ;
extern int sv_speed_left ;     //this value is 0~255
extern double pv_speed_left ;
extern double set_speed_left;
extern double e_speed_left; //error of speed = set_speed - pv_speed
extern double e_speed_pre_left;  //last error of speed
extern double e_speed_sum_left;  //sum error of speed
extern double pwm_pulse_left;     //this value is 0~255
extern double kp_left;
extern double ki_left;
extern double kd_left;
extern int timer1_counter_left; //for timer
extern int i_left;




extern int encoder_right;
extern int m_direction_right;
extern int sv_speed_right ;     //this value is 0~255
extern double pv_speed_right ;
extern double set_speed_right;
extern double e_speed_right ; //error of speed = set_speed - pv_speed
extern double e_speed_pre_right;  //last error of speed
extern double e_speed_sum_right;  //sum error of speed
extern double pwm_pulse_right ;     //this value is 0~255
extern double kp_right ;
extern double ki_right ;
extern double kd_right ;
extern int timer1_counter_right; //for timer
extern int i_right;


/*****servo********/
extern const int servoPin;
//extern int idel_angle;
extern int angleStep;

extern int angleMin;
extern int angleMax;

void servo_check();
void servo_set();

/****left motor*****/
extern const byte pin_left_a;    //for encoder pulse A
extern const byte pin_left_b ;   //for encoder pulse B
extern const byte pin_left_fwd ; //for H-bridge: run motor forward
extern const byte pin_left_bwd ; //for H-bridge: run motor backward
extern const byte pin_left_pwm ; //for H-bridge: motor speed


/****Right motor*****/
extern const byte pin_right_a ;   //for encoder pulse A
extern const byte pin_right_b ;   //for encoder pulse B
extern const byte pin_right_fwd ; //for H-bridge: run motor forward
extern const byte pin_right_bwd ; //for H-bridge: run motor backward
extern const byte pin_right_pwm ; //for H-bridge: motor speed


/****mics****/
#define LF       0x0A  //ASCII value of \n
extern int idx;
