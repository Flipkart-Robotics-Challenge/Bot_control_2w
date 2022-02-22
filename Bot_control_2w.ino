#include <WiFi.h>
#include <string.h>
#include <analogWrite.h>
#include "include_utils.h"


int idx;
int temp = 0;

/*const int drive_distance = 100;   // cm
const int motor_power = 255;      // 0-255
const int motor_offset = 5;       // Diff. when driving straight
const int wheel_d = 43;           // Wheel diameter (mm)
const float wheel_c = PI * wheel_d; */ // Wheel circumference (mm)


/*****info***********/
int count_per_rev = 700;
int encoder = 0;

/*******wifi config************/
#ifndef STASSID
#define STASSID "R&D-LABS"//"realme 3 Pro"//"Black Shark 2"//"Honor Play"
#define STAPSK  "12345678"//"3dampr0!"
#endif
//#define n_wifi  5
char* ssid[n_wifi] = {"IA_Lab","IA Lab","SUITS","realme 3 Pro","R&D-LABS","Black Shark 2"};
char* password[n_wifi] = {"IAlab@2022","rosdeveloper","WFH@wifi.com","3dampr0!","","123123@edel.1"};


//char* ssid = STASSID;
//char* password = STAPSK;

/***Create an instance of the server****/
// specify the port to listen on as an argument
WiFiServer server(6491);

WiFiClient client;


/********timer interrupt*******/

const int counts_per_rev = 700;   // (4 pairs N-S) * (48:1 gearbox) * (2 falling/rising edges) = 384

volatile int interrupts;
int totalInterrupts;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


/**********com_data********/
char message[6] = {0};
int distance,direction,angle,action;




int milli =0;

void setup() {
  
  // serial moniter steup
  Serial.begin(115200);   

  // I/O Pin cinfig
  Set_Pins(); 
  servo_idel();
  // Connect to WiFi network
  Wifi_Init(ssid,password);    
  
  //Indication of successful wifi connection
  digitalWrite(LED_BUILTIN, HIGH); 

  // Start the server
  server.begin();
  Serial.println(F("Server started"));

  // Print the IP address
  Serial.println(WiFi.localIP());


  
  
  // Configure Prescaler to 80, as our timer runs @ 80Mhz
  // Giving an output of 80,000,000 / 80 = 1,000,000 ticks / second
  timer = timerBegin(0, 80, true);                
  timerAttachInterrupt(timer, &IRAM_ATTR_onTime, true);    
  // Fire Interrupt every 100k ticks, so 0.1s
  timerAlarmWrite(timer, 100000, true);      
  //timerAlarmEnable(timer);
  //--------------------------timer setup
  
}

void loop() 
{
   esp_task_wdt_reset();

  if (interrupts > 0) {
    portENTER_CRITICAL(&timerMux);
    interrupts--;
    portEXIT_CRITICAL(&timerMux);
    totalInterrupts++;
    
  }


  // Check if a client has connected
  if(client_check()){
    //Serial.println("inside client");
    //milli = millis();  
    strcpy(message,Recieve(&client));
    Serial.println(message);
    if(message[0] != '7') Acknowledge(ack);
    command(message);
    //if(message[0] == '7') Acknowledge(servo_done);
    motor_start = true;
  }

}


int client_check()
{
  client = server.available();
  return client?1:0;
}
void Acknowledge(int code)
{
  Transmit(&client,str(code));  
}
