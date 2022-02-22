#include <WiFi.h>
#include<string.h>
#include "include_utils.h"

int led_stat = 0;
int retry;
int cmd_received;
int wifi_no = 0;
void Wifi_Init(char** ssid,char** pass)
{
    wifi_no = -1;
    CONNECT:
    wifi_no++;
    if(wifi_no>n_wifi-1)
    {
      Serial.println("Try different WiFi!! \nOr\n Reset the MCU");
      while(1);
      return;
    }
    Serial.println();
    Serial.println();
    Serial.print(F("Connecting to "));
    Serial.print(*(ssid+wifi_no));
    Serial.print(" -> ");
    Serial.println(*(pass+wifi_no));

    //WiFi.mode(WIFI_STA);
    
    retry = 0;
    
    WiFi.begin(*(ssid+wifi_no), *(pass+wifi_no));
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        digitalWrite(LED_BUILTIN, led_stat= !led_stat);
        Serial.print(F("."));
        if(retry++>10) goto CONNECT;
    }
    Serial.println();
    Serial.println(F("WiFi connected"));
    digitalWrite(LED_BUILTIN, 1);

}


void server_init(WiFiServer* server)
{
    server->begin();
}
char data[10]={0};

char* Recieve(WiFiClient* client)
{
    /*WiFiClient client = server->available();
    if (!client) {
        return NULL;
    }*/
    Serial.println("new client");
    String req = client->readStringUntil(0);
    //Serial.println(req);
    int str_len = req.length() + 1;
    char req_array[str_len];
    req.toCharArray(req_array, str_len);
    strcpy(data,req_array);
    //Serial.println(strlen(data));
    return data;
}

void Transmit(WiFiClient* client,char* data)
{
    /*WiFiClient client = server->available();
    if (!client) {
        return;
    }*/
    client->print(data); //rturn string length
    //Serial.println(F("Disconnecting from client"));
}
WiFiClient* client_copy = NULL;
WiFiClient* client_check(WiFiServer* server)
{
    WiFiClient client = server->available();
    client_copy = &client;
    return client_copy;
}
