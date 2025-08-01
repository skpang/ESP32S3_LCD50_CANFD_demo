/*

https://www.skpang.co.uk/collections/esp32-boards/products/esp32s3-5-lcd-with-can-fd-and-can-bus

Ensure these libraries are installed:
https://github.com/pierremolinaro/acan2517FD
https://github.com/handmade0octopus/ESP32-TWAI-CAN
https://github.com/lvgl/lvgl

*/



#include <ESP32-TWAI-CAN.hpp>
#include "LVGL_Driver.h"
#include "Display_ST7701.h"  
#include "ui.h"
#include "ui_helpers.h"
#include "canfd.h"


int led  = 3;
int bl = 1;
uint8_t brightness;
uint8_t can_start = 0;
uint32_t frame_count = 0;
String can2_data;
#define CAN_TX   15  // Connects to CTX
#define CAN_RX   16  // Connects to CRX
CanFrame rxFrame; 
void canReceiver() {
   char buff[5];

  // try to parse packet
  if(ESP32Can.readFrame(rxFrame, 0)) { // 1000 is the timeout value
    // Communicate that a packet was recieved
    sprintf(buff,"%02X",rxFrame.identifier);
    lv_label_set_text(uic_ccanID, buff);
    sprintf(buff,"%d",rxFrame.data_length_code);
    lv_label_set_text(uic_ccanLen, buff);

    Serial.printf("Classic CAN received ID: %03X Len:%d  Data: ", rxFrame.identifier,rxFrame.data_length_code);
    can2_data = " ";
    // Communicate packet information
    for(int i = 0; i <= rxFrame.data_length_code - 1; i ++) {
      Serial.printf("%02x ",rxFrame.data[i]); // Transmit value from the frame 
      sprintf(buff,"%02X",rxFrame.data[i]);
      can2_data += String(" ") + buff; 
      }
    Serial.println(" ");
     lv_label_set_text(uic_ccCANdata, can2_data.c_str());
  } 
}


void canSender() {
  static uint8_t i=0;
  digitalWrite(led, HIGH);

  CanFrame testFrame = { 0 };
  testFrame.identifier = 0x7df;  // Sets the ID
  testFrame.extd = 0; // Set extended frame to false
  testFrame.data_length_code = 8; // Set length of data - change depending on data sent
  testFrame.data[0] = i++; // Write data to buffer. data is not sent until writeFrame() is called.
  testFrame.data[1] = 0x12;
  testFrame.data[2] = 0x34;
  testFrame.data[3] = 0x56;
  testFrame.data[4] = 0x78;
  testFrame.data[5] = 0x9a;
  testFrame.data[6] = 0xbc;
  testFrame.data[7] = 0xde;

  ESP32Can.writeFrame(testFrame); // transmit frame

  frame_count++;

  digitalWrite(led, LOW);
   
}
void Driver_Loop(void *parameter)
{
  while(1)
  {
      if(can_start == 1)
      {
        canSender();  // call function to send data through CAN
        canfd_sendframe();
  
      }

    vTaskDelay(pdMS_TO_TICKS(100));
    
  }
}
void CAN_init(void)
{
  ESP32Can.setPins(CAN_TX, CAN_RX);
  // Start the CAN bus at 500 kbps
  if(ESP32Can.begin(ESP32Can.convertSpeed(500))) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }
}

void Driver_Init()
{
//  Flash_test();
  //BAT_Init();
  I2C_Init();
  //TCA9554PWR_Init(0x00);   
 // Set_EXIO(EXIO_PIN8,Low);
  //PCF85063_Init();
 // QMI8658_Init(); 
  
  xTaskCreatePinnedToCore(
    Driver_Loop,     
    "Other Driver task",   
    4096,                
    NULL,                 
    3,                    
    NULL,                
    0                    
  );
}

void setup()
{
  pinMode(led, OUTPUT);
  pinMode(bl, OUTPUT);
  digitalWrite(bl, HIGH); 
  Serial.begin(115200);
  digitalWrite(led, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(100);                      // wait for a second
  digitalWrite(led, LOW);   // turn the LED off by making the voltage LOW
  delay(100);
  digitalWrite(led, HIGH);  // turn the LED on (HIGH is the voltage level)

  delay(200);
  Serial.println("###############################################");
  Serial.println("ESP32 S3 LCD 5.0in Starting....");
 
  brightness = 10;

  Driver_Init();
  LCD_Init();    

  Serial.println("ESP32S3 5.0in LCD with LVGL skpang.co.uk 08/2025");
  Serial.println((String)"Memory available in PSRAM (after LCD init): " +ESP.getFreePsram());
  String LVGL_Arduino = "LVGL ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.println( LVGL_Arduino );

  Lvgl_Init();
  ui_init();
  digitalWrite(led, LOW); 
  lv_label_set_text(ui_version,LVGL_Arduino.c_str());
  
  String bright = "Brightness : ";
  bright += brightness;
  lv_label_set_text(ui_brightness,bright.c_str());

  CAN_init();
  canfd_init();
  can_start = 1;
}

void loop()
{
  Lvgl_Loop();
  canReceiver();
  canfd_receiveframe();
  vTaskDelay(pdMS_TO_TICKS(5));
}