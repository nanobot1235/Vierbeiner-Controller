#include "core2.h"
#include "BluetoothSerial.h"

//#define BTSerial Serial2
BluetoothSerial BTSerial;

TaskHandle_t task_loop1;

#define BUTTON_PIN_BITMASK 0b0000000000000110000000000000000000000000//
//                     pin: *39     *31     *23     *15    *7        //

const double toleranz = 0.5;

float oldbodycoords[2][3];
float bodycoords[2][3];
float walk[3];
float oldwalk[3];

uint8_t address[6]  = {0x00, 0x12, 0x06, 0x01, 0x51, 0x15};

 
void setupCore2();
static void esploop1(void* pvParameters);
void xTaskCreatePinnedToCore(void* pvParameters);

void bye(){
  digitalWrite(LEDPin, HIGH);
  Serial.println("bye");
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

int offsets[6];
byte potPins[] = {32, 33, 34, 0, 0, 35};
int potIn[sizeof(potPins)];
//double maxpos[] = {Coord_x, Coord_y, Coord_z, 0, 0, Coord_e};

byte buttonPins[] = {26, 25};
unsigned long lastButtenPress[sizeof(buttonPins)];
bool pinPressed[sizeof(buttonPins)];

void setup() {
  Serial.begin(115200);
  pinMode(25, INPUT);  
  pinMode(26, INPUT);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LOW);
  BTSerial.begin("vierbeiner Controller", true);
  if (!BTSerial.connect(address)){
    bye();
    delay(1000);
  }
  setupCore2();

  for (byte b = 0; b<sizeof(potPins); b++){
    offsets[b] = analogRead(potPins[b]);
  }

}

void loop() {

  if(!BTSerial.connected(1000)){
    u8g2.sleepOn();
    bye();
  }

  for (byte b = 0; b < sizeof(pinPressed); b++)  {
    pinPressed[b] = false;
  }  
//for debuncing 
  for(byte b = 0; b < sizeof(buttonPins); b++){
    if(digitalRead(buttonPins[b])){
      if((millis() - lastButtenPress[b]) > debounceTollerance){
        lastButtenPress[b] = millis();
        pinPressed[b] = true;
      }
    }
  }

  if(pinPressed[0]){
    BTSerial.print(0.0);
    BTSerial.print("o");
  }
  
  if(pinPressed[1]){
    mode++;
    mode %= 5;
  }

  for (byte b = 0; b < sizeof(potPins); b++){
    int i = analogRead(potPins[b]);

    if (i<2048){
      potIn[b] = mapf(i, 0.0, offsets[b],  0, 2047);
    }
    else{
      potIn[b] = mapf(i, offsets[b], 4095.0,  2048, 4095);
    }
  }

  switch (mode)  {
    case 0:
      bodycoords[0][0] = mapf(potIn[0], 0.0, 4095,  Coord_x, -Coord_x);
      bodycoords[0][1] = mapf(potIn[1], 0.0, 4095, -Coord_y,  Coord_y);
      bodycoords[0][2] = mapf(potIn[2], 0.0, 4095,  Coord_z, -Coord_z);
    //bodycoords[1][0] = mapf(potIn[3], 0.0, 4095,  Coord_ ,  Coord_ );
    //bodycoords[1][1] = mapf(potIn[4], 0.0, 4095,  Coord_ ,  Coord_ );
      bodycoords[1][2] = mapf(potIn[5], 0.0, 4095,  Coord_e, -Coord_e);
    /*
      Serial.print(bodycoords[0][0]);
      Serial.print("x ");
      Serial.print(bodycoords[0][1]);
      Serial.print("y ");
      Serial.print(bodycoords[0][2]);
      Serial.print("z ");
      Serial.print(bodycoords[1][2]);
      Serial.println("e ");
    */
      
      if (abs(bodycoords[0][0] - oldbodycoords[0][0])>toleranz){
        oldbodycoords[0][0] = bodycoords[0][0];
        BTSerial.print(bodycoords[0][0]);
        BTSerial.print("x ");
      }
      if (abs(bodycoords[0][1] - oldbodycoords[0][1])>toleranz){
        oldbodycoords[0][1] = bodycoords[0][1];
        BTSerial.print(bodycoords[0][1]);
        BTSerial.print("y ");
      }
      if (abs(bodycoords[0][2] - oldbodycoords[0][2])>toleranz){
        oldbodycoords[0][2] = bodycoords[0][2];
        BTSerial.print(bodycoords[0][2]);
        BTSerial.print("z ");
      }
      if (abs(bodycoords[1][2] - oldbodycoords[1][2])>toleranz){
        oldbodycoords[1][2] = bodycoords[1][2];
        BTSerial.print(bodycoords[1][2]);
        BTSerial.print("e ");
      }
    break;
       
    case 1:
  	  bodycoords[0][0] = mapf(potIn[0], 0.0, 4095,  Coord_x, -Coord_x);
    //bodycoords[0][1] = mapf(potIn[1], 0.0, 4095, -Coord_y,  Coord_y);
    //bodycoords[0][2] = mapf(potIn[2], 0.0, 4095,  Coord_z, -Coord_z);
      bodycoords[1][0] = mapf(potIn[1], 0.0, 4095, -Coord_q,  Coord_q);
      bodycoords[1][1] = mapf(potIn[2], 0.0, 4095,  Coord_w, -Coord_w);
      bodycoords[1][2] = mapf(potIn[5], 0.0, 4095,  Coord_e, -Coord_e);
      
      if (abs(bodycoords[0][0] - oldbodycoords[0][0])>toleranz){
        oldbodycoords[0][0] = bodycoords[0][0];
        BTSerial.print(bodycoords[0][0]);
        BTSerial.print("x ");
      }
      if (abs(bodycoords[1][0] - oldbodycoords[1][0])>toleranz){
        oldbodycoords[1][0] = bodycoords[1][0];
        BTSerial.print(bodycoords[1][0]);
        BTSerial.print("q ");
      }
      if (abs(bodycoords[1][1] - oldbodycoords[1][1])>toleranz){
        oldbodycoords[1][1] = bodycoords[1][1];
        BTSerial.print(bodycoords[1][1]);
        BTSerial.print("w ");
      }
      if (abs(bodycoords[1][2] - oldbodycoords[1][2])>toleranz){
        oldbodycoords[1][2] = bodycoords[1][2];
        BTSerial.print(bodycoords[1][2]);
        BTSerial.print("e ");
      }
    break;
    
    case 2:
  	  walk[0] = mapf(potIn[0], 0.0, 4095,  Coord_f, -Coord_f);
    //bodycoords[0][1] = mapf(potIn[1], 0.0, 4095, -Coord_y,  Coord_y);
    //bodycoords[0][2] = mapf(potIn[2], 0.0, 4095,  Coord_z, -Coord_z);
    //bodycoords[1][0] = mapf(potIn[1], 0.0, 4095, -Coord_q,  Coord_q);
    //bodycoords[1][1] = mapf(potIn[2], 0.0, 4095,  Coord_w, -Coord_w);
    //bodycoords[1][2] = mapf(potIn[5], 0.0, 4095,  Coord_e, -Coord_e);
      
      if (abs(walk[0] - oldwalk[0])>toleranz){
        oldwalk[0] = walk[0];
        BTSerial.print(walk[0]);
        BTSerial.print("f ");
      }
      
    break;
    
    default:

    break;
  }
}

void setupCore2() {
	bigLock = NULL;
	xTaskCreatePinnedToCore(
		esploop1,               /* Task function. */
		"loop1",                /* name of task. */
		10000,                  /* Stack size of task */
		NULL,                   /* parameter of the task */
		1,                      /* priority of the task */
		&task_loop1,            /* Task handle to keep track of created task */
		!xPortGetCoreID());     /* pin task to core 0 */
	bigLock = xSemaphoreCreateMutex();
}


static void esploop1(void* pvParameters) {
	setup2();

	while (1) {
		loop2();
	}
}

