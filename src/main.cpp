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
byte pins[] = {32, 33, 34, 0, 0, 35};
double maxpos[] = {Coord_x, Coord_y, Coord_z, 0, 0, Coord_e};

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

  for (byte b = 0; b<6; b++){
    offsets[b] = analogRead(pins[b]);
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

  for(byte b = 0; b < sizeof(buttonPins); b++){
    if(digitalRead(buttonPins[b])){
      if((millis() - lastButtenPress[b]) > debounceTollerance){
        lastButtenPress[b] = millis();
        pinPressed[b] = true;
      }
    }
  }

  if(pinPressed[1]){
    Serial.println("Hai");
  }

  if(pinPressed[0]){
    BTSerial.print(0.0);
    BTSerial.print("o");
  }

  for (byte b = 0; b<6; b++){
    int i = analogRead(pins[b]);

    if (i<2048){
      bodycoords[b/3][b%3] = mapf(i, 0.0, offsets[b],  maxpos[b], 0);
    }
    else{
      bodycoords[b/3][b%3] = mapf(i, offsets[b], 4095.0,  0, -maxpos[b]);
    }
  }

/*
  bodycoords[0][0] = mapf(analogRead(pins[0]) - offsets[0], 0.0, 4095,  Coord_x, -Coord_x);
  bodycoords[0][1] = mapf(analogRead(pins[1]) - offsets[1], 0.0, 4095, -Coord_y,  Coord_y);
  bodycoords[0][2] = mapf(analogRead(pins[2]) - offsets[2], 0.0, 4095,  Coord_z, -Coord_z);
//bodycoords[1][0] = mapf(analogRead(pins[3]) - offsets[3], 0.0, 4095,  Coord_ ,  Coord_ );
//bodycoords[1][1] = mapf(analogRead(pins[4]) - offsets[4], 0.0, 4095,  Coord_ ,  Coord_ );
  bodycoords[1][2] = mapf(analogRead(pins[5]) - offsets[5], 0.0, 4095,  Coord_e, -Coord_e);
  */




  Serial.print(bodycoords[0][0]);
  Serial.print("x ");
  Serial.print(bodycoords[0][1]);
  Serial.print("y ");
  Serial.print(bodycoords[0][2]);
  Serial.print("z ");
  Serial.print(bodycoords[1][2]);
  Serial.println("e ");
  
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
  
  delay(200);
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

