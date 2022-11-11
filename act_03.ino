#include <esp_wpa2.h>
#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials


// Insert Firebase project API Key
#define API_KEY "AIzaSyCJp3wOiDbR_zhVGDhq88HtoQ6bXp8NqQg"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://semana-i-esp32-default-rtdb.firebaseio.com/" 

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

const int trigPin = 5;
const int LM35_pin = 14;
const int PIN_TO_SENSOR = 16;
const int ledPin = 21;
const int echoPin = 18;
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

unsigned char contador = 0;
unsigned long sendDataPrevMillis = 0;

bool signupOK = false;

String displayVal = "";
String ledVal = "";
int presence = 0;

int ADC_VALUE = 0;
int pinStateCurrent = LOW;
int pinStatePrevious = LOW;
int LEDs[] = {27,26,25,33,32,2,13};       // for ESP32 microcontroller
int zero[] = {0, 1, 1, 1, 1, 1, 1};       // cero
int one[] = {0, 0, 0, 0, 1, 1, 0};        // uno
int two[] = {1, 0, 1, 1, 0, 1, 1};        // dos
int three[] = {1, 0, 0, 1, 1, 1, 1};      // tres
int four[] = {1, 1, 0, 0, 1, 1, 0};       // cuatro 
int five[] = {1, 1, 0, 1, 1, 0, 1};       // cinco
int six[] = {1, 1, 1, 1, 1, 0, 1};        // seis
int seven[] = {1, 0, 0, 0, 1, 1, 1};      // siete
int eight[] = {1, 1, 1, 1, 1, 1, 1};      // ocho
int nine[] = {1, 1, 0, 1, 1, 1, 1};       // nueve
int ten[] = {1, 1, 1, 0, 1, 1, 1};        // diez, A
int eleven[] = {1, 1, 1, 1, 1, 0, 0};     // once, b
int twelve[] = {0, 1, 1, 1, 0, 0, 1};     // doce, C
int thirthteen[] = {1, 0, 1, 1, 1, 1, 0}; // trece, d
int fourthteen[] = {1, 1, 1, 1, 0, 0, 1}; // catorce, E
int fifthteen[] = {1, 1, 1, 0, 0, 0, 1};  // quince, F

float c_value = 0;
float distanceCm;
float distanceInch;

long duration;

void setup() {
     // WPA2 enterprise magic starts here
    WiFi.disconnect(true);      
    WiFi.mode(WIFI_STA);   //init wifi mode
    Serial.printf("Connecting to WiFi: %s ", ssid);
    //esp_wifi_sta_wpa2_ent_set_ca_cert((uint8_t *)incommon_ca, strlen(incommon_ca) + 1);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
    //esp_wifi_sta_wpa2_ent_enable();
    esp_wpa2_config_t configW = WPA2_CONFIG_INIT_DEFAULT();
    esp_wifi_sta_wpa2_ent_enable(&configW);
    // WPA2 enterprise magic ends here
    WiFi.begin(ssid);
  
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    /* Assign the api key (required) */
    config.api_key = API_KEY;

    /* Assign the RTDB URL (required) */
    config.database_url = DATABASE_URL;

    /* Sign up */
    if (Firebase.signUp(&config, &auth, "", "")) {
        Serial.println("ok");
        signupOK = true;
    } else {
        Serial.printf("%s\n", config.signer.signupError.message.c_str());
    }

    /* Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    /* ----------------------------------------------------------------------- */
    Serial.begin(115200);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(PIN_TO_SENSOR, INPUT);
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(ledPin, ledChannel);
    for (int i = 0; i<7; i++) pinMode(LEDs[i], OUTPUT);
}

void loop() {
    fun_LM35();
    fun_Ojitos();
    fun_PIR();
    /* ----------------------------------------------------------------------- */
    if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 2000 || sendDataPrevMillis == 0)) {
        sendDataPrevMillis = millis();
    
        // Write LM35 value
        if (Firebase.RTDB.setFloat(&fbdo, "test/temperatura", c_value)) {
            Serial.println("PASSED");
            Serial.println("PATH: " + fbdo.dataPath());
            Serial.println("TYPE: " + fbdo.dataType());
        } else {
            Serial.println("FAILED");
            Serial.println("REASON: " + fbdo.errorReason());
        }

        // Write Ultrasonic value
        if (Firebase.RTDB.setFloat(&fbdo, "test/distancia", distanceCm)) {
            Serial.println("PASSED");
            Serial.println("PATH: " + fbdo.dataPath());
            Serial.println("TYPE: " + fbdo.dataType());
        } else {
            Serial.println("FAILED");
            Serial.println("REASON: " + fbdo.errorReason());
        }

        // Write PIR value
        if (Firebase.RTDB.setInt(&fbdo, "test/presencia", presence)) {
            Serial.println("PASSED");
            Serial.println("PATH: " + fbdo.dataPath());
            Serial.println("TYPE: " + fbdo.dataType());
        } else {
            Serial.println("FAILED");
            Serial.println("REASON: " + fbdo.errorReason());
        }
        
        //Read display number
        if (Firebase.RTDB.getString(&fbdo, "/test/display")) {
            if (fbdo.dataType() == "string") {
                displayVal = fbdo.stringData();
                segment_display(displayVal.toInt());
                Serial.println("Display: " + displayVal);
            }
        } else Serial.println(fbdo.errorReason());
        
        //Read led pwm value
        if (Firebase.RTDB.getString(&fbdo, "/test/led")) {
            if (fbdo.dataType() == "string") {
                ledVal = fbdo.stringData();
                ledcWrite(ledChannel, ledVal.toInt());
                Serial.println("LED: " + ledVal);
            }
        } else Serial.println(fbdo.errorReason());
    }
}

void fun_LM35() {
    ADC_VALUE = analogRead(LM35_pin);
    delay(1000);
    c_value = (((float)ADC_VALUE * 3300)/4095.0)/10.0;
    Serial.print("Temperatura: ");
    Serial.print(c_value);
    Serial.println("°C");
    delay(1000);
}

void fun_Ojitos() {
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
  
    // Calculate the distance
    distanceCm = duration * SOUND_SPEED/2;
  
    // Convert to inches
    distanceInch = distanceCm * CM_TO_INCH;
  
    Serial.print("Distancia (cm): ");
    Serial.println(distanceCm);
    Serial.print("Distancia (inch): ");
    Serial.println(distanceInch);
    delay(500);
}

void fun_PIR() {
  pinStatePrevious = pinStateCurrent;                           // store old state
  pinStateCurrent = digitalRead(PIN_TO_SENSOR);                 // read new state

  if (pinStatePrevious == LOW && pinStateCurrent == HIGH) {      // pin state change: LOW -> HIGH
    presence = 1;
    Serial.println("Motion detected!");
  } else if (pinStatePrevious == HIGH && pinStateCurrent == LOW) { // pin state change: HIGH -> LOW
    presence = 0;
    Serial.println("Motion stopped!");
  }
  delay(2000);
}

void fun_Display() {
    segment_display(contador);
    delay(1000);
    if(contador < 15) contador++;
    else contador = 0;
}


void fun_LED() {
  // increase the LED brightness
  for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    // changing the LED brightness with PWM
    ledcWrite(ledChannel, dutyCycle);
    delay(15);
  }

  // decrease the LED brightness
  for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
    // changing the LED brightness with PWM
    ledcWrite(ledChannel, dutyCycle);
    delay(15);
  }
}

void segment_display(unsigned char valor) {
    switch(valor) {
        case 0:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], zero[i]);
                    break;
        case 1:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], one[i]);
                    break;
        case 2:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], two[i]);
                    break;
        case 3:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], three[i]);
                    break;
        case 4:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], four[i]);
                    break;
        case 5:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], five[i]);
                    break;
        case 6:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], six[i]);
                    break;
        case 7:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], seven[i]);
                    break;
        case 8:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], eight[i]);
                    break;
        case 9:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], nine[i]);
                    break;
        case 10:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], ten[i]);
                    break;
        case 11:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], eleven[i]);
                    break;
        case 12:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], twelve[i]);
                    break;
        case 13:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], thirthteen[i]);
                    break;
        case 14:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], fourthteen[i]);
                    break;
        case 15:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], fifthteen[i]);
                    break; 
        default:
                    for (int i = 0; i<7; i++) digitalWrite(LEDs[i], zero[i]);
                    break;          
    }
}
