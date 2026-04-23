#include <ultrasonic.h> 

#define PIEZO_TRANSMITTER 9
#define ANALYZER_MILK_LEVEL 3
#define PIEZO_RECEIVER A0
#define VALVE1 5
#define VALVE2 6

long beep_start;

static bool analyzer_milk_level = false;
static bool analysis_completed = false;
static bool sampling_active = false;

float amplitude_sum = 0;
long sample_count = 0;

float lr;
float Aa;
float fat;
float snf;

volatile bool flow = false;

void setup() {
  
  delay(1000);
  Serial.begin(9600);
  
  pinMode(PIEZO_TRANSMITTER, OUTPUT);
  setup_ultrasonic();

  pinMode(ANALYZER_MILK_LEVEL, INPUT);
  
  pinMode(VALVE1, OUTPUT);
  pinMode(VALVE2, OUTPUT);

  digitalWrite(VALVE1, HIGH);
  digitalWrite(VALVE2, LOW);

}

void loop() {

  if (digitalRead(ANALYZER_MILK_LEVEL) == HIGH && !analyzer_milk_level) {

    delay(1500);
    digitalWrite(VALVE2,HIGH);

    analyzer_milk_level = true;
    sampling_active = true;
    analysis_completed = false;

    beep_start = millis();

    amplitude_sum = 0;
    sample_count = 0;

    tone(PIEZO_TRANSMITTER, 3000); 
  }

  else if (digitalRead(ANALYZER_MILK_LEVEL) == LOW) {

    analyzer_milk_level = false;
  }

  if (sampling_active) {

    if (millis() - beep_start <= 10000) {

      amplitude_sum += analogRead(PIEZO_RECEIVER);
      sample_count++;
    }

    else {

      noTone(PIEZO_TRANSMITTER);

      sampling_active = false;

      if (sample_count > 0) {

        float avg_amplitude = amplitude_sum / sample_count;

        float distance = read_ultrasonic_sensor();

        lr = -10.526 * distance + 80;

        Aa = 1023 - avg_amplitude;

        fat = (0.1016 * Aa * Aa) + (0.2288 * Aa) + 0.4918;

        snf = lr / 4 + 0.2 * fat + 0.36;

        analysis_completed = true;

      }
    }
  }


  if (analysis_completed) {
        Serial.print(fat, 2);
        Serial.print(",");
        Serial.println(snf, 2);

    digitalWrite(VALVE1, LOW);
      delay(15000);

    digitalWrite(VALVE1, HIGH);


    analysis_completed = false;
  }
}
