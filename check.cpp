#include <Arduino.h>
#include <pthread.h>


const int adcPin = 2;

float voltage = 0;
float slope = -0.583;
int intercept = 420;
float temp;
int tempV;
float finalV;
int testTimes = 50; //testcases per average
int delayTime = 10; //ms between each test case
int totalDelay = 300; //time between each average

pthread_t tid;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);


  //create thread
  pthread_create(&tid, NULL, worker, NULL);

  delay(2000);

  //end the thread execution
  pthread_join(tid, NULL);
}

void *worker(void *args){

for(int i = 0; i < 30; i++){
        Serial.print("This is a test for everything\n");
        Serial.print("Please work\n");
    }

    return NULL;
}

void loop() {
  //voltage = analogRead(adcPin); //Read voltage between resistor and diode
  //voltage = constrain(voltage, 0, 4095);  //max range for voltage
  //voltage = map(voltage, 0, 4095, 0, 3300); //map range to 0 to 3.3 V

  //Voltage read at room temp of (21C): 710
  //room temp pt2 (27): 740
  //Voltage Read in ice water (2.1C): 783
  //Voltage read with lighter of (39C): 650
  //Lighter pt2 (62C): 520
  //temp = (voltage)/-2;  //
  int totalV = 0;

  for (int i = 0; i < testTimes; i++){
    tempV = analogRead(adcPin); //Read voltage between resistor and diode
    tempV = constrain(tempV, 0, 4095);  //max range for voltage
    //Serial.println(tempV);

    totalV += tempV;
    delay(delayTime);
  }
  finalV = (float)totalV/testTimes;
  Serial.println(finalV);
  finalV = finalV/4095.0*3300.0;
  temp = finalV * (slope) + intercept;
  Serial.print("Voltage: ");
  Serial.println(finalV);
  Serial.print("Temperature: ");
  Serial.println(temp);
  delay(totalDelay);
}