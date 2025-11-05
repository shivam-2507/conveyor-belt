#include <Arduino.h>
#include <pthread.h>

// Watchdog timer timeout in seconds
#define WDT_TIMEOUT 5
#define DISTANCE_THRESHOLD 10.0

const int trigPin = 5;
const int echoPin = 12;

// define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;
bool watchdog_kicked = false;
pthread_mutex_t mutex;
pthread_t tid;
unsigned long timer1 = 0;
unsigned long timer2 = 0;


void setup()
{
  Serial.begin(115200);     // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

  // MAYBE GET RID OF THIS
  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); // Enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);               // Add current thread to WDT watch
  Serial.println("WDT configured successfully!");
}

void create_threads(){
  pthread_create( &tid, NULL, timer2, NULL);
}

// timer2
void *timer2(void * args){

  while(1){
    if(watchdog_kicked = true)
  }
  unsigned long current_time = millis();



}

float readDistance(){

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distanceCm = duration * SOUND_SPEED / 2;

  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);

  return distanceCm;
}


// watchdog helper function
void watchdog_kicking(void){
  watchdog_kicked = true;
  if(distanceCm > 10 && time > 3){

  }
  return;
}

//timer 1
int timer1(float distanceCm)
{
  timer1 = millis();

  if(distance > 5){
    pthread_mutex_lock(&mutex);
    watchdog_kicked = true;
    pthread_mutex_unlock(&mutex);
  } 
  
  unsigned long current_time = millis();
  timer1 = current_time - timer1;
  return timer1;


}


//ending the project showcase
void end_loop(){
  pthread_join( &tid, NULL);

  pthread_mutex_destroy(&mutex);
  return;
}



void loop()
{

}