#include <Arduino.h>
#include <pthread.h>

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

unsigned long timer1_start = 0;
unsigned long timer2_start = 0;

void *timer2(void *args)
{
  while (true)
  {
    pthread_mutex_lock(&mutex);
    bool kicked = watchdog_kicked;
    pthread_mutex_unlock(&mutex);

    if (kicked == true)
    {
      unsigned long start_time = millis();
      while(1){

        if((millis() - start_time) > 3000)
        {
          Serial.print("Object is not there anymore\n");

          unsigned long elapsed = start_time - timer1_start;
          
          Serial.print("\n\nObject detection time: ");
          Serial.println(elapsed);
          Serial.print("\n\n");

        }
      }
    }
  }

  return NULL;
}

void setup()
{
  Serial.begin(115200);     // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

  pthread_mutex_init(&mutex, NULL);

  pthread_create(&tid, NULL, timer2, NULL);
}

// timer2 for the watchdog thread

float readDistance()
{
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

void loop()
{
  distanceCm = readDistance();

  // we start timer1 when object detected
  if(distanceCm < 3 && timer1_start == 0 ) 
  {
    timer1_start = millis();
    Serial.println("Timer1 started");
  }

  // we keep looping watchdog to be NOT kicked
  if(distanceCm < 3)
  {
    watchdog_kicked = false;
  }

  // Kick watchdog if distance > 5 during detection 
  if(distanceCm > 5 && timer1_start != 0)
  { 
    pthread_mutex_lock(&mutex);
    watchdog_kicked = true; 
    pthread_mutex_unlock(&mutex);
  }
}