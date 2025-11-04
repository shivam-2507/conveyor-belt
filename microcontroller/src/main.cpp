#include <Arduino.h>
#include <esp_task_wdt.h>

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

bool watchDog(float distanceCm)
{

  if (distanceCm > 10)
  {
    return true;
  }

  return false;
}

int timer(float distanceCm)
{
  int timeInMilliseconds = 0;

  if (distanceCm < 5)
  {
    timeInMilliseconds++;
    delay(1);
  }

  if (!watchDog(distanceCm))
  {
    Serial.println("Something went wrong!");
    return -1;
  }

  return timeInMilliseconds;
}

void loop()
{
  esp_task_wdt_reset();

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distanceCm = duration * SOUND_SPEED / 2;

  distanceInch = distanceCm * CM_TO_INCH;

  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
  // Serial.print("Distance (inch): ");
  // Serial.println(distanceInch);

  delay(1000);
}