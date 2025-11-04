#include <Arduino.h>
#include <esp_task_wdt.h>

// Watchdog timer timeout in seconds
#define WDT_TIMEOUT 10
#define DISTANCE_THRESHOLD 10.0
#define DISTANCE_TIMEOUT 5000 // 5 seconds in milliseconds

const int trigPin = 5;
const int echoPin = 12;

// define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

// Watchdog variables
unsigned long lastGoodDistanceTime = 0;
bool distanceWatchdogActive = false;
SemaphoreHandle_t distanceMutex;

void setup()
{
  Serial.begin(115200);     // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

  // Initialize the watchdog timer
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

// Watchdog task running in parallel
void distanceWatchdogTask(void *parameter)
{
  while (true)
  {
    float currentDistance;
    unsigned long currentTime = millis();
    
    // Safely read the distance value
    xSemaphoreTake(distanceMutex, portMAX_DELAY);
    currentDistance = distanceCm;
    xSemaphoreGive(distanceMutex);
    
    // Check if distance is greater than threshold
    if (currentDistance > DISTANCE_THRESHOLD)
    {
      if (!distanceWatchdogActive)
      {
        // First time exceeding threshold, start timer
        distanceWatchdogActive = true;
        lastGoodDistanceTime = currentTime;
        Serial.println("⚠️ Distance > 10cm - Watchdog timer started");
      }
      else
      {
        // Check if timeout exceeded
        unsigned long elapsedTime = currentTime - lastGoodDistanceTime;
        if (elapsedTime >= DISTANCE_TIMEOUT)
        {
          Serial.println("❌ Distance > 10cm for 5 seconds! Resetting ESP32...");
          delay(100); // Give time for serial to print
          ESP.restart(); // Reset the microcontroller
        }
        else
        {
          // Print countdown
          Serial.print("⏱️ Distance watchdog: ");
          Serial.print((DISTANCE_TIMEOUT - elapsedTime) / 1000.0, 1);
          Serial.println(" seconds remaining");
        }
      }
    }
    else
    {
      // Distance is within safe range, reset watchdog
      if (distanceWatchdogActive)
      {
        Serial.println("✅ Distance back to safe range - Watchdog reset");
        distanceWatchdogActive = false;
      }
    }
    
    delay(100); // Check every 100ms
  }
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

  delay(1000);
}