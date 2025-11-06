#include <Arduino.h>

const int trigPin = 5;
const int echoPin = 12;

// define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;
bool watchdog_kicked = false;

SemaphoreHandle_t mutex; // FreeRTOS mutex (better than pthread)

unsigned long timer1_start = 0;
unsigned long timer2_start = 0;

// FreeRTOS task (replaces pthread function)
void timer2Task(void *parameter)
{
  while (true)
  {
    xSemaphoreTake(mutex, portMAX_DELAY); // Lock
    bool kicked = watchdog_kicked;
    unsigned long t2_start = timer2_start;
    unsigned long t1_start = timer1_start;
    xSemaphoreGive(mutex); // Unlock

    if (kicked == true)
    {
      unsigned long current_time = millis();
      if (current_time - t2_start > 3000) // 3 seconds
      {
        unsigned long elapsed = t1_start - t2_start;
        Serial.print("\n\nObject detection time: ");
        Serial.println(elapsed);
        Serial.print("\n\n");

        // Reset after reporting
        xSemaphoreTake(mutex, portMAX_DELAY);
        watchdog_kicked = false;
        xSemaphoreGive(mutex);
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // 👈 IMPORTANT: Check every 100ms (prevents CPU hogging)
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Create FreeRTOS mutex
  mutex = xSemaphoreCreateMutex();

  // Create FreeRTOS task pinned to Core 0
  xTaskCreatePinnedToCore(
      timer2Task,   // Task function
      "Timer2Task", // Task name
      4096,         // Stack size (bytes) - increased for safety
      NULL,         // Parameters
      1,            // Priority (1 = low, higher number = higher priority)
      NULL,         // Task handle
      0             // Core 0 (main loop runs on Core 1)
  );

  Serial.println("✅ FreeRTOS task created on Core 0");
}

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

  // Start timer1 when object detected
  if (distanceCm < 3 && timer1_start == 0)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    timer1_start = millis();
    xSemaphoreGive(mutex);
    Serial.println("Timer1 started");
  }

  // Kick watchdog if distance > 5 during detection
  if (distanceCm > 5 && timer1_start != 0)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    watchdog_kicked = true;
    timer2_start = millis();
    xSemaphoreGive(mutex);
    Serial.println("Timer2 started (watchdog kicked)");
  }

  delay(100); // Check every 100ms
}