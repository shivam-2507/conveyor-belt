#include <Arduino.h>

const int trigPin = 5;
const int echoPin = 12;

int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 14;

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

// define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

SemaphoreHandle_t mutex;

unsigned long timer1_start = 0;
unsigned long timer2_start = 0;
bool timer1_active = false;
bool timer2_active = false;

// Motor control task - runs continuously forever
void motorTask(void *parameter)
{
  while (true)
  {
    // Move motor forward continuously
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    ledcWrite(pwmChannel, dutyCycle);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Timer2 watchdog task - monitors if distance > 5 for more than 3 seconds
void timer2Task(void *parameter)
{
  while (true)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    bool t2_active = timer2_active;
    unsigned long t2_start = timer2_start;
    unsigned long t1_start = timer1_start;
    xSemaphoreGive(mutex);

    if (t2_active)
    {
      unsigned long elapsed = millis() - t2_start;

      if (elapsed >= 3000) // 3 seconds
      {
        // Calculate time difference: timer2 - timer1
        unsigned long time_diff = t2_start - t1_start;

        Serial.println("\n========================================");
        Serial.print("⏱️  Object detection time: ");
        Serial.print(time_diff);
        Serial.println(" ms");
        Serial.println("========================================\n");

        // Reset timers
        xSemaphoreTake(mutex, portMAX_DELAY);
        timer1_active = false;
        timer2_active = false;
        timer1_start = 0;
        timer2_start = 0;
        xSemaphoreGive(mutex);
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  // Configure PWM
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);

  // Create FreeRTOS mutex
  mutex = xSemaphoreCreateMutex();

  // Create motor task - runs forever
  xTaskCreatePinnedToCore(
      motorTask,
      "MotorTask",
      2048,
      NULL,
      1,
      NULL,
      0);

  // Create timer2 watchdog task
  xTaskCreatePinnedToCore(
      timer2Task,
      "Timer2Task",
      4096,
      NULL,
      1,
      NULL,
      0);

  Serial.println("✅ System initialized");
  Serial.println("✅ Motor running continuously");
  Serial.println("✅ Timer tasks ready");
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

  return distanceCm;
}

void loop()
{
  distanceCm = readDistance();

  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);

  xSemaphoreTake(mutex, portMAX_DELAY);
  bool t1_active = timer1_active;
  bool t2_active = timer2_active;
  xSemaphoreGive(mutex);

  // Logic: If distance < 5, start timer1
  if (distanceCm < 5 && !t1_active)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    timer1_start = millis();
    timer1_active = true;
    xSemaphoreGive(mutex);

    Serial.println("🟢 Timer1 started - Object detected!");
  }

  // Logic: If distance > 5 AND timer1 is active, start timer2 and show warning
  if (distanceCm > 5 && t1_active && !t2_active)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    timer2_start = millis();
    timer2_active = true;
    xSemaphoreGive(mutex);

    Serial.println("⚠️  WARNING: Distance exceeded 5cm - Timer2 started!");
    Serial.println("⏳ Monitoring for 3 seconds...");
  }

  delay(100);
}
