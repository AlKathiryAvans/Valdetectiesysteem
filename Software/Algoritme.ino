#include <SparkFunLIS3DH.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>


#define BUZZER_PIN 14

const float FREEFALL_THR = 0.3;
const float IMPACT_THR   = 2.3;
const float INACT_THR    = 0.30;

const int INACT_TIME     = 5000;
const int FREEFALL_TIME  = 150;
const int UPDATE_DELAY   = 10;

const float alpha = 0.12;

// Accelerometer manager
class Accelerometer {
private:
    float gx, gy, gz;
    float lx, ly, lz;
    
    LIS3DH accel;
    
    static const int BUF_SIZE = (INACT_TIME/1000) * (1000/UPDATE_DELAY);
    float magBuffer[BUF_SIZE];
    int magIndex = 0;
    bool bufferFilled = false;
    
public:
    float ax, ay, az;
    float mag;

    Accelerometer() : accel(I2C_MODE, 0x19), gx(0), gy(0), gz(0) {}

    bool begin() {
        accel.settings.accelSampleRate = 100;
        accel.settings.accelRange = 8;
        accel.settings.xAccelEnabled = 1;
        accel.settings.yAccelEnabled = 1;
        accel.settings.zAccelEnabled = 1;

        
        if(accel.begin() != IMU_SUCCESS)
            return false;
        return true;
    }

    float safeRead(float v) {
        if (isnan(v) || fabs(v) > accel.settings.accelRange) return 0.0;
        return v;
    }

    void update() {
        ax = safeRead(accel.readFloatAccelX());
        ay = safeRead(accel.readFloatAccelY());
        az = safeRead(accel.readFloatAccelZ());

        // Low-pass (gravity)
        gx = alpha * ax + (1 - alpha) * gx;
        gy = alpha * ay + (1 - alpha) * gy;
        gz = alpha * az + (1 - alpha) * gz;

        // Linear acceleration
        lx = ax - gx;
        ly = ay - gy;
        lz = az - gz;

        mag = sqrt(lx * lx + ly * ly + lz * lz);

        // buffer
        magBuffer[magIndex] = mag;
        magIndex = (magIndex + 1) % BUF_SIZE;
        if (magIndex == 0) bufferFilled = true;
    }

    float getAverageMag() {
        int count = bufferFilled ? BUF_SIZE : magIndex;
        if (count == 0) return 0;
        float sum = 0;
        for (int i = 0; i < count; i++) sum += magBuffer[i];
        return sum / count;
    }
};


// Detectie algoritme
class FallDetector {
private:
    unsigned long freefallStart = 0;
    unsigned long impactTime = 0;

public:
    bool fallDetected = false;
    bool inFreeFall = false;
    bool impactDetected = false;

    SemaphoreHandle_t mutex;

    FallDetector() {}

    bool begin() {
        mutex = xSemaphoreCreateMutex();
        if (mutex == NULL) {
            Serial.println("FallDetector mutex creation failed!");
            return false;
        }
        return true;
    }

    void update(Accelerometer &acc) {
        xSemaphoreTake(mutex, portMAX_DELAY);

        float mag = acc.mag;

        // Start free-fall
        if (!inFreeFall && mag < FREEFALL_THR) {
            inFreeFall = true;
            freefallStart = millis();
        }

        // Detect impact after free-fall
        if (inFreeFall && mag > IMPACT_THR) {
            if (millis() - freefallStart > FREEFALL_TIME) {
                impactDetected = true;
                impactTime = millis();
                fallDetected = false;
            }
            inFreeFall = false;
        }

        // Detect inactivity after impact
        if (impactDetected) {
            float avgMag = acc.getAverageMag();

            if (avgMag > INACT_THR) {
                impactDetected = false;
                inFreeFall = false;
                fallDetected = false;
            } 
            else if (millis() - impactTime > INACT_TIME) {
                fallDetected = true;
                impactDetected = false;
            }
        }

        xSemaphoreGive(mutex);
    }

    bool getFallState() {
        xSemaphoreTake(mutex, portMAX_DELAY);
        bool state = fallDetected;
        xSemaphoreGive(mutex);
        return state;
    }
};


// Buzzer
class BuzzerAlarm {
private:
    int pin;
public:
    BuzzerAlarm(int buzzerPin) {
        pin = buzzerPin;
    }
    void begin() {
        pinMode(pin, OUTPUT);
    }

    void update(bool active) {
        if (active) {
            tone(pin, 2500);
            vTaskDelay(pdMS_TO_TICKS(200));
            noTone(pin);
            vTaskDelay(pdMS_TO_TICKS(200));
            tone(pin, 1500);
            vTaskDelay(pdMS_TO_TICKS(200));
            noTone(pin);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            noTone(pin);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
};


// Objects
Accelerometer accelM;
FallDetector fallDet;
BuzzerAlarm buzzer(BUZZER_PIN);


// Tasks
void sensorTask(void *pv) {
    for (;;) {
        accelM.update();
        vTaskDelay(pdMS_TO_TICKS(UPDATE_DELAY));
    }
}

void fallTask(void *pv) {
    for (;;) {
        fallDet.update(accelM);
        vTaskDelay(pdMS_TO_TICKS(UPDATE_DELAY));
    }
}

void buzzerTask(void *pv) {
    for (;;) {
        buzzer.update(fallDet.getFallState());
    }
}

void reportTask(void *pv) {
    for (;;) {
        Serial.printf("mag:%.3f,x:%.3f,y:%.3f,z:%.3f,freefall:%d,impact:%d,fall:%d\n",
                      accelM.mag, accelM.ax, accelM.ay, accelM.az,
                      fallDet.inFreeFall,
                      fallDet.impactDetected,
                      fallDet.fallDetected);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}



void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Starting Fall Detection");

    if (!accelM.begin()) {
        Serial.println("Accelerometer failed!");
        while (true) delay(1000);
    }

    if (!fallDet.begin()) {
        Serial.println("Mutex init failed!");
        while (true) delay(1000);
    }

    buzzer.begin();

    // Task creation error handling
    if (xTaskCreatePinnedToCore(sensorTask, "Sensor", 4096, NULL, 2, NULL, 0) != pdPASS)
        Serial.println("SensorTask kon niet worden aangemaakt");

    if (xTaskCreatePinnedToCore(fallTask, "Fall", 4096, NULL, 2, NULL, 0) != pdPASS)
        Serial.println("FallTask kon niet worden aangemaakt");

    if (xTaskCreatePinnedToCore(buzzerTask, "Buzzer", 2048, NULL, 1, NULL, 1) != pdPASS)
        Serial.println("BuzzerTask kon niet worden aangemaakt");

    if (xTaskCreatePinnedToCore(reportTask, "Report", 4096, NULL, 1, NULL, 1) != pdPASS)
        Serial.println("ReportTask kon niet worden aangemaakt");
}

void loop() {}