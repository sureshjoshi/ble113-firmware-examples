#include "Wire.h"
#include <SoftwareSerial.h>

#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

SoftwareSerial ble(9,8);

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    pinMode(9, INPUT);
    pinMode(8, OUTPUT);
    // initialize serial communication
	ble.begin(9600);
	Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

// byte testData[] = {0x01,0x02,0x03,0x04, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}; 
byte mpuData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// {axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gyl}

void loop() {
    // read raw accel/gyro measurements from device
    delay(2000);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    mpuData[0] = (byte)(ax >> 8);
    mpuData[1] = (byte)(ax & 0xFF); 
    mpuData[2] = (byte)(ay >> 8);
    mpuData[3] = (byte)(ay & 0xFF);
    mpuData[4] = (byte)(az >> 8);
    mpuData[5] = (byte)(az & 0xFF);

    mpuData[6] = (byte)(gx >> 8);
    mpuData[7] = (byte)(gx & 0xFF);
    mpuData[8] = (byte)(gy >> 8);
    mpuData[9] = (byte)(gy & 0xFF);
    mpuData[10] = (byte)(gz >> 8);
    mpuData[11] = (byte)(gz & 0xFF);    

    ble.write(mpuData, 12);
                
    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}


