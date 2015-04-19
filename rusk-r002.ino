// This #include statement was automatically added by the Spark IDE.
#include "FOXFIRE_EmonLib/FOXFIRE_EmonLib.h"

// This #include statement was automatically added by the Spark IDE.
#include "FOXFIRE_BMP085/FOXFIRE_BMP085.h"

// This #include statement was automatically added by the Spark IDE.
#include "FOXFIRE_Si70xx/FOXFIRE_Si70xx.h"

// This #include statement was automatically added by the Spark IDE.
#include "FOXFIRE_Si1132/FOXFIRE_Si1132.h"

// This #include statement was automatically added by the Spark IDE.
#include "FOXFIRE_MPU6050/FOXFIRE_MPU6050.h"

//********************************************************************************
//********************************************************************************
//********************************************************************************

// Initialize application variables
int SENSORDELAY = 5000; // milliseconds (runs x1)
int EVENTSDELAY = 1000; // milliseconds (runs x10)
int SLEEP_DELAY = 45; // seconds (runs x1) - should get about 24 hours on 2100mAH - 0 to disable
String SLEEP_DELAY_MIN = "15"; // seconds - easier to store as string then convert to int
String SLEEP_DELAY_STATUS = "OK"; // always OK to start with

int I2CEN = D2;
int ALGEN = D3;
int LED = D7;

int SOUND = A1;
int SOUNDV = 0; //Raw Peak-to-Peak Level/Amplitude

int POWR1 = A2;
int POWR2 = A3;
int POWR3 = A4;
double POWR1V = 0; //Watts
double POWR2V = 0; //Watts
double POWR3V = 0; //Watts

int SOILT = A6;
int SOILTV = 0; //Raw 0-4095

int SOILH = A7;
int SOILHV = 0; //Raw 0-4095

bool BMP180OK = false;
float BMP180Pressure = 0; //hPa
float BMP180Temperature = 0; //Celsius
float BMP180Altitude = 0; //Meters

bool Si7020OK = false;
float Si7020Temperature = 0; //Celsius
float Si7020Humidity = 0; //%Relative Humidity

bool Si1132OK = false;
float Si1132UVIndex = 0; //UV Index scoring is as follows: 1-2  -> Low, 3-5  -> Moderate, 6-7  -> High, 8-10 -> Very High, 11+  -> Extreme
float Si1132Visible = 0; //Lux
float Si1132InfraRd = 0; //Lux

bool ACCELOK = false;
int16_t ax, ay, az;
int16_t gx, gy, gz;

//********************************************************************************
//********************************************************************************
//********************************************************************************

// Initialize cloud API float-to-string-hack
// https://community.spark.io/t/get-reads-all-return-same-variable-value/1290/7
// These values are mapped and updated in the postSensingVariableMap() function
char Sound[63] = "0";
char Power[63] = "0.00 0.00 0.00";
char SoilTnH[63] = "0 0";
char PreTAlt[63] = "0.00 0.00 0.00";
char AmbiTnH[63] = "0.00 0.00";
char UVVisIR[63] = "0.00 0.00 0.00";
char AcclXYZ[63] = "0 0 0";
char GyroXYZ[63] = "0 0 0";

//********************************************************************************
//********************************************************************************
//********************************************************************************

// Power
EnergyMonitor emon1;
EnergyMonitor emon2;
EnergyMonitor emon3;

// Weather
Adafruit_BMP085_Unified bmp180 = Adafruit_BMP085_Unified(10085);
FOXFIRE_Si70xx si7020;
FOXFIRE_Si1132 si1132;

// MPU6050
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu6050(0x68);

//********************************************************************************
//********************************************************************************
//********************************************************************************

/*
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/

void setPinsMode() {
    pinMode(I2CEN, OUTPUT);
    pinMode(ALGEN, OUTPUT);
    pinMode(LED, OUTPUT);

    pinMode(POWR1, INPUT);
    pinMode(POWR2, INPUT);
    pinMode(POWR3, INPUT);

    pinMode(SOILT, INPUT);
    pinMode(SOILH, INPUT);
}

int setSleepDelay(String SLEEP_DELAY_STR) {
    if (SLEEP_DELAY_STR.toInt() > SLEEP_DELAY_MIN.toInt())
    {
        SLEEP_DELAY = SLEEP_DELAY_STR.toInt();
        SLEEP_DELAY_STATUS = "OK";
        return SLEEP_DELAY;
    }
    else
    {
        SLEEP_DELAY_STATUS = "SLEEP DELAY VALUE MUST BE GREATER THAN " + SLEEP_DELAY_MIN + " SECONDS";
        return -1;
    }
}

int readSoundLevel() {
    unsigned int sampleWindow = 50; // Sample window width in milliseconds (50 milliseconds = 20Hz)
    unsigned long endWindow = millis() + sampleWindow;  // End of sample window

    unsigned int signalSample = 0;
    unsigned int signalMin = 1024; // Minimum is the lowest signal below which we assume silence
    unsigned int signalMax = signalMin; // Maximum signal starts out the same as the Minimum signal

    // collect data for milliseconds equal to sampleWindow
    while (millis() < endWindow)
    {
        signalSample = analogRead(SOUND);
        if (signalSample > signalMax)
        {
            signalMax = signalSample;  // save just the max levels
        }
        else if (signalSample < signalMin)
        {
            signalMin = signalSample;  // save just the min levels
        }
    }

    SOUNDV = signalMax - signalMin;  // max - min = peak-peak amplitude

    return 1;
}

int readPower() {
    // If voltage is fixed, e.g. 240V,
    // and resistance is known/assumed,
    // then the only variation is amperage.
    POWR1V = (emon1.calcIrms(1480) * 1000); //Watts
    POWR2V = (emon2.calcIrms(1480) * 1000); //Watts
    POWR3V = (emon3.calcIrms(1480) * 1000); //Watts

    return 1;
}

int readSoilTemperature() {
    SOILTV = analogRead(SOILT);

    return 1;
}

int readSoilHumidity() {
    SOILHV = analogRead(SOILH);

    return 2;
}

int readWeatherBMP180() {
    if (BMP180OK)
    {
        /* Get a new sensor event */
        sensors_event_t event;
        bmp180.getEvent(&event);

        // Read weather measurements from device
        if (event.pressure)
        {
            BMP180Pressure = event.pressure;

            /* First we get the current temperature from the BMP085 */
            bmp180.getTemperature(&BMP180Temperature);

            /* Then convert the atmospheric pressure, SLP and temp to altitude */
            /* Update this next line with the current SLP for better results */
            float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
            BMP180Altitude = bmp180.pressureToAltitude(seaLevelPressure, BMP180Pressure, BMP180Temperature);
        }
    }

    return BMP180OK ? 1 : 0;
}

int readWeatherSi7020() {
    if (Si7020OK)
    {
        Si7020Temperature = si7020.readTemperature();
        Si7020Humidity = si7020.readHumidity();
    }

    return Si7020OK ? 2 : 0;
}

int readWeatherSi1132() {
    if (Si1132OK)
    {
        Si1132UVIndex = si1132.readUV() / 100;
        Si1132Visible = si1132.readVisible();
        Si1132InfraRd = si1132.readIR();
    }

    return Si1132OK ? 3 : 0;
}

int readMotion() {
    if (ACCELOK)
    {
        // read raw accel/gyro measurements from device
        mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    }

    return ACCELOK ? 1 : 0;
}

void postSensingVariableMap() {
    sprintf(Sound, "%i", SOUNDV);
    sprintf(Power, "%.2f %.2f %.2f", POWR1V, POWR2V, POWR3V);
    sprintf(SoilTnH, "%i %i", SOILTV, SOILHV);
    sprintf(PreTAlt, "%.2f %.2f %.2f", BMP180Pressure, BMP180Temperature, BMP180Altitude);
    sprintf(AmbiTnH, "%.2f %.2f", Si7020Temperature, Si7020Humidity);
    sprintf(UVVisIR, "%.2f %.2f %.2f", Si1132UVIndex, Si1132Visible, Si1132InfraRd);
    sprintf(AcclXYZ, "%i %i %i", ax, ay, az);
    sprintf(GyroXYZ, "%i %i %i", gx, gy, gz);
}

void postSensingEventPublish() {
    // Publish update events
    // --> up to a burst of 4 at rate of 1 per second
    // --> testing delay of 1.0 seconds after each event
    Spark.publish("Ready", NULL, String(EVENTSDELAY / 1000).toInt() * 8, PRIVATE); delay(EVENTSDELAY);
    Spark.publish("Sound", String(Sound), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
    Spark.publish("Power", String(Power), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
    Spark.publish("SoilTnH", String(SoilTnH), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
    Spark.publish("PreTAlt", String(PreTAlt), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
    Spark.publish("AmbiTnH", String(AmbiTnH), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
    Spark.publish("UVVisIR", String(UVVisIR), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
    Spark.publish("AcclXYZ", String(AcclXYZ), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
    Spark.publish("GyroXYZ", String(GyroXYZ), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
}

//********************************************************************************
//********************************************************************************
//********************************************************************************

void setup() {
    // Open serial over USB
    Serial.begin(9600);

    // Sets the I2C clock speed
    //Wire.setSpeed(CLOCK_SPEED_400KHZ);

    // Initialize IO pins
    setPinsMode();

    // Initialize cloud API variables
    Spark.variable("Sound", &Sound, STRING);
    Spark.variable("Power", &Power, STRING);
    Spark.variable("SoilTnH", &SoilTnH, STRING);
    Spark.variable("PreTAlt", &PreTAlt, STRING);
    Spark.variable("AmbiTnH", &AmbiTnH, STRING);
    Spark.variable("UVVisIR", &UVVisIR, STRING);
    Spark.variable("AcclXYZ", &AcclXYZ, STRING);
    Spark.variable("GyroXYZ", &GyroXYZ, STRING);

    // Initialize cloud API functions
    Spark.function("sleepDelay", setSleepDelay);
}

void loop(void) {
    if (Spark.connected())
    {
        Serial.print("RUSK --> Device --> "); Serial.println(Spark.deviceID());

        //********************************************************************************
        //********************************************************************************
        //********************************************************************************

        // Power up sensors
        digitalWrite(I2CEN, HIGH);
        digitalWrite(ALGEN, HIGH);

        // Initialize BMP180
        BMP180OK = bmp180.begin();
        // Initialize Si7020
        Si7020OK = si7020.begin();
        // Initialize Si1132
        Si1132OK = si1132.begin();
        // Initialize MPU6050 and verify connection
        mpu6050.initialize();
        ACCELOK = mpu6050.testConnection();

        // Allow sensors to warm up
        delay(SENSORDELAY);

        //********************************************************************************
        //********************************************************************************
        //********************************************************************************

        Serial.print("RUSK --> Sound --> "); Serial.println(readSoundLevel());
        Serial.print("SoundNw:\t"); Serial.println(SOUNDV);

        Serial.print("RUSK --> Power --> "); Serial.println(readPower());
        Serial.print("WattsNw:\t");
        Serial.print(POWR1V); Serial.print("\t");
        Serial.print(POWR2V); Serial.print("\t");
        Serial.println(POWR3V);

        Serial.print("RUSK --> Soil --> "); Serial.println(readSoilTemperature() + readSoilHumidity());
        Serial.print("SoilTem:\t"); Serial.println(SOILTV);
        Serial.print("SoilHum:\t"); Serial.println(SOILHV);

        //********************************************************************************
        //********************************************************************************
        //********************************************************************************

        Serial.print("RUSK --> Weather --> "); Serial.println(readWeatherBMP180() + readWeatherSi7020() + readWeatherSi1132());

        // display values
        Serial.print("BMPTemp:\t"); Serial.println(BMP180Temperature);
        Serial.print("BMPPres:\t"); Serial.println(BMP180Pressure);
        Serial.print("BMPAlti:\t"); Serial.println(BMP180Altitude);
        Serial.print("AirTemp:\t"); Serial.println(Si7020Temperature);
        Serial.print("AirHumi:\t"); Serial.println(Si7020Humidity);
        Serial.print("UVIndx0:\t"); Serial.println(Si1132UVIndex);
        Serial.print("Visible:\t"); Serial.println(Si1132Visible);
        Serial.print("InfraRd:\t"); Serial.println(Si1132InfraRd);

        //********************************************************************************
        //********************************************************************************
        //********************************************************************************

        Serial.print("RUSK --> Motion --> "); Serial.println(readMotion());

        // display tab-separated accel x/y/z values
        Serial.print("AcclXYZ:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.println(az);

        // display tab-separated gyro x/y/z values
        Serial.print("GyroXYZ:\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);

        //********************************************************************************
        //********************************************************************************
        //********************************************************************************

        Serial.println("RUSK --> Device --> Post Sensing Activities");

        // run post sensing activity
        digitalWrite(LED, HIGH);
        postSensingVariableMap();
        postSensingEventPublish();
        digitalWrite(LED, LOW);

        //********************************************************************************
        //********************************************************************************
        //********************************************************************************

        if (SLEEP_DELAY > 0)
        {
            // Power down sensors
            digitalWrite(I2CEN, LOW);
            digitalWrite(ALGEN, LOW);

            //********************************************************************************
            //********************************************************************************
            //********************************************************************************

            Serial.print("RUSK --> Sleep --> "); Serial.println(SLEEP_DELAY);
            Serial.print("RUSK --> Sleep --> "); Serial.println(SLEEP_DELAY_STATUS);

            // publish sleep event
            Spark.publish("Sleep", String(SLEEP_DELAY), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);

            // run internal processes and invoke light sleep mode
            Spark.process();
            Spark.sleep(SLEEP_DELAY);
        }
    }
    else
    {
        if (WiFi.ready())
        {
            Spark.connect();
        }
    }
}