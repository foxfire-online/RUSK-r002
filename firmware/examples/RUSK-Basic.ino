//********************************************************************************
//********************************************************************************
//********************************************************************************

// Initialize application variables
int SENSORDELAY = 3000; // milliseconds (runs x1)
int EVENTSDELAY = 1000; // milliseconds (runs x10)
int OTAUPDDELAY = 7000; // milliseconds (runs x1)
int SLEEP_DELAY = 0; // 40 seconds (runs x1) - should get about 24 hours on 2100mAH, 0 to disable and use RELAX_DELAY instead
String SLEEP_DELAY_MIN = "15"; // seconds - easier to store as string then convert to int
String SLEEP_DELAY_STATUS = "OK"; // always OK to start with
int RELAX_DELAY = 10; // seconds (runs x1) - no power impact, just idle/relaxing

// Variables for the I2C scan
byte I2CERR, I2CADR;

//********************************************************************************
//********************************************************************************
//********************************************************************************

int I2CEN = D2;
int ALGEN = D3;
int LED = D7;

int SOUND = A0;
double SOUNDV = 0; //Volts Peak-to-Peak Level/Amplitude

int POWR1 = A1;
int POWR2 = A2;
int POWR3 = A3;
double POWR1V = 0; //Watts
double POWR2V = 0; //Watts
double POWR3V = 0; //Watts

int SOILT = A4;
double SOILTV = 0; //Celsius: Temperature (C) = Vout*41.67-40 :: Temperature (F) = Vout*75.006-40

int SOILH = A5;
double SOILHV = 0; //Volumetric Water Content (VWC): http://www.vegetronix.com/TechInfo/How-To-Measure-VWC.phtml

bool BMP180OK = false;
double BMP180Pressure = 0; //hPa
double BMP180Temperature = 0; //Celsius
double BMP180Altitude = 0; //Meters

bool Si7020OK = false;
double Si7020Temperature = 0; //Celsius
double Si7020Humidity = 0; //%Relative Humidity

bool Si1132OK = false;
double Si1132UVIndex = 0; //UV Index scoring is as follows: 1-2  -> Low, 3-5  -> Moderate, 6-7  -> High, 8-10 -> Very High, 11+  -> Extreme
double Si1132Visible = 0; //Lux
double Si1132InfraRd = 0; //Lux

bool ACCELOK = false;
int cx, cy, cz, ax, ay, az, gx, gy, gz;
double tm; //Celsius

//********************************************************************************
//********************************************************************************
//********************************************************************************

// Initialize cloud API float-to-string-hack
// https://community.spark.io/t/get-reads-all-return-same-variable-value/1290/7
// These values are mapped and updated in the postSensingVariableMap() function
char Sound[63] = "0.00";
char Power[63] = "0.00 0.00 0.00";
char SoilTnH[63] = "0.00 0.00";
char PreTAlt[63] = "0.00 0.00 0.00";
char AmbiTnH[63] = "0.00 0.00";
char UVVisIR[63] = "0.00 0.00 0.00";
char CompXYZ[63] = "0 0 0";
char AcclXYZ[63] = "0 0 0";
char GyroXYZ[63] = "0 0 0";

//********************************************************************************
//********************************************************************************
//********************************************************************************

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setPinsMode() {
    pinMode(I2CEN, OUTPUT);
    pinMode(ALGEN, OUTPUT);
    pinMode(LED, OUTPUT);

    pinMode(SOUND, INPUT);

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
    unsigned int signalMin = 4095; // Minimum is the lowest signal below which we assume silence
    unsigned int signalMax = 0; // Maximum signal starts out the same as the Minimum signal

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

    //SOUNDV = signalMax - signalMin;  // max - min = peak-peak amplitude
    SOUNDV = mapFloat((signalMax - signalMin), 0, 4095, 0, 3.3);

    return 1;
}

int readPower() {
    // Vrms for apparent power readings (when no AC-AC voltage sample is present)
    byte Vrms = 240; //This is basic residential or socket voltage

    // Power
    EnergyMonitor emon1;
    EnergyMonitor emon2;
    EnergyMonitor emon3;

    // Calibration factor = CT ratio / burden resistance
    // (100A / 0.05A) / 33 Ohms burden = 60.606
    // (100A / 0.05A) / 22 Ohms burden = 90.909
    emon1.current(POWR1, 90.909);// Current: input pin, calibration.
    emon2.current(POWR2, 90.909);// Current: input pin, calibration.
    emon3.current(POWR3, 90.909);// Current: input pin, calibration.

    // Calculate Apparent Power
    // If voltage is fixed, e.g. 240V,
    // and resistance is known/assumed,
    // then the only variation is amperage.
    POWR1V = (emon1.calcIrms(1480) * Vrms); //Watts: 1480 is number of samples.
    POWR2V = (emon2.calcIrms(1480) * Vrms); //Watts: 1480 is number of samples.
    POWR3V = (emon3.calcIrms(1480) * Vrms); //Watts: 1480 is number of samples.

    return 1;
}

int readSoilTemperature() {
    //Formula basd on THERM200 documentation (Voltage Output Equation)
    //http://www.vegetronix.com/Products/THERM200/

    SOILTV = (mapFloat(analogRead(SOILT), 0, 4095, 0, 3.3) * 41.67) - 40;

    return 1;
}

int readSoilHumidity() {
    // Formula based on VH400 documentation (Voltage Output Curves)
    // http://vegetronix.com/Products/VH400/VH400-Piecewise-Curve.phtml

    float voltage = mapFloat(analogRead(SOILH), 0, 4095, 0, 3.3);

    if (0 < voltage && voltage <= 1.1)
    {
        SOILHV = (10 * voltage) - 1;
    }
    else if (1.1 < voltage && voltage <= 1.3)
    {
        SOILHV = (25 * voltage) - 17.5;
    }
    else if (1.3 < voltage && voltage <= 1.82)
    {
        SOILHV = (48.08 * voltage) - 47.5;
    }
    else if (1.82 < voltage && voltage <= 2.2)
    {
        SOILHV = (26.32 * voltage) - 7.89;
    }
    else
    {
        // All undocumented and out of range values get a value of -1
        SOILHV = -1;
    }

    return 2;
}

int readWeatherBMP180() {
    Adafruit_BMP085_Unified bmp180 = Adafruit_BMP085_Unified(10085);
    BMP180OK = bmp180.begin(); // Initialize BMP180
    float bufferPressure = 0;
    float bufferTemperature = 0;

    if (BMP180OK)
    {
        /* Get a new sensor event */
        sensors_event_t event;
        bmp180.getEvent(&event);

        // Read weather measurements from device
        if (event.pressure)
        {
            bufferPressure = event.pressure;
            /* The pressure is initially loaded into a buffer variable */
            /* We load it into the published variable this way to be compatible with the Spark.variable data types*/
            BMP180Pressure = bufferPressure;

            /* First we get the current temperature from the BMP085 */
            bmp180.getTemperature(&bufferTemperature);
            /* The temperature is initially loaded into a buffer variable */
            /* We load it into the published variable this way to be compatible with the Spark.variable data types*/
            BMP180Temperature = bufferTemperature;

            /* Then convert the atmospheric pressure, SLP and temp to altitude */
            /* Update this next line with the current SLP for better results */
            float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
            BMP180Altitude = bmp180.pressureToAltitude(seaLevelPressure, bufferPressure, bufferTemperature);
        }
    }

    return BMP180OK ? 1 : 0;
}

int readWeatherSi7020() {
    FOXFIRE_Si70xx si7020;
    Si7020OK = si7020.begin(); // Initialize Si7020

    if (Si7020OK)
    {
        Si7020Temperature = si7020.readTemperature();
        Si7020Humidity = si7020.readHumidity();
    }

    return Si7020OK ? 2 : 0;
}

int readWeatherSi1132() {
    FOXFIRE_Si1132 si1132;
    Si1132OK = si1132.begin(); // Initialize Si1132

    if (Si1132OK)
    {
        Si1132UVIndex = si1132.readUV() / 100;
        Si1132Visible = si1132.readVisible();
        Si1132InfraRd = si1132.readIR();
    }

    return Si1132OK ? 3 : 0;
}

int readMotion() {
    FOXFIRE_MPU9150 mpu9150;
    ACCELOK = mpu9150.begin(mpu9150._addr_motion); // Initialize MPU9150

    if (ACCELOK)
    {
        // Clear the 'sleep' bit to start the sensor.
        mpu9150.writeSensor(mpu9150._addr_motion, MPU9150_PWR_MGMT_1, 0);

        // Set up compass
        mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x00); //PowerDownMode
        mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x0F); //SelfTest
        mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x00); //PowerDownMode

        mpu9150.writeSensor(mpu9150._addr_motion, 0x24, 0x40); //Wait for Data at Slave0
        mpu9150.writeSensor(mpu9150._addr_motion, 0x25, 0x8C); //Set i2c address at slave0 at 0x0C
        mpu9150.writeSensor(mpu9150._addr_motion, 0x26, 0x02); //Set where reading at slave 0 starts
        mpu9150.writeSensor(mpu9150._addr_motion, 0x27, 0x88); //set offset at start reading and enable
        mpu9150.writeSensor(mpu9150._addr_motion, 0x28, 0x0C); //set i2c address at slv1 at 0x0C
        mpu9150.writeSensor(mpu9150._addr_motion, 0x29, 0x0A); //Set where reading at slave 1 starts
        mpu9150.writeSensor(mpu9150._addr_motion, 0x2A, 0x81); //Enable at set length to 1
        mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x01); //overvride register
        mpu9150.writeSensor(mpu9150._addr_motion, 0x67, 0x03); //set delay rate
        mpu9150.writeSensor(mpu9150._addr_motion, 0x01, 0x80);

        mpu9150.writeSensor(mpu9150._addr_motion, 0x34, 0x04); //set i2c slv4 delay
        mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x00); //override register
        mpu9150.writeSensor(mpu9150._addr_motion, 0x6A, 0x00); //clear usr setting
        mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x01); //override register
        mpu9150.writeSensor(mpu9150._addr_motion, 0x6A, 0x20); //enable master i2c mode
        mpu9150.writeSensor(mpu9150._addr_motion, 0x34, 0x13); //disable slv4

        // Read all sensor values which the sensor provides
        // Formated all values as x, y, and z in order for
        // Compass, Gyro, Acceleration. The First value is
        // the temperature.

        tm = ( (double) mpu9150.readSensor(mpu9150._addr_motion, MPU9150_TEMP_OUT_L, MPU9150_TEMP_OUT_H) + 12412.0 ) / 340.0;
        cx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_XOUT_L, MPU9150_CMPS_XOUT_H);
        cy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_YOUT_L, MPU9150_CMPS_YOUT_H);
        cz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_ZOUT_L, MPU9150_CMPS_ZOUT_H);
        ax = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
        ay = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
        az = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H);
        gx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_XOUT_L, MPU9150_GYRO_XOUT_H);
        gy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_YOUT_L, MPU9150_GYRO_YOUT_H);
        gz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_ZOUT_L, MPU9150_GYRO_ZOUT_H);
    }

    return ACCELOK ? 1 : 0;
}

void postSensingVariableMap() {
    sprintf(Sound, "%.2f", SOUNDV);
    sprintf(Power, "%.2f %.2f %.2f", POWR1V, POWR2V, POWR3V);
    sprintf(SoilTnH, "%.2f %.2f", SOILTV, SOILHV);
    sprintf(PreTAlt, "%.2f %.2f %.2f", BMP180Pressure, BMP180Temperature, BMP180Altitude);
    sprintf(AmbiTnH, "%.2f %.2f", Si7020Temperature, Si7020Humidity);
    sprintf(UVVisIR, "%.2f %.2f %.2f", Si1132UVIndex, Si1132Visible, Si1132InfraRd);
    sprintf(CompXYZ, "%i %i %i", cx, cy, cz);
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
    Spark.publish("CompXYZ", String(CompXYZ), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
    Spark.publish("AcclXYZ", String(AcclXYZ), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
    Spark.publish("GyroXYZ", String(GyroXYZ), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);
}

//********************************************************************************
//********************************************************************************
//********************************************************************************

void setup() {
    // Open serial over USB
    Serial.begin(9600);

    // Disable Interrupts
    noInterrupts();

    // Initialize IO pins
    setPinsMode();

    // Initialize cloud API variables (using 9 of 10 available)
    Spark.variable("Sound", &Sound, STRING);
    Spark.variable("Power", &Power, STRING);
    Spark.variable("SoilTnH", &SoilTnH, STRING);
    Spark.variable("PreTAlt", &PreTAlt, STRING);
    Spark.variable("AmbiTnH", &AmbiTnH, STRING);
    Spark.variable("UVVisIR", &UVVisIR, STRING);
    Spark.variable("CompXYZ", &CompXYZ, STRING);
    Spark.variable("AcclXYZ", &AcclXYZ, STRING);
    Spark.variable("GyroXYZ", &GyroXYZ, STRING);

    // Initialize custom cloud API variables here
    // or edit above to monitor specific values for
    // services like IFTTT (last free of 10)
    Spark.variable("IF3T_Visible", &Si1132Visible, DOUBLE);

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

        // Allow sensors to warm up
        delay(SENSORDELAY);

        //********************************************************************************
        //********************************************************************************
        //********************************************************************************

        //Initialize the 'Wire' class for I2C Scan
        Wire.begin();

        // Scan for I2C devices
        Serial.println("RUSK --> ScanI2C --> Scanning I2C for devices...");
        for (I2CADR = 1; I2CADR < 127; I2CADR++)
        {
            // The i2c_scanner uses the return value of
            // the Write.endTransmisstion to see if
            // a device did acknowledge to the address.
            Wire.beginTransmission(I2CADR);
            I2CERR = Wire.endTransmission();

            if (I2CERR == 0)
            {
                Serial.print("RUSK --> ScanI2C --> Device at 0x");
                if (I2CADR < 16)
                {
                    Serial.print("0");
                }
                Serial.print(I2CADR, HEX);
                Serial.println("");
            }
        }

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
        Serial.print("UVIndex:\t"); Serial.println(Si1132UVIndex);
        Serial.print("Visible:\t"); Serial.println(Si1132Visible);
        Serial.print("InfraRd:\t"); Serial.println(Si1132InfraRd);

        //********************************************************************************
        //********************************************************************************
        //********************************************************************************

        Serial.print("RUSK --> Motion --> "); Serial.println(readMotion());

        // display tab-separated compass x/y/z values
        Serial.print("CompXYZ:\t");
        Serial.print(cx); Serial.print("\t");
        Serial.print(cy); Serial.print("\t");
        Serial.println(cz);

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

        if (SLEEP_DELAY > 0)
        {
            // Power down sensors
            digitalWrite(I2CEN, LOW);
            digitalWrite(ALGEN, LOW);
        }

        //********************************************************************************
        //********************************************************************************
        //********************************************************************************

        Serial.println("RUSK --> Device --> Post Sensing Activities");

        // run post sensing activity
        postSensingVariableMap();
        postSensingEventPublish();


        Serial.println("RUSK --> Device --> OTA Update Window");

        // set aside time to do OTA updates
        digitalWrite(LED, HIGH);
        Spark.process(); delay(OTAUPDDELAY);
        digitalWrite(LED, LOW);

        //********************************************************************************
        //********************************************************************************
        //********************************************************************************

        if (SLEEP_DELAY > 0)
        {
            Serial.print("RUSK --> Sleep --> "); Serial.println(SLEEP_DELAY);
            Serial.print("RUSK --> Sleep --> "); Serial.println(SLEEP_DELAY_STATUS);

            // publish sleep event
            Spark.publish("Sleep", String(SLEEP_DELAY), SLEEP_DELAY, PRIVATE); delay(EVENTSDELAY);

            // invoke light sleep mode
            Spark.sleep(SLEEP_DELAY);
        }
        else
        {
            Serial.print("RUSK --> Relax --> "); Serial.println(RELAX_DELAY);

            // publish relax event
            Spark.publish("Relax", String(RELAX_DELAY), RELAX_DELAY, PRIVATE); delay(EVENTSDELAY);

            // pause briefly anyway
            delay(RELAX_DELAY * 1000);
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