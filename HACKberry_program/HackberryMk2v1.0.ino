/*
 *  Arduino micro code for HACKberry Mk2.
 *  Origially created by exiii Inc.
 *  edited by Kouki Shinjo on 2018/04/26
 */
#include <Servo.h>

//Settings
const boolean isRight = 0;//right:1, left:0
const int outThumbMax = 95;//right:open, left:close
const int outIndexMax = 100;//right:open, left:close
const int outOtherMax = 75;//right:open, left:close
const int outThumbMin = 10;//right:close, left:open
const int outIndexMin = 0;//right:close, left:open
const int outOtherMin = 20;//right:close, left:open
const int speedMax = 6;
const int speedMin = 0;
const int speedReverse = -3;
const int thSpeedReverse = 15;//0-100
const int thSpeedZero = 30;//0-100

// onSerial is true if you want to print data live to the serial monitor
const boolean onSerial = 0; // 1 is not recommended

//Pin
int pinButtonCalib; //start calibration
int pinButtonTBD; // No function implemented yet.
int pinButtonThumb; // open/close thumb
int pinButtonOther; //lock/unlock other three fingers
int pinServoIndex;
int pinServoOther;
int pinServoThumb;
int pinSensor; //sensor input

//Hardware
Servo servoIndex; //index finger
Servo servoOther; //other three fingers
Servo servoThumb; //thumb

//Software
boolean isThumbOpen = 1;
boolean isOtherLock = 0;
int swCount0,swCount1,swCount2,swCount3 = 0;
int sensorValue = 0; // value read from the sensor
int sensorMax = 700;
int sensorMin = 0;
int speed = 0;
int position = 0;
const int positionMax = 100;
const int positionMin = 0;
int prePosition = 0;
int outThumb,outIndex,outOther = 90;
int outThumbOpen,outThumbClose,outIndexOpen,outIndexClose,outOtherOpen,outOtherClose;

void setup() {
    // If onSerial, initialize serial output
    if (onSerial)  Serial.begin(9600);

    // Pin Configuration
    pinButtonCalib = A6;
    pinButtonTBD   = A7;
    pinButtonThumb = A0;
    pinButtonOther = 10;
    pinServoIndex  = 5;
    pinServoOther  = 6;
    pinServoThumb  = 9;
    pinSensor      = A1;

    if(isRight){    // If the arm is for the right hand, the open positions are max and the closed positions are min
        outThumbOpen=outThumbMax; outThumbClose=outThumbMin;
        outIndexOpen=outIndexMax; outIndexClose=outIndexMin;
        outOtherOpen=outOtherMax; outOtherClose=outOtherMin;
    } else {        // If the arm is for the left hand, the open positionsa are min and the closed positions are max
        outThumbOpen=outThumbMin; outThumbClose=outThumbMax;
        outIndexOpen=outIndexMin; outIndexClose=outIndexMax;
        outOtherOpen=outOtherMin; outOtherClose=outOtherMax;
    }
    
    // Instantiate servo objects
    servoIndex.attach(pinServoIndex);//index
    servoOther.attach(pinServoOther);//other
    servoThumb.attach(pinServoThumb);//thumb

    // Initialize input pins
    pinMode(pinButtonCalib, INPUT_PULLUP);
    pinMode(pinButtonTBD,   INPUT_PULLUP);
    pinMode(pinButtonThumb, INPUT_PULLUP);
    pinMode(pinButtonOther, INPUT_PULLUP);
}

void loop() {
    //==waiting for calibration==
    if(onSerial) Serial.println("======Waiting for Calibration======");
    while (1) {
        // At startup, constantly write to servos to open index, other, and thumb
        servoIndex.write(outIndexOpen);
        servoOther.write(outOtherOpen);
        servoThumb.write(outThumbOpen);
        if(onSerial) serialMonitor();
        delay(10);
        
        // pinButtonCalib is active low
        // When the calibration button is pressed, run calibration and break out of the loop
        if (readButton(pinButtonCalib) == LOW) {
            calibration();
            break;
        }
    }
    //==control==
    position = positionMin;
    prePosition = positionMin;
    while (1) {
        // If calibration button is pressed for 10 iterations (250 ms), run calibration
        if (readButton(pinButtonCalib) == LOW) swCount0 += 1;
        else swCount0 = 0;
        if (swCount0 == 10) {
            swCount0 = 0;
            calibration();
        }
        
        // If TBD button is pressed for 10 iterations (250 ms), run the TBD function (which is nothing for now)
        if (readButton(pinButtonTBD) == LOW) swCount1 += 1;
        else swCount1 = 0;
        if (swCount1 == 10) {
            swCount1 = 0;
            // Do something here
            while (readButton(pinButtonTBD) == LOW) delay(1);
        }
        
        // If thumb button is pressed for 10 iterations (250 ms), toggle the thumb open
        if (readButton(pinButtonThumb) == LOW) swCount2 += 1;
        else swCount2 = 0;
        if (swCount2 == 10) {
            swCount2 = 0;
            isThumbOpen = !isThumbOpen;
            
            // Make sure the toggle doesn't occur multiple times by waiting until the button is released
            while (readButton(pinButtonThumb) == LOW) delay(1);
        }
        
        // If other button is pressed for 10 iterations (250 ms), toggle other open
        if (readButton(pinButtonOther) == LOW) swCount3 += 1;//A3
        else swCount3 = 0;
        if (swCount3 == 10) {
            swCount3 = 0;
            isOtherLock = !isOtherLock;
            
            // Make sure the toggle doesn't occur multiple times by waiting until the button is released
            while (readButton(pinButtonOther) == LOW) delay(1);
        }

        // Read sensor, using average over 10 reads
        sensorValue = readSensor();
        
        // Each iteration should be 25 ms
        delay(25);
        
        // Bound the sensor value within sensorMin and sensorMax
        if(sensorValue<sensorMin) sensorValue=sensorMin;
        else if(sensorValue>sensorMax) sensorValue=sensorMax;
        
        // Compute new index finger servo percentage
        sensorToPosition();
        // Convert index finger percentage to servo position
        outIndex = map(position, positionMin, positionMax, outIndexOpen, outIndexClose);
        servoIndex.write(outIndex);
        
        // If the other fingers are not locked, move them in sync with the index finger
        if (!isOtherLock){
            outOther = map(position, positionMin, positionMax, outOtherOpen, outOtherClose);
            servoOther.write(outOther);
        }
        
        // Open or close the thumb according to the state of the toggle isThumbOpen
        if(isThumbOpen) servoThumb.write(outThumbOpen);
        else servoThumb.write(outThumbClose);
        
        if(onSerial) serialMonitor();
    }
}

/*
* functions
*/
boolean isDigitalPin(const int pin) {
    return (pin >= 0) && (pin <= 19) ? true : false;
}

boolean readButton(const int pin) {
    if ( isDigitalPin(pin) ) {
        return digitalRead(pin);
    } else {
        if (analogRead(pin) > 512) return HIGH;
        else return LOW;
    }
}

int readSensor() {
    int i, sval;
    for (i = 0; i < 10; i++) {
        sval += analogRead(pinSensor);
    }
    sval = sval/10;
    return sval;
}

void sensorToPosition(){
    /**
     * This functions converts the sensor value to a change in index finger position, within certain limits,
     * thus preventing the finger from snapping to a new position too quickly
     */
    
    // Convert sensorValue to percentage, based on sensorMin and sensorMax
    int tmpVal = map(sensorValue, sensorMin, sensorMax, 100, 0);
    // If 0 <= tmpVal < 15, speed = -3
    if(tmpVal<thSpeedReverse) speed=speedReverse;
    // If 15 <= tmpVal < 30, speed = 0
    else if(tmpVal<thSpeedZero) speed=speedMin;
    // If 30 <= tmpVal < 100, speed = map tmpVal to be in the domain of 0 to 6
    else speed=map(tmpVal,40,100,speedMin,speedMax);
    
    // Target position is current position + speed
    position = prePosition + speed;
    // Bound position between positionMin and positionMax
    if (position < positionMin) position = positionMin;
    if (position > positionMax) position = positionMax;
    // Okay this seems redundant, they could've just done prePosition += speed and bounded prePosition
    prePosition = position;
}

void calibration() {
    // Set index finger's position to completely open
    outIndex=outIndexOpen;
    // Open index and thumb, and close other
    servoIndex.write(outIndexOpen);
    servoOther.write(outOtherClose);
    servoThumb.write(outThumbOpen);
    
    // Set position and prePosition to min
    position=positionMin;
    prePosition=positionMin;

    // Wait 200 ms for servos to reach this state
    delay(200);
    if(onSerial) Serial.println("======calibration start======");

    // Initialize sensorMax and sensorMin to some arbitrary values on the order of the sensor reading
    sensorMax = readSensor();
    sensorMin = sensorMax - 50;
    unsigned long time = millis();
    // For exactly 4 seconds, update min and max values
    while ( millis() < time + 4000 ) { // millis seems unnecessary here when delay is still being used
        sensorValue = readSensor();
        delay(25);
        // Update sensorMin and sensorMax based on current reading
        if ( sensorValue < sensorMin ) sensorMin = sensorValue;
        else if ( sensorValue > sensorMax )sensorMax = sensorValue;

        // Update target position based on the current sensor value and the newly calculated sensor min and max values
        sensorToPosition();
        
        // Map position (which is currently a percentage) to an index finger servo position
        outIndex = map(position, positionMin, positionMax, outIndexOpen, outIndexClose);
        
        // Move index finger
        servoIndex.write(outIndex);

        if(onSerial) serialMonitor();
    }
    if(onSerial)  Serial.println("======calibration finish======");
    return;
}

//Print values to monitor
void serialMonitor(){
    Serial.print("Min="); Serial.print(sensorMin);
    Serial.print(",Max="); Serial.print(sensorMax);
    Serial.print(",sensor="); Serial.print(sensorValue);
    Serial.print(",speed="); Serial.print(speed);
    Serial.print(",position="); Serial.print(position);
    Serial.print(",outIndex="); Serial.print(outIndex);
    Serial.print(",isThumbOpen="); Serial.print(isThumbOpen);
    Serial.print(",isOtherLock="); Serial.println(isOtherLock);
}
