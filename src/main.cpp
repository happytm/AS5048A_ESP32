/*

 */
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>

///////////////////////////////////////////////////////////////////////////////
//
// new SPI
//
static const int spiClk = 1000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
//SPIClass * hspi = NULL;
//
///////////////////////////////////////////////////////////////////////////////

// Define this if you want to run as an Access Point.  If undefined it will connect to the
// SSID with the password below....

#define AP
// uncomment the next line to turn on debugging
#define DEBUGGING

char ssid[] = "braapppp"; //  your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)

const char *apssid = "ESPap";
const char *appassword = "gofish";

int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// These defines are for the AS5048
// for example...
// http://ams.com/eng/Support/Demoboards/Position-Sensors/Rotary-Magnetic-Position-Sensors/AS5048A-Adapterboard

#define AS5048_CMD_READ 0x4000
#define AS5048_REG_AGC 0x3FFD
#define AS5048_REG_MAG 0x3FFE
#define AS5048_REG_DATA 0x3FFF
#define AS5048_REG_ERR 0x1
#define AS5048_CMD_NOP 0x0

#define numToAverage 20
#define discardNumber 2     // 2x this number must be less than numToAverage
int azimuthSensorPin=15;
int altitudeSensorPin=5;
byte cmd_highbyte = 0;
byte cmd_lowbyte = 0;
byte alt_data_highbyte = 0;
byte alt_data_lowbyte = 0;
byte azt_data_highbyte = 0;
byte azt_data_lowbyte = 0;
word command = 0;
word data = 0;
unsigned int value = 0;
unsigned int rawData;
double increment = (double) 360 / 16384;  // this is the smallest angular increment that is reportable by the sensors
double angle = 0;
int i = 0;
int del = 10;

// the arrays that will store the most recent numToAverage angles
double smoothAzimuthAngles[numToAverage];
double smoothAltitudeAngles[numToAverage];
// the value of the current Azimuth and Altitude angle that we will report back to Sky Safari
double advancedCircularSmoothAzimuthAngle = 0;
double advancedCircularSmoothAltitudeAngle = 0;

WiFiServer server(23);

boolean alreadyConnected = false; // whether or not the client was connected previously

///////////////////////////////////////////////////////////////////////////////
//
//  Function definitions
//
// pad the Tics value with leading zeros and return a string
String PadTic(unsigned int tic, String Sign){
  String paddedTic;
  if (tic < 10)
    paddedTic = "0000" + String(tic);
  else if ( tic < 100 )
    paddedTic = "000" + String(tic);
  else if ( tic < 1000 )
    paddedTic = "00" + String(tic);
  else if ( tic < 10000 )
    paddedTic = "0" + String(tic);
  else if ( tic < 100000 )
    paddedTic = "" + String(tic);
  paddedTic = Sign + paddedTic;
  return paddedTic;
}

// Calculate Even parity of word
byte calcEvenParity(word value) {
  byte count = 0;
  byte i;
  // loop through the 16 bits
  for (i = 0; i < 16; i++) {
    // if the rightmost bit is 1 increment our counter
    if (value & 0x1) {
      count++;
    }
    // shift off the rightmost bit
    value >>=1;
  }
  // all odd binaries end in 1
  return count & 0x1;
}

// Read the sensor REG_DATA register
unsigned int readSensor(int cs){
  unsigned int data;
  unsigned int data_highbyte;
  unsigned int data_lowbyte;
  pinMode(cs, OUTPUT);
  command = AS5048_CMD_READ | AS5048_REG_DATA;                        // Set up the command we will send
  command |= calcEvenParity(command) <<15;                            // assign the parity bit
  cmd_highbyte = highByte(command);                                   // split it into bytes
  cmd_lowbyte = lowByte(command);                                     //
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);                                             // Drop ssl to enable the AS5048's
  data_highbyte = vspi->transfer(cmd_highbyte);                         // send the initial read command
  data_lowbyte = vspi->transfer(cmd_lowbyte);
  digitalWrite(cs, HIGH);                                            // disable the AS5048's
  digitalWrite(cs, LOW);                                             // Drop ssl to enable the AS5048's
  data_highbyte = vspi->transfer(cmd_highbyte);                         // send the initial read command
  data_lowbyte = vspi->transfer(cmd_lowbyte);
  digitalWrite(cs, HIGH);                                            // disable the AS5048's
  vspi->endTransaction();
  data = data_highbyte;                                               // Store the high byte in my 16 bit varriable
  data = data << 8;                                                   // shift left 8 bits
  data = data | data_lowbyte;                                         // tack on the low byte
  #ifdef DEBUGGING
    Serial.println();
    Serial.print("Sent Command: ");
    Serial.println(command, BIN);
    Serial.print("To register: ");
    Serial.println(AS5048_REG_DATA, BIN);
    Serial.println(data);
  #endif
  return data;
}

// This just trims the bottom 14 bits off of a sensor read
unsigned int readTic(int cs){
  unsigned int rawData;
  unsigned int realData;
  rawData = readSensor(cs);
  realData = rawData & 0x3fff;
  return realData;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// Return the minimal angular separation for two angeles.  Returns between 0 and 180 for any two input values
double angularSeparation(double angleOne, double angleTwo){
  double retVal = 0;
  retVal = fabs(angleOne - angleTwo);
  if ( retVal > 180 ) {
   retVal = 360 - retVal;
  }
  return retVal;

}
// return the circular mean of the angles using the atan2 method
// takes a pointer to an array of angles
double circularAverage( double *anglesToAverage){
  int j;
  int k;
  double totalX = 0;
  double averageX = 0;
  double totalY = 0;
  double averageY = 0;
  double angle = 0;
  double retVal = 0;
  k = numToAverage - ( 2 * discardNumber );
  for ( j = 0; j < k ; j++) {
    yield();
   totalX += cos((double) ((anglesToAverage[j] * PI) / 180));
   totalY += sin((double) ((anglesToAverage[j] * PI) / 180));
  }
  averageX = totalX / k;
  averageY = totalY / k;
  if ( averageX == 0 && averageY == 0 ) {
    angle = 0;                                 // just to be safe define a value where atan2 is undefined
  } else {
    angle = atan2(averageY , averageX);
    if (angle >= 0) {
      // if the returned value of angle is positive it is a positive rotation (CCW) between 0 and 180 degress
      retVal = (double) ((angle / PI) * 180);
    } else {
      // convert the negative angle to a positive rotation from 0 degrees (CCW)
      retVal =  (double) ((( 2 * PI ) + angle) / PI ) * 180;
    }
  }
  return retVal;
}

// advancedCircularSmooth
// take a new sensor reading, the current advancedCircularSmooth value and an array of past sensor readings
// return a good estimate of the most likely value that the sensor should read by doing the following
//
//   insert the new sensor reading into the oldest position in our array of past sensor readings
//   create a copy of the array for sorting and averaging
//   create an array of indexes that will represent the order of the original indicies in the sorted array
//   for each value in our past sensor values array
//     calculate the distance from the mean and store that value in a new array of (distance from mean)
//   do an insertion sort on this mean distance array and record the transpositions in the index array
//   throw out the bottom 10% and top 10% of values
//   use that array to find the new angular mean.
double advancedCircularSmooth(double newAngle, int axis, double currentCircSmoothValue, double *pastSensorReadings) {
  int j, k;
  int bottom, top;
  static int currentPosition1;  // this is a hack to take into account we are calling this function on two different arrays
  static int currentPosition2;  //
  int currentPosition;
  double tempFloat;
  int tempInt;
  double sortedAngles[numToAverage];
  double sortedDeltas[numToAverage];
  int sortedIndex[numToAverage];
  double anglesToAverage[numToAverage - 2 * discardNumber];
  double currentAverageAngle;
  // increment the couter based on the axis we are smoothing
  if ( axis == 1 ) {
    currentPosition1 = ( currentPosition1 + 1) % numToAverage;
    currentPosition = currentPosition1;
  } else {
    currentPosition2 = ( currentPosition2 + 1) % numToAverage;
    currentPosition = currentPosition2;
  }
  pastSensorReadings[currentPosition] = newAngle;           // put the new value into the array
  yield();
  for ( j = 0; j < numToAverage; j++ ) {
     sortedAngles[j] = pastSensorReadings[j];               // create our array for sorting
     sortedDeltas[j] = angularSeparation(currentCircSmoothValue, pastSensorReadings[j]);
     sortedIndex[j] = j;                                    // create our array of inexes to store sorted order
  }
  // do an insertion sort on the deltas array and apply the transformations to the index array
  // and the sorted Angles array at the same time
  for ( k = 1 ; k < numToAverage; k++){
    // for all but the first element look at the remaining elements till a smaller element is found
    j = k;
    yield();
    while (j > 0 && sortedDeltas[j-1] > sortedDeltas[j]) {
      tempFloat = sortedDeltas[j];
      sortedDeltas[j] = sortedDeltas[j-1];
      sortedDeltas[j-1] = tempFloat;
      tempFloat = sortedAngles[j];
      sortedAngles[j] = sortedAngles[j-1];
      sortedAngles[j-1] = tempFloat;
      tempInt = sortedIndex[j];
      sortedIndex[j] = sortedIndex[j-1];
      sortedIndex[j-1] = tempInt;
      j -= 1;
      yield();
    }
  }
  // now create a smaller array discarding the top and bottom discardNumber values.
  for ( j = discardNumber; j < numToAverage - discardNumber; j++ ) {
    anglesToAverage[j - discardNumber] = sortedAngles[j - discardNumber];
  }
  return (double) circularAverage(anglesToAverage);
}


// convert an angle to tics
unsigned int angleToTics( double angle){
  unsigned int retVal = 0;
  retVal = (unsigned int) (((angle + (increment / 2)) / 360 ) * 16384);
  if ( retVal > 16383 ) {
    retVal = retVal - 16384;
  }
  return retVal;
}

// convert tics to angle
double ticsToAngle ( unsigned int tics) {
  double retVal;
  retVal = tics * increment;
  return retVal;
}

// end forward declares
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//
//  setup
//
void setup() {
  //Initialize serial
  Serial.begin(115200);
  delay(1000);

  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  //hspi = new SPIClass(HSPI);

  #ifdef AP
    Serial.println("Setting up WiFi Access Point");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apssid);
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
  #else
    // attempt to connect to Wifi network:
    while ( status != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);

      // wait 10 seconds for connection:
      delay(5000);
    }
  #endif

  // start the server:
  server.begin();
  // you're connected now, so print out the status:
  #ifdef AP
  #else
    printWifiStatus();
  #endif

  vspi->begin();                                                        // Wake up the buss
  //SPI.setBitOrder(MSBFIRST);                                          // AS5048 is a Most Significant Bit first
  //SPI.setDataMode(SPI_MODE1);                                         // AS5048 uses Mode 1
  // fill up our smoothing arrays with data
  for (i = 1; i <= numToAverage; i++){
    rawData = readTic(azimuthSensorPin);
    advancedCircularSmoothAzimuthAngle = advancedCircularSmooth(ticsToAngle(rawData), 1, advancedCircularSmoothAzimuthAngle, smoothAzimuthAngles);
    rawData = readTic(altitudeSensorPin);
    advancedCircularSmoothAltitudeAngle = advancedCircularSmooth(ticsToAngle(rawData), 2, advancedCircularSmoothAltitudeAngle, smoothAltitudeAngles);
  }
}
//  end setup
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//
// main
//
void loop() {
  // wait for a new client:
  rawData = readTic(azimuthSensorPin);
  rawData &= 0x3FFE; // discard the least significant bit(s)
  advancedCircularSmoothAzimuthAngle = advancedCircularSmooth(ticsToAngle(rawData), 1, advancedCircularSmoothAzimuthAngle, smoothAzimuthAngles);
  #ifdef DEBUGGING
    Serial.print("Raw Angle: ");
    Serial.print(ticsToAngle(rawData));
    Serial.print(" CircSmooth Az: ");
    Serial.print(advancedCircularSmoothAzimuthAngle);
  #endif
  rawData = readTic(altitudeSensorPin);
  rawData &= 0x3FFE; // discard the least significant bit(s)
  advancedCircularSmoothAltitudeAngle = advancedCircularSmooth(ticsToAngle(rawData), 2, advancedCircularSmoothAltitudeAngle, smoothAltitudeAngles);
  #ifdef DEBUGGING
    Serial.print("Raw Angle: ");
    Serial.print(ticsToAngle(rawData));
    Serial.print(" CircSmooth Al: ");
    Serial.println(advancedCircularSmoothAltitudeAngle);
  #endif
  delay(1000);

  WiFiClient thisClient = server.available();
  // when the client sends the first byte, say hello:
  while (thisClient) {
    if (!alreadyConnected) {
      alreadyConnected = true;
    }
    if (thisClient.connected()) {
      if (thisClient.available() > 0) {
        // if there are chars to read....
        // lets print a response and discard the rest of the bytes
        thisClient.print(PadTic(angleToTics(advancedCircularSmoothAzimuthAngle), "+"));
        thisClient.print("\t");
        thisClient.print(PadTic(angleToTics(advancedCircularSmoothAltitudeAngle), "+"));
        thisClient.print("\r\n");
        #ifdef DEBUGGING
          Serial.print("Azimuth tic: ");
          Serial.print(PadTic(angleToTics(advancedCircularSmoothAzimuthAngle), "-"));
          Serial.print(" Altitude tic: ");
          Serial.println(PadTic(angleToTics(advancedCircularSmoothAltitudeAngle), "-"));
        #endif
        // discard remaining bytes
        thisClient.flush();
      }
    }
    else {
      thisClient.stop();
      alreadyConnected = false;
    }
  }
}
//
// end main
//
///////////////////////////////////////////////////////////////////////////////
