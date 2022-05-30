/*
  Interface for Laser cooling system
  - monitoring water flow
  - switching the pump
  - signaling errors
*/

#define VERSION "laser_ctrl_0.31"

#include <OneWire.h>
// #include <DallasTemperature.h>
// #include <Streaming.h>

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <EEPROM.h>

#define FALSE 0
#define TRUE 1

#define I2C_ADDR  0x3c
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SLEEP_DELAY_IN_SECONDS  1
#define TEMPHIST_DEPTH 64
#define MAX_TEMPS 2

// counterhandling
#define PULSE_HIST 64

//---------------------------
// PIN definitions

// data cable connected to D4 pin
#define ONE_WIRE_BUS 4

// flow meter tacho signal
// use HW counter timer0
#define TACHO_INPUT 5

// detect if laser is on
#define LASER_ON_PIN  7

// enable PUMP and COOLER
#define EN_PUMP 11
#define EN_COOL 12

// signal that cooling is ok
#define COOLING_STATUS_OK 8

//---------------------------

#define MIN_FLOW  10    // flow rate we need to detect
#define FLOW_DETECT  2   // seconds we need to see min_flow
#define FLOW_FAIL  5   // seconds we wait to see min_flow

#define RDY_DWELL 15    // max dwelling time before we flush again

//---------------------------

union busAddress {
  byte raw[8];
  long data[2];
};

#define RTC_BLOCK_OFFSET 496

struct dataMem {
  long timestamp;
  long reboots;
};

struct rtcMem {
  struct dataMem data;
  long crc32;
};

//rtcMem rtcCache;

//-----------------------------------------------
volatile unsigned long flowRate = 0;
volatile byte flowDetect = 0;

//volatile unsigned long hist[ PULSE_HIST]
volatile byte pulse_index = 0;

OneWire oneWire(ONE_WIRE_BUS);

byte sensorCount = 0;
busAddress sensorIDs[ MAX_TEMPS];

float temperature[MAX_TEMPS];
static byte in = 0;
char hist[MAX_TEMPS][TEMPHIST_DEPTH];

byte tempAvail = FALSE;

//-----------------------------------------------

#define STATE_NONE  -1
#define STATE_INIT  0
#define STATE_READY 1
#define STATE_PURGE 2
#define STATE_ON    3
#define STATE_ERROR 4

unsigned long state_count = 0;
char state = STATE_NONE;

//-----------------------------------------------

void reboot() {
  Serial.println( "rebooting ...");
  
  //rtcCache.data.reboots++;
  //updateSettings();

  delay( 5000);
}

void safe_delay( unsigned long dly) {
  while( dly >= 1000) {
    //Serial.print( dly);
    //Serial.print( ".");
    //Serial.flush();
    delay( 1000);
    dly -= 1000;
  }
/*
  Serial.print( dly);
  Serial.print( " ");
  Serial.flush();
*/
  if ( dly > 0) delay( dly);
}

//---------------------------

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

byte crcCheck( byte* buf, byte maxIdx) {
  byte crc = OneWire::crc8( buf, maxIdx);
  if ( crc != buf[maxIdx]) {
    Serial.print("crc8");
    Serial.print( maxIdx, DEC);
    Serial.print(" ");
    Serial.print( crc, HEX);
    Serial.print( " != ");
    Serial.println( buf[maxIdx], HEX);
    return TRUE;
  }

  return FALSE;
}

//---------------------------
/*
void updateSettings() {
    rtcCache.crc32 = calculateCRC32( (uint8_t *)&rtcCache.data, sizeof(rtcCache.data));
    EEPROM.put( RTC_BLOCK_OFFSET, rtcCache);
    //ESP.rtcUserMemoryWrite( RTC_BLOCK_OFFSET, (uint32_t*)&rtcCache, sizeof(rtcCache));
}

byte retrieveSettings() {
  //ESP.rtcUserMemoryRead( RTC_BLOCK_OFFSET, (uint32_t*)&rtcCache, sizeof(rtcCache));
  EEPROM.get( RTC_BLOCK_OFFSET, rtcCache);
  uint32_t crcNow = calculateCRC32( (uint8_t *)&rtcCache.data, sizeof(rtcCache.data));
  
  if ( rtcCache.crc32 != crcNow) {
    rtcCache.data.timestamp = 0;
    rtcCache.data.reboots = 0;
#ifdef LOGLEVEL
    Serial.print( "crc error [");
    Serial.print( rtcCache.crc32, 16);
    Serial.print( "] != [");
    Serial.print( crcNow, 16);
    Serial.println( "]. resetting params ...");
#endif
    return FALSE;
  } else {
    return TRUE;
  }
}
*/
//---------------------------

void setupTacho() {
  // reset Timer 1
  TCCR1A = 0;
  TCCR1B = 0;
  // reset Timer 2
  TCCR2A = 0;
  TCCR2B = 0;

  // Timer 1 - counts events on pin D5
  TIMSK1 = 1;   // interrupt on Timer 1 overflow

  // ----------- the gate -----------
  // Timer 2 - gives us our 1 ms counting interval
  // 16 MHz clock (62.5 ns per tick) - prescaled by 1024
  //  counter increments every 64 µs.
  // So we count 125 of them, giving exactly 1000 µs (1 ms)
  TCCR2A = 2;   // CTC mode
  OCR2A  = 156; // count up to 249  (zero relative!!!!)
  // Timer 2 - interrupt on match (ie. every 1 ms)
  TIMSK2 = 2;   // enable Timer2 Interrupt
  // Reset prescalers
  GTCCR = 2;    // reset prescaler now
  // start Timer 2
  TCCR2B = 7 ;  // prescaler of 1024 = 64us = 25
  // set counter-2 to zero
  TCNT2 = 0;
  // ----------- the gate -----------

  // set counter-1 to zero:
  TCNT1 = 0;
  // start Timer 1
  // External clock source on T1 pin (D5). Clock on rising edge.
  TCCR1B = 7;
}

ISR (TIMER1_OVF_vect) {
  // count number of Counter1 overflows
}

//****** gate time *******************************************
/*  Timer2 Interrupt Service is invoked by
    hardware Timer-2 every 1 ms = 1000 Hz
    16Mhz / 128 / 125 = 1000 Hz            */

ISR (TIMER2_COMPA_vect) {
  // calculate total count
  if ( pulse_index >= PULSE_HIST) {
    flowRate = TCNT1;
    TCNT1 = 0;
    pulse_index = 0;
    return;
  }

  pulse_index++;
}

//------------------------------------

void onewireTrigger() {
  byte sensors = 0;

  //Loop through all DS1820
  sensors = 0;
  while ( oneWire.search( sensorIDs[sensors].raw)) {

    //Topic is built from a static String plus the ID of the DS18B20
    if ( crcCheck( sensorIDs[sensors].raw, 7)) {
      continue;
    }
    oneWire.reset();
    
    // the first ROM byte indicates which chip
    switch (sensorIDs[sensors].raw[0]) {
      case 0x28:
        break;
      default:
        Serial.println("skip");
        continue;
    }
    
    oneWire.select( sensorIDs[sensors].raw);
    oneWire.write( 0x44, 1);        // start conversion, with parasite power on at the end
    byte present = oneWire.reset();

    sensors++;

    if (sensors > MAX_TEMPS) break;
  }
  oneWire.reset_search();
  sensorCount = sensors;
}

void onewireRead() {
  byte i, sensor = 0;
  byte data[10];
    
  while ( sensor < sensorCount) {
    oneWire.select( sensorIDs[sensor].raw);
    oneWire.write( 0xBE);         // Read Scratchpad

    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = oneWire.read();
    }
    byte present = oneWire.reset();
    
    if ( crcCheck( data, 8)) {
      continue;
    }

    int16_t raw = (data[1] << 8) | data[0];

    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time

    //convert RAW Temperature to String
    //String raw_temp = String(raw, DEC);
    //convert RAW Temperature to celsius
    float temp = raw * 0.0625;

    if ( /*rtcCache.data.timestamp ||*/ temp != 85.0) {
      // act on temperature
      addTemp( sensor, temp);
    } else {
      Serial.println("skip");
    }
    sensor++;
  }

  //Serial.print( "dev #");
  //Serial.println( devCount);

  //rtcCache.data.timestamp++;
/*
  if ( 0 == (rtcCache.data.timestamp % 60)) {
    updateSettings();
  }
 */
}

//---------------------------

void addTemp( byte sensor, float value) {
  temperature[ sensor] = value;
  hist[sensor][ in] = sensor + (char) value;

  if ( sensor) ++in %= TEMPHIST_DEPTH;
}

void drawTempHist() {
  byte i, low, high;

  for( i=0; i < TEMPHIST_DEPTH; i++) {
    low = hist[0][ (i+in) % TEMPHIST_DEPTH];
    high = hist[1][ (i+in) % TEMPHIST_DEPTH];
    display.drawFastVLine( i+64, 64-high, high - low, SSD1306_WHITE);
  }
}
void drawTemp() {
  char sensorData[6];
  byte i;

  display.setTextColor(SSD1306_WHITE);
  for( i=0; i < MAX_TEMPS; i++) {
    display.setCursor(32, i ? 1 : 56);
    dtostrf(temperature[ i], 4, 2, sensorData);
    display.print( sensorData);
  }
}

void drawFlow() {
  byte level = digitalRead( 10);

  display.drawFastVLine( 63, 0, 64, SSD1306_WHITE);

  display.setCursor( 0,32);             // Start at top-left corner
  display.setTextSize(1);             // Draw 2X-scale text
  
  //display.setTextColor(SSD1306_WHITE);
  //display.println( flowRate);
  display.drawFastHLine( 0, 30, 63, SSD1306_WHITE);
  display.drawPixel( 1 + (MIN_FLOW / 2), 31, SSD1306_WHITE);

  display.drawFastHLine( 0, 32, 1+ min( 62, flowRate / 2), SSD1306_WHITE);
  display.drawFastHLine( 0, 33, 1+ min( 62, flowRate / 2), SSD1306_WHITE);

  display.drawPixel( 1 + (MIN_FLOW / 2), 34, SSD1306_WHITE);
  display.drawFastHLine( 0, 35, 63, SSD1306_WHITE);
  
  display.setCursor(0, 44);
  display.print( level ? "WATER LOW" : "water ok");
}

//-----------------------------------------------

void setupDisplay() {
  Serial.print( "display [");
  Serial.print( I2C_ADDR, 16);
  Serial.println( "]");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if( !display.begin( SSD1306_SWITCHCAPVCC, I2C_ADDR)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setCursor(0,28);             // Start at top-left corner
  display.setTextSize(1);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.println( VERSION);
  display.display();

  delay( 1000);
}

void setup() {
  // setup serial port
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(COOLING_STATUS_OK, OUTPUT);

  pinMode(EN_PUMP, OUTPUT);
  pinMode(EN_COOL, OUTPUT);
  pinMode(LASER_ON_PIN, INPUT);

  Serial.println( VERSION);
/*
  if ( ! retrieveSettings()) {
    updateSettings();
  }
*/
  Wire.begin();
  Wire.setClock(400000L);
/*
  Serial.print( "time [");
  Serial.print( rtcCache.data.timestamp);
  Serial.println( "]");
*/
  // setup OneWire bus
  //DS18B20.begin();
  
  setupDisplay();
  go_none();
  setupTacho();
}

void pump( bool set) {
  digitalWrite(EN_PUMP, set ? HIGH : LOW);
}

void cool( bool set) {
  digitalWrite(EN_COOL, set ? HIGH : LOW);
}

void laser( bool ok) {
  digitalWrite(LED_BUILTIN, ok ? HIGH : LOW);
  digitalWrite(COOLING_STATUS_OK, ok ? HIGH : LOW);
}

bool is_firing() {
  return digitalRead( LASER_ON_PIN);
}

void logState( String pref="") {
  char status[8] = "";
  /*
  Serial.print( "[");
  Serial.print( rtcCache.data.timestamp);
  Serial.print( "] ");
  */
  Serial.print( pref);
  Serial.print( "STATE ");
  Serial.print( state_count);
  Serial.print( "-");
  Serial.print( flowDetect);
  Serial.print( " ");
  switch( state) {
    case STATE_INIT: strcpy( status, "INIT"); break;
    case STATE_READY: strcpy( status, "READY"); break;
    case STATE_PURGE: strcpy( status, "PURGE"); break;
    case STATE_ON: strcpy( status, "ON"); break;
    case STATE_ERROR: strcpy( status, "ERROR"); break;
    case STATE_NONE: strcpy( status, "NONE"); break;
    default:
      sprintf( status, "[%i]", state);
  }
  Serial.println( status);

  display.setCursor(0,16);             // Start at top-left corner
  display.setTextSize(1);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.println( status);
}

//-----------------------------------------------

void loop() {
  if ( tempAvail) {
    onewireRead();
  }
  
  display.clearDisplay();
  display.drawFastVLine( 63, 0, 64, SSD1306_WHITE);
  drawFlow();
  drawTempHist();
  drawTemp();

  onewireTrigger();
  tempAvail = TRUE;

  if ( flowRate <= MIN_FLOW) flowDetect = 0;
  else flowDetect++;
          
  switch( state) {
    case STATE_INIT:
      if (flowDetect >= FLOW_DETECT) {
        go_ready();
      }
      if ( state_count >= FLOW_FAIL) go_error();
    break;

    case STATE_READY:
      if ( is_firing()) {
        go_on();
      } else {
        if ( state_count > RDY_DWELL) {
          go_purge();
        }
      }
    break;

    case STATE_PURGE:
      if ( is_firing()) {
        go_on();
      } else {
        if ( flowDetect >= FLOW_DETECT) go_ready();
        else {
          if (state_count >= FLOW_FAIL) {
            go_error();
          }
        }
      }
    break;

    case STATE_ON:
      if (state_count >= FLOW_FAIL) {
        if ( is_firing()) {
          if ( flowDetect < FLOW_DETECT) {
            go_error();
          }
        } else {
          if ( flowDetect >= FLOW_DETECT) go_ready();
          else go_error();
        }
      }
    break;

    case STATE_ERROR:
      if ( state_count >= FLOW_DETECT) go_init();
    break;

    case STATE_NONE:
    default:
      go_init();
  }
  
  logState();
  state_count++;
  display.display();
    
  safe_delay(1000);     // maybe 750ms is enough, maybe not
}

void go_init() {
  pump( TRUE);
  cool( TRUE);
  laser( FALSE);
  setState( STATE_INIT);
}

void go_ready() {
  pump( FALSE);
  cool( TRUE);
  laser( TRUE);
  setState( STATE_READY);
}

void go_purge() {
  pump( TRUE);
  cool( TRUE);
  laser( TRUE);
  setState( STATE_PURGE);
}
void go_on() {
  pump( TRUE);
  cool( TRUE);
  laser( TRUE);
  setState( STATE_ON);
}

void go_error() {
  pump( FALSE);
  cool( FALSE);
  laser( FALSE);
  setState( STATE_ERROR);
}

void go_none() {
  pump( FALSE);
  cool( FALSE);
  laser( FALSE);
  setState( STATE_NONE);
}

void setState( char st) {
  state = st;
  state_count = 0;
  logState( "=> ");
}
