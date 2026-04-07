#include <Arduino.h>


#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */


#define  LMIC_DEBUG_LEVEL = 1 
#define CFG_au915

#define LORA_GAIN 14
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif


void buildPacket(uint8_t txBuffer[9]);
void do_send(osjob_t* j);
float loopPowerAds(int channel);
void  displayValues();

/*
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
//cbb26990ffbccf7313672a5fa3ec20e3
static const PROGMEM u1_t NWKSKEY[16] = {0xF9, 0xD9, 0xC0, 0xFB, 0xA0, 0x8D, 0x7D, 0x46, 0xC0, 0x30, 0xD7, 0xC8, 0x41, 0x5C, 0x5A, 0x96};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0x8B, 0x03, 0xEB, 0x14, 0xED, 0xEA, 0x42, 0x77, 0xEA, 0x19, 0x58, 0xCB, 0xD2, 0xC4, 0xEE, 0x07};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260DD8FC ; // <-- Change this address for every node!
*/

/*
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
//cbb26990ffbccf7313672a5fa3ec20e3
static const PROGMEM u1_t NWKSKEY[16] = {0x28, 0xF6, 0xFA, 0xA2, 0x63, 0xA8, 0x61, 0x73, 0x95, 0xC3, 0xAA, 0x06, 0x3B, 0x9A, 0x41, 0xCE};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0xFE, 0x93, 0xBF, 0x1D, 0x7A, 0x50, 0x5B, 0x9A, 0x2B, 0xF2, 0x27, 0x0F, 0xD1, 0xAE, 0x62, 0xF2};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260D550F ; // <-- Change this address for every node!
*/
//config for  DHT Lorawan
static const PROGMEM u1_t NWKSKEY[16] = {0x9C, 0x49, 0x24, 0x6B, 0x55, 0xC3, 0x23, 0x1E, 0xDB, 0x36, 0xE4, 0x7C, 0x9B, 0xCA, 0x38, 0x72};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0xB1, 0x20, 0xEC, 0x5F, 0x9B, 0x26, 0x8B, 0xD0, 0xF8, 0x56, 0x38, 0x8B, 0x88, 0xE6, 0x93, 0x61};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260DE74F;


int n_packet=0;
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//static uint8_t mydata[4];
uint8_t txBuffer[36];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60; // 300=5 minutos

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,   // For TTGO 14, T-Beam 23
    .dio = {26, 35, 34}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};


#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

unsigned long startTime= 0;
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET    16 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define DISPLAY 1

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}
int count =5;
void do_send(osjob_t* j) {  
  // Check if there is not a current TX/RX job running
      
  //displayValues();
 
      if (LMIC.opmode & OP_TXRXPEND)
      {
        Serial.println(F("OP_TXRXPEND, not sending"));
        //LoraStatus = "OP_TXRXPEND, not sending";
      }
      else
      { 
          //led.clear();
          //led.drawString(0,0,"fake packet");
          buildPacket(txBuffer);

          n_packet++;
          LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
          Serial.println(F("Packet queued"));
          //LoraStatus = "Packet queued";
      }

  // Next TX is scheduled after TX_COMPLETE event.
  
}


void setupLoRaWAN()
{
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band  
 // LMIC_setupChannel(8, 994000000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  2);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);

#elif defined(CFG_as923)
  // Set up the channels used in your country. Only two are defined by default,
  // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
  // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

  // ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#else
# error Region not supported
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF9, LORA_GAIN);


  // Start job
  do_send(&sendjob);
 
}



float VOLTAGE= 127;
void setup() {
  // Inicializando os comandos no TTGO lora32 (placa usada para ensino na unioeste).
  Serial.begin(115200);
  
  Serial.println("Iniciando");
  Wire.begin(/*sda*/ 4, /*scl*/ 15, 1000000);

  
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(200);
  digitalWrite(OLED_RESET, HIGH);
  

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1.5); 
  display.setTextColor(SSD1306_WHITE);
  displayValues();


  ads.setDataRate(RATE_ADS1115_860SPS);
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);

  setupLoRaWAN();  
  startTime=millis();
}

unsigned long displayTime=0;
float Irms0=0;
float Irms1=0;
float Vrms0=0;
float Vrms1=0;
float accIrms0=0;
float accIrms1=0;
float accVrms0=0;
float accVrms1=0;

int N=0;

void loop()
{
  unsigned long currentTime= millis();
  if (currentTime - displayTime > 10000) {
    //Serial.println(Irms0, 3);
    displayValues();
    displayTime= currentTime;
  }
  Vrms0=loopPowerAds(0);
  Vrms1=loopPowerAds(1);
  
  Irms0= Vrms0*60;
  Irms1= Vrms1*60;
  
  if (Irms0 < 0.025) Irms0=0;
  if (Irms1 < 0.025) Irms1=0;
  
  accVrms0 += Vrms0;
  accVrms1 += Vrms1;
  accIrms0 += Irms0;
  accIrms1 += Irms1;

  N++;
  os_runloop_once();
  currentTime= millis()- currentTime;
  if (currentTime >0)
    delay(currentTime);

}

float KwH0=0;
float KwH1=0;
void buildPacket(uint8_t txBuffer[36])
{
 //temp DHT
  float Imean0= accIrms0/N;
  float Imean1= accIrms1/N;
  unsigned long time=(millis()- startTime);
  if (isnan(KwH0)) KwH0=0;
  if (isnan(KwH1)) KwH1=0; 

  KwH0+= (Imean0*VOLTAGE*(float)(time/1000))/3600;
  KwH1+= (Imean1*VOLTAGE*(float)(time/1000))/3600;
 
  memcpy(&txBuffer[0], &Imean0, sizeof(float));
  memcpy(&txBuffer[4], &Imean1, sizeof(float));  
  memcpy(&txBuffer[8], &time, sizeof(time/1000));
  memcpy(&txBuffer[16], &KwH0, sizeof(float));
  memcpy(&txBuffer[20], &KwH1, sizeof(float));
  memcpy(&txBuffer[24], &N, sizeof(int));
  memcpy(&txBuffer[28], &accVrms0, sizeof(float));
  memcpy(&txBuffer[32], &accVrms1, sizeof(int));

  

  Serial.print("Lora Send I0= ");
  Serial.print(Imean0, 3);
  Serial.print("   I1=");
  Serial.println(Imean1,3);
  Serial.print("   kwH0=");
  Serial.println(KwH0,3);
  Serial.print("   kwH1=");
  Serial.println(KwH1,3);
  

  accVrms0=0;
  accVrms1=0;
  accIrms0=0;
  accIrms1=0;
  N=0;
  startTime=millis();



}


void displayValues() {
  // tempo de espera entre cada leitura.
  // Ler o valor da temperatura e imprimir esse valor no monitor serial.
  //led.clear();
 
   // led.drawString(0,0,"pacote: ");
   // led.drawString(5,0, n_packet);    
     display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Irms0: ");
  display.print(Irms0, 3);
  display.println(" A ");
 // display.setCursor(10, 30);
  display.print("Potencia0: ");
  display.print(Irms0*VOLTAGE, 2);
  display.println(" VA");
  display.print("Irms1: ");
  display.print(Irms1, 3);
  display.println(" A ");
 // display.setCursor(10, 30);
  display.print("Potencia1: ");
  display.print(Irms1*VOLTAGE, 2);
  display.println(" VA");
  display.display();

 
}

const float VREF = 3.3;        // tensão do ESP32
const int ADC_RES = 4095;      // 12 bits
float loopPowerAds(int channel) {
  const int N = 165 ;  // número de amostras aprox. 250ms

  float sum = 0;
  float offset = 0;
  //Serial.println("Start loop ads");
  // medir offset médio
  for (int i = 0; i < N; i++) {
   // offset +=  ads.getLastConversionResults();
    offset +=  ads.readADC_SingleEnded(channel);
    //Serial.print(offset);
    
  }
  offset = offset / N;
  Serial.println(offset);
  // 2 calcular RMS
  
  int init = micros();
  float sample=0;
  for (int i = 0; i < N; i++) {
    float sample =  ads.readADC_SingleEnded(channel) - offset;
  /*  if (sample < 50) 
       sample=0;
    else {
      Serial.print(sample); Serial.print(" ");
    }*/
    
    //float voltage = (sample * VREF) / ADC_RES;
   float voltage=  ads.computeVolts(sample);
   sum += voltage * voltage;
  }
  int end= micros()-init;

  /*Serial.print("time ");
  Serial.println(end);
  Serial.println(sum, 6);
*/
  float Vrms = sqrt(sum / N);


  return Vrms;


}
