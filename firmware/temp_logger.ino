
//to get M0 ports use this reference
//https://github.com/arduino/ArduinoCore-samd/blob/master/variants/arduino_mzero/variant.cpp


#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <Adafruit_MAX31865.h>

#if defined(ARDUINO_FEATHER_ESP32) // Feather Huzzah32
  #define TFT_CS         14
  #define TFT_RST        15
  #define TFT_DC         32

#elif defined(ESP8266)
  #define TFT_CS         4
  #define TFT_RST        16                                            
  #define TFT_DC         5

#else
  // For the breakout board, you can use any 2 or 3 pins.
  // These pins will also work for the 1.8" TFT shield.
  #define TFT_CS        8 //PA06
  #define TFT_RST       32 // PA28 // Or set to -1 and connect to Arduino RESET pin
  #define TFT_DC         20 //PB11
  #define TFT_MOSI       27//  PA04
  #define TFT_SCLK       28//  PA05


  #define MAX_A0       25//  PB08
  #define MAX_A1       26//  PB09
  #define MAX_A2       21//  PB10


#endif


// Use software SPI: CS, DI, DO, CLK // PA17 PA18 PA16 PA19
Adafruit_MAX31865 thermo = Adafruit_MAX31865(13, 10, 11, 12);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK,TFT_RST);
  // int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst)

// 
float p = 3.1415926;


#include <Ethernet.h>
#include "wiring_private.h" // pinPeripheral() function
// SPI
//#define PIN_SPI_MISO  (2u) //PA08
//#define PIN_SPI_MOSI  (1u) // PA10
//#define PIN_SPI_SCK   (0u) // PA11
#define PIN_SPI_SS    (3u) // PA09
#define ETH_CS  3 //PA09
#define PERIPH_SPI           sercom2
#define ETH_RESETn  24 //PA02

#define SS_PIN_DEFAULT  3
//  SPIClassSAMD(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI, SercomSpiTXPad, SercomRXPad);
SPIClassSAMD ETH_SPI (&sercom2, 2, 0, 1, SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0);
SPIClassSAMD MAX_SPI (&sercom1, 2, 0, 1, SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0);

uint8_t Eth_buf[16];



	
	inline static void setSS() {
		digitalWrite(ETH_CS, LOW);
	}
	inline static void resetSS() {
		digitalWrite(ETH_CS, HIGH);
}

uint16_t W5500_read(uint16_t addr, uint8_t *buf, uint16_t len)
{
	uint8_t cmd[4];

		setSS();
		if (addr < 0x100) {
			// common registers 00nn
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = 0x00;
      SerialUSB.println("addr < 0x100");
		} else if (addr < 0x8000) {
			// socket registers  10nn, 11nn, 12nn, 13nn, etc
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = ((addr >> 3) & 0xE0) | 0x08;
       SerialUSB.println("addr < 0x8000");
		} else if (addr < 0xC000) {
			// transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
			//  10## #nnn nnnn nnnn
      SerialUSB.println("addr < 0xC000");
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x10;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x10; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x10; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x10; // 2K buffers
			#endif
		} else {
        SerialUSB.println("else");
			// receive buffers
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x18;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x18; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x18; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x18; // 2K buffers
			#endif
		}
    SerialUSB.println("lets transfer  CMD over SPI");
		//ETH_SPI.transfer(cmd, 3);
    	for (uint16_t i=0; i < 3; i++) {
				ETH_SPI.transfer(cmd[i]);
			}
    SerialUSB.println("CMD transfer done");
		memset(buf, 0, len);
    SerialUSB.println("memset done");
		//ETH_SPI.transfer(buf, len);
      for (uint16_t i=0; i < len; i++) {
				ETH_SPI.transfer(buf[i]);
			}
      SerialUSB.println("transfer done");
		resetSS();
	
	return len;
}


uint16_t W5500_write(uint16_t addr, const uint8_t *buf, uint16_t len)
{
	uint8_t cmd[8];

		setSS();
		if (addr < 0x100) {
			// common registers 00nn
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = 0x04;
		} else if (addr < 0x8000) {
			// socket registers  10nn, 11nn, 12nn, 13nn, etc
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = ((addr >> 3) & 0xE0) | 0x0C;
		} else if (addr < 0xC000) {
			// transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
			//  10## #nnn nnnn nnnn
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x14;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x14; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x14; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x14; // 2K buffers
			#endif
		} else {
			// receive buffers
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x1C;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x1C; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x1C; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x1C; // 2K buffers
			#endif
		}
		if (len <= 5) {
			for (uint8_t i=0; i < len; i++) {
				cmd[i + 3] = buf[i];
			}
			ETH_SPI.transfer(cmd, len + 3);
		} else {
			ETH_SPI.transfer(cmd, 3);
#ifdef SPI_HAS_TRANSFER_BUF
			ETH_SPI.transfer(buf, NULL, len);
#else
			// TODO: copy 8 bytes at a time to cmd[] and block transfer
			for (uint16_t i=0; i < len; i++) {
				ETH_SPI.transfer(buf[i]);
			}
#endif
		}
		resetSS();
	
	return len;
}




void setup(void) {
  while (!SerialUSB) ;
  SerialUSB.begin(9600);





  // Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab

  SerialUSB.println("Initialized");

  uint16_t time = millis();
  time = millis() - time;

  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.println("Hello World!");



  ETH_SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));


  //mySPI.begin();
  // Assign pins 11, 12, 13 to SERCOM functionality
  pinPeripheral(2, PIO_SERCOM_ALT);  //PIO_SERCOM_ALT for sercom2
  pinPeripheral(1, PIO_SERCOM_ALT);
  pinPeripheral(0, PIO_SERCOM_ALT);
  pinPeripheral(3, PIO_SERCOM_ALT);

  pinMode(ETH_RESETn, OUTPUT);
  pinMode(ETH_CS, OUTPUT);
  digitalWrite(ETH_RESETn, 0);
  delay(100); 
  digitalWrite(ETH_RESETn, 1);


 pinMode(MAX_A0, OUTPUT);
 pinMode(MAX_A1, OUTPUT);
 pinMode(MAX_A2, OUTPUT);
 digitalWrite(MAX_A0, 0);
 digitalWrite(MAX_A1, 0);
 digitalWrite(MAX_A2, 0);

  SerialUSB.println("Adafruit MAX31865 PT100 Sensor Test!");
  thermo.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
   
digitalWrite(MAX_A0, 1);
 digitalWrite(MAX_A1, 0);
 digitalWrite(MAX_A2, 0);

thermo.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary

digitalWrite(MAX_A0, 0);
 digitalWrite(MAX_A1, 1);
 digitalWrite(MAX_A2, 0);

thermo.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary


  SerialUSB.println("reading ETH chip");
  W5500_read(0, Eth_buf, 16);

  for (int i = 0; i < 16; i++) SerialUSB.print(Eth_buf[i], HEX);
  delay(100);
  SerialUSB.println("reading ETH done");






}

void loop() {

  digitalWrite(MAX_A0, 0);
 digitalWrite(MAX_A1, 0);
 digitalWrite(MAX_A2, 0);
  uint16_t rtd = thermo.readRTD();

  SerialUSB.print("RTD value0: "); SerialUSB.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  SerialUSB.print("Ratio0 = "); SerialUSB.println(ratio,8);
  SerialUSB.print("Resistance0 = "); SerialUSB.println(RREF*ratio,8);
  SerialUSB.print("Temperature0 = "); SerialUSB.println(thermo.temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    SerialUSB.print("Fault 0x"); SerialUSB.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      SerialUSB.println("RTD High Threshold0"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      SerialUSB.println("RTD Low Threshold0"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      SerialUSB.println("REFIN- > 0.85 x Bias0"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      SerialUSB.println("REFIN- < 0.85 x Bias - FORCE- open0"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      SerialUSB.println("RTDIN- < 0.85 x Bias - FORCE- open0"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      SerialUSB.println("Under/Over voltage0"); 
    }
    thermo.clearFault();
  }
  SerialUSB.println();
  delay(1000);


digitalWrite(MAX_A0, 1);
 digitalWrite(MAX_A1, 0);
 digitalWrite(MAX_A2, 0);
  rtd = thermo.readRTD();

  SerialUSB.print("RTD value1: "); SerialUSB.println(rtd);
  ratio = rtd;
  ratio /= 32768;
  SerialUSB.print("Ratio1 = "); SerialUSB.println(ratio,8);
  SerialUSB.print("Resistance1 = "); SerialUSB.println(RREF*ratio,8);
  SerialUSB.print("Temperature1= "); SerialUSB.println(thermo.temperature(RNOMINAL, RREF));

  // Check and print any faults
   fault = thermo.readFault();
  if (fault) {
    SerialUSB.print("Fault 0x"); SerialUSB.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      SerialUSB.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      SerialUSB.println("RTD Low Threshold1"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      SerialUSB.println("REFIN- > 0.85 x Bias1"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      SerialUSB.println("REFIN- < 0.85 x Bias - FORCE- open1"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      SerialUSB.println("RTDIN- < 0.85 x Bias - FORCE- open1"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      SerialUSB.println("Under/Over voltage1"); 
    }
    thermo.clearFault();
  }
  SerialUSB.println();
  delay(1000);


      SerialUSB.println("toggling SPI ETH");
    while(1)
    {
      
      digitalWrite(ETH_CS, 0);
      ETH_SPI.transfer16(0x5571);
      digitalWrite(ETH_CS, 1);
      ETH_SPI.endTransaction();
    }

}







void tftPrintTest() {
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(1500);
  tft.setCursor(0, 0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(p, 6);
  tft.println(" Want pi?");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(ST77XX_WHITE);
  tft.println("Sketch has been");
  tft.println("running for: ");
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print(millis() / 1000);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(" seconds.");
}


