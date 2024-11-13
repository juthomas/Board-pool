#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <Adafruit_AHTX0.h>
#include "RTClib.h"
#include <PCA95x5.h>

#define PIN_PB3 11
#define PIN_PB5 13

#define PIN_PB0 8
#define PIN_PB1 9
#define PIN_PB2 10
#define PIN_PB4 12

#define PIN_PD3 3
#define PIN_PD5 5
#define PIN_PD6 6
#define PIN_PD2 2
#define PIN_PD4 4

#define PIN_PC0 14
#define PIN_PC1 15
#define PIN_PC2 16
#define PIN_PC3 17

#define NUMPIXELS  3
#define DATAPIN    PIN_PB3
#define CLOCKPIN   PIN_PB5

#define LED_D1     PIN_PB0
#define LED_D2     PIN_PB1
#define LED_D3     PIN_PB2
#define LED_D4     PIN_PB4

#define LED_RGB_R  PIN_PD3
#define LED_RGB_G  PIN_PD5
#define LED_RGB_B  PIN_PD6

#define BUT_1      PIN_PD2
#define BUT_2      PIN_PD4

#define ADC_POT    PIN_PC0
#define ADC_LDR    PIN_PC1
#define ADC_NTC    PIN_PC2

//A. pin11
//B. pin7
//C. pin4
//D. pin2
//E. pin1
//F. pin10
//G. pin5
//DP pin3

//CAT0 12
//CAT1 9
//CAT2 8
//CAT3 6

#define SEG_A     PCA95x5::Port::P06
#define SEG_B     PCA95x5::Port::P16
#define SEG_C     PCA95x5::Port::P13
#define SEG_D     PCA95x5::Port::P11
#define SEG_E     PCA95x5::Port::P10
#define SEG_F     PCA95x5::Port::P05
#define SEG_G     PCA95x5::Port::P14
#define SEG_DP    PCA95x5::Port::P12

#define CAT_0     PCA95x5::Port::P07
#define CAT_1     PCA95x5::Port::P04
#define CAT_2     PCA95x5::Port::P17
#define CAT_3     PCA95x5::Port::P15

#define BUT_3     PCA95x5::Port::P00
#define LEDS_D11  PCA95x5::Port::P01
#define LEDS_D10  PCA95x5::Port::P02
#define LEDS_D09  PCA95x5::Port::P03

#define NUMBER_0 ((1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C) | (1 << SEG_D) | (1 << SEG_E) | (1 << SEG_F))
#define NUMBER_1 ((1 << SEG_B) | (1 << SEG_C))
#define NUMBER_2 ((1 << SEG_A) | (1 << SEG_B) | (1 << SEG_D) | (1 << SEG_E) | (1 << SEG_G))
#define NUMBER_3 ((1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C) | (1 << SEG_D) | (1 << SEG_G))
#define NUMBER_4 ((1 << SEG_B) | (1 << SEG_C) | (1 << SEG_F) | (1 << SEG_G))
#define NUMBER_5 ((1 << SEG_A) | (1 << SEG_C) | (1 << SEG_D) | (1 << SEG_F) | (1 << SEG_G))
#define NUMBER_6 ((1 << SEG_A) | (1 << SEG_C) | (1 << SEG_D) | (1 << SEG_E) | (1 << SEG_F) | (1 << SEG_G))
#define NUMBER_7 ((1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C))
#define NUMBER_8 ((1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C) | (1 << SEG_D) | (1 << SEG_E) | (1 << SEG_F) | (1 << SEG_G))
#define NUMBER_9 ((1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C) | (1 << SEG_D) | (1 << SEG_F) | (1 << SEG_G))


#define DIGIT_4 ((1 << CAT_0) | (1 << CAT_1) | (1 << CAT_2)) // DIGIT_1
#define DIGIT_3 ((1 << CAT_0) | (1 << CAT_1) | (1 << CAT_3)) // DIGIT_2
#define DIGIT_2 ((1 << CAT_0) | (1 << CAT_2) | (1 << CAT_3)) // DIGIT_3
#define DIGIT_1 ((1 << CAT_1) | (1 << CAT_2) | (1 << CAT_3)) // DIGIT_4

enum MODE {
	MODE_CTN,
	MODE_TIME,
	MODE_POT,
	MODE_LDR,
	MODE_NTC,
	MODE_AHT20_TEMP,
};

 #define USE_AHT20

Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);
RTC_PCF8563 rtc;
PCA9555 ioex;
Adafruit_AHTX0 aht;

uint16_t  segs[] = { NUMBER_0, NUMBER_1, NUMBER_2, NUMBER_3, NUMBER_4, NUMBER_5, NUMBER_6, NUMBER_7, NUMBER_8, NUMBER_9 };
uint16_t digits[4];
uint8_t  sw1_pressed = 0;
uint8_t  mode        = 0;
uint8_t  i           = 0;
uint32_t bright      = 30;
uint8_t  ctn         = 0;
uint32_t timer       = 0;
int      test        = 0;

struct pixel {
	uint8_t g;
	uint8_t r;
	uint8_t b;
};

struct pixel rgb2uint32(byte r, byte g, byte b){
	struct pixel p = {g,r,b};
	return p;
}

struct pixel Wheel(byte WheelPos) {
	if(WheelPos < 85) {
		return rgb2uint32(WheelPos * 3, 255 - WheelPos * 3, 0);
	} else if(WheelPos < 170) {
		WheelPos -= 85;
		return rgb2uint32(255 - WheelPos * 3, 0, WheelPos * 3);
	} else {
		WheelPos -= 170;
		return rgb2uint32(0, WheelPos * 3, 255 - WheelPos * 3);
	}
}


void setup() {
	Serial.begin(115200);

	pinMode(LED_D1, OUTPUT);
	pinMode(LED_D2, OUTPUT);
	pinMode(LED_D3, OUTPUT);
	pinMode(LED_D4, OUTPUT);

	digitalWrite(LED_D1, HIGH);
	digitalWrite(LED_D2, HIGH);
	digitalWrite(LED_D3, HIGH);
	digitalWrite(LED_D4, HIGH);

	pinMode(LED_RGB_R, OUTPUT);
	pinMode(LED_RGB_G, OUTPUT);
	pinMode(LED_RGB_B, OUTPUT);

	pinMode(BUT_1, INPUT);
	pinMode(BUT_2, INPUT);

	pinMode(ADC_POT, INPUT);
	pinMode(ADC_LDR, INPUT);
	pinMode(ADC_NTC, INPUT);


	strip.begin();
	strip.show();
	Wire.begin();

	// Initialisation du RTC avec limite de tentatives
	int rtc_attempts = 0;
	const int MAX_RTC_ATTEMPTS = 10;
	while (!rtc.begin()) {
		Serial.println("Couldn't find RTC");
			delay(1000);
			rtc_attempts++;
			if (rtc_attempts >= MAX_RTC_ATTEMPTS) {
				Serial.println("RTC initialization failed, proceeding without RTC");
				break;
			}
	}

	if (!digitalRead(BUT_1) || rtc.lostPower()) {
		Serial.println("RTC is NOT initialized, let's set the time!");
			rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
	}
	rtc.start();
	Serial.println("RTC OK");

	// Initialisation de l'AHT20 avec limite de tentatives
	#ifdef USE_AHT20
		int aht_attempts = 0;
		const int MAX_AHT_ATTEMPTS = 10;
		while(!aht.begin()) {
			Serial.println("Couldn't find AHT");
			delay(1000);
			aht_attempts++;
			if (aht_attempts >= MAX_AHT_ATTEMPTS) {
				Serial.println("AHT20 initialization failed, proceeding without AHT20");
				break;
			}
		}
		if (aht_attempts < MAX_AHT_ATTEMPTS) {
			Serial.println("AHT20 OK");
		}
	#endif

	ioex.attach(Wire);
	ioex.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
	ioex.direction(PCA95x5::Direction::OUT_ALL);
	ioex.direction(BUT_3, PCA95x5::Direction::IN);
	ioex.write(PCA95x5::Level::L_ALL);

	Serial.println("Setup terminé.");
}

void set_number(uint32_t v) {
	digits[0] = DIGIT_1 | (segs[v / 1000 % 10]) | (1 << LEDS_D09) | (1 << LEDS_D10) | (1 << LEDS_D11);
	digits[1] = DIGIT_2 | (segs[v / 100 % 10]) | (1 << LEDS_D09) | (1 << LEDS_D10) | (1 << LEDS_D11);
	digits[2] = DIGIT_3 | (segs[v / 10 % 10] ) | (1 << LEDS_D09) | (1 << LEDS_D10) | (1 << LEDS_D11);
	digits[3] = DIGIT_4 | (segs[v % 10] );
}

void loop() {
	// check APA102

	struct pixel p = Wheel(i+0);
	strip.setPixelColor(0, p.r * bright / 255, p.g * bright / 255, p.b * bright / 255);
	p = Wheel(i+30);
	strip.setPixelColor(1, p.r * bright / 255, p.g * bright / 255, p.b * bright / 255);
	p = Wheel(i+60);
	strip.setPixelColor(2, p.r * bright / 255, p.g * bright / 255, p.b * bright / 255);
	strip.show();


	// check RGB LED
	p = Wheel(i+90);
	analogWrite(3, p.r * bright / 255);
	analogWrite(5, p.g * bright / 255);
	analogWrite(6, p.b * bright / 255);


	// check 7seg

	ioex.write((1 << CAT_0) | (1 << CAT_1) | (1 << CAT_2) | (1 << CAT_3) | (1 << LEDS_D09) | ( 1 << LEDS_D10) | ( 1 << LEDS_D11));
	delayMicroseconds(1);

	ioex.write(digits[ctn]);
	ctn = (ctn+1)%4;
	delayMicroseconds(200);

	if (timer < millis()) {
		switch (mode) {
			case MODE_CTN:
				set_number(test++);
				break;
			case MODE_TIME:
			{
				DateTime now = rtc.now();
				set_number(now.hour()*100+now.minute());
				break;
			}
			case MODE_POT:
				set_number(analogRead(ADC_POT));
				break;
			case MODE_LDR:
				set_number(analogRead(ADC_LDR));
				break;
			case MODE_NTC:
				set_number(analogRead(ADC_NTC));
				break;
			#ifdef USE_AHT20
				case MODE_AHT20_TEMP:
					sensors_event_t humidity, temp;
					aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
					Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
					Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
					// Serial.println(temp.temperature);
					set_number(temp.temperature);
          break;
			#endif
			default:
				mode = 0;
				break;
		}

		// check i2c expander input

		// if (ioex.read(BUT_3))
		// 	digitalWrite(LED_D3, HIGH);
		// else
		// 	digitalWrite(LED_D3, LOW);

		timer = millis() + 50; // wait 1 seconds
	}

	// check buttons

	if (!sw1_pressed && !digitalRead(BUT_1)) {
		mode++;
		sw1_pressed = 1;
		delay(20);
	} else {
		if (sw1_pressed && digitalRead(BUT_1)) {
			sw1_pressed = 0;
		}
	}
	
	if (digitalRead(BUT_2))
		digitalWrite(LED_D2, HIGH);
	else
		digitalWrite(LED_D2, LOW);

	uint32_t v = analogRead(ADC_LDR);
	bright = map(v, 0, 1024, 5, 12);
	i += 1;
}
