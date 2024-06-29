
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "tones.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Standard Arduino Pins
#define digitalPinToPortReg(P) \
	(((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define digitalPinToDDRReg(P) \
	(((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define digitalPinToPINReg(P) \
	(((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define digitalPinToBit(P) \
	(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))

#define digitalReadFast(P) bitRead(*digitalPinToPINReg(P), digitalPinToBit(P))

#define digitalWriteFast(P, V) bitWrite(*digitalPinToPortReg(P), digitalPinToBit(P), (V))

const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//--------- Ringbuf parameters ----------
uint8_t Ringbuffer[256];
uint8_t RingWrite = 0;
uint8_t RingRead = 0;
volatile uint8_t RingCount = 0;

const int LFilterPin = A4;
const int VolPin = A3;
float vol = 1.0;
float alpha = 0.1;
int16_t filteredOutput = 0;

//-----------------------------------------
ISR(TIMER1_COMPA_vect)
{

	//-------------------  Ringbuffer handler -------------------------

	if (RingCount)
	{ // If entry in FIFO..
		int16_t rawOutput = Ringbuffer[RingRead++];
		//filteredOutput = (alpha * rawOutput) + ((1 - alpha) * filteredOutput);
		//amplifiedOutput = filteredOutput * vol;
		
		OCR2A = rawOutput * 0.5; // Output LSB of 16-bit DAC
		
		//uncomment this for the amplified out
		//OCR2A = amplifiedOutput;
		RingCount--;
	}

	//-----------------------------------------------------------------
}

// Define the array of arrays
const uint8_t *const Tones[8] PROGMEM = {tone_0, tone_1, tone_2, tone_3, tone_4, tone_5, tone_6, tone_7};
int buttonPins[8] = {2, 3, 4, 5, 6, 7, 8, 9}; // or byte?
bool currentButtonStates[8] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};
bool lastButtonStates[8] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
const int pitchPin = A0;
uint8_t phaccs[8];
uint8_t pitches[8] = {1, 1, 1, 1, 1, 1, 1, 1};
uint16_t samplelens[8] = {1014, 1014, 1014, 1014, 1014, 1014, 1014, 1014};
uint16_t samplecnts[8];
uint16_t samplepnts[8];

uint8_t oldPORTB;
uint8_t oldPORTD;

int16_t total;
uint8_t divider;
uint8_t MUX = 0;
char ser = ' ';

void setup()
{
  Serial.begin(9600);
	OSCCAL = 0xFF;

	// Drumtrigger inputs
	for (int i = 0; i < 8; i++)
	{ // byte
		pinMode(buttonPins[i], INPUT_PULLUP);
	}

	pinMode(pitchPin, INPUT);
	// pinMode(LFilterPin, INPUT);

	// 8-bit PWM DAC pin
	pinMode(11, OUTPUT);

	// Set up Timer 1 to send a sample every interrupt.
	cli();
	// Set CTC mode
	// Have to set OCR1A *after*, otherwise it gets reset to 0!
	TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
	TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
	// No prescaler
	TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
	// Set the compare register (OCR1A).
	// OCR1A is a 16-bit register, so we have to do this with
	// interrupts disabled to be safe.
	// OCR1A = F_CPU / SAMPLE_RATE;
	// Enable interrupt when TCNT1 == OCR1A
	TIMSK1 |= _BV(OCIE1A);
	OCR1A = 400; // 40KHz Samplefreq

	// Set up Timer 2 to do pulse width modulation on D11

	// Use internal clock (datasheet p.160)
	ASSR &= ~(_BV(EXCLK) | _BV(AS2));

	// Set fast PWM mode  (p.157)
	TCCR2A |= _BV(WGM21) | _BV(WGM20);
	TCCR2B &= ~_BV(WGM22);

	// Do non-inverting PWM on pin OC2A (p.155)
	// On the Arduino this is pin 11.
	TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
	TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
	// No prescaler (p.158)
	TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

	// Set initial pulse width to the first sample.
	OCR2A = 128;

	// set timer0 interrupt at 61Hz
	TCCR0A = 0; // set entire TCCR0A register to 0
	TCCR0B = 0; // same for TCCR0B
	TCNT0 = 0;	// initialize counter value to 0
	// set compare match register for 62hz increments
	OCR0A = 255; // = 61Hz
	// turn on CTC mode
	TCCR0A |= (1 << WGM01);
	// Set CS01 and CS00 bits for prescaler 1024
	TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00); // 1024 prescaler

	TIMSK0 = 0;

	// set up the ADC
	ADCSRA &= ~PS_128; // remove bits set by Arduino library
	// Choose prescaler PS_128.
	ADCSRA |= PS_128;
	ADMUX = 64;
	sbi(ADCSRA, ADSC);
	

	sei();
}

void loop()
{
    // int value = analogRead(LFilterPin);
    // alpha = map(value, 0, 1023, 1, 99) / 100.0;
	//------ Add current sample word to ringbuffer FIFO --------------------
  for (int i = 0; i < 8; i++) 
  {
    if (currentButtonStates[0] == LOW)
   { 
    //Serial.print(0);
        Serial.println(" pressed");}

            }
	if (RingCount < 255)
	{ // if space in ringbuffer
		total = 0;

		// we can't wrap the following a for loop, for some reason when we do the samples get corrupted, or we have side-effects/compiler optimising issues
		if (currentButtonStates[0] == LOW)
		{
			phaccs[0] += pitches[0];
			if (phaccs[0] & 128)
			{
				phaccs[0] &= 127;
				samplepnts[0]++;
			}
			total += (pgm_read_byte_near(Tones[0] + samplepnts[0]) - 128);
		}

		if (currentButtonStates[1] == LOW)
		{
			phaccs[1] += pitches[1];
			if (phaccs[1] & 128)
			{
				phaccs[1] &= 127;
				samplepnts[1]++;
			}
			total += (pgm_read_byte_near(Tones[1] + samplepnts[1]) - 128);
		}

		if (currentButtonStates[2] == LOW)
		{
			phaccs[2] += pitches[2];
			if (phaccs[2] & 128)
			{
				phaccs[2] &= 127;
				samplepnts[2]++;
			}
			total += (pgm_read_byte_near(Tones[2] + samplepnts[2]) - 128);
		}

		if (currentButtonStates[3] == LOW)
		{
			phaccs[3] += pitches[3];
			if (phaccs[3] & 128)
			{
				phaccs[3] &= 127;
				samplepnts[3]++;
			}
			total += (pgm_read_byte_near(Tones[3] + samplepnts[3]) - 128);
		}

		if (currentButtonStates[4] == LOW)
		{
			phaccs[4] += pitches[4];
			if (phaccs[4] & 128)
			{
				phaccs[4] &= 127;
				samplepnts[4]++;
			}
			total += (pgm_read_byte_near(Tones[4] + samplepnts[4]) - 128);
		}

		if (currentButtonStates[5] == LOW)
		{
			phaccs[5] += pitches[5];
			if (phaccs[5] & 128)
			{
				phaccs[5] &= 127;
				samplepnts[5]++;
			}
			total += (pgm_read_byte_near(Tones[5] + samplepnts[5]) - 128);
		}

		if (currentButtonStates[6] == LOW)
		{
			phaccs[6] += pitches[6];
			if (phaccs[6] & 128)
			{
				phaccs[6] &= 127;
				samplepnts[6]++;

			}
			total += (pgm_read_byte_near(Tones[6] + samplepnts[6]) - 128);
		}

		if (currentButtonStates[7] == LOW)
		{
			phaccs[7] += pitches[7];
			if (phaccs[7] & 128)
			{
				phaccs[7] &= 127;
				samplepnts[7]++;
			}
			total += (pgm_read_byte_near(Tones[7] + samplepnts[7]) - 128);
		}

		total >>= 1;
		if (!(PINB & 4))
			total >>= 1;
		total += 128;
		if (total > 255)
			total = 255;

            for (int i = 0; i < 8; i++) {
            currentButtonStates[i] = digitalRead(buttonPins[i]);}
        //     if (lastButtonStates[i] == HIGH && currentButtonStates[i] == LOW) {
        //         trigger(i);
        //     }
        //     lastButtonStates[i] = currentButtonStates[i];
        // }

		cli();
		Ringbuffer[RingWrite] = total;
		RingWrite++;
		RingCount++;
		sei();
	}}
