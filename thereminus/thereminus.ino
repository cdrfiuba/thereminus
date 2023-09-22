#include <avr/pgmspace.h>
#include "samples.h"

enum playingModes {CONTINUOUS, QUANTIZED, SAMPLES};
playingModes playingMode = CONTINUOUS;

enum scales {CHROMATIC, C_PENTATONIC, A_MAJOR};
scales quantizedModeScale = CHROMATIC;

const byte INITIAL_NOTE_JUMP_DISTANCE = 3;
byte noteJumpDistance = INITIAL_NOTE_JUMP_DISTANCE;

const int PIN_BUTTON = 9;
const int PIN_LED = 13;
const int MIN_DISTANCE = 1;
const int MAX_DISTANCE = 100;
const byte CONTINUOUS_MODE_VOLUME = 8; // from 0 to 15
const byte QUANTIZED_MODE_VELOCITY = 64; // from 0 to 127
const int CONTINUOUS_MODE_MAX_FREQ = 2400;
const int HYSTERESIS_IN_CM = 3;
const int MIDI_BASE_NOTE = 57;

byte rangeUp;
byte rangeDown;
byte segment = 0;
byte previousSegment = 0;
byte midiNote = 0;
int num_notes = 0;

char debugStringBuffer[60];
bool debugMode = true;
// sprintf + serial of 20 bytes takes ~200us
// sprintf + serial of 10 bytes takes ~144us
// sprintf + serial of  5 bytes takes ~108us
#define serialDebug(...) \
  if (debugMode) { \
    sprintf(debugStringBuffer, __VA_ARGS__); \
    Serial.print(debugStringBuffer); \
  }

// for led cube
const int COLUMN1_PIN = A0;
const int COLUMN2_PIN = A1;
const int COLUMN3_PIN = A2;
const int COLUMN4_PIN = A3;
const int COLUMN5_PIN = A4;
const int COLUMN6_PIN = A5;
const int COLUMN7_PIN = 2;
const int COLUMN8_PIN = 3;
const int COLUMN9_PIN = 4;
const int ROW1_PIN = 5;
const int ROW2_PIN = 6;
const int ROW3_PIN = 7;

#define ColumnOn(N) digitalWrite(COLUMN ## N ## _PIN, LOW);
#define ColumnOff(N) digitalWrite(COLUMN ## N ## _PIN, HIGH);
#define RowOn(N) digitalWrite(ROW ## N ## _PIN, LOW);
#define RowOff(N) digitalWrite(ROW ## N ## _PIN, HIGH);
#define ColumnsOff() ColumnOff(1);ColumnOff(2);ColumnOff(3);ColumnOff(4);ColumnOff(5);ColumnOff(6);ColumnOff(7);ColumnOff(8);ColumnOff(9);
#define LEN(a) (uint16_t)(sizeof a / sizeof *a)

/**
 * from cdrfiuba/picopico
 * */

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define NUM_VOICES         3
#define DEFAULT_OCTAVE     4
#define DEFAULT_VOL        15
#define DEFAULT_PW         0xA0

// Note frequencies for 8th octave (highest):
//   freqs = [4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902]
//   [int((2**16 / 20000.0) * f) for f in freqs]
const uint16_t scale[] = {13716, 14532, 15397, 16311, 17281, 18310, 19398, 20552, 21774, 23068, 24441, 25893};

// Volume amplitude lookup table (16 entries)
const uint8_t amp[] = {0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240};

// Noise period counter (12 entries)
const uint16_t noisePeriods[] = {1200, 1760, 2360, 2960, 3760, 4720, 7080, 9440, 14160, 18880, 37800, 65535};

enum Waveform { PULSE, SAW, TRI, NOISE, SAMPLE };

struct Voice {
  // Player-related registers
  uint8_t   note;                   // Note (0-11)
  uint8_t   octave;                 // Octave (0-7)
  uint8_t   volume;                 // Volume (0-15)

  // Internal registers for sound generation
  volatile Waveform waveform;
  volatile int16_t  acc;            // Phase accumulator
  volatile uint16_t freq;           // Frequency delta
  volatile uint8_t  amp;            // Amplitude
  volatile int8_t   pw;             // Pulse width
  volatile const void* sample;      // pointer to sample
  volatile uint16_t sample_size;    // size of the sample array
  volatile uint16_t sample_counter; // counter for the current index of the sample
  volatile uint16_t sample_tick;    // counter for correcting the sample rate
  volatile bool sample_must_play;   // flag for starting the playback of a sample
  volatile bool sample_is_playing;  // flag for playing back each byte of the sample
};

// Note buffer
volatile uint16_t lfsr = 1;
volatile char lfsrOut = 0;
volatile signed char oldTemp = 0; // FIXME change variable name

// Global tick counter
volatile uint16_t timer0_tick = 0;
volatile uint8_t ticks = 0;
volatile bool nextTick = false;
volatile bool nextBigTick = false;

Voice voices[NUM_VOICES] = {};

ISR(TIMER0_COMPA_vect) {
  sound_generation();
}
void sound_generation() {
  // tick counter for time keeping
  timer0_tick++;
  if (timer0_tick > 111) { // 333 = 16.66ms, 111 = 5.55ms
    timer0_tick = 0;
    // if the tick counter fires before the last tick was completed,
    // while still playing, toggle the led
    //if (nextTick && playing) {
    //    digitalWrite(PIN_LED, abs(digitalRead(PIN_LED) - 1));
    //}
    // ticks and big ticks (every 1 second)
    nextTick = true;
    ticks++;
    if (ticks > 90) { // 5.55ms * 180 = 1000ms
      ticks = 0;
      nextBigTick = true;
      digitalWrite(PIN_LED, abs(digitalRead(PIN_LED) - 1));
    }
  }

  // inspired by http://www.technoblogy.com/show?2E6L
  unsigned char temp;
  signed char stemp, mask, out = 0;
  Voice* v;
  for (int i = 0; i < NUM_VOICES; i++) {
    v = &voices[i];
    switch (v->waveform) {
      case PULSE:
        v->acc += v->freq;
        temp = (v->acc >> 8) & v->pw;
        out += (temp ? v->amp : 0) >> 2;
        break;
      case TRI:
        v->acc += v->freq / 2;
        stemp = v->acc >> 8;
        mask = stemp >> 7;
        if (v->amp != 0) {
          if (v->volume > 12) {  // 13 14 15
            out += (stemp ^ mask) >> 1;
          } else if (v->volume > 8) {  // 9 10 11 12
            out += (stemp ^ mask) >> 2;
          } else if (v->volume > 4) {  // 5 6 7 8
            out += (stemp ^ mask) >> 3;
          } else if (v->volume > 0) {  // 1 2 3 4
            out += (stemp ^ mask) >> 4;
          }
        }
        break;
      case SAW:
        v->acc += v->freq;
        temp = v->acc >> 8;
        out += (temp ? v->amp : 0) >> 2;
        break;
      case NOISE:
        v->acc += v->freq * 8;
        stemp = (v->acc >> 8) & 0x80;
        // if temp != oldTemp, trigger the LFSR to generate a new pseudorandom value
        if (stemp != oldTemp) {
          lfsrOut = (lfsr & 1) ^ ((lfsr & 2) >> 1);  // output is bit 0 XOR bit 1
          lfsr = (lfsr >> 1) | (lfsrOut << 14);      // shift and include output on bit 15
          oldTemp = stemp;
        }
        out += (lfsrOut ? v->amp : 0) >> 2;
        break;
      case SAMPLE:
        if (v->sample_must_play == true) {
          v->sample_must_play = false;
          v->sample_counter = 0;
          v->sample_is_playing = true;
        }
        if (v->sample_is_playing) {
          v->sample_counter++;
          out += pgm_read_byte_near((char*)v->sample + v->sample_counter - 1);
          if (v->sample_counter >= v->sample_size - 1) {
            v->sample_is_playing = false;
            v->sample_counter = 0;
          }
        } else {
          out += 128;
        }
        break;
      }
  }

  // notes on the noise channels:
  // This noise generator is somewhat based on the mechanism found in the NES APU.
  // The NES has a linear-feedback shift register for generating pseudorandom numbers.
  // It starts with a register set to 1, and when the period counter reaches 0, it
  // clocks the shift register.
  // The LFSR performs an Exclusive OR between bit 0 and bit 1, then shifts to the
  // right, and sets/resets bit 15 based on the exclusive OR result.

  OCR1B = out;
}

/**
 * from dreamster/dreamster-lib
 * */

// global sonars for use in ISR
enum SonarStates {
  SONAR_BEGIN_TRIGGER = 'T',
  SONAR_WAITING_ECHO = 'W',
  SONAR_MEASURING_PING = 'M',
  SONAR_COOLDOWN = 'C',
  SONAR_STANDBY = 'S',
};
struct Sonar {
  unsigned int distance = 0;
  char state = SONAR_STANDBY;
  unsigned int ticks = 0;
  unsigned int cooldown_ticks = 0;
  int trigger_pin;
  int echo_pin;
};
enum Sonars {
  A = 0,
  B = 1,
  C = 2,
  NUM_SONARS = 1,
};
Sonar sonar[NUM_SONARS];
int next_sonar = A;

void sonar_timer_callback() {
  bool all_sonars_in_standby = true;
  // sonars state machine
  for (int i = 0; i < NUM_SONARS; i++) {
    switch (sonar[i].state) {
      case SONAR_BEGIN_TRIGGER:
        // trigger ping and move to next state
        digitalWrite(sonar[i].trigger_pin, HIGH);
        delayMicroseconds(5); // needs 10us in HIGH, and digitalWrite takes 5us
        digitalWrite(sonar[i].trigger_pin, LOW);
        sonar[i].state = SONAR_WAITING_ECHO;
      break;
      case SONAR_WAITING_ECHO:
        // awaiting start of ping reflection (echo)
        sonar[i].ticks++;
        if (digitalRead(sonar[i].echo_pin) == HIGH) {
          // measure first tick and move to next state
          sonar[i].ticks = 1;
          sonar[i].state = SONAR_MEASURING_PING;
        }
        // timeout, shouldn't take longer than 400us (8 cycles of a 40KHz burst),
        // plus some overhead
        if (sonar[i].ticks > 16) { // 464us
          sonar[i].ticks = 1;
          sonar[i].state = SONAR_MEASURING_PING;
        }
      break;
      case SONAR_MEASURING_PING:
        // count ticks until the echo pin is low
        if (digitalRead(sonar[i].echo_pin) == HIGH) {
          sonar[i].ticks++;
        } else {
          // when pin is low, count each tick as 1 cm, and move to next state
          sonar[i].distance = sonar[i].ticks / 2;
          sonar[i].cooldown_ticks = (sonar[i].cooldown_ticks + (sonar[i].ticks * 8)) / 2;
          sonar[i].ticks = 0;
          sonar[i].state = SONAR_COOLDOWN;
        }
        // timeout
        if (sonar[i].ticks > 800) { // 23ms
          sonar[i].distance = 0;
          sonar[i].cooldown_ticks = (sonar[i].cooldown_ticks + (sonar[i].ticks * 8)) / 2;
          sonar[i].ticks = 0;
          sonar[i].state = SONAR_COOLDOWN;
        }
        break;
      case SONAR_COOLDOWN:
        // wait for a while before triggering next ping
        sonar[i].ticks++;
        if (sonar[i].ticks > sonar[i].cooldown_ticks) { // 380 ticks = 11ms
          sonar[i].ticks = 0;
          sonar[i].state = SONAR_STANDBY;
        }
      break;
      case SONAR_STANDBY:
        // wait
      break;
    }
    if (sonar[i].state != SONAR_STANDBY) {
      all_sonars_in_standby = false;
    }
  }

  // if all sonars are on standby, proceed to ping the next sonar
  if (all_sonars_in_standby) {
    sonar[next_sonar].state = SONAR_BEGIN_TRIGGER;
    next_sonar++;
    if (next_sonar == NUM_SONARS) next_sonar = A;
  }
}
ISR(TIMER3_COMPA_vect) {
  sonar_timer_callback();
}

bool isButtonPressed (int pinButton, int buttonPressedValue = LOW, bool waitForRelease = false) {
  static int buttonPressed = false;
    
  // detect button press (and optionally wait for release)
  if (digitalRead(pinButton) == buttonPressedValue) {
    if (buttonPressed) return false;
    buttonPressed = true;

    // _delay_ms(100); // debounce
    if (waitForRelease) {
      while (digitalRead(pinButton) == buttonPressedValue) {
        // wait for release
        _delay_ms(100);
      }
      _delay_ms(10);
      // button releases, continue
    } else {
      if (digitalRead(pinButton) == buttonPressedValue) {
        return true;
      }
    }
    // confirmed that button was pressed
    return true;
  }
  buttonPressed = false;
  return false;
}
void ledsBy10Segments(int segment) {
  RowOn(1); RowOn(2); RowOn(3);
  ColumnsOff();
  switch (segment) {
    case 1: ColumnOn(1) break;
    case 2: ColumnOn(2) break;
    case 3: ColumnOn(3) break;
    case 4: ColumnOn(4) break;
    case 5: ColumnOn(5) break;
    case 6: ColumnOn(6) break;
    case 7: ColumnOn(7) break;
    case 8: ColumnOn(8) break;
    case 9: ColumnOn(9) break;
  }
}
void ledsBy27Segments(int segment) {
  RowOff(1); RowOff(2); RowOff(3);
  ColumnsOff();
  switch (segment) {
    case 1: RowOn(1); ColumnOn(1); break;
    case 2: RowOn(1); ColumnOn(2); break;
    case 3: RowOn(1); ColumnOn(3); break;
    case 4: RowOn(1); ColumnOn(4); break;
    case 5: RowOn(1); ColumnOn(5); break;
    case 6: RowOn(1); ColumnOn(6); break;
    case 7: RowOn(1); ColumnOn(7); break;
    case 8: RowOn(1); ColumnOn(8); break;
    case 9: RowOn(1); ColumnOn(9); break;

    case 10: RowOn(2); ColumnOn(1); break;
    case 11: RowOn(2); ColumnOn(2); break;
    case 12: RowOn(2); ColumnOn(3); break;
    case 13: RowOn(2); ColumnOn(4); break;
    case 14: RowOn(2); ColumnOn(5); break;
    case 15: RowOn(2); ColumnOn(6); break;
    case 16: RowOn(2); ColumnOn(7); break;
    case 17: RowOn(2); ColumnOn(8); break;
    case 18: RowOn(2); ColumnOn(9); break;

    case 19: RowOn(3); ColumnOn(1); break;
    case 20: RowOn(3); ColumnOn(2); break;
    case 21: RowOn(3); ColumnOn(3); break;
    case 22: RowOn(3); ColumnOn(4); break;
    case 23: RowOn(3); ColumnOn(5); break;
    case 24: RowOn(3); ColumnOn(6); break;
    case 25: RowOn(3); ColumnOn(7); break;
    case 26: RowOn(3); ColumnOn(8); break;
    case 27: RowOn(3); ColumnOn(9); break;
  }
}
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  cli();

  // from dreamster/dreamster-lib
  sonar[A].trigger_pin = 0;
  sonar[A].echo_pin = 1;
  // sonar[B].trigger_pin = 2;
  // sonar[B].echo_pin = 3;
  // sonar[C].trigger_pin = 4;
  // sonar[C].echo_pin = 5;

  for (int i = 0; i < NUM_SONARS; i++) {
    pinMode(sonar[i].trigger_pin, OUTPUT);
    pinMode(sonar[i].echo_pin, INPUT);
  }

  // set timer 3 to interrupt every 58us
  TIMSK3 &= ~(1<<OCIE3A);        // Disable Timer3 interrupt
  TCCR3A = 0;                    // Set Timer3 prescaler to 8, CTC mode.
  TCCR3B = (1<<CS31 | 1<<WGM32); // Set Timer3 prescaler to 8, CTC mode.
  OCR3A = 57;                    // Set interrupt every 58us (115)
  TIMSK3 |= (1<<OCIE3A);         // Enable Timer3 interrupt.

  // from cdrfiuba/picopico
  // TCCR1A => COM1A1 COM1A0 COM1B1 COM1B0 – – WGM11 WGM10
  // TCCR1B => ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10

  // Set up Timer/Counter1 for PWM output
  sbi(TCCR1B, CS10); // 1:1 prescale
  cbi(TCCR1B, CS11);
  cbi(TCCR1B, CS12);
  sbi(TCCR1A, WGM10); // Fast PWM Mode, 8bit
  cbi(TCCR1A, WGM11);
  sbi(TCCR1B, WGM12);
  cbi(TCCR1B, WGM13);
  cbi(TCCR1A, COM1B0); // Clear OC1B on Compare Match
  sbi(TCCR1A, COM1B1);

  OCR1B = 128;
  pinMode(10, OUTPUT);            // Enable PWM output pin

  // Set up Timer/Counter0 for 20kHz interrupt to output samples.
  // The resulting frequency is calculated similar to the CTC mode.
  // F_CPU / (2 * PRESCALER * (1 + OCR0A))
  TCCR0A = 0;
  TCCR0B = 0;
  sbi(TCCR0A, WGM00);
  sbi(TCCR0A, WGM01);
  sbi(TCCR0B, WGM02); // Fast PWM
  sbi(TCCR0B, CS01); // 1/8 prescale
  // OCR0A = 49; // Divide by 800 (40KHz)
  OCR0A = 99; // Divide by 1600 (20KHz)

  // On Timer0, enable timer compare match, disable overflow
  TIMSK0 = 1 << OCIE0A | 0 << TOIE0;

  // voice initialization
  for (int i = 0; i < NUM_VOICES; i++) {
    voices[i].octave = DEFAULT_OCTAVE;
    voices[i].volume = DEFAULT_VOL;
    voices[i].pw = DEFAULT_PW;
  }
  // voices
  voices[0].waveform = PULSE;
  voices[1].waveform = PULSE;
  voices[2].waveform = PULSE;

  /* led cube config */
  pinMode(COLUMN1_PIN, OUTPUT);
  pinMode(COLUMN2_PIN, OUTPUT);
  pinMode(COLUMN3_PIN, OUTPUT);
  pinMode(COLUMN4_PIN, OUTPUT);
  pinMode(COLUMN5_PIN, OUTPUT);
  pinMode(COLUMN6_PIN, OUTPUT);
  pinMode(COLUMN7_PIN, OUTPUT);
  pinMode(COLUMN8_PIN, OUTPUT);
  pinMode(COLUMN9_PIN, OUTPUT);
  pinMode(ROW1_PIN, OUTPUT);
  pinMode(ROW2_PIN, OUTPUT);
  pinMode(ROW3_PIN, OUTPUT);
  RowOff(1); RowOff(2); RowOff(3);
  ColumnsOff();

  sei();
}

void loop() {
  if (!nextTick) {
    return;   
  }
  nextTick = false;

  if (playingMode == CONTINUOUS) {
    // play sound according to distance, focusing on a small octave range, 
    // but with a lot of precision, to emulate a true theremin
    for (int i = 0; i < NUM_SONARS; i++) {
      voices[i].waveform = PULSE;
      voices[i].amp = 0;
      RowOff(1); RowOff(2); RowOff(3);
      ColumnsOff();
      if (sonar[i].distance < MIN_DISTANCE || sonar[i].distance > MAX_DISTANCE) continue;
      int mapped_distance = map(sonar[i].distance, MIN_DISTANCE, MAX_DISTANCE, 1, 27);
      ledsBy27Segments(mapped_distance);
      
      voices[i].amp = amp[CONTINUOUS_MODE_VOLUME];
      // multiplying the usable range (from 0 to 60, or from 0 to 100) by 16 
      // provides approximately one octave of continuous sound, going from 
      // around 250Hz to 500Hz when setting the base freq to 800
      int32_t freq = CONTINUOUS_MODE_MAX_FREQ - (sonar[i].distance) * 16;
      if (freq < 0) freq = 0;
      voices[i].freq = (uint16_t)freq;
      
      // reduce volume if far
      int distanceFromMax = MAX_DISTANCE - sonar[i].distance;
      if (distanceFromMax < CONTINUOUS_MODE_VOLUME) {
        voices[i].amp = amp[distanceFromMax];
      }
      serialDebug("%.4d %.3d\n",voices[i].freq, sonar[i].distance);
    }
  }

  if (playingMode == QUANTIZED) {
    // play sound according to distance
    for (int i = 0; i < NUM_SONARS; i++) {
      voices[i].waveform = PULSE;
      // when outside the range, mute sound and proceed to next sonar
      if (sonar[i].distance < MIN_DISTANCE || sonar[i].distance > MAX_DISTANCE) {
        RowOff(1); RowOff(2); RowOff(3);
        ColumnsOff();
        voices[i].amp = 0;
        continue;
      }

      // set hysteresis as a range going from bottom to top or from top to bottom.
      // each zone determines the current note, which can only be changed
      // when surpassing the hysteresis zone
      // >> 11111111 22222222 33333333 44444444 >>
      //    --1-- ?? --2-- ?? --3-- ?? --4-- ??
      // << 11111 22222222 33333333 44444444 55 <<
      
      if (quantizedModeScale == CHROMATIC) {
        num_notes = 24;
        rangeUp = ceil(sonar[i].distance * num_notes / MAX_DISTANCE);
        rangeDown = ceil((sonar[i].distance + HYSTERESIS_IN_CM) * num_notes / MAX_DISTANCE);
        if (rangeUp == rangeDown || abs(segment - previousSegment) >= noteJumpDistance) {
          segment = rangeUp;
        }
        previousSegment = rangeUp;
        midiNote = segment + MIDI_BASE_NOTE;

        // leds
        ledsBy10Segments(segment - 7);
      }
      if (quantizedModeScale == C_PENTATONIC) {
        num_notes = 10;
        rangeUp = ceil(sonar[i].distance * num_notes / MAX_DISTANCE);
        rangeDown = ceil((sonar[i].distance + HYSTERESIS_IN_CM) * num_notes / MAX_DISTANCE);

        if (rangeUp == rangeDown || abs(segment - previousSegment) >= noteJumpDistance) {
          segment = rangeUp;
        }
        previousSegment = segment;
        const int NOTES[num_notes + 1] = {
          57, 60, 62, 65, 67, 69, 72, 74, 77, 79, 81
        };
        midiNote = NOTES[segment];

        ledsBy10Segments(segment);
      }
      if (quantizedModeScale == A_MAJOR) {
        num_notes = 24;
        rangeUp = ceil(sonar[i].distance * num_notes / MAX_DISTANCE);
        rangeDown = ceil((sonar[i].distance + HYSTERESIS_IN_CM) * num_notes / MAX_DISTANCE);
        if (rangeUp == rangeDown || abs(segment - previousSegment) >= noteJumpDistance) {
          segment = rangeUp;
        }
        previousSegment = segment;
        const int NOTES[num_notes + 1] = {
          42, 44, 45, 47, 49, 50, 52, 
          54, 56, 57, 59, 61, 62, 64, 
          66, 68, 69, 71, 73, 74, 76,
          78, 80, 81, 83
        };
        midiNote = NOTES[segment];

        // leds
        ledsBy27Segments(segment);
      }
      serialDebug("%.2d %.2d %.2d %.3d\n", rangeUp, rangeDown, midiNote, sonar[i].distance);
      
      // voice/note configuration
      voices[i].octave = ((midiNote - midiNote % 12) / 12) - 1;
      voices[i].note = ((midiNote % 12) + 2) - 2;
      voices[i].volume = QUANTIZED_MODE_VELOCITY / 8;
      if (voices[i].volume > 15) voices[i].volume = 15;
      voices[i].amp = amp[voices[i].volume];
      voices[i].freq = scale[voices[i].note] >> (8 - (voices[i].octave % 8));
    }
  }
  if (playingMode == SAMPLES) {
    for (int i = 0; i < NUM_SONARS; i++) {
      if (sonar[i].distance < MIN_DISTANCE || sonar[i].distance > MAX_DISTANCE) {
        RowOff(1); RowOff(2); RowOff(3);
        ColumnsOff();
        previousSegment = 0;
        continue;
      }

      num_notes = 10;
      rangeUp = ceil(sonar[i].distance * num_notes / MAX_DISTANCE);
      rangeDown = ceil((sonar[i].distance + HYSTERESIS_IN_CM) * num_notes / MAX_DISTANCE);
      if (rangeUp == rangeDown || abs(segment - previousSegment) >= noteJumpDistance) {
        segment = rangeUp;
      }

      voices[i].waveform = SAMPLE;
      if (segment != previousSegment) {
        if (segment == 2) {
          serialDebug("kick\n");
          voices[i].sample = kick;
          voices[i].sample_size = LEN(kick);
          voices[i].sample_must_play = true;
        }
        if (segment == 3) {
          serialDebug("hihat\n");
          voices[i].sample = hihat;
          voices[i].sample_size = LEN(hihat);
          voices[i].sample_must_play = true;
        }
        if (segment == 4) {
          serialDebug("snare\n");
          voices[i].sample = snare;
          voices[i].sample_size = LEN(snare);
          voices[i].sample_must_play = true;
        }
      }
      serialDebug("%.2d %.2d %.3d\n", segment, previousSegment, sonar[i].distance);
      if (segment > 0) previousSegment = segment;

      // leds
      ColumnsOff();
      if (segment >= 2 && segment <= 4) {
        ledsBy10Segments(segment - 1);
      }
    }
  }

  if (isButtonPressed(PIN_BUTTON, LOW, true)) {
    serialDebug("new playing mode: ");
    if (playingMode == CONTINUOUS) {
      playingMode = QUANTIZED;
      quantizedModeScale = CHROMATIC;
      serialDebug("quantized - chromatic");
    } else if (playingMode == QUANTIZED && quantizedModeScale == CHROMATIC) {
      playingMode = QUANTIZED;
      quantizedModeScale = C_PENTATONIC;
      serialDebug("quantized - C pentatonic");
    } else if (playingMode == QUANTIZED && quantizedModeScale == C_PENTATONIC) {
      playingMode = QUANTIZED;
      quantizedModeScale = A_MAJOR;
      serialDebug("quantized - A major");
    } else if (playingMode == QUANTIZED && quantizedModeScale == A_MAJOR) {
      playingMode = SAMPLES;
      serialDebug("samples");
    } else if (playingMode == SAMPLES) {
      playingMode = CONTINUOUS;
      serialDebug("continuous");
    }
//    if (playingMode == CONTINUOUS) {
//      playingMode = QUANTIZED;
//      quantizedModeScale = C_PENTATONIC;
//      serialDebug("quantized - C pentatonic");
//    } else if (playingMode == QUANTIZED && quantizedModeScale == C_PENTATONIC) {
//      playingMode = CONTINUOUS;
//      serialDebug("continuous");
//    }
    serialDebug("\n");
  }
  
  // when not sending serial data, wait a very small time,
  // or else loop() doesn't run
  if (!debugMode) {
    _delay_us(1);
  }
  
}
