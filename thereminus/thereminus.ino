enum playingModes {CONTINUOUS, QUANTIZED};
const byte INITIAL_PLAYING_MODE = CONTINUOUS;
byte playingMode = INITIAL_PLAYING_MODE;

enum scales {CHROMATIC, C_PENTATONIC, A_MAJOR};
const byte INITIAL_QUANTIZED_MODE_SCALE = CHROMATIC;
byte quantizedModeScale = INITIAL_QUANTIZED_MODE_SCALE;

const byte INITIAL_NOTE_JUMP_DISTANCE = 2;
byte noteJumpDistance = INITIAL_NOTE_JUMP_DISTANCE;

const int PIN_BUTTON = 9;
const int PIN_LED = 13;
const int MIN_DISTANCE = 1;
const int MAX_DISTANCE = 100;
const byte CONTINUOUS_MODE_VOLUME = 8; // from 0 to 15
const byte QUANTIZED_MODE_VELOCITY = 64; // from 0 to 127
const int CONTINUOUS_MODE_MAX_FREQ = 2400;
const int HYSTERESIS_IN_CM = 3;
const int NOTE_DIVISIONS = 10;
const int NUM_NOTES = ceil((MAX_DISTANCE - MIN_DISTANCE) / NOTE_DIVISIONS);
const int MIDI_BASE_NOTE = 57;

byte rangeUp;
byte rangeDown;
byte midiNote = 0;
byte previousMidiNote = 0;

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
int cubeled_state = 0;

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

enum Waveform { PULSE, SAW, TRI, NOISE };

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
    if (timer0_tick > 111) { // 333 = 16.65ms, 111 = 5.55ms
        timer0_tick = 0;
        // if the tick counter fires before the last tick was completed,
        // while still playing, toggle the led
        //if (nextTick && playing) {
        //    digitalWrite(PIN_LED, abs(digitalRead(PIN_LED) - 1));
        //}
        // ticks and big ticks (every 1 second)
        nextTick = true;
        ticks++;
        if (ticks > 27) { // 5.55ms * 27 = 149.85ms
            ticks = 0;
            nextBigTick = true;
            digitalWrite(PIN_LED, abs(digitalRead(PIN_LED) - 1));
        }
    }

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
                    if (v->volume > 12) { // 13 14 15
                        out += (stemp ^ mask) >> 1;
                    } else if (v->volume > 8) { // 9 10 11 12
                        out += (stemp ^ mask) >> 2;
                    } else if (v->volume > 4) { // 5 6 7 8
                        out += (stemp ^ mask) >> 3;
                    } else if (v->volume > 0) { // 1 2 3 4
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
    Voice* v = &voices[i];
    v->octave = DEFAULT_OCTAVE;
    v->volume = DEFAULT_VOL;
    v->pw = DEFAULT_PW;
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
      RowOff(1); RowOff(2); RowOff(3);
      ColumnsOff();
      voices[i].amp = 0;
      if (sonar[i].distance < MIN_DISTANCE || sonar[i].distance > MAX_DISTANCE) continue;
      int mapped_distance = map(sonar[i].distance, MIN_DISTANCE, MAX_DISTANCE, 1, 27);
      if (mapped_distance == 1) { RowOn(1); ColumnOn(1);}
      if (mapped_distance == 2) { RowOn(1); ColumnOn(2);}
      if (mapped_distance == 3) { RowOn(1); ColumnOn(3);}
      if (mapped_distance == 4) { RowOn(1); ColumnOn(4);}
      if (mapped_distance == 5) { RowOn(1); ColumnOn(5);}
      if (mapped_distance == 6) { RowOn(1); ColumnOn(6);}
      if (mapped_distance == 7) { RowOn(1); ColumnOn(7);}
      if (mapped_distance == 8) { RowOn(1); ColumnOn(8);}
      if (mapped_distance == 9) { RowOn(1); ColumnOn(9);}

      if (mapped_distance == 10) {RowOn(2); ColumnOn(1);}
      if (mapped_distance == 11) {RowOn(2); ColumnOn(2);}
      if (mapped_distance == 12) {RowOn(2); ColumnOn(3);}
      if (mapped_distance == 13) {RowOn(2); ColumnOn(4);}
      if (mapped_distance == 14) {RowOn(2); ColumnOn(5);}
      if (mapped_distance == 15) {RowOn(2); ColumnOn(6);}
      if (mapped_distance == 16) {RowOn(2); ColumnOn(7);}
      if (mapped_distance == 17) {RowOn(2); ColumnOn(8);}
      if (mapped_distance == 18) {RowOn(2); ColumnOn(9);}

      if (mapped_distance == 19) {RowOn(3); ColumnOn(1);}
      if (mapped_distance == 20) {RowOn(3); ColumnOn(2);}
      if (mapped_distance == 21) {RowOn(3); ColumnOn(3);}
      if (mapped_distance == 22) {RowOn(3); ColumnOn(4);}
      if (mapped_distance == 23) {RowOn(3); ColumnOn(5);}
      if (mapped_distance == 24) {RowOn(3); ColumnOn(6);}
      if (mapped_distance == 25) {RowOn(3); ColumnOn(7);}
      if (mapped_distance == 26) {RowOn(3); ColumnOn(8);}
      if (mapped_distance == 27) {RowOn(3); ColumnOn(9);}
      
      Voice* v = &voices[i];

      v->amp = amp[CONTINUOUS_MODE_VOLUME];
      // multiplying the usable range (from 0 to 60, or from 0 to 100) by 16 
      // provides approximately one octave of continuous sound, going from 
      // around 250Hz to 500Hz when setting the base freq to 800
      int32_t freq = CONTINUOUS_MODE_MAX_FREQ - (sonar[i].distance) * 16;
      if (freq < 0) freq = 0;
      v->freq = (uint16_t)freq;
      
      // reduce volume if far
      int distanceFromMax = MAX_DISTANCE - sonar[i].distance;
      if (distanceFromMax < CONTINUOUS_MODE_VOLUME) {
        v->amp = amp[distanceFromMax];
      }
      serialDebug("%.4d %.2d\n", v->freq, sonar[i].distance);
    }
  }

  if (playingMode == QUANTIZED) {
    // play sound according to distance
    for (int i = 0; i < NUM_SONARS; i++) {
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
      
      // set range that skips the hysteresis to allow for faster response times
      if (quantizedModeScale == CHROMATIC) noteJumpDistance = 2;

      rangeUp = ceil(sonar[i].distance / NOTE_DIVISIONS);
      rangeDown = ceil((sonar[i].distance + HYSTERESIS_IN_CM) / NOTE_DIVISIONS);
      /*
      if (rangeUp == rangeDown) {
        midiNote = MIDI_BASE_NOTE + rangeUp;
      }
      if (abs(midiNote - previousMidiNote) >= noteJumpDistance) {
        midiNote = MIDI_BASE_NOTE + rangeUp;
      }
      previousMidiNote = MIDI_BASE_NOTE + rangeUp;
      */
      if (quantizedModeScale == CHROMATIC) {
        //  no restrictions
      }
      if (quantizedModeScale == C_PENTATONIC) {
        /*
        RowOn(1); RowOn(2); RowOn(3);
        midiNote = map(sonar[i].distance, MIN_DISTANCE, MAX_DISTANCE, 9, 0);
        */
        if (rangeUp == rangeDown) {
          midiNote = rangeUp;
        }
        if (midiNote == 0) {midiNote = 57; ColumnsOff();}
        if (midiNote == 1) {midiNote = 60; ColumnsOff(); ColumnOn(1);}
        if (midiNote == 2) {midiNote = 62; ColumnsOff(); ColumnOn(2);}
        if (midiNote == 3) {midiNote = 65; ColumnsOff(); ColumnOn(3);}
        if (midiNote == 4) {midiNote = 67; ColumnsOff(); ColumnOn(4);}
        if (midiNote == 5) {midiNote = 69; ColumnsOff(); ColumnOn(5);}
        if (midiNote == 6) {midiNote = 72; ColumnsOff(); ColumnOn(6);}
        if (midiNote == 7) {midiNote = 74; ColumnsOff(); ColumnOn(7);}
        if (midiNote == 8) {midiNote = 77; ColumnsOff(); ColumnOn(8);}
        if (midiNote == 9) {midiNote = 79; ColumnsOff(); ColumnOn(9);}
        
      }
      if (quantizedModeScale == A_MAJOR) {
        const int index = map(sonar[i].distance, MIN_DISTANCE, MAX_DISTANCE, NUM_NOTES - 1, 0);
        /*const int NOTES[NUM_NOTES] = {
          42, 44, 45, 47, 49, 50, 52, 
          54, 56, 57, 59, 61, 62, 64, 
          66, 68, 69, 71, 73, 74, 76,
          78, 80, 81
        };
        midiNote = NOTES[index];*/
      }
      serialDebug("%.2d %.2d %.2d %.2d\n", rangeUp, rangeDown, midiNote, sonar[i].distance);
      
      // voice/note configuration
      Voice* v = &voices[i];
      v->octave = ((midiNote - midiNote % 12) / 12) - 1;
      v->note = ((midiNote % 12) + 2) - 2;
      v->volume = QUANTIZED_MODE_VELOCITY / 8;
      if (v->volume > 15) v->volume = 15;
      v->amp = amp[v->volume];
      v->freq = scale[v->note] >> (8 - (v->octave % 8));
    }
  }

  // for (int i = 0; i < NUM_SONARS; i++) {
    // serialDebug("%.2d %.4d ", i, sonar[i].distance);
  // }
  // serialDebug("\n");
  
  if (isButtonPressed(PIN_BUTTON, LOW, true)) {
    serialDebug("new playing mode: ");
    // if (playingMode == CONTINUOUS) {
      // playingMode = QUANTIZED;
      // quantizedModeScale = CHROMATIC;
      // serialDebug("quantized - chromatic");
    // } else if (playingMode == QUANTIZED && quantizedModeScale == CHROMATIC) {
      // playingMode = QUANTIZED;
      // quantizedModeScale = C_PENTATONIC;
      // serialDebug("quantized - C pentatonic");
    // } else if (playingMode == QUANTIZED && quantizedModeScale == C_PENTATONIC) {
      // playingMode = QUANTIZED;
      // quantizedModeScale = A_MAJOR;
      // serialDebug("quantized - A major");
    // } else if (playingMode == QUANTIZED && quantizedModeScale == A_MAJOR) {
      // playingMode = CONTINUOUS;
      // serialDebug("continuous");
    // }
    if (playingMode == CONTINUOUS) {
      playingMode = QUANTIZED;
      quantizedModeScale = C_PENTATONIC;
      serialDebug("quantized - C pentatonic");
    } else if (playingMode == QUANTIZED && quantizedModeScale == C_PENTATONIC) {
      playingMode = CONTINUOUS;
      serialDebug("continuous");
    }
    serialDebug("\n");
  }
  
  // when not sending serial data, wait a very small time,
  // or else loop() doesn't run
  if (!debugMode) {
    _delay_us(1);
  }
  
  // cubeled debug, from cdrfiuba/pcbdesarrollo
  /*if (nextBigTick) {
      nextBigTick = false;
      cubeled_state++;
      if (cubeled_state == 1) {         ColumnOn(1); 
                                        RowOff(2); RowOff(3); RowOn(1);
      } else if (cubeled_state == 2) {  ColumnOff(1); ColumnOn(2);
      } else if (cubeled_state == 3) {  ColumnOff(2); ColumnOn(3);
      } else if (cubeled_state == 4) {  ColumnOff(3); ColumnOn(6);
      } else if (cubeled_state == 5) {  ColumnOff(6); ColumnOn(5);
      } else if (cubeled_state == 6) {  ColumnOff(5); ColumnOn(4);
      } else if (cubeled_state == 7) {  ColumnOff(4); ColumnOn(7);
      } else if (cubeled_state == 8) {  ColumnOff(7); ColumnOn(8);
      } else if (cubeled_state == 9) {  ColumnOff(8); ColumnOn(9);
      } else if (cubeled_state == 10) { RowOff(1); RowOn(2);
      } else if (cubeled_state == 11) { ColumnOff(9); ColumnOn(8);
      } else if (cubeled_state == 12) { ColumnOff(8); ColumnOn(7);
      } else if (cubeled_state == 13) { ColumnOff(7); ColumnOn(4);
      } else if (cubeled_state == 14) { ColumnOff(4); ColumnOn(5);
      } else if (cubeled_state == 15) { ColumnOff(5); ColumnOn(6);
      } else if (cubeled_state == 16) { ColumnOff(6); ColumnOn(3);
      } else if (cubeled_state == 17) { ColumnOff(3); ColumnOn(2);
      } else if (cubeled_state == 18) { ColumnOff(2); ColumnOn(1);
      } else if (cubeled_state == 19) { RowOff(2); RowOn(3);
      } else if (cubeled_state == 20) { ColumnOff(1); ColumnOn(2);
      } else if (cubeled_state == 21) { ColumnOff(2); ColumnOn(3);
      } else if (cubeled_state == 22) { ColumnOff(3); ColumnOn(6);
      } else if (cubeled_state == 23) { ColumnOff(6); ColumnOn(5);
      } else if (cubeled_state == 24) { ColumnOff(5); ColumnOn(4);
      } else if (cubeled_state == 25) { ColumnOff(4); ColumnOn(7);
      } else if (cubeled_state == 26) { ColumnOff(7); ColumnOn(8);
      } else if (cubeled_state == 27) { ColumnOff(8); ColumnOn(9);
      } else if (cubeled_state == 28) { RowOff(3); ColumnOff(9);
                                        RowOn(2);  ColumnOn(5);
      } else if (cubeled_state == 29) { ColumnOff(5);
      } else if (cubeled_state == 30) {
        cubeled_state = 0;
      }
  }*/
  
}
