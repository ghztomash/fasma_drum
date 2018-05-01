/*
 Teensy Drum Machine v1.2
 Hardware: Teensy Drum v0.2 (Fasma Festival)
 By Tomash Ghz

 TODO:
 - Gain & Mastering of sounds.
 
 Change Log:
 + Power Saving mode (turn off LEDs), Sleep mode.
 -60mA run mode.
 -xmA sleep mode.
 -450mAh Alkaline 9V battery -> 7.5 hours.
 + Save automation to EEPROM
 + Chaining Patterns
 + Parameter initialization.
 + Pitch linear to exponential
 + Reduce kick pitch range.
 + Investigate Noise
 + Pmod Automation Float.
 + Crush Automation.
 + Automation Constrain for all voices.
 + Triggers light up in Pattern Mode.
 + Preview restore parameters.
 + External clock polarity
 + Copy patterns backwards bug.
 + Optimize for memory
 + LPF on kick out.
 + Preview sounds when stopped

 */
#include <Snooze.h>
#include <Bounce.h>
#include <Audio.h>
#include <synth_simple_drum.h>
#include <AudioExtended.h>
#include <AnalogDebounce.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "AudioSampleHat1.h"
#include "AudioSampleSnare2.h"

// sleep objects
SnoozeDigital digital;
SnoozeUSBSerial usb;
SnoozeAudio audio;
SnoozeBlock config(digital, usb, audio);

// audio objects
AudioSynthSimpleDrum drum1;          //xy=431,197
AudioEffectBitcrusher bitCrusher1;
AudioFilterStateVariable filter1;
AudioSynthSimpleDrum drum2;          //xy=399,244
AudioEffectBitcrusher bitCrusher2;
AudioFilterStateVariable filter2;
AudioPlayMemorySample sample1;
AudioEffectBitcrusher bitCrusher3;
AudioPlayMemorySample sample2;
AudioEffectBitcrusher bitCrusher4;
AudioSynthSimpleDrum tickDrum;
AudioMixer4 mixer1;         //xy=737,265
AudioMixer4 mixer2;
AudioOutputAnalog i2s1;           //xy=979,214
AudioConnection patchdrum1Cord1(drum1, 0, bitCrusher1, 0);
AudioConnection patchdrum1Cord2(bitCrusher1, 0, filter1, 0);
AudioConnection patchdrum1Cord3(filter1, 0, mixer1, 0);
AudioConnection patchdrum2Cord1(drum2, 0, bitCrusher2, 0);
AudioConnection patchdrum2Cord2(bitCrusher2, 0, filter2, 0);
AudioConnection patchdrum2Cord3(filter2, 1, mixer1, 1);
AudioConnection patchCordSample1f1(sample1, 0, bitCrusher3, 0);
AudioConnection patchCordSample1f2(bitCrusher3, 0, mixer1, 2);
AudioConnection patchCordSample2f1(sample2, 0, bitCrusher4, 0);
AudioConnection patchCordSample2f2(bitCrusher4, 0, mixer1, 3);
AudioConnection patchMixer1(mixer1, 0, mixer2, 0);
AudioConnection patchTick(tickDrum, 0, mixer2, 1);
AudioConnection patchMixer2(mixer2, 0, i2s1, 0);

// hardware pin definitions
const byte padAPin = 6;
const byte padBPin = 7;
const byte padCPin = 8;
const byte padDPin = 9;

const byte analogInPin = A3;
const byte analogPin[4] = { A6, A1, A2, A0 };
const byte led[4] = { 2, 23, 13, 12 };
boolean ledState[4] = { false, false, false, false };
const byte ledRGB[3] = { 4, 3, 5 };
const byte calibrationRGB[3] = { 254, 254, 253 };
const byte trigInPin = 0;
const byte trigOutPin = 1;

// global variables
bool longPress = false;
bool patternSelectLed = false;
int patternSelectLedTimer = 0;
bool patternCopy = false;
uint8_t patternToCopy = 0;

uint16_t bpm = 0;
boolean externalClock = false;
boolean clockHigh = false;
uint32_t step_ms = 0;
uint32_t step_us16 = 0;
uint32_t patternCopyTimer = 0;
uint32_t sleepTimer = 0;
bool sleeping = false;

float quantizeCoef = 0.55;

static uint32_t internalTimerRate = 10UL; //62500UL;
static uint16_t internalTimerPrio = 244;

uint32_t lastStep = 0;
uint32_t lastt = 0;
uint32_t recTime = 0;
boolean recPressed = false;
boolean patternSelect = false;

boolean patternSaved = false;
uint32_t patternSavedLEDTimer = 0;

uint8_t selectedVoice = 0;
uint8_t selectedPattern = 0;

uint8_t currentStep = 0;
boolean stepChanged = true;

uint8_t currentBar = 0;
uint8_t chainOrder[8];
uint8_t chainCount = 1;
uint8_t chainMode = 0;
uint8_t chainAdded = 0;

int16_t parameterLock[3][4][4][16];
float parameterLockPmod[4][4][16];
boolean parameterIsLocked[4][4][16];
boolean sequence[4][4][16];
boolean playing = true;
boolean tick = false;
boolean record = true;
boolean eraseMode = false;
boolean muted[4] = { false, false, false, false };

boolean noteJustAdded[4] = { false, false, false, false };

IntervalTimer INTERNAL_timer;

const byte playButtonPin = 18;
const byte eraseButtonPin = 11;
const byte recordButtonPin = 19;
const byte patternButtonPin = 10;
Bounce playButton = Bounce(playButtonPin, 10);
Bounce eraseButton = Bounce(eraseButtonPin, 10);
Bounce recordButton = Bounce(recordButtonPin, 10);
Bounce patternButton = Bounce(patternButtonPin, 10);

Bounce padA = Bounce(padAPin, 5);
Bounce padB = Bounce(padBPin, 5);
Bounce padC = Bounce(padCPin, 5);
Bounce padD = Bounce(padDPin, 5);

AnalogDebounce tempoPot = AnalogDebounce(A3, false, 25);
AnalogDebounce pitchPot[4];
AnalogDebounce lengthPot[4];
AnalogDebounce pmodPot[4];
AnalogDebounce crushPot[4];

boolean isPlaying[4] = { false, false, false, false };
int sDecay[4] = { 50, 50, 50, 50 };

int adcCounter = 0;

int pitchOnPress[4];
int lengthOnPress[4];
float pmodOnPress[4];
int crushOnPress[4];
int pitch[4];
int length[4];
float pmod[4];
int crush[4];

boolean debug = false;

uint8_t i = 0;
uint8_t j = 0;

void FASTRUN isrTrigger() {
	// external clock trigger
	cli();
	if(!externalClock) {
		INTERNAL_timer.end(); // disable internal clock
	}
	externalClock = true;
	if(playing) {
		currentStep++;
		clockHigh = digitalReadFast(trigInPin);
		digitalWriteFast(trigOutPin,clockHigh);
		//clockHigh=!clockHigh;
		if(currentStep>=16) {
			currentStep=0;
			if(chainMode) {
				currentBar++;
				if(currentBar>=chainCount)
				currentBar=0;
				selectedPattern=chainOrder[currentBar];
			}
		}
		stepChanged=true;
		lastt=(micros()-lastStep);
		step_us16 = lastt;
		lastStep = micros();
	}
	sei();
}

void FASTRUN isrInternalTimer() {
	// internal clock
	if((micros()-(lastStep))>step_us16) {
		currentStep++;
		digitalWriteFast(trigOutPin,clockHigh);
		clockHigh=!clockHigh;
		if(currentStep>=16) {
			currentStep=0;
			if(chainMode) {
				currentBar++;
				if(currentBar>=chainCount)
				currentBar=0;
				selectedPattern=chainOrder[currentBar];
			}
		}
		stepChanged=true;
		lastt=(micros()-lastStep);
		lastStep = micros();
	}
}

float FASTRUN linToExp(int index, float fRange) {
	//convert linear values to exponential
	float ratio = pow(fRange, 1/1024.0);
	return pow(ratio,index);
}

void setup() {
	for (i = 0; i < 4; i++) {
		// configure voice LEDs
		pinMode(led[i], OUTPUT);
		digitalWrite(led[i], LOW);
		// clear the sequences
		for (j = 0; j < 16; j++) {
			sequence[0][i][j] = false;
			sequence[1][i][j] = false;
			sequence[2][i][j] = false;
			sequence[3][i][j] = false;
		}

		//configure the potentiometers
		pitchPot[i] = AnalogDebounce(analogPin[0], false, 25);
		pitchPot[i].disable();
		pitchPot[i].init(512);
		pitch[i] = 512;
		lengthPot[i] = AnalogDebounce(analogPin[1], false, 25);
		lengthPot[i].disable();
		lengthPot[i].init(512);
		length[i] = 138;
		pmodPot[i] = AnalogDebounce(analogPin[2], false, 25);
		pmodPot[i].disable();
		pmodPot[i].init(512);
		pmod[i] = 0.5;
		crushPot[i] = AnalogDebounce(analogPin[3], false, 25);
		crushPot[i].init(0);
		crushPot[i].disable();
		crush[i] = 16;
	}

	// load patterns from EEPROM
	if (EEPROM.read(0) == 0x42) {
		loadPattern(0);
		loadPattern(1);
		loadPattern(2);
		loadPattern(3);
	} else {
		savePattern(0);
		savePattern(1);
		savePattern(2);
		savePattern(3);
		EEPROM.write(0, 0x42);
	}

	// enable voice 0 potentiometers
	pitchPot[0].enable();
	lengthPot[0].enable();
	pmodPot[0].enable();
	crushPot[0].enable();

	// configure the RGB LED
	for (i = 0; i < 3; i++) {
		pinMode(ledRGB[i], OUTPUT);
		digitalWrite(ledRGB[i], HIGH);
	}

	// configure rest of the pins
	pinMode(trigOutPin, OUTPUT);
	pinMode(trigInPin, INPUT);
	pinMode(playButtonPin, INPUT_PULLUP);
	pinMode(eraseButtonPin, INPUT_PULLUP);
	pinMode(recordButtonPin, INPUT_PULLUP);
	pinMode(patternButtonPin, INPUT_PULLUP);
	pinMode(padAPin, INPUT_PULLUP);
	pinMode(padBPin, INPUT_PULLUP);
	pinMode(padCPin, INPUT_PULLUP);
	pinMode(padDPin, INPUT_PULLUP);

	pinMode(22, OUTPUT); //noise debug

	// define and enable audio
	AudioMemory(128);

	AudioNoInterrupts();

	mixer1.gain(0, 1.5);
	mixer1.gain(1, 1.4);
	mixer1.gain(2, 0.7);
	mixer1.gain(3, 0.65);

	drum1.frequency(36);
	drum1.length(138);
	drum1.secondMix(0.0);
	drum1.pitchMod(0.50);

	filter1.frequency(540);
	filter1.resonance(0.707);

	filter2.frequency(60);
	filter2.resonance(0.707);

	bitCrusher1.bits(16);
	bitCrusher1.sampleRate(44100);

	drum2.frequency(80);
	drum2.length(138);
	drum2.secondMix(0.8);
	drum2.pitchMod(0.50);

	bitCrusher2.bits(16);
	bitCrusher2.sampleRate(44100);

	sample2.setSpeed(1.0);
	sample2.setLength(138);
	//sample2.setSample(AudioSampleSine100hz,17665);
	sample2.setSample(AudioSampleHat1, 4417);
	//sample2.setSample(AudioSamplePad,12225);
	sample1.setSpeed(1.0);
	sample1.setLength(138);
	sample1.setSample(AudioSampleSnare2, 4673);

	bitCrusher3.bits(16);
	bitCrusher3.sampleRate(44100);
	bitCrusher4.bits(16);
	bitCrusher4.sampleRate(44100);

	tickDrum.frequency(800);
	tickDrum.length(10);
	tickDrum.secondMix(0.0);
	tickDrum.pitchMod(0);

	AudioInterrupts();

	// enable interrupts
	attachInterrupt(trigInPin, isrTrigger, CHANGE);
	INTERNAL_timer.begin(isrInternalTimer, internalTimerRate);
	INTERNAL_timer.priority(internalTimerPrio);

	analogWriteResolution(8);
	analogWriteFrequency(4, 187500); //187500
	analogWriteFrequency(3, 187500);
	analogWriteFrequency(5, 187500);

	analogReference (EXTERNAL);
	//analogReadAveraging(1);

	// setup Sleep wake up pins
	digital.pinMode(playButtonPin, INPUT_PULLUP, CHANGE);
	digital.pinMode(eraseButtonPin, INPUT_PULLUP, CHANGE);
	digital.pinMode(recordButtonPin, INPUT_PULLUP, CHANGE);
	digital.pinMode(patternButtonPin, INPUT_PULLUP, CHANGE);
	digital.pinMode(padAPin, INPUT_PULLUP, CHANGE);
	digital.pinMode(padBPin, INPUT_PULLUP, CHANGE);
	digital.pinMode(padCPin, INPUT_PULLUP, CHANGE);
	digital.pinMode(padDPin, INPUT_PULLUP, CHANGE);
}

void FASTRUN readPads() {
	// read drum pads
	if ( padA.update() ) {
		if ( padA.read() == LOW) {

			if ((!eraseMode)&&(!patternSelect)) { // select voice
				disablePots(selectedVoice);
				selectedVoice = 0;

				if((!record)&&(!playing)) {
					AudioNoInterrupts();
					drum1.length(constrain(length[0],1,512));
					drum1.pitchMod(constrain(pmod[0],0.0,1.0));
					bitCrusher1.bits(constrain(crush[0],4,16));
					drum1.frequency(linToExp(constrain(pitch[0],0,1024),100)+25);
					AudioInterrupts();
					drum1.noteOn();
				}
			}
			if (patternSelect) { // select pattern
				selectedPattern = 0;
				if(!chainAdded) {
					clearChain();
					chainMode=0;
				}
				if(chainMode)
				addChain(selectedPattern);
				if(!patternCopy) {
					patternCopy = true;
					patternToCopy = 0;
				} else { // copy the previously selected pattern
					copyPattern(patternToCopy,0);
					patternCopy = false;
				}
			} else if ((!eraseMode)&&(record)) { // record step
				if (!muted[0]) {
					AudioNoInterrupts();
					drum1.length(constrain(length[0],1,512));
					drum1.pitchMod(constrain(pmod[0],0.0,1.0));
					bitCrusher1.bits(constrain(crush[0],4,16));
					drum1.frequency(linToExp(constrain(pitch[0],0,1024),100)+25);
					AudioInterrupts();
					drum1.noteOn();
				}
				if ((micros()-lastStep)>(step_us16*quantizeCoef)) {
					sequence[selectedPattern][0][(currentStep+1)%16]=true;
					noteJustAdded[0]=true;
				}
				else
				{
					sequence[selectedPattern][0][currentStep]=true;

				}
			}
			if ((eraseMode)&&(!record)) { // mute
				muted[0]=!muted[0];
			}
			if ((eraseMode)&&(record)) { // erase
				for (i=0; i<16; i++) {
					sequence[selectedPattern][0][i]=false;
					parameterLock[0][selectedPattern][0][i]=0;
					parameterLock[1][selectedPattern][0][i]=0;
					parameterLock[2][selectedPattern][0][i]=0;
					parameterLockPmod[selectedPattern][0][i]=0;
				}
			}
		} else {
			if (patternSelect) { // select pattern
				patternCopy = false;
			}
		}
		sleepTimer=millis();
		sleeping=false;
	}

	//========

	if ( padB.update() ) {
		if ( padB.read() == LOW) {
			if ((!eraseMode)&&(!patternSelect)) { // select voice
				if(selectedVoice!=1)
				disablePots(selectedVoice);
				selectedVoice = 1;

				if((!record)&&(!playing)) {
					AudioNoInterrupts();
					drum2.length(constrain(length[1],1,512));
					drum2.pitchMod(constrain(pmod[1],0.0,1.0));
					bitCrusher2.bits(constrain(crush[1],4,16));
					drum2.frequency(linToExp(constrain(pitch[1],0,1024),200)+65);
					AudioInterrupts();
					drum2.noteOn();
				}
			}
			if (patternSelect) {
				selectedPattern = 1;
				if(!chainAdded) {
					clearChain();
					chainMode=0;
				}
				if(chainMode)
				addChain(selectedPattern);
				if(!patternCopy) {
					patternCopy = true;
					patternToCopy = 1;
				} else { // copy the previously selected pattern
					copyPattern(patternToCopy,1);
					patternCopy = false;
				}
			} else if ((!eraseMode)&&(record)) { // record step
				if(!muted[1]) {
					AudioNoInterrupts();
					drum2.length(constrain(length[1],1,512));
					drum2.pitchMod(constrain(pmod[1],0.0,1.0));
					bitCrusher2.bits(constrain(crush[1],4,16));
					drum2.frequency(linToExp(constrain(pitch[1],0,1024),200)+65);
					AudioInterrupts();
					drum2.noteOn();
				}
				if((micros()-lastStep)>(step_us16*quantizeCoef)) {
					sequence[selectedPattern][1][(currentStep+1)%16]=true;
					noteJustAdded[1]=true;
				}
				else
				sequence[selectedPattern][1][currentStep]=true;
			}
			if((eraseMode)&&(!record)) { // mute
				muted[1]=!muted[1];
			}
			if((eraseMode)&&(record)) { // erase
				for (i=0; i<16; i++) {
					sequence[selectedPattern][1][i]=false;
					parameterLock[0][selectedPattern][1][i]=0;
					parameterLock[1][selectedPattern][1][i]=0;
					parameterLock[2][selectedPattern][1][i]=0;
					parameterLockPmod[selectedPattern][1][i]=0;
				}
			}
		} else {
			if (patternSelect) { // select pattern
				patternCopy = false;
			}
		}
		sleepTimer=millis();
		sleeping=false;
	}

	//========

	if ( padC.update() ) {
		if( padC.read() == LOW) {
			if((!eraseMode)&&(!patternSelect)) { // select voice
				if(selectedVoice!=2)
				disablePots(selectedVoice);
				selectedVoice = 2;

				if((!record)&&(!playing)) {
					AudioNoInterrupts();
					sample1.setLength(length[2]);
					sample1.pitchMod(constrain(pmod[2],0.0,1.0));
					sample1.setSpeed(constrain((((pitch[2])/1024.0)+0.5),0.0,2.0));
					bitCrusher3.bits(constrain(crush[2],4,16));
					AudioInterrupts();
					sample1.play();
				}
			}
			if (patternSelect) { // select pattern
				selectedPattern = 2;
				if(!chainAdded) {
					clearChain();
					chainMode=0;
				}
				if(chainMode)
				addChain(selectedPattern);
				if(!patternCopy) {
					patternCopy = true;
					patternToCopy = 2;
				} else { // copy the previously selected pattern
					copyPattern(patternToCopy,2);
					patternCopy = false;
				}
			} else if((!eraseMode)&&(record)) { // record step
				if(!muted[2]) {
					AudioNoInterrupts();
					sample1.setLength(length[2]);
					sample1.pitchMod(constrain(pmod[2],0.0,1.0));
					sample1.setSpeed(constrain((((pitch[2])/1024.0)+0.5),0.0,2.0));
					bitCrusher3.bits(constrain(crush[2],4,16));
					AudioInterrupts();
					sample1.play();
				}
				if((micros()-lastStep)>(step_us16*quantizeCoef)) {
					sequence[selectedPattern][2][(currentStep+1)%16]=true;
					noteJustAdded[2]=true;
				}
				else
				sequence[selectedPattern][2][currentStep]=true;
			}
			if((eraseMode)&&(!record)) { // mute
				muted[2]=!muted[2];
			}
			if((eraseMode)&&(record)) { // erase
				for (i=0; i<16; i++) {
					sequence[selectedPattern][2][i]=false;
					parameterLock[0][selectedPattern][2][i]=0;
					parameterLock[1][selectedPattern][2][i]=0;
					parameterLock[2][selectedPattern][2][i]=0;
					parameterLockPmod[selectedPattern][2][i]=0;
				}
			}
		} else {
			if (patternSelect) { // select pattern
				patternCopy = false;
			}
		}
		sleepTimer=millis();
		sleeping=false;
	}

	//========

	if( padD.update() ) {
		if( padD.read() == LOW) {
			if((!eraseMode)&&(!patternSelect)) { //select the voice
				if(selectedVoice!=3)
				disablePots(selectedVoice);
				selectedVoice = 3;

				if((!record)&&(!playing)) {
					AudioNoInterrupts();
					sample2.setLength(length[3]);
					sample2.pitchMod(constrain(pmod[3],0.0,1.0));
					sample2.setSpeed(constrain((((pitch[3])/1024.0)+0.5),0.0,2.0));
					bitCrusher4.bits(constrain(crush[3],4,16));
					AudioInterrupts();
					sample2.play();
				}
			}
			if (patternSelect) { // select pattern
				selectedPattern = 3;
				if(!chainAdded) {
					clearChain();
					chainMode=0;
				}
				if(chainMode)
				addChain(selectedPattern);
				if(!patternCopy) {
					patternCopy = true;
					patternToCopy = 3;
				} else { // copy the previously selected pattern
					copyPattern(patternToCopy,3);
					patternCopy = false;
				}
			} else if((!eraseMode)&&(record)) { // record step
				if(!muted[3]) {
					AudioNoInterrupts();
					sample2.setLength(length[3]);
					sample2.pitchMod(constrain(pmod[3],0.0,1.0));
					sample2.setSpeed(constrain((((pitch[3])/1024.0)+0.5),0.0,2.0));
					bitCrusher4.bits(constrain(crush[3],4,16));
					AudioInterrupts();
					sample2.play();
				}
				if((micros()-lastStep)>(step_us16*quantizeCoef)) {
					sequence[selectedPattern][3][(currentStep+1)%16]=true;
					noteJustAdded[3]=true;
				}
				else
				sequence[selectedPattern][3][currentStep]=true;
			}
			if((eraseMode)&&(!record)) { // mute
				muted[3]=!muted[3];
			}
			if((eraseMode)&&(record)) { // erase
				for (i=0; i<16; i++) {
					sequence[selectedPattern][3][i]=false;
					parameterLock[0][selectedPattern][3][i]=0;
					parameterLock[1][selectedPattern][3][i]=0;
					parameterLock[2][selectedPattern][3][i]=0;
					parameterLockPmod[selectedPattern][3][i]=0;
				}
			}
		} else {
			if (patternSelect) { // select pattern
				patternCopy = false;
			}
		}
		sleepTimer=millis();
		sleeping=false;
	}
}

void FASTRUN readPots() {
	//digitalWriteFast(22,HIGH);
	// do one ADC conversion at a time
	switch(adcCounter) {
		case 0:
		pitchPot[selectedVoice].read();
		break;
		case 1:
		lengthPot[selectedVoice].read();
		break;
		case 2:
		pmodPot[selectedVoice].read();
		break;
		case 3:
		crushPot[selectedVoice].read();
		break;
		case 4:
		tempoPot.read();
		break;
	}

	adcCounter++;

	if(adcCounter>4)
	adcCounter=0;

	// read BPM Pot
	if(tempoPot.hasChanged()) {
		if(!externalClock) {
			bpm = tempoPot.getValue()/5.68+60;
			step_ms = (60000000.0 / bpm);
			step_us16 = (15000000.0 / bpm);
		}
	}

	// pitch
	if(pitchPot[selectedVoice].hasChanged()) {
		pitch[selectedVoice] = pitchPot[selectedVoice].getValue();
	}

	if(lengthPot[selectedVoice].hasChanged()) {
		length[selectedVoice] = lengthPot[selectedVoice].getValue()/4 +10;
	}

	if(pmodPot[selectedVoice].hasChanged()) {
		pmod[selectedVoice] = pmodPot[selectedVoice].getValue()/1024.0;
	}

	if(crushPot[selectedVoice].hasChanged()) {
		crush[selectedVoice] = 16-(crushPot[selectedVoice].getValue()/85);
	}
	//digitalWriteFast(22,LOW);
}

void FASTRUN readButtons() {
	// play button
	if ( playButton.update() ) {
		if ( playButton.read() == LOW) {

			if(!patternSelect) {
				playing = !playing;

				// reset step and stop timer
				INTERNAL_timer.end();
				externalClock = false;
				clockHigh = false;
				currentStep = 16;
				lastStep = 0;
				if(playing)
				INTERNAL_timer.begin(isrInternalTimer, internalTimerRate);
			} else { // enable pattern chaining mode
				//Serial.println(F("pattern chaining"));
				chainMode = 1;
				chainAdded = true;
			}
		}
		sleepTimer=millis();
		sleeping=false;
	}

	// mute erase button
	if(!patternSelect) {
		if ( eraseButton.update() ) {
			if (eraseButton.read()== HIGH) {
				if(!record) { // turn off all mute LEDs
					for(i=0; i<4; i++) {
						digitalWriteFast(led[i],LOW);
					}
				}
				eraseMode=false;
			} else {
				eraseMode=true;
			}
			sleepTimer=millis();
			sleeping=false;
		}
	} else {
		if ( eraseButton.update() ) {
			if (eraseButton.read()== LOW) {
				erasePattern(selectedPattern);
			}
			sleepTimer=millis();
			sleeping=false;
		}
	}

	// record button
	if ( recordButton.update() ) {
		if ( recordButton.read() == LOW) {
			if(patternSelect) { // save pattern
				savePattern(selectedPattern);
			} else if ((eraseMode)&&(record)) { // randomize voice automation
				//Serial.println(F("Randomize"));
				randomizeAutomation();
			} else { // enable record
				recPressed = true;
				recTime = millis();
			}
			longPress=false;
		} else {
			recPressed=false;

			if((!patternSelect)&&(!eraseMode)) {

				if(!longPress)
				record = !record;

				if(record) {
					recPressed = true;
					recTime = millis();
					setRGB(0,HIGH);
				} else {
					tick = false;
					if(playing)
					setRGB(1,HIGH);
				}
			}
			if(longPress) {
				//Serial.println("Released");
				pitchPot[selectedVoice].init(pitchOnPress[selectedVoice]);
				pitchPot[selectedVoice].disable();
				lengthPot[selectedVoice].init(lengthOnPress[selectedVoice]*4-10);
				lengthPot[selectedVoice].disable();
				pmodPot[selectedVoice].init((int)(pmodOnPress[selectedVoice]*1024));
				pmodPot[selectedVoice].disable();
				crushPot[selectedVoice].init((16-crushOnPress[selectedVoice])*85);
				crushPot[selectedVoice].disable();
			}
			longPress=false;
		}
		sleepTimer=millis();
		sleeping=false;
	}
	if( recordButton.read() == LOW) { // long press enable metronome
		if((!record)&&(recPressed)) {
			if(millis()-recTime>500) {
				recPressed=false;
				tick=!tick;
				//record = !record;
				setRGB(1,LOW);// indicate tick is on by turing off green led for a bit
			}
		} else if((record)&&(recPressed)) {
			if(millis()-recTime>500) {
				if(!longPress) {
					pitchOnPress[selectedVoice] = pitch[selectedVoice];
					lengthOnPress[selectedVoice] = length[selectedVoice];
					pmodOnPress[selectedVoice] = pmod[selectedVoice];
					crushOnPress[selectedVoice] = crush[selectedVoice];
					//Serial.println("LongPess");
					longPress=true;
				}
				if(abs(pitch[selectedVoice]-pitchOnPress[selectedVoice])>10) {
					// pitch
					parameterLock[0][selectedPattern][selectedVoice][(currentStep+1)%16] = pitch[selectedVoice]-pitchOnPress[selectedVoice];
				}
				if(abs(length[selectedVoice]-lengthOnPress[selectedVoice])>10) {
					// length
					parameterLock[1][selectedPattern][selectedVoice][(currentStep+1)%16] = length[selectedVoice]-lengthOnPress[selectedVoice];
				}
				if(abs(pmod[selectedVoice]-pmodOnPress[selectedVoice])>0.10) {
					// pmod
					parameterLockPmod[selectedPattern][selectedVoice][(currentStep+1)%16] = pmod[selectedVoice]-pmodOnPress[selectedVoice];
				}
				if(abs(crush[selectedVoice]-crushOnPress[selectedVoice])>2) {
					// crush
					parameterLock[2][selectedPattern][selectedVoice][(currentStep+1)%16] = crushOnPress[selectedVoice]-crush[selectedVoice];
				}
			}
		}
		sleepTimer=millis();
		sleeping=false;
	}

	// pattern button
	if(!eraseMode) {
		if ( patternButton.update() ) {
			if ( patternButton.read() == LOW) {
				patternSelect=true;
				//clearChain();
			} else {
				for(i=0; i<4; i++) { // turn off pattern LEDs
					digitalWriteFast(led[i],LOW);
				}
				patternSaved = false;
				patternCopy = false;
				patternSelect = false;
				chainAdded = false;
				if(record)
				setRGB(0,HIGH);
				else if (playing)
				setRGB(1,HIGH);
			}
			sleepTimer=millis();
			sleeping=false;
		}
	}
}

void loop() {
	// main loop
	//===================================================
	readButtons();
	readPads();
	readPots();
	//===================================================

	if (stepChanged) {
		// metronome tick
		if ((record) && (tick) && (currentStep % 4 == 0)) {
			if (currentStep == 0)
				tickDrum.frequency(1760);
			else
				tickDrum.frequency(880);
			tickDrum.noteOn();
		}

		if (sequence[selectedPattern][0][currentStep]) {
			if (!muted[0]) {
				if (!noteJustAdded[0]) {
					AudioNoInterrupts();
					// have to check if the current step is currently pressed down for Param Lock.
					// if yes, play pitch on press + paramlock.
					// if no check if the current step has param lock
					// all other steps must play 
					if (longPress && (selectedVoice == 0)) { // play preview pitches
						// its not pressed but has param lock
						drum1.length(
								constrain(
										lengthOnPress[0]
												+ parameterLock[1][selectedPattern][0][currentStep],
										1, 512));
						drum1.pitchMod(
								constrain(
										pmodOnPress[0]
												+ parameterLockPmod[selectedPattern][0][currentStep],
										0.0, 1.0));
						bitCrusher1.bits(
								constrain(
										crushOnPress[0]
												- parameterLock[2][selectedPattern][0][currentStep],
										4, 16));
						drum1.frequency(
								linToExp(
										constrain(
												pitchOnPress[0]
														+ parameterLock[0][selectedPattern][0][currentStep],
												0, 1024), 100) + 25);
						// has no param lock
					} else { // play current pitch + param lock
						drum1.length(
								constrain(
										length[0]
												+ parameterLock[1][selectedPattern][0][currentStep],
										1, 512));
						drum1.pitchMod(
								constrain(
										pmod[0]
												+ parameterLockPmod[selectedPattern][0][currentStep],
										0.0, 1.0));
						bitCrusher1.bits(
								constrain(
										crush[0]
												- parameterLock[2][selectedPattern][0][currentStep],
										4, 16));
						drum1.frequency(
								linToExp(
										constrain(
												pitch[0]
														+ parameterLock[0][selectedPattern][0][currentStep],
												0, 1024), 100) + 25);
					}
					AudioInterrupts();
					drum1.noteOn();
				}
				noteJustAdded[0] = false;
				if (!patternSelect) {
					if (ledState[0]) {
						digitalWriteFast(led[0], LOW);
						ledState[0] = false;
					} else {
						if (!sleeping) {
							digitalWriteFast(led[0], HIGH);
							ledState[0] = true;
						}
					}
				}
			}
		}

		if (sequence[selectedPattern][1][currentStep]) {
			if (!muted[1]) {
				if (!noteJustAdded[1]) {
					AudioNoInterrupts();
					// have to check if the current step is currently pressed down for Param Lock.
					// if yes, play pitch on press + paramlock.
					// if no check if the current step has param lock
					// all other steps must play 
					if (longPress && (selectedVoice == 1)) { // play preview pitches
						// its not pressed but has param lock
						drum2.length(
								constrain(
										lengthOnPress[1]
												+ parameterLock[1][selectedPattern][1][currentStep],
										1, 512));
						drum2.pitchMod(
								constrain(
										pmodOnPress[1]
												+ parameterLockPmod[selectedPattern][1][currentStep],
										0.0, 1.0));
						bitCrusher2.bits(
								constrain(
										crushOnPress[1]
												- parameterLock[2][selectedPattern][1][currentStep],
										4, 16));
						drum2.frequency(
								linToExp(
										constrain(
												pitchOnPress[1]
														+ parameterLock[0][selectedPattern][1][currentStep],
												0, 1024), 200) + 65);
						filter2.frequency(
								linToExp(
										constrain(
												pitchOnPress[1]
														+ parameterLock[0][selectedPattern][1][currentStep],
												0, 1024), 200) + 65);
						//Serial.println("Preview");
					} else { // play current pitch + param lock
						drum2.length(
								constrain(
										length[1]
												+ parameterLock[1][selectedPattern][1][currentStep],
										1, 512));
						drum2.pitchMod(
								constrain(
										pmod[1]
												+ parameterLockPmod[selectedPattern][1][currentStep],
										0.0, 1.0));
						bitCrusher2.bits(
								constrain(
										crush[1]
												- parameterLock[2][selectedPattern][1][currentStep],
										4, 16));
						drum2.frequency(
								linToExp(
										constrain(
												pitch[1]
														+ parameterLock[0][selectedPattern][1][currentStep],
												0, 1024), 200) + 65);
						filter2.frequency(
								linToExp(
										constrain(
												pitch[1]
														+ parameterLock[0][selectedPattern][1][currentStep],
												0, 1024), 200) + 65);
						//Serial.println("Current");
					}
					AudioInterrupts();
					drum2.noteOn();
				}
				noteJustAdded[1] = false;
				if (!patternSelect) {
					if (ledState[1]) {
						digitalWriteFast(led[1], LOW);
						ledState[1] = false;
					} else {
						if (!sleeping) {
							digitalWriteFast(led[1], HIGH);
							ledState[1] = true;
						}
					}
				}
			}
		}

		if (sequence[selectedPattern][2][currentStep]) {
			if (!muted[2]) {
				if (!noteJustAdded[2]) {
					AudioNoInterrupts();
					if (longPress && (selectedVoice == 2)) { // play preview pitches
						sample1.setLength(
								lengthOnPress[2]
										+ parameterLock[1][selectedPattern][2][currentStep]);
						sample1.pitchMod(
								constrain(
										pmodOnPress[2]
												+ parameterLockPmod[selectedPattern][2][currentStep],
										0.0, 1.0));
						sample1.setSpeed(
								constrain(
										(((pitchOnPress[2]
												+ parameterLock[0][selectedPattern][2][currentStep])
												/ 1024.0) + 0.5), 0.0, 2.0));
						bitCrusher3.bits(
								constrain(
										crushOnPress[2]
												- parameterLock[2][selectedPattern][2][currentStep],
										4, 16));
					} else { // play current pitch + param lock
						sample1.setLength(
								length[2]
										+ parameterLock[1][selectedPattern][2][currentStep]);
						sample1.pitchMod(
								constrain(
										pmod[2]
												+ parameterLockPmod[selectedPattern][2][currentStep],
										0.0, 1.0));
						sample1.setSpeed(
								constrain(
										(((pitch[2]
												+ parameterLock[0][selectedPattern][2][currentStep])
												/ 1024.0) + 0.5), 0.0, 2.0));
						bitCrusher3.bits(
								constrain(
										crush[2]
												- parameterLock[2][selectedPattern][2][currentStep],
										4, 16));
					}
					AudioInterrupts();
					sample1.play();
				}
				noteJustAdded[2] = false;

				if (!patternSelect) {
					if (ledState[2]) {
						digitalWriteFast(led[2], LOW);
						ledState[2] = false;
					} else {
						if (!sleeping) {
							digitalWriteFast(led[2], HIGH);
							ledState[2] = true;
						}
					}
				}
			}
		}

		if (sequence[selectedPattern][3][currentStep]) {
			if (!muted[3]) {
				if (!noteJustAdded[3]) {
					AudioNoInterrupts();
					if (longPress && (selectedVoice == 3)) { // play preview pitches
						sample2.setLength(
								lengthOnPress[3]
										+ parameterLock[1][selectedPattern][3][currentStep]);
						sample2.pitchMod(
								constrain(
										pmod[3]
												+ parameterLockPmod[selectedPattern][3][currentStep],
										0.0, 1.0));
						sample2.setSpeed(
								constrain(
										(((pitchOnPress[3]
												+ parameterLock[0][selectedPattern][3][currentStep])
												/ 1024.0) + 0.5), 0.0, 2.0));
						bitCrusher4.bits(
								constrain(
										crushOnPress[3]
												- parameterLock[2][selectedPattern][3][currentStep],
										4, 16));
					} else { // play current pitch + param lock
						sample2.setLength(
								length[3]
										+ parameterLock[1][selectedPattern][3][currentStep]);
						sample2.pitchMod(
								constrain(
										pmod[3]
												+ parameterLockPmod[selectedPattern][3][currentStep],
										0.0, 1.0));
						sample2.setSpeed(
								constrain(
										(((pitch[3]
												+ parameterLock[0][selectedPattern][3][currentStep])
												/ 1024.0) + 0.5), 0.0, 2.0));
						bitCrusher4.bits(
								constrain(
										crush[3]
												- parameterLock[2][selectedPattern][3][currentStep],
										4, 16));
					}
					AudioInterrupts();
					sample2.play();
				}
				noteJustAdded[3] = false;

				if (!patternSelect) {
					if (ledState[3]) {
						digitalWriteFast(led[3], LOW);
						ledState[3] = false;
					} else {
						if (!sleeping) {
							digitalWriteFast(led[3], HIGH);
							ledState[3] = true;
						}
					}
				}
			}
		}
		
		// debugging print outs
		if (debug) {
			Serial.print(F("\033[2J\033[0;0f"));
			//Serial.print("\033[0;20fA \033[0;40fB \033[0;60fC \033[0;80fD\n");
			Serial.print(F("\t\t\tA \t\tB \t\tC \t\tD\n"));
			Serial.print(F("Current Pitch:\t\t"));
			//Serial.println(constrain(pitch[selectedVoice]+parameterLock[0][selectedPattern][selectedVoice][currentStep],25,2000));
			Serial.print(
					linToExp(
							constrain(
									pitch[0]
											+ parameterLock[0][selectedPattern][0][currentStep],
									0, 1024), 100) + 25);
			Serial.print(F(" \t\t"));
			Serial.print(
					linToExp(
							constrain(
									pitch[1]
											+ parameterLock[0][selectedPattern][1][currentStep],
									0, 1024), 200) + 65);
			Serial.print(F(" \t\t"));
			Serial.print(
					constrain(
							(((pitch[2]
									+ parameterLock[0][selectedPattern][2][currentStep])
									/ 1024.0) + 0.5), 0.0, 2.0));
			Serial.print(F(" \t\t"));
			Serial.println(
					constrain(
							(((pitch[3]
									+ parameterLock[0][selectedPattern][3][currentStep])
									/ 1024.0) + 0.5), 0.0, 2.0));

			Serial.print(F("\nPitch Pot:\t\t"));
			Serial.print(pitchPot[0].getValue());
			Serial.print(F("\t\t"));
			Serial.print(pitchPot[1].getValue());
			Serial.print(F("\t\t"));
			Serial.print(pitchPot[2].getValue());
			Serial.print(F("\t\t"));
			Serial.print(pitchPot[3].getValue());

			//}
			Serial.print(F("\n\nCurrent Length:\t\t"));
			//Serial.print(lengthOnPress[selectedVoice]);
			//Serial.print("\t");
			//Serial.print(length[selectedVoice]);

			Serial.print(
					constrain(
							length[0]
									+ parameterLock[1][selectedPattern][0][currentStep],
							1, 512));
			Serial.print(F("\t\t"));
			Serial.print(
					constrain(
							length[1]
									+ parameterLock[1][selectedPattern][1][currentStep],
							1, 512));
			Serial.print(F("\t\t"));
			Serial.print(
					constrain(
							length[2]
									+ parameterLock[1][selectedPattern][2][currentStep],
							1, 512));
			Serial.print(F("\t\t"));
			Serial.print(
					constrain(
							length[3]
									+ parameterLock[1][selectedPattern][3][currentStep],
							1, 512));

			Serial.print(F("\nLength Pot:\t\t"));
			Serial.print(lengthPot[0].getValue());
			Serial.print(F("\t\t"));
			Serial.print(lengthPot[1].getValue());
			Serial.print(F("\t\t"));
			Serial.print(lengthPot[2].getValue());
			Serial.print(F("\t\t"));
			Serial.print(lengthPot[3].getValue());

			Serial.print(F("\n\nCurrent Pmod:\t\t"));
			Serial.print(
					constrain(
							pmod[0]
									+ parameterLockPmod[selectedPattern][0][currentStep],
							0.0, 1.0));
			Serial.print(F("\t\t"));
			Serial.print(
					constrain(
							pmod[1]
									+ parameterLockPmod[selectedPattern][1][currentStep],
							0.0, 1.0));
			Serial.print(F("\t\t"));
			Serial.print(
					constrain(
							pmod[2]
									+ parameterLockPmod[selectedPattern][2][currentStep],
							0.0, 1.0));
			Serial.print(F("\t\t"));
			Serial.print(
					constrain(
							pmod[2]
									+ parameterLockPmod[selectedPattern][2][currentStep],
							0.0, 1.0));

			Serial.print(F("\n\nCurrent Crush: "));
			Serial.print(crushPot[selectedVoice].getValue());
			Serial.print(F("\t"));
			Serial.print((16 - (crush[selectedVoice])) * 85);
			Serial.print(F("\t"));
			Serial.print(crush[selectedVoice]);
			Serial.print(F("\t"));
			Serial.print(crushOnPress[selectedVoice]);
			Serial.print(F("\t"));
			Serial.println(
					crush[selectedVoice]
							- parameterLock[2][selectedPattern][selectedVoice][currentStep]); // 16 to 4
		}

		if (debug) {
			Serial.println("");
			for (i = 0; i < 16; i++) {
				if (i % 4 == 0)
					Serial.print(F("+\t"));
				else
					Serial.print(F("_\t"));
			}
			Serial.println(F(""));
			for (i = 0; i < currentStep; i++)
				Serial.print(F("\t"));
			Serial.println(F("v"));
			Serial.println(F(""));
			Serial.print(F("\033[0;37;40m"));
			for (j = 0; j < 4; j++) {
				for (i = 0; i < 16; i++) {
					if (sequence[selectedPattern][j][i])
						Serial.print(F("\033[1;30;47m"));
					else
						Serial.print(F("\033[0;37;40m"));
					Serial.print(sequence[selectedPattern][j][i]);
					Serial.print(F("\t"));
				}
				Serial.print(F("\033[0;37;40m"));
				Serial.println(F(""));
			}
			for (i = 0; i < currentStep; i++)
				Serial.print(F("\t"));
			Serial.println(F("^"));
			Serial.println("");

			for (i = 0; i < 16; i++) {
				Serial.print(
						parameterLock[0][selectedPattern][selectedVoice][i]);
				Serial.print(F("\t"));
			}
			Serial.println("");
			for (i = 0; i < 16; i++) {
				Serial.print(
						parameterLock[1][selectedPattern][selectedVoice][i]);
				Serial.print(F("\t"));
			}
			Serial.println("");
			for (i = 0; i < 16; i++) {
				Serial.print(
						parameterLock[2][selectedPattern][selectedVoice][i]);
				Serial.print(F("\t"));
			}
			Serial.println("");
			for (i = 0; i < 16; i++) {
				Serial.print(
						parameterLockPmod[selectedPattern][selectedVoice][i]);
				Serial.print(F("\t"));
			}
			Serial.println(F(""));
		}

		if (debug) {
			Serial.println(F("\n************************"));
			Serial.print(F("BPM: "));
			Serial.print(bpm);
			Serial.print(F("\nCPU: "));
			Serial.print(AudioProcessorUsage());
			Serial.print(F("\t"));
			Serial.println(AudioProcessorUsageMax());
			Serial.print(F("Memory: "));
			Serial.print(AudioMemoryUsage());
			Serial.print(F("\t"));
			Serial.println(AudioMemoryUsageMax());
			Serial.print(F("Erase Mode: "));
			Serial.println(eraseMode);
		}

		if (debug) {
			Serial.println(F("\n******** Chain **********"));
			Serial.print(F("Chain Count: "));
			Serial.print(chainCount);
			Serial.print(F("\tCurrentPattern: "));
			Serial.print(selectedPattern);
			Serial.print(F("\tCurrent Bar: "));
			Serial.println(currentBar);
			for (i = 0; i < 8; i++) {
				Serial.print(F(" "));
				Serial.print(chainOrder[i]);
			}
		}

		// blink in tempo
		if (currentStep % 4 == 0) {
			if (!sleeping) {
				setRGB(0, HIGH);
				setRGB(1, HIGH);
				setRGB(2, HIGH);
			}
		}
		stepChanged = false;
	}

	updateLEDs();

	if ((!sleeping) & ((millis() - sleepTimer) > 1200000)) { // sleep after 20 minutes
		if (playing) { // continue playing just turn off the LEDs
			sleeping = true;
			//Serial.println("Going to light sleep");

			setRGB(0, LOW);
			setRGB(1, LOW);
			setRGB(2, LOW);

			digitalWriteFast(led[0], LOW);
			digitalWriteFast(led[1], LOW);
			digitalWriteFast(led[2], LOW);
			digitalWriteFast(led[3], LOW);
		} else { // turn off the LEDs and put the MCU to low power mode
			sleeping = true;
			//Serial.println("Going to deep sleep");

			setRGB(0, LOW);
			setRGB(1, LOW);
			setRGB(2, LOW);

			digitalWriteFast(led[0], LOW);
			digitalWriteFast(led[1], LOW);
			digitalWriteFast(led[2], LOW);
			digitalWriteFast(led[3], LOW);

			Snooze.sleep(config); // put MCU to sleep
		}
	}
}

void updateLEDs() {
	// update LEDs state
	if (!sleeping) {
		if (micros() - lastStep > 20000) { // tempo indication
			if (record) {
				setRGB(1, LOW);
				setRGB(2, LOW);
			} else {
				setRGB(0, LOW);
				setRGB(2, LOW);
				if (!playing)
					setRGB(1, LOW);
			}
		}

		if ((eraseMode) && (record)) { // erase mode
			setRGB(0, HIGH);
			setRGB(2, HIGH);
			setRGB(1, LOW);
		}
		if (patternSelect) { // pattern select mode
			if ((patternSaved) && (millis() - patternSavedLEDTimer) < 100) {
				setRGB(0, HIGH);
				setRGB(1, LOW);
				setRGB(2, LOW);
			} else if (chainMode) {
				setRGB(0, LOW);
				setRGB(1, HIGH);
				setRGB(2, HIGH);
			} else {
				setRGB(0, LOW);
				setRGB(1, LOW);
				setRGB(2, HIGH);
			}
			for (i = 0; i < 4; i++)
				if (i != selectedPattern)
					digitalWriteFast(led[i], LOW);

			if (patternCopy) {
				if (millis() - patternSelectLedTimer > 50) { // flash copy pattern
					patternSelectLedTimer = millis();
					patternSelectLed = !patternSelectLed;
					digitalWriteFast(led[selectedPattern], patternSelectLed);
				}
			} else {
				if (millis() - patternSelectLedTimer > 100) { // flash selected pattern
					patternSelectLedTimer = millis();
					patternSelectLed = !patternSelectLed;
					digitalWriteFast(led[selectedPattern], patternSelectLed);
				}
			}

		} else if ((eraseMode) && (!record)) { // display mute LEDs
			for (i = 0; i < 4; i++) {
				digitalWriteFast(led[i], !muted[i]);
				//ledState[i]=false;
			}
			setRGB(0, HIGH);
			setRGB(1, HIGH);
			setRGB(2, LOW);
		} else if (micros() - lastStep > 50000) { // step trigger LED

			for (i = 0; i < 4; i++) {
				if (ledState[i]) {
					digitalWriteFast(led[i], LOW);
					ledState[i] = false;
				}
			}

			if (!ledState[selectedVoice]) {
				digitalWrite(led[selectedVoice], HIGH);
				ledState[selectedVoice] = true;
			}
		}
	}
}

void disablePots(int v) {
	// lock potentiometers
	pitchPot[v].disable();
	lengthPot[v].disable();
	pmodPot[v].disable();
	crushPot[v].disable();
}

void setRGB(int p, int v) {
	// set RGB led on or off with calibration
	if (v == HIGH)
		analogWrite(ledRGB[p], calibrationRGB[p]);
	else if (v == LOW)
		analogWrite(ledRGB[p], 256);
}

void loadPattern(byte p) {
	// restore pattern from EEPROM
	byte a;
	byte b;
	int address = 1 + p * 8;
	// 2 bytes per voice
	// 4 voices -> 8 bytes
	// 4 patterns -> 32 bytes
	for (i = 0; i < 4; i++) {
		a = EEPROM.read(address + i * 2);
		b = EEPROM.read(address + i * 2 + 1);

		//Serial.print("loading voice ");
		//Serial.println(i);
		//Serial.println(a, BIN);
		//Serial.println(b, BIN);

		for (j = 0; j < 8; j++) {
			sequence[p][i][j] = (((a >> j) & 1) == 1);
			sequence[p][i][j + 8] = (((b >> j) & 1) == 1);
		}
	}
	loadAutomation(p);
}

void loadAutomation(byte p) {
	// load automation from EEPROM
	int address = 1023 + p * 256;
	byte b1, b2, b3, b4;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 16; j++) {
			b1 = EEPROM.read(address + i * 64 + j * 4);
			b2 = EEPROM.read(address + i * 64 + j * 4 + 1);
			b3 = EEPROM.read(address + i * 64 + j * 4 + 2);
			b4 = EEPROM.read(address + i * 64 + j * 4 + 3);

			// restore 8 bit to 12bit
			parameterLock[0][p][i][j] = (b1 << 3) - 1024;
			parameterLock[1][p][i][j] = (b2 << 1) - 256;
			parameterLockPmod[p][i][j] = (b3 / 128.0) - 1.0;
			parameterLock[2][p][i][j] = b4 - 16;
		}
	}
}

void savePattern(byte p) {
	// save pattern to EEPROM
	byte a;
	byte b;
	int address = 1 + p * 8;
	// 2 bytes per voice
	// 4 voices -> 8 bytes
	// 4 patterns -> 32 bytes
	for (i = 0; i < 4; i++) {
		a = 0;
		b = 0;
		for (j = 0; j < 8; j++) {
			a += ((sequence[p][i][j]) & 1) << j;
			b += ((sequence[p][i][j + 8]) & 1) << j;
		}
		//EEPROM.write(0, 0x69);
		EEPROM.write(address + i * 2, a);
		EEPROM.write(address + i * 2 + 1, b);
		//Serial.print("saving voice ");
		//Serial.println(i);
		//Serial.println(a, BIN);
		//Serial.println(b, BIN);
	}
	saveAutomation(p);

	patternSaved = true;
	patternSavedLEDTimer = millis();
}

void saveAutomation(byte p) {
	// save automation to EEPROM
	// 1024 bytes automation data (reduced to 8bit)
	// 64 bytes per voice
	// 4 voices -> 256 bytes
	// 4 patters -> 1024 bytes

	int address = 1023 + p * 256;
	byte b1, b2, b3, b4;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 16; j++) {
			b1 = (parameterLock[0][p][i][j] + 1024) >> 3;
			b2 = (parameterLock[1][p][i][j] + 256) >> 1;
			b3 = (parameterLockPmod[p][i][j] + 1) * 128;
			b4 = parameterLock[2][p][i][j] + 16;

			EEPROM.write(address + i * 64 + j * 4, b1);
			EEPROM.write(address + i * 64 + j * 4 + 1, b2);
			EEPROM.write(address + i * 64 + j * 4 + 2, b3);
			EEPROM.write(address + i * 64 + j * 4 + 3, b4);
		}

	}

}

void randomizeAutomation() {
	// fill automation with random values
	for (j = 0; j < 16; j++) {
		parameterLock[0][selectedPattern][selectedVoice][j] = random(-1024,
				1024);
		parameterLock[1][selectedPattern][selectedVoice][j] = random(-256, 256);
		parameterLockPmod[selectedPattern][selectedVoice][j] = random(-1024,
				1024) / 1024.0;
		parameterLock[2][selectedPattern][selectedVoice][j] = random(-12, 12);
	}
}

void copyPattern(byte from, byte to) {
	// copy pattern and automation
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 16; j++) {
			sequence[to][i][j] = sequence[from][i][j];
			parameterLock[0][to][i][j] = parameterLock[0][from][i][j];
			parameterLock[1][to][i][j] = parameterLock[1][from][i][j];
			parameterLock[2][to][i][j] = parameterLock[2][from][i][j];
			parameterLockPmod[to][i][j] = parameterLockPmod[from][i][j];
		}
	}
}

void erasePattern(byte p) {
	// erase pattern and automation
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 16; j++) {
			sequence[p][i][j] = false;
			parameterLock[0][p][i][j] = 0;
			parameterLock[1][p][i][j] = 0;
			parameterLock[2][p][i][j] = 0;
			parameterLockPmod[p][i][j] = 0;
		}
	}
}

void addChain(uint8_t c) {
	// add a pattern to the chain sequence
	chainOrder[chainCount] = c;
	if (chainCount < 8)
		chainCount++;
}

void clearChain() {
	// clear the chain sequence
	addChain(selectedPattern);
	chainCount = 0;
}
