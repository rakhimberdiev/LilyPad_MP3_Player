// "Player" example sketch for Lilypad MP3 Player
// Mike Grusin, SparkFun Electronics
// http://www.sparkfun.com

// This sketch turns the Lilypad MP3 Player into a basic MP3
// player that you can control with the optional rotary encoder.

// HARDWARE

// To use this sketch, an optional RGB rotary encoder must be soldered
// to the MP3 Player board. The board is designed to hold SparkFun
// part number COM-10892. https://www.sparkfun.com/products/10982

// SOFTWARE

// This sketch requires the following libraries. These are included
// in the Lilypad MP3 Player firmware zip file and must be copied to
// a "libraries" folder in your Arduino sketch directory:

// Uses the SdFat library by William Greiman, which is supplied
// with this archive, or download from http://code.google.com/p/sdfatlib/

// Uses the SFEMP3Shield library by Porter and Flaga, which is supplied
// with this archive, or download from http://www.billporter.info/

// Uses the PinChangeInt library by Lex Talionis, which is supplied 
// with this archive, or download from http://code.google.com/p/arduino-pinchangeint/

// BASIC OPERATION:

// Place your audio files in the root directory of the SD card.
// Your files MUST have one of the following extensions: MP3, WAV,
// MID (MIDI), MP4, WMA, AAC, FLA, OGG. Note that this is solely to
// prevent the VS1053 from locking up from being fed non-audio data
// (files without one of the above extensions are quietly skipped).
// You can rename any playable file to any of the above extensions,
// or add additional extensions to the isPlayable() function below.
// See the VS1053 datasheet for the audio file types it can play.

// The control logic is a bit different from Sparkfun's original.
//
// The player starts playing immediately upon boot (what's the point
// in a programmable payer if you have to control in manually, right?)
//
// To start and stop play back, press and release the knob (as in 
// the original, sort of, but we don't care about press duration).
//
// Just turning the knob switches tracks.
//
// To adjust volume, press and turn the knob.
//
// Light scheme: blue for playing, red for stopped, yellow when the
// knob is down, but no further decision is yet made (by releasing or
// turning, which would indicate start/stop or volume control,
// respectively). Green for volume adjustment in progress.


// SERIAL DEBUGGING

// This sketch can output serial debugging information if desired
// by changing the global variable "debugging" to true. Note that
// this will take away trigger inputs 4 and 5, which are shared 
// with the TX and RX lines. You can keep these lines connected to
// trigger switches and use the serial port as long as the triggers
// are normally open (not grounded) and remain ungrounded while the
// serial port is in use.

// License:
// We use the "beerware" license for our firmware. You can do
// ANYTHING you want with this code. If you like it, and we meet
// someday, you can, but are under no obligation to, buy me a
// (root) beer in return.

// Have fun! 
// -your friends at SparkFun

// Revision history:
// 1.0 initial release MDG 2013/1/31

// Required libraries:

#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include <SFEMP3Shield.h>
#include <PinChangeInt.h>
#include <EEPROM.h>

// Set debugging to true to get serial messages:

boolean debugging = true;

//Defining this will randomize tracks
#define RANDOMIZE true
#define MAX_TRACKS 100

int tracks_found, current_track;
uint16_t track_files[MAX_TRACKS];
char track[13];



// Possible modes (first and last are there to make
// rotating through them easier):

#define UNDEFINED_MODE 0
#define KNOB_UP_MODE 1
#define KNOB_DOWN_MODE 2
#define VOLUME_CONTROL_MODE 3

// Default initial volume for the MP3 chip. 0 is the loudest, 255
// is the lowest.

uint8_t volume = 40;

// Set loop_all to true if you would like to automatically
// start playing the next file after the current one ends:

boolean loop_all = true;

// LilyPad MP3 pin definitions:

#define TRIG1 A0
#define ROT_LEDG A1
#define SHDN_GPIO1 A2
#define ROT_B A3
#define TRIG2_SDA A4
#define TRIG3_SCL A5
#define RIGHT A6
#define LEFT A7

#define TRIG5_RXI 0
#define TRIG4_TXO 1
#define MP3_DREQ 2
#define ROT_A 3
#define ROT_SW 4
#define ROT_LEDB 5
#define MP3_CS 6
#define MP3_DCS 7
#define MP3_RST 8
#define SD_CS 9
#define ROT_LEDR 10
#define MOSI 11
#define MISO 12
#define SCK 13

// RGB LED colors (for common anode LED, 0 is on, 1 is off)

#define OFF B111
#define RED B110
#define GREEN B101
#define YELLOW B100
#define BLUE B011
#define PURPLE B010
#define CYAN B001
#define WHITE B000

//EEPROM constants

#define EEPROM_SCHEMA 0x01 //Used to do a quick check that EEPROM has our data, not something else
#define EEPROM_SCHEMA_ADDRESS 0
#define EEPROM_VOLUME_ADDRESS 1 //This is where we will store our volume setting


// Global variables and flags for interrupt request functions:

volatile int rotary_counter = 0; // Current "position" of rotary encoder (increments CW) 
volatile boolean rotary_change = false; // Will turn true if rotary_counter has changed
volatile boolean rotary_direction; // Direction rotary encoder was turned (true = CW)
volatile boolean button_pressed = false; // Will turn true if the button has been pushed
volatile boolean button_released = false; // Will turn true if the button has been released (sets button_downtime)
volatile unsigned long button_downtime = 0L; // ms the button was pushed before release


// Library objects:

SdFat sd;
SdFile file;
SFEMP3Shield MP3player;


void setup()
{
  byte result;

  if (debugging)
  {
    Serial.begin(9600);
    Serial.println(F("Lilypad MP3 Player - adapted by Bulat Rakhimberdiev, 2014-2015."));

    // ('F' places constant strings in program flash to save RAM)

    Serial.print(F("Free RAM = "));
    Serial.println(FreeRam(), DEC);
  }

  randomSeed(analogRead(1));
  
  
  // Set up I/O pins:

  pinMode(TRIG1, INPUT);
  digitalWrite(TRIG1, HIGH); // turn on weak pullup
  pinMode(MP3_CS, OUTPUT);
  pinMode(SHDN_GPIO1, OUTPUT);
  pinMode(ROT_B, INPUT);
  digitalWrite(ROT_B, HIGH); // turn on weak pullup
  pinMode(TRIG2_SDA, INPUT);
  digitalWrite(TRIG1, HIGH); // turn on weak pullup
  pinMode(TRIG3_SCL, INPUT);
  digitalWrite(TRIG1, HIGH); // turn on weak pullup
  pinMode(TRIG5_RXI, INPUT);
  digitalWrite(TRIG5_RXI, HIGH); // turn on weak pullup
  pinMode(TRIG4_TXO, INPUT);
  digitalWrite(TRIG4_TXO, HIGH); // turn on weak pullup
  pinMode(ROT_A, INPUT);
  digitalWrite(ROT_A, HIGH); // turn on weak pullup
  pinMode(ROT_SW, INPUT);
  // switch is common anode with external pulldown, do not turn on pullup
  pinMode(MP3_DREQ, INPUT);
  pinMode(ROT_LEDB, OUTPUT);
  pinMode(ROT_LEDG, OUTPUT);
  pinMode(MP3_DCS, OUTPUT);
  pinMode(MP3_RST, OUTPUT);
  pinMode(ROT_LEDR, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);


  setLEDcolor(YELLOW);

  // Turn off amplifier chip / turn on MP3 mode:

  digitalWrite(SHDN_GPIO1, LOW);

  // Initialize the SD card:

  if (debugging) Serial.println(F("Initializing SD card... "));
  
  for (int i = 0; i < 10; i++)
  {
    result = sd.begin(SD_SEL, SPI_FULL_SPEED);
    if (result == 1) break;
  }

  if (result != 1)
  {
    if (debugging) Serial.println(F("Failed to open SD card. Halting"));
    errorBlink(1,RED);
  }
  else 
    if (debugging) Serial.println(F("OK"));

  // Build the play list
  scanCard();
  if (tracks_found == 0)
  {
    if (debugging) Serial.println(F("No playable files (MP3, WAV, MID, MP4, WMA, FLA, OGG, AAC) in root folder. Halting."));
    errorBlink(1,YELLOW);
  }
  if (RANDOMIZE) randomizeTracks();
  
  
  // Get initial track:
  startOver();
  goToTrack(0);

  //Initialize the MP3 chip:

  if (debugging) Serial.println(F("Initializing MP3 chip... "));

  result = MP3player.begin();

  // Check result, 0 and 6 are OK:

  if((result != 0) && (result != 6))
  {
    if (debugging)
    {
      Serial.print(F("error "));
      Serial.println(result);
    }
    errorBlink(result,BLUE);
  }
  else
    if (debugging) Serial.println(F("OK"));

  // Set up interrupts. We'll use the standard external interrupt
  // pin for the rotary, but we'll use the pin change interrupt
  // library for the button:

  attachInterrupt(1,rotaryIRQ,CHANGE);
  PCintPort::attachInterrupt(ROT_SW, &buttonIRQ, CHANGE);
  rotary_change = false; //for some wierd reason, the global initiaization fails after a powerup/

  initVolumeControl();

  // Uncomment to get a directory listing of the SD card:
  //sd.ls(LS_R | LS_DATE | LS_SIZE);

  // Turn on amplifier chip:

  digitalWrite(SHDN_GPIO1, HIGH);
  delay(2);

}


void buttonIRQ()
{
  // Button press interrupt request function (IRQ).
  // This function is called *automatically* when the button
  // changes state.

  // Process rotary encoder button presses and releases, including
  // debouncing (extra "presses" from noisy switch contacts).

  // If button is pressed, the button_pressed flag is set to true.
  // (Manually set this to false after handling the change.)

  // If button is released, the button_released flag is set to true,
  // and button_downtime will contain the duration of the button
  // press in ms. (Set this to false after handling the change.)

  // Raw information from PinChangeInt library:

  // Serial.print("pin: ");
  // Serial.print(PCintPort::arduinoPin);
  // Serial.print(" state: ");
  // Serial.println(PCintPort::pinState);

  static boolean button_state = false;
  static unsigned long start, end;

  if ((PCintPort::pinState == HIGH) && (button_state == false)) 
    // Button was up, but is currently being pressed down
  {
    // Discard button presses too close together (debounce)
    start = millis();
    if (start > (end + 10)) // 10ms debounce timer
    {
      button_state = true;
      button_pressed = true;
    }
  }
  else if ((PCintPort::pinState == LOW) && (button_state == true))
    // Button was down, but has just been released
  {
    // Discard button releases too close together (debounce)
    end = millis();
    if (end > (start + 10)) // 10ms debounce timer
    {
      button_state = false;
      button_released = true;
      button_downtime = end - start;
    }
  }
}


void rotaryIRQ()
{
  // Rotary encoder interrupt request function (IRQ).
  // This function is called *automatically* when the
  // rotary encoder changes state.

  // Process input from the rotary encoder.
  // The rotary "position" is held in rotary_counter, increasing
  // for CW rotation (changes by one per detent).

  // If the position changes, rotary_change will be set true.
  // (You may manually set this to false after handling the change).

  // This function will automatically run when rotary encoder input A
  // transitions in either direction (low to high or high to low).
  // By saving the state of the A and B pins through two interrupts,
  // we'll determine the direction of rotation.

  // Int rotary_counter will be updated with the new value, and boolean
  // rotary_change will be true if there was a value change.

  // Based on concepts from Oleg at circuits@home (http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros)
  // Unlike Oleg's original code, this code uses only one interrupt and
  // has only two transition states; it has less resolution but needs only
  // one interrupt, is very smooth, and handles switchbounce well.

  static unsigned char rotary_state = 0; // Current and previous encoder states

  rotary_state <<= 2;  // Remember previous state
  rotary_state |= (digitalRead(ROT_A) | (digitalRead(ROT_B) << 1));  // Mask in current state
  rotary_state &= 0x0F; // Zero upper nybble

  //Serial.println(rotary_state,HEX);

  if (rotary_state == 0x09) // From 10 to 01, increment counter. Also try 0x06 if unreliable.
  {
    rotary_counter++;
    rotary_direction = true;
    rotary_change = true;
  }
  else if (rotary_state == 0x03) // From 00 to 11, decrement counter. Also try 0x0C if unreliable.
  {
    rotary_counter--;
    rotary_direction = false;
    rotary_change = true;
  }
}


void loop()
{
  // "Static" variables are initalized once the first time
  // the loop runs, but they keep their values through
  // successive loops.

  static uint8_t current_mode = UNDEFINED_MODE;
  static boolean playing = true; //play on startup

  if (current_mode == UNDEFINED_MODE) current_mode = setMode(KNOB_UP_MODE, playing);

  // rotaryIRQ() sets the flag rotary_counter to true
  // if the knob position has changed. We can use this flag
  // to do something in the main loop() each time there's
  // a change. We'll clear this flag when we're done, which
  // lets us run this if() once for each change.

  if (rotary_change)
  {
    if (debugging)
    {
      Serial.print(F("Rotary changed, direction "));
      Serial.println(rotary_direction);
    }
    switch (current_mode)
    {
    case KNOB_UP_MODE:
      switchTrack(rotary_direction, playing);
      break;
    case KNOB_DOWN_MODE:
      current_mode = setMode(VOLUME_CONTROL_MODE, playing);
      changeVolume(rotary_direction);
      break;
    case VOLUME_CONTROL_MODE:
      changeVolume(rotary_direction);
      break;
    }
    rotary_change = false; // Clear flag
  }

  // The button IRQ also sets several flags to true, one for
  // button_pressed, one for button_released. Like the rotary
  // flag, we'll clear these when we're done handling them:

  if (button_pressed)
  {
    if (debugging) Serial.println(F("Button pressed."));

    switch (current_mode)
    {
    case KNOB_UP_MODE:
      current_mode = setMode(KNOB_DOWN_MODE, playing);
      break;
    default:
      //This should not happen
      if (debugging) Serial.println(F("Unexpected: button pressed in while already down!"));
      break;
    }
    button_pressed = false; // Clear flag

  }

  if (button_released)
  {
    if (debugging)
    {
      Serial.println(F("Button released."));
    }

    switch (current_mode)
    {
    case KNOB_DOWN_MODE:
      playing = !playing;
      current_mode = setMode(KNOB_UP_MODE, playing);
      break;
    case VOLUME_CONTROL_MODE:
      current_mode = setMode(KNOB_UP_MODE, playing);
      break;
    default:
      //This should not happen
      if (debugging) Serial.println(F("Unexpected: button released in while already up!"));
      break;
    }
    button_released = false; // Clear flags
  }

  // Handle "last track ended" situations
  // (should we play the next track?)

  // Are we in "playing" mode, and has the
  // current file ended?

  if (playing && !MP3player.isPlaying())
  {
    if (debugging) Serial.println(F("Done playing track"));
    if (loop_all)
    {
      if (debugging) Serial.println(F("Switching to next track"));
      switchTrack(true, true);
    }
    else
      playing = false;
  }
}

uint8_t setMode(uint8_t targetMode, boolean playing)
{
  Serial.print(F("Setting mode to: "));
  Serial.println(targetMode);
  Serial.print(F("Playing: "));
  Serial.println(playing);

  switch (targetMode)
  {
  case KNOB_UP_MODE:
    if (playing)
    {
      if (!MP3player.isPlaying()) startPlaying();
      setLEDcolor(BLUE);
    }
    else
    {
      if (MP3player.isPlaying()) stopPlaying();
      setLEDcolor(RED);
    }
    break;
  case KNOB_DOWN_MODE:
    setLEDcolor(YELLOW);
    break;
  case VOLUME_CONTROL_MODE:
    setLEDcolor(GREEN);
    break;
  default:
    setLEDcolor(OFF);
  }
  return targetMode;
}

void switchTrack(boolean direction, boolean playing)
{

  // Before switching to a new audio file, we MUST
  // stop playing before accessing the SD directory:

  if (playing)
    stopPlaying();

  // Get the next file:

  if (direction)
    getNextTrack();
  else
    getPrevTrack();

  // If we were previously playing, let's start again!

  if (playing) startPlaying(); 

  if (debugging)
  {
    Serial.print(F("current track "));
    Serial.println(track);
  }
}

void changeVolume(boolean direction)
{
  // Increment or decrement the volume.
  // This is handled internally in the VS1053 MP3 chip.
  // Lower numbers are louder (0 is the loudest).
  // Useful volume levels are between ~150 (lowest) and 0 (loudest), so stay between these.

  if (volume < 149 && direction == false)
    volume += 2;

  if (volume > 1 && direction == true)
    volume -= 2;

  setAndStoreVolume();

  if (debugging)
  {
    Serial.print(F("volume "));
    Serial.println(volume);
  }
}

int add_loopover(int sum, int low, int high)
{
  int range = high - low + 1;
  if (range < 1) return -1; //invalid thresholds
  if (sum > high) return low + (sum % range);
  if (sum < low) return high - ((low - sum) % range) + 1;
  return sum;
}

void getNextTrack()
{
  goToTrack(add_loopover(current_track + 1, 0, tracks_found - 1));
}


void getPrevTrack()
{
  goToTrack(add_loopover(current_track - 1, 0, tracks_found - 1));
}

void scanCard()
{
  if (debugging) Serial.println(F("Scanning directory..."));
  sd.chdir("/",true); // Index beginning of root directory
  int file_index = 0;
  tracks_found = 0;
  for (int i = 0; i < MAX_TRACKS; i++)
  {
    if (file.openNext(sd.vwd(), O_READ))
    {

      file.getFilename(track);
      if (isPlayable())
      {
        track_files[i] = file_index;
        tracks_found++;
        if (debugging)
        {
          Serial.print(F("Adding file "));
          Serial.print(track);
          Serial.print(F(" (id "));
          Serial.print(file_index);
          Serial.print(F(") as track "));
          Serial.println(i);
        }
      }
      file.close();
      file_index++;
      if (file_index == 0xFFFF) break;
    }
    else
    {
      break;
    }
  }
  Serial.print("Tracks found: ");
  Serial.println(tracks_found);
}

void randomizeTracks()
{
  int target_position;
  uint16_t tmp;
  for (int i = 0; i < tracks_found; i++)
  {
    target_position = int(random(0, tracks_found));
    if (debugging)
    {
      Serial.print(F("Random position generated: "));
      Serial.println(target_position);
    }
    tmp = track_files[target_position];
    track_files[target_position] = track_files[i];
    track_files[i] = tmp;
  }
}

void startOver()
{
  sd.chdir("/",true); // Index beginning of root directory
  current_track = 0;
}

void goToTrack(int target_track)
{
  uint16_t target_file = track_files[target_track];
  uint16_t current_file = track_files[current_track];
  if (target_file > current_file)
  {
    for (uint16_t i = current_file; i < target_file; i++)
    {
      file.close();
      file.openNext(sd.vwd(), O_READ);
    }
  }
  else
  {
    startOver();
    for (uint16_t i = 0; i <= target_file; i++)
    {
      file.close();
      file.openNext(sd.vwd(), O_READ);
    }
  }
  file.getFilename(track);
  current_track = target_track;
  if (debugging)
  {
    Serial.print(F("Moved to track "));
    Serial.print(current_track);
    Serial.print(F(", file "));
    Serial.print(target_file);
    Serial.print(F(": "));
    Serial.println(track);
  }
}

void startPlaying()
{
  setLEDcolor(CYAN);

  int result;

  if (debugging)
  {
    Serial.print(F("playing "));
    Serial.print(track);
    Serial.print(F("..."));
  }  

  mute(); //this is a hack to suppress the click at the start of playback

  result = MP3player.playMP3(track);
  if (result == 0)
  {
    delay(50);
    unmute();
    setLEDcolor(BLUE);
  }


  if (debugging)
  {
    Serial.print(F(" result "));
    Serial.println(result);  
  }
}


void stopPlaying()
{
  if (debugging) Serial.println(F("stopping playback"));
  mute();
  MP3player.stopTrack();
}


boolean isPlayable()
{
  // Check to see if a filename has a "playable" extension.
  // This is to keep the VS1053 from locking up if it is sent
  // unplayable data.

  char *extension;

  extension = strrchr(track,'.');
  extension++;
  if (
  (strcasecmp(extension,"MP3") == 0) ||
    (strcasecmp(extension,"WAV") == 0) ||
    (strcasecmp(extension,"MID") == 0) ||
    (strcasecmp(extension,"MP4") == 0) ||
    (strcasecmp(extension,"WMA") == 0) ||
    (strcasecmp(extension,"FLA") == 0) ||
    (strcasecmp(extension,"OGG") == 0) ||
    (strcasecmp(extension,"AAC") == 0)
    )
    return true;
  else
    return false;
}


void setLEDcolor(unsigned char color)
{
  // Set the RGB LED in the (optional) rotary encoder
  // to a specific color. See the color #defines at the
  // start of this sketch.

  digitalWrite(ROT_LEDR,color & B001);
  digitalWrite(ROT_LEDG,color & B010);
  digitalWrite(ROT_LEDB,color & B100);  
}


void errorBlink(int blinks, byte color)
{
  // This function will blink the RGB LED in the rotary encoder
  // (optional) a given number of times and repeat forever.
  // This is so you can see error codes without having to use
  // the serial monitor window.

  int x;

  while(true) // Loop forever
  {
    for (x=0; x < blinks; x++) // Blink a given number of times
    {
      setLEDcolor(color);
      delay(250);
      setLEDcolor(OFF);
      delay(250);
    }
    delay(1500); // Longer pause between blink-groups
  }
}

boolean eepromValid()
{
  uint8_t eepromSchemaVersion;
  eepromSchemaVersion = EEPROM.read(EEPROM_SCHEMA_ADDRESS);
  return (eepromSchemaVersion == EEPROM_SCHEMA);
}

void initEeprom()
{
  EEPROM.write(EEPROM_SCHEMA_ADDRESS, EEPROM_SCHEMA);
  if (debugging)
  {
    Serial.println(F("Initted EEPROM."));
  }
}

void storeVolume()
{
  if (!eepromValid()) initEeprom();
  EEPROM.write(EEPROM_VOLUME_ADDRESS, volume);
  if (debugging)
  {
    Serial.print(F("Stored volume to EEPROM: "));
    Serial.println(volume);
  }
}

uint8_t volumeFromStore()
{
  if (eepromValid())
  {
    return EEPROM.read(EEPROM_VOLUME_ADDRESS);
  }
  else
  {
    return 0;
  }
}

void initVolumeControl()
{
  if (eepromValid())
  {
    volume = volumeFromStore();
    if (debugging)
    {
      Serial.print(F("Read volume from EEPROM: "));
      Serial.println(volume);
    }
  }
  setAndStoreVolume();
}

void setAndStoreVolume()
{
  MP3player.setVolume(volume, volume); // Set player volume (same for both left and right channels)
  if (volume != volumeFromStore()) storeVolume();
}

void mute()
{
  MP3player.setVolume(0xFEFE,0xFEFE);
}

void unmute()
{
  MP3player.setVolume(volume,volume);
}















