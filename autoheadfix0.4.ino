#include <AccelStepper.h>
#include <HX711.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>

//#define SEED_LEVER_ENABLED

#define MOUSE_LOADCELL_DOUT_PIN 2
#define MOUSE_LOADCELL_SCK_PIN 3
#define ARM_LOADCELL_DOUT_PIN 15
#define ARM_LOADCELL_SCK_PIN 14
#define START_PIN 4
// TODO : sleep
#define SEED_DIRECTION_PIN 5
#define SEED_STEP_PIN 6
#define SEED_LIMIT_SWITCH 13
#ifdef SEED_LEVER_ENABLED
#define SEED_LEVER 9
#endif  
#define HEADPOST_PIN_A 8
#define HEADPOST_PIN_B 0
#define HEADPOST_DIRECTION_PIN 12
#define HEADPOST_STEP_PIN 11
#define HEADPOST_LIMIT_SWITCH 7
#define CHIP_SELECT 10

AccelStepper * seedStepper = new AccelStepper(AccelStepper::DRIVER, SEED_STEP_PIN, SEED_DIRECTION_PIN);
AccelStepper * headpostStepper = new AccelStepper(AccelStepper::DRIVER, HEADPOST_STEP_PIN, HEADPOST_DIRECTION_PIN);
HX711 mouseScale;
HX711 armbarScale;

#define SPS 80.0f
#define SAMPLES 10

const float samplePeriod = (1000.0f/SPS)*SAMPLES;

float loadBuffer[10];
float runningSum = 0.0f;
int p = 0;

#define HEADFIX_SPEED 20000.0f
#define HEADFIX_ACCEL 10000.0f
#define HEADFIX_DISTANCE 40

#define SEED_SPEED 20000.0f
#define SEED_ACCEL 5000.0f
#define SEED_FETCH_DISTANCE 950
#define SEED_PRESENT_DISTANCE 150

#define COUNTS_PER_GRAM 1.507554220188709e+03

enum SeedState { IDLE = 0, RETRIEVING_SEED = 1, SEED_RETRIEVED = 2, RAISING_SEED = 3, SEED_READY = 4, PRESENTING_SEED = 5, SEED_AVAILABLE = 6 };
volatile SeedState seedState = IDLE;

enum HeadpostState { DISENGAGED = 0, FREE = 1, FIXING = 2, SETTLING = 3, FIXED = 4, RELEASING = 5 };
volatile HeadpostState headpostState = DISENGAGED;

unsigned long startFixation = 0;
unsigned long fixationTime = 23000;
unsigned long startFree = 0;
unsigned long freeTime = 500; // gotta give the mice time to get away
const bool isHeadFix = true;
const bool isTimedPresentation = true;
const unsigned long presentationInterval = 5000;
const unsigned long fixationDelta = 2000;
const unsigned int nFixesPerIncrement = 5;
const unsigned int nStrugglesPerIncrement = 1;
float struggleThreshold = 10.0f;
float rewardThreshold = 5.0f;
const float struggleDelta = 1.0f;
float headfixLoad = 0.0f;
float currentLoad = 0.0f;
float currentForce = 0.0f;
unsigned int successfulFixes = 0;
unsigned int selfReleases = 0;

unsigned long startPresentation = 0;
unsigned long endPresentation = 0;
const unsigned long presentationTime = 1000;
/*unsigned long startRetraction = 0;
const unsigned long retractionTime = 10000;*/

// make sure to update the header written to the SD card as well (TODO : there must be a better way)
struct LogEntry {
  unsigned long t = 0;
  byte seedState = IDLE;
  byte headpostState = DISENGAGED;
  byte padding[2] = {0, 0};
  unsigned long fixationTime = fixationTime;
  float mouseWeight = 0.0f;
  float struggleThreshold = struggleThreshold;
  float armbarForce = 0.0f;
  float rewardThreshold = rewardThreshold;
};

bool isSDCardPresent;
File logFile;
String logFileName;

LogEntry logEntry;

void setup() {
  // put your setup code here, to run once:
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(SEED_DIRECTION_PIN, OUTPUT);
  pinMode(SEED_STEP_PIN, OUTPUT);
  pinMode(SEED_LIMIT_SWITCH, INPUT);
#ifdef SEED_LEVER_ENABLED
  pinMode(SEED_LEVER, INPUT_PULLUP);
#endif
  pinMode(HEADPOST_PIN_A, INPUT_PULLUP);
  pinMode(HEADPOST_PIN_B, INPUT_PULLUP);
  pinMode(HEADPOST_DIRECTION_PIN, OUTPUT);
  pinMode(HEADPOST_STEP_PIN, OUTPUT);
  pinMode(HEADPOST_LIMIT_SWITCH, INPUT_PULLUP);

  seedStepper->setMaxSpeed(SEED_SPEED);
  seedStepper->setAcceleration(SEED_ACCEL);

  headpostStepper->setMaxSpeed(HEADFIX_SPEED);
  headpostStepper->setAcceleration(HEADFIX_ACCEL);

  Serial.begin(115200);

  mouseScale.begin(MOUSE_LOADCELL_DOUT_PIN, MOUSE_LOADCELL_SCK_PIN);
  mouseScale.set_gain(64); // TODO : why?
  mouseScale.tare(80);
  mouseScale.set_scale(COUNTS_PER_GRAM);

#ifdef SEED_LEVER_ENABLED
  armbarScale.begin(ARM_LOADCELL_DOUT_PIN, ARM_LOADCELL_SCK_PIN);
  armbarScale.set_gain(64);
  armbarScale.tare(80);
  armbarScale.set_scale(COUNTS_PER_GRAM); // TODO : calibrate
#endif

  for (int ii = 0; ii < SAMPLES; ++ii) {
    loadBuffer[ii] = mouseScale.get_units();
  }

  p = SAMPLES-1;

  isSDCardPresent = SD.begin(CHIP_SELECT);

  if (!isSDCardPresent) {
    Serial.println("# No SD card present.");
    return;
  }

  RTC_PCF8523 rtc; // the rtc doesn't have the time resolution that we need for logging, so just use to name the log file

  if (!rtc.begin()) {
    Serial.println("# RTC not present or malfunctioning.");
    return;
  }

  if (!rtc.initialized()) {
    Serial.println("# Initialising RTC with current date and time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  DateTime now = rtc.now();

  // fat32 filenames are at most 8 characters long, so only store the month, day, hour, and minute in the filename
#ifdef __AVR__
  char logFileNamePrefix[8];

  int success = sprintf(logFileNamePrefix, "%02d%02d%02d%02d", now.month(), now.day(), now.hour(), now.minute());
#else
  // For reasons I cannot fathom, writing to the last character of a fixed-
  // length char array cause Dues but not AVR-based boards to crash. However, if
  // you make the array one bigger, the conversion to String includes the
  // trailing null. To get around this, on Due we write to an array one bigger 
  // necessary, then assign a char array of undefined length the same address,
  // with some filthy casts along the way.
  char * logFileNamePrefix;
  char logFileNamePrefixTemp[9];

  int success = sprintf(logFileNamePrefixTemp, "%02d%02d%02d%02d", now.month(), now.day(), now.hour(), now.minute());
#endif

  if (success < 1 || success > 8) {
    // TODO : actually test this with the new SAM3X-safe code on both AVR-based boards and Due
    logFileName = (char*)"logfile.dat";
  } else {
#ifndef __AVR__
    logFileNamePrefix = (char*)logFileNamePrefixTemp;
#endif
    logFileName = String((char*)logFileNamePrefix) + ".dat";
  }

  Serial.print("# The log file name is ");
  Serial.println(logFileName);

  Serial.print("# sizeof(LogEntry) is ");
  Serial.println(sizeof(LogEntry));

  if (SD.exists(logFileName)) {
    SD.remove(logFileName);
  }

  logFile = SD.open(logFileName, FILE_WRITE);

  if (isSDCardPresent && logFile) {
    int totalWritten = 0;
  
    totalWritten += logFile.println("// Automatic Head-Fixation Trainer for Mouse Food-Handling");
    totalWritten += logFile.println("{");
    totalWritten += logFile.println("  \"version\": 0.3,");
    totalWritten += logFile.print("  \"log_entry_size\": ");
    totalWritten += logFile.print(sizeof(LogEntry));
    totalWritten += logFile.println(",");
    totalWritten += logFile.println("  \"log_entry_format\": {");
    totalWritten += logFile.println("    \"time\": \"uint32\",");
    totalWritten += logFile.println("    \"seed_state\": \"uint8\",");
    totalWritten += logFile.println("    \"headpost_state\": \"uint8\",");
    totalWritten += logFile.println("    \"padding\": \"uint16\",");
    totalWritten += logFile.println("    \"fixation_time\": \"uint32\",");
    totalWritten += logFile.println("    \"mouse_weight\": \"float32\",");
    totalWritten += logFile.println("    \"struggle_threshold\": \"float32\",");
    totalWritten += logFile.println("    \"armbar_force\": \"float32\",");
    totalWritten += logFile.println("    \"reward_threshold\": \"float32\"");
    totalWritten += logFile.println("  },");
    totalWritten += logFile.print("  \"initial_head_fixation_time\": ");
    totalWritten += logFile.print(fixationTime);
    totalWritten += logFile.println(",");
    totalWritten += logFile.print("  \"head_fixation_time_increment\": ");
    totalWritten += logFile.print(fixationDelta);
    totalWritten += logFile.println(",");
    totalWritten += logFile.print("  \"successes_per_increment\": ");
    totalWritten += logFile.print(nFixesPerIncrement);
    totalWritten += logFile.println(",");
    totalWritten += logFile.print("  \"initial_self_release_threshold\": ");
    totalWritten += logFile.print(struggleThreshold);
    totalWritten += logFile.println(",");
    totalWritten += logFile.print("  \"self_release_threshold_increment\": ");
    totalWritten += logFile.print(struggleDelta);
    totalWritten += logFile.println(",");
    totalWritten += logFile.print("  \"struggles_per_increment\": ");
    totalWritten += logFile.print(nStrugglesPerIncrement);
    totalWritten += logFile.println(",");
    totalWritten += logFile.print("  \"load_cell_scale\": ");
    totalWritten += logFile.print(COUNTS_PER_GRAM);
    totalWritten += logFile.println(",");
    totalWritten += logFile.print("  \"logging_state_time\": ");
    char format[22] = "\"YYYY-MM-DD hh:mm:ss\"";
    totalWritten += logFile.print(rtc.now().toString(format));
    totalWritten += logFile.println(",");
    totalWritten += logFile.print("  \"header_size\": ");
    totalWritten += 24; // four digits plus comma \r\n plus closing brace plus \r\n plus // DATA START plus \r\n
    
    char buffer[4];
    sprintf(buffer,"%04d",totalWritten);
    byte lastWrite = logFile.println(buffer);
    lastWrite += logFile.println("}");
    lastWrite += logFile.println("// DATA START");
  
    if (lastWrite != 24) {
      Serial.print("# error: last write should have been 24 bytes but actual number of bytes was: ");
      Serial.print(lastWrite);
      while (true) {
        // don't start the state machine if this happens
      }
    }
  }
}

void loop() {
  /*String toPrint = "";
  toPrint += millis();
  toPrint += "\t";
  toPrint += seedState;
  toPrint += "\t";
  toPrint += headpostState;
  toPrint += "\t";
  toPrint += successfulFixes;
  toPrint += "\t";
  toPrint += fixationTime;
  toPrint += "\t";
  toPrint += headfixLoad;*/

#ifdef SEED_LEVER_ENABLED

  if (armbarScale.is_ready()) {
    currentForce = armbarScale.get_units(); // TODO : low pass filter like for mouse weight?
    Serial.println(currentForce);
  }
#endif

  if (mouseScale.is_ready()) {
    p = (p + 1) % SAMPLES;
    runningSum -= loadBuffer[(++p) % SAMPLES];
    float reading = mouseScale.get_units();
    runningSum += reading;
    loadBuffer[p] = reading;
    currentLoad = runningSum/SAMPLES;
    logEntry.t = millis();

    if (isSDCardPresent & logFile) {
      logEntry.seedState = seedState;
      logEntry.headpostState = headpostState;
      logEntry.fixationTime = fixationTime;
      logEntry.mouseWeight = currentLoad;
      logEntry.struggleThreshold = struggleThreshold;
      logEntry.armbarForce = currentForce;
      logEntry.rewardThreshold = rewardThreshold;
      
      logFile.write((byte*)&logEntry,sizeof(LogEntry));

      if (!seedStepper->run() && !headpostStepper->run()) { // don't flush if we're in the middle of a motor move
        logFile.flush();
      }
    }
    
    /*Serial.print(logEntry.t);
    Serial.print("\t");
    Serial.println(currentLoad);*/
  }
  
  //Serial.println(toPrint);
  
  switch (seedState) {
    case IDLE:
      if (digitalRead(SEED_LIMIT_SWITCH) == HIGH) {
        seedStepper->setSpeed(-1000.0f);
        seedStepper->runSpeed();
      } else {
        seedStepper->setCurrentPosition(0);
      
        if (digitalRead(START_PIN) == LOW) {
          seedState = SEED_RETRIEVED;
          
          if (isHeadFix) {
            headpostState = FREE;
          }
        }
      }
      break;
    case RETRIEVING_SEED:
      if (!seedStepper->run()) {
        //startRetraction = millis();
        seedState = SEED_RETRIEVED;
      }
      break;
    case SEED_RETRIEVED:
      if (!isHeadFix || !isTimedPresentation || millis() - endPresentation > presentationInterval) {
        seedStepper->move(SEED_FETCH_DISTANCE);        
        seedState = RAISING_SEED;
      }
      break;
    case RAISING_SEED:
      if (!seedStepper->run()) {
          seedState = SEED_READY;
      }
      break;
    case SEED_READY:
      if ((isTimedPresentation || abs(currentForce) > rewardThreshold) && ((isHeadFix && (headpostState == FIXING || headpostState == SETTLING || headpostState == FIXED)) || (!isHeadFix && digitalRead(HEADPOST_PIN_A) == LOW && digitalRead(HEADPOST_PIN_B) == LOW))) {
        seedStepper->move(SEED_PRESENT_DISTANCE);
        seedState = PRESENTING_SEED;
      }
      break;
    case PRESENTING_SEED:
      if (!seedStepper->run()) {
        startPresentation = millis();
        seedState = SEED_AVAILABLE;
      }
      break;
    case SEED_AVAILABLE:
      if (millis() - startPresentation > presentationTime) {
        endPresentation = millis();
        seedStepper->move(-SEED_PRESENT_DISTANCE-SEED_FETCH_DISTANCE);
        seedState = RETRIEVING_SEED;
      }
      break;
    default:
      break;
  }

  switch (headpostState) {
    case DISENGAGED:
      if (isHeadFix && digitalRead(HEADPOST_LIMIT_SWITCH) == HIGH) {
        headpostStepper->setSpeed(-1000.0f);
        headpostStepper->runSpeed();
      } else {
        headpostStepper->setCurrentPosition(0);
      }
      break;
    case FREE:
      endPresentation = 0;
      if (/*seedState == SEED_READY && */millis() - startFree > freeTime && digitalRead(HEADPOST_PIN_A) == LOW && digitalRead(HEADPOST_PIN_B) == LOW) {
        headpostStepper->move(HEADFIX_DISTANCE);
        headpostState = FIXING;
      }
      break;
    case FIXING:
      if (!headpostStepper->run()) {
        startFixation = millis();
        headpostState = SETTLING;
      }  
      break;
    case SETTLING:
      if (millis() - startFixation >= samplePeriod) {
        headfixLoad = currentLoad;
        headpostState = FIXED;
      }
      break;
    case FIXED:
      if (currentLoad < headfixLoad - struggleThreshold || currentLoad > headfixLoad + struggleThreshold) {
        // self-release: reset successful fix counter and increment self release counter
        successfulFixes = 0;
        if (++selfReleases >= nStrugglesPerIncrement) {
          selfReleases = 0;
          struggleThreshold += struggleDelta;
        }
        headpostState = RELEASING;
      } else if (digitalRead(HEADPOST_PIN_A) == HIGH || digitalRead(HEADPOST_PIN_B) == HIGH) {
        // mechanical head-fixation failure: do not increment either counter
        headpostState = RELEASING;
      } else if (millis() - startFixation > fixationTime) {
        // timeout: reset self release counter and increment successful fix counter
        selfReleases = 0;
        if (++successfulFixes >= nFixesPerIncrement) {
          successfulFixes = 0;
          fixationTime += fixationDelta;
        }
        headpostState = RELEASING;
      }
      break;
    case RELEASING:
      // keep retracting until we hit the limit switch
      headpostStepper->move(-HEADFIX_DISTANCE);
      headpostStepper->run();

      if (digitalRead(HEADPOST_LIMIT_SWITCH) == LOW) {
        headpostStepper->setCurrentPosition(0);
        startFree = millis();
        headpostState = FREE;
      }
      break;
    default:
      break;
  }
}
