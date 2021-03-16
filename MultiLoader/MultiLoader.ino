/* TODO
   Fix menu scrolling
   Add external interrupt for read data WORKING
*/

#define DEBUG true

#include <SdFat.h>
#include "U8glib.h"
#include <avr/wdt.h>

//--------------------Setup pins----------------------
#define uiKeyUp A0
#define uiKeyDown A1
#define uiKeyOk A2
#define uiKeyCancel 8

#define MSPIM_SCK 3  // port D bit 4
#define MSPIM_SS 5  // port D bit 5
#define BB_MISO 6  // port D bit 6
#define BB_MOSI 7  // port D bit 7

#define BB_MISO_PORT PIND
#define BB_MOSI_PORT PORTD
#define BB_SCK_PORT PORTD
#define BB_SCK_BIT 3
#define BB_MISO_BIT 6
#define BB_MOSI_BIT 7
//--------------------Setup pins----------------------

#define KEY_NONE 0
#define KEY_UP 1
#define KEY_DOWN 2
#define KEY_OK 5
#define KEY_CANCEL 6

// target board reset goes to here
const byte RESET = MSPIM_SS;

// SD chip select pin
#define chipSelect 4
//--------------------Setup pins----------------------

uint8_t uiKeyCodeFirst = KEY_NONE;
uint8_t uiKeyCodeSecond = KEY_NONE;
uint8_t uiKeyCode = KEY_NONE;

String menu_strings[20]; //20
uint8_t fileCounter = 0;

uint8_t menu_current = 0;
uint8_t menu_redraw_required = 0;
uint8_t last_key_code = KEY_NONE;

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);  // I2C / TWI
SdFat sd;

typedef enum {  // status "messages"
  MSG_NO_SD_CARD,                     // cannot open SD card
  MSG_CANNOT_OPEN_FILE,               // canoot open file 'wantedFile' (above)
  MSG_LINE_TOO_LONG,                  // line on disk too long to read
  MSG_LINE_TOO_SHORT,                 // line too short to be valid
  MSG_LINE_DOES_NOT_START_WITH_COLON, // line does not start with a colon
  MSG_INVALID_HEX_DIGITS,             // invalid hex where there should be hex
  MSG_BAD_SUMCHECK,                   // line fails sumcheck
  MSG_LINE_NOT_EXPECTED_LENGTH,       // record not length expected
  MSG_UNKNOWN_RECORD_TYPE,            // record type not known
  MSG_NO_END_OF_FILE_RECORD,          // no 'end of file' at end of file
  MSG_FILE_TOO_LARGE_FOR_FLASH,       // file will not fit into flash

  MSG_CANNOT_ENTER_PROGRAMMING_MODE,  // cannot program target chip
  MSG_NO_BOOTLOADER_FUSE,             // chip does not have bootloader
  MSG_CANNOT_FIND_SIGNATURE,          // cannot find chip signature
  MSG_UNRECOGNIZED_SIGNATURE,         // signature not known
  MSG_BAD_START_ADDRESS,              // file start address invalid
  MSG_VERIFICATION_ERROR,             // verification error after programming
  MSG_FLASHED_OK,                     // flashed OK

  MSG_PROCESS_ACTION,                 // action process
  MSG_PROCESS_VERIFICATION,           // verifying
  MSG_PROCESS_WRITING,                // writing
} msgType;

const unsigned int ENTER_PROGRAMMING_ATTEMPTS = 2;

// control speed of programming
const byte BB_DELAY_MICROSECONDS = 6;

const unsigned long NO_PAGE = 0xFFFFFFFF;
const int MAX_FILENAME = 13;

// actions to take
enum {
  checkFile,
  verifyFlash,
  writeToFlash,
};

// copy of fuses/lock bytes found for this processor
byte fuses [5];

// meaning of bytes in above array
enum {
  lowFuse,
  highFuse,
  extFuse,
  lockByte,
  calibrationByte
};

// structure to hold signature and other relevant data about each chip
typedef struct {
  byte sig [3];
  const char * desc;
  unsigned long flashSize;
  unsigned int baseBootSize;
  unsigned long pageSize;     // bytes
  byte fuseWithBootloaderSize;  // ie. one of: lowFuse, highFuse, extFuse
  byte timedWrites;    // if pollUntilReady won't work by polling the chip
} signatureType;

const unsigned long kb = 1024;
const byte NO_FUSE = 0xFF;

// see Atmega datasheets
const signatureType signatures [] PROGMEM =
{
  //     signature        description   flash size   bootloader  flash  fuse
  //                                                     size    page    to
  //                                                             size   change
  // Atmega328 family
  //{ { 0x1E, 0x92, 0x0A }, "ATmega48PA",   4 * kb,         0,    64,  NO_FUSE, false },
  //{ { 0x1E, 0x93, 0x0F }, "ATmega88PA",   8 * kb,       256,   128,  extFuse, false },
  //{ { 0x1E, 0x94, 0x0B }, "ATmega168PA", 16 * kb,       256,   128,  extFuse, false },
  { { 0x1E, 0x95, 0x0F }, "ATmega328P",  32 * kb,       512,   128,  highFuse, false }
};  // end of signatures

char name[MAX_FILENAME] = { 0 };  // current file name

// number of items in an array
#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

// programming commands to send via SPI to the chip
enum {
  progamEnable = 0xAC,

  // writes are preceded by progamEnable
  chipErase = 0x80,
  writeLockByte = 0xE0,
  writeLowFuseByte = 0xA0,
  writeHighFuseByte = 0xA8,
  writeExtendedFuseByte = 0xA4,

  pollReady = 0xF0,

  programAcknowledge = 0x53,

  readSignatureByte = 0x30,
  readCalibrationByte = 0x38,

  readLowFuseByte = 0x50,       readLowFuseByteArg2 = 0x00,
  readExtendedFuseByte = 0x50,  readExtendedFuseByteArg2 = 0x08,
  readHighFuseByte = 0x58,      readHighFuseByteArg2 = 0x08,
  readLockByte = 0x58,          readLockByteArg2 = 0x00,

  readProgramMemory = 0x20,
  writeProgramMemory = 0x4C,
  loadExtendedAddressByte = 0x4D,
  loadProgramMemory = 0x40,

};  // end of enum

// which program instruction writes which fuse
const byte fuseCommands [4] = { writeLowFuseByte, writeHighFuseByte, writeExtendedFuseByte, writeLockByte };

// types of record in .hex file
enum {
  hexDataRecord,  // 00
  hexEndOfFile,   // 01
  hexExtendedSegmentAddressRecord, // 02
  hexStartSegmentAddressRecord,  // 03
  hexExtendedLinearAddressRecord, // 04
  hexStartLinearAddressRecord // 05
};

void ShowMessage (const byte which)
{
#if DEBUG
  switch (which)
  {
    // problems with SD card or finding the file
    case MSG_NO_SD_CARD:                      Serial.println (F("No SD card")); break;
    case MSG_CANNOT_OPEN_FILE:                Serial.println (F("Cannot open file")); break;

    // problems reading the .hex file
    case MSG_LINE_TOO_LONG:                   Serial.println (F("Line too long")); break;
    case MSG_LINE_TOO_SHORT:                  Serial.println (F("Line too short")); break;
    case MSG_LINE_DOES_NOT_START_WITH_COLON:  Serial.println (F("Line doesn't start with colon")); break;
    case MSG_INVALID_HEX_DIGITS:              Serial.println (F("Invalid HEX digits")); break;
    case MSG_BAD_SUMCHECK:                    Serial.println (F("Bad checksumm")); break;
    case MSG_LINE_NOT_EXPECTED_LENGTH:        Serial.println (F("Line not expected lenght")); break;
    case MSG_UNKNOWN_RECORD_TYPE:             Serial.println (F("Unknown record type")); break;
    case MSG_NO_END_OF_FILE_RECORD:           Serial.println (F("No end of file record")); break;

    // problems with the file contents
    case MSG_FILE_TOO_LARGE_FOR_FLASH:        Serial.println (F("File too large for flash")); break;

    // problems programming the chip
    case MSG_CANNOT_ENTER_PROGRAMMING_MODE:   Serial.println (F("Cannot enter to programming mode")); break;
    case MSG_NO_BOOTLOADER_FUSE:              Serial.println (F("No bootloader fuse")); break;
    case MSG_CANNOT_FIND_SIGNATURE:           Serial.println (F("Cannot find signature")); break;
    case MSG_UNRECOGNIZED_SIGNATURE:          Serial.println (F("Unrecognized signature")); break;
    case MSG_BAD_START_ADDRESS:               Serial.println (F("Bad start adress")); break;
    case MSG_VERIFICATION_ERROR:              Serial.println (F("Verification error")); break;
    case MSG_FLASHED_OK:                      Serial.println (F("Flashed ok!!!")); break;
    case MSG_PROCESS_ACTION:                  Serial.println(F("Action...")); break;
    case MSG_PROCESS_VERIFICATION:            Serial.println(F("Verifying...")); break;
    case MSG_PROCESS_WRITING:            Serial.println(F("Writing...")); break;

    default:                                   Serial.println (F("---")); break;   // unknown error
  }
#endif //DEBUG
}

// Bit Banged SPI transfer
byte BB_SPITransfer (byte c)
{
  byte bit;

  for (bit = 0; bit < 8; bit++)
  {
    // write MOSI on falling edge of previous clock
    if (c & 0x80)
      BB_MOSI_PORT |= bit (BB_MOSI_BIT);
    else
      BB_MOSI_PORT &= ~bit (BB_MOSI_BIT);
    c <<= 1;

    // read MISO
    c |= (BB_MISO_PORT & bit (BB_MISO_BIT)) != 0;

    // clock high
    BB_SCK_PORT |= bit (BB_SCK_BIT);

    // delay between rise and fall of clock
    delayMicroseconds (BB_DELAY_MICROSECONDS);

    // clock low
    BB_SCK_PORT &= ~bit (BB_SCK_BIT);

    // delay between rise and fall of clock
    delayMicroseconds (BB_DELAY_MICROSECONDS);
  }

  return c;
}  // end of BB_SPITransfer


// if signature found in above table, this is its index
int foundSig = -1;
byte lastAddressMSB = 0;
// copy of current signature entry for matching processor
signatureType currentSignature;

// execute one programming instruction ... b1 is command, b2, b3, b4 are arguments
//  processor may return a result on the 4th transfer, this is returned.
byte program (const byte b1, const byte b2 = 0, const byte b3 = 0, const byte b4 = 0)
{
  noInterrupts ();
  BB_SPITransfer (b1);
  BB_SPITransfer (b2);
  BB_SPITransfer (b3);
  byte b = BB_SPITransfer (b4);
  interrupts ();
  return b;
} // end of program

// read a byte from flash memory
byte readFlash (unsigned long addr)
{
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB)
  {
    program (loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }  // end if different MSB

  return program (readProgramMemory | high, highByte (addr), lowByte (addr));
} // end of readFlash

// write a byte to the flash memory buffer (ready for committing)
void writeFlash (unsigned long addr, const byte data)
{
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address
  program (loadProgramMemory | high, 0, lowByte (addr), data);
} // end of writeFlash


// convert two hex characters into a byte
//    returns true if error, false if OK
bool hexConv (const char * (& pStr), byte & b)
{

  if (!isxdigit (pStr [0]) || !isxdigit (pStr [1]))
  {
    ShowMessage (MSG_INVALID_HEX_DIGITS);
    return true;
  } // end not hex

  b = *pStr++ - '0';
  if (b > 9)
    b -= 7;

  // high-order nybble
  b <<= 4;

  byte b1 = *pStr++ - '0';
  if (b1 > 9)
    b1 -= 7;

  b |= b1;

  return false;  // OK
}  // end of hexConv

// poll the target device until it is ready to be programmed
void pollUntilReady ()
{
  if (currentSignature.timedWrites)
    delay (10);  // at least 2 x WD_FLASH which is 4.5 mS
  else
  {
    while ((program (pollReady) & 1) == 1)
    {}  // wait till ready
  }  // end of if
}  // end of pollUntilReady

unsigned long pagesize;
unsigned long pagemask;
unsigned long oldPage;
unsigned int progressBarCount;

// clear entire temporary page to 0xFF in case we don't write to all of it
void clearPage ()
{
  unsigned int len = currentSignature.pageSize;
  for (unsigned int i = 0; i < len; i++)
    writeFlash (i, 0xFF);
}  // end of clearPage

// commit page to flash memory
void commitPage (unsigned long addr)
{
  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB)
  {
    program (loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }  // end if different MSB

  ShowMessage(MSG_PROCESS_WRITING);

  program (writeProgramMemory, highByte (addr), lowByte (addr));
  pollUntilReady ();

  clearPage();  // clear ready for next page full
}  // end of commitPage

// write data to temporary buffer, ready for committing
void writeData (const unsigned long addr, const byte * pData, const int length)
{
  // write each byte
  for (int i = 0; i < length; i++)
  {
    unsigned long thisPage = (addr + i) & pagemask;
    // page changed? commit old one
    if (thisPage != oldPage && oldPage != NO_PAGE)
      commitPage (oldPage);
    // now this is the current page
    oldPage = thisPage;
    // put byte into work buffer
    writeFlash (addr + i, pData [i]);
  }  // end of for

}  // end of writeData

// count errors
unsigned int errors;

void verifyData (const unsigned long addr, const byte * pData, const int length)
{
  // check each byte
  for (int i = 0; i < length; i++)
  {
    unsigned long thisPage = (addr + i) & pagemask;
    // page changed? show progress
    if (thisPage != oldPage && oldPage != NO_PAGE)
      ShowMessage(MSG_PROCESS_VERIFICATION);
    // now this is the current page
    oldPage = thisPage;

    byte found = readFlash (addr + i);
    byte expected = pData [i];
    if (found != expected)
      errors++;
  }  // end of for

}  // end of verifyData

bool gotEndOfFile;
unsigned long extendedAddress;

unsigned long lowestAddress;
unsigned long highestAddress;
unsigned long bytesWritten;
unsigned int lineCount;

/*
  Line format:

  :nnaaaatt(data)ss

  Where:
  :      = a colon

  (All of below in hex format)

  nn     = length of data part
  aaaa   = address (eg. where to write data)
  tt     = transaction type
           00 = data
           01 = end of file
           02 = extended segment address (changes high-order byte of the address)
           03 = start segment address *
           04 = linear address *
           05 = start linear address *
  (data) = variable length data
  ss     = sumcheck

              We don't use these

*/

// returns true if error, false if OK
bool processLine (const char * pLine, const byte action)
{
  if (*pLine++ != ':')
  {
    ShowMessage (MSG_LINE_DOES_NOT_START_WITH_COLON);
    return true;  // error
  }

  const int maxHexData = 40;
  byte hexBuffer [maxHexData];
  int bytesInLine = 0;

  if (action == checkFile)
    if (lineCount++ % 40 == 0)
      ShowMessage(MSG_PROCESS_ACTION);

  // convert entire line from ASCII into binary
  while (isxdigit (*pLine))
  {
    // can't fit?
    if (bytesInLine >= maxHexData)
    {
      ShowMessage (MSG_LINE_TOO_LONG);
      return true;
    } // end if too long

    if (hexConv (pLine, hexBuffer [bytesInLine++]))
      return true;
  }  // end of while

  if (bytesInLine < 5)
  {
    ShowMessage (MSG_LINE_TOO_SHORT);
    return true;
  }

  // sumcheck it

  byte sumCheck = 0;
  for (int i = 0; i < (bytesInLine - 1); i++)
    sumCheck += hexBuffer [i];

  // 2's complement
  sumCheck = ~sumCheck + 1;

  // check sumcheck
  if (sumCheck != hexBuffer [bytesInLine - 1])
  {
    ShowMessage (MSG_BAD_SUMCHECK);
    return true;
  }

  // length of data (eg. how much to write to memory)
  byte len = hexBuffer [0];

  // the data length should be the number of bytes, less
  //   length / address (2) / transaction type / sumcheck
  if (len != (bytesInLine - 5))
  {
    ShowMessage (MSG_LINE_NOT_EXPECTED_LENGTH);
    return true;
  }

  // two bytes of address
  unsigned long addrH = hexBuffer [1];
  unsigned long addrL = hexBuffer [2];

  unsigned long addr = addrL | (addrH << 8);

  byte recType = hexBuffer [3];

  switch (recType)
  {
    // stuff to be written to memory
    case hexDataRecord:
      lowestAddress  = min (lowestAddress, addr + extendedAddress);
      highestAddress = max (lowestAddress, addr + extendedAddress + len - 1);
      bytesWritten += len;

      switch (action)
      {
        case checkFile:  // nothing much to do, we do the checks anyway
          break;

        case verifyFlash:
          verifyData (addr + extendedAddress, &hexBuffer [4], len);
          break;

        case writeToFlash:
          writeData (addr + extendedAddress, &hexBuffer [4], len);
          break;
      } // end of switch on action
      break;

    // end of data
    case hexEndOfFile:
      gotEndOfFile = true;
      break;

    // we are setting the high-order byte of the address
    case hexExtendedSegmentAddressRecord:
      extendedAddress = ((unsigned long) hexBuffer [4]) << 12;
      break;

    // ignore these, who cares?
    case hexStartSegmentAddressRecord:
    case hexExtendedLinearAddressRecord:
    case hexStartLinearAddressRecord:
      break;

    default:
      ShowMessage (MSG_UNKNOWN_RECORD_TYPE);
      return true;
  }  // end of switch on recType

  return false;
} // end of processLine

//------------------------------------------------------------------------------
// returns true if error, false if OK
bool readHexFile (const char * fName, const byte action)
{
  const int maxLine = 80;
  char buffer[maxLine];
  ifstream sdin (fName);
  int lineNumber = 0;
  gotEndOfFile = false;
  extendedAddress = 0;
  errors = 0;
  lowestAddress = 0xFFFFFFFF;
  highestAddress = 0;
  bytesWritten = 0;
  progressBarCount = 0;

  pagesize = currentSignature.pageSize;
  pagemask = ~(pagesize - 1);
  oldPage = NO_PAGE;

  // check for open error
  if (!sdin.is_open())
  {
    ShowMessage (MSG_CANNOT_OPEN_FILE);
    return true;
  }

  switch (action)
  {
    case checkFile:
      break;

    case verifyFlash:
      break;

    case writeToFlash:
      program (progamEnable, chipErase);   // erase it
      delay (20);  // for Atmega8
      pollUntilReady ();
      clearPage();  // clear temporary page
      break;
  } // end of switch

  while (sdin.getline (buffer, maxLine))
  {
    lineNumber++;
    int count = sdin.gcount();
    if (sdin.fail())
    {
      ShowMessage (MSG_LINE_TOO_LONG);
      return true;
    }  // end of fail (line too long?)

    // ignore empty lines
    if (count > 1)
    {
      if (processLine (buffer, action))
      {
        return true;  // error
      }
    }
  }    // end of while each line

  if (!gotEndOfFile)
  {
    ShowMessage (MSG_NO_END_OF_FILE_RECORD);
    return true;
  }

  switch (action)
  {
    case writeToFlash:
      // commit final page
      if (oldPage != NO_PAGE)
        commitPage (oldPage);
      break;

    case verifyFlash:
      if (errors > 0)
      {
        ShowMessage (MSG_VERIFICATION_ERROR);
        return true;
      }  // end if
      break;

    case checkFile:
      break;
  }  // end of switch

  return false;
}  // end of readHexFile


// returns true if managed to enter programming mode
bool startProgramming ()
{

  byte confirm;
  pinMode (RESET, OUTPUT);
  digitalWrite (MSPIM_SCK, LOW);
  pinMode (MSPIM_SCK, OUTPUT);
  pinMode (BB_MOSI, OUTPUT);
  unsigned int timeout = 0;

  // we are in sync if we get back programAcknowledge on the third byte
  do
  {
    // regrouping pause
    delay (100);

    // ensure SCK low
    noInterrupts ();
    digitalWrite (MSPIM_SCK, LOW);
    // then pulse reset, see page 309 of datasheet
    digitalWrite (RESET, HIGH);
    delayMicroseconds (10);  // pulse for at least 2 clock cycles
    digitalWrite (RESET, LOW);
    interrupts ();

    delay (25);  // wait at least 20 mS
    noInterrupts ();
    BB_SPITransfer (progamEnable);
    BB_SPITransfer (programAcknowledge);
    confirm = BB_SPITransfer (0);
    BB_SPITransfer (0);
    interrupts ();

    if (confirm != programAcknowledge)
    {
      if (timeout++ >= ENTER_PROGRAMMING_ATTEMPTS)
        return false;
    }  // end of not entered programming mode

  } while (confirm != programAcknowledge);
  return true;  // entered programming mode OK
}  // end of startProgramming

void stopProgramming ()
{
  // turn off pull-ups
  digitalWrite (RESET, LOW);
  digitalWrite (MSPIM_SCK, LOW);
  digitalWrite (BB_MOSI, LOW);
  digitalWrite (BB_MISO, LOW);

  // set everything back to inputs
  pinMode (RESET, INPUT);
  pinMode (MSPIM_SCK, INPUT);
  pinMode (BB_MOSI, INPUT);
  pinMode (BB_MISO, INPUT);

} // end of stopProgramming

void getSignature ()
{
  foundSig = -1;
  lastAddressMSB = 0;

  byte sig [3];
  for (byte i = 0; i < 3; i++)
  {
    sig [i] = program (readSignatureByte, 0, i);
  }  // end for each signature byte

  for (unsigned int j = 0; j < NUMITEMS (signatures); j++)
  {
    memcpy_P (&currentSignature, &signatures [j], sizeof currentSignature);

    if (memcmp (sig, currentSignature.sig, sizeof sig) == 0)
    {
      foundSig = j;
      // make sure extended address is zero to match lastAddressMSB variable
      program (loadExtendedAddressByte, 0, 0);
      return;
    }  // end of signature found
  }  // end of for each signature

  ShowMessage (MSG_UNRECOGNIZED_SIGNATURE);
}  // end of getSignature

void getFuseBytes ()
{
  fuses [lowFuse]   = program (readLowFuseByte, readLowFuseByteArg2);
  fuses [highFuse]  = program (readHighFuseByte, readHighFuseByteArg2);
  fuses [extFuse]   = program (readExtendedFuseByte, readExtendedFuseByteArg2);
  fuses [lockByte]  = program (readLockByte, readLockByteArg2);
  fuses [calibrationByte]  = program (readCalibrationByte);
}  // end of getFuseBytes


// write specified value to specified fuse/lock byte
void writeFuse (const byte newValue, const byte instruction)
{
  if (newValue == 0)
    return;  // ignore

  program (progamEnable, instruction, 0, newValue);
  pollUntilReady ();
}  // end of writeFuse

// returns true if error, false if OK
bool updateFuses (const bool writeIt)
{
  unsigned long addr;
  unsigned int  len;

  byte fusenumber = currentSignature.fuseWithBootloaderSize;

  // if no fuse, can't change it
  if (fusenumber == NO_FUSE)
  {
    //    ShowMessage (MSG_NO_BOOTLOADER_FUSE);   // maybe this doesn't matter?
    return false;  // ok return
  }

  addr = currentSignature.flashSize;
  len = currentSignature.baseBootSize;

  if (lowestAddress == 0)
  {
    // don't use bootloader
    fuses [fusenumber] |= 1;
  }
  else
  {
    byte newval = 0xFF;

    if (lowestAddress == (addr - len))
      newval = 3;
    else if (lowestAddress == (addr - len * 2))
      newval = 2;
    else if (lowestAddress == (addr - len * 4))
      newval = 1;
    else if (lowestAddress == (addr - len * 8))
      newval = 0;
    else
    {
      ShowMessage (MSG_BAD_START_ADDRESS);
      return true;
    }

    if (newval != 0xFF)
    {
      newval <<= 1;
      fuses [fusenumber] &= ~0x07;   // also program (clear) "boot into bootloader" bit
      fuses [fusenumber] |= newval;
    }  // if valid

  }  // if not address 0

  if (writeIt)
  {
    writeFuse (fuses [fusenumber], fuseCommands [fusenumber]);
  }

  return false;
}  // end of updateFuses

//------------------------------------------------------------------------------
//      SETUP
//------------------------------------------------------------------------------
void setup ()
{
  pinMode(uiKeyUp, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeyDown, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeyOk, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeyCancel, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(RESET, OUTPUT);
  Serial.begin(9600);

  digitalWrite(RESET, LOW);

  while (!sd.begin (chipSelect, SPI_FULL_SPEED))
  {
    ShowMessage (MSG_NO_SD_CARD);
    delay (1000);
  }

  printDirectory(sd.open("/"));
  menu_redraw_required = 1;     // force initial redraw

}  // end of setup

void printDirectory(File dir) {
  char TempNameVar[20];

  while (true) {
    File entry =  dir.openNextFile();
    if (!entry) break;
    if (!entry.isDir()) {
      entry.getName(TempNameVar, 20);
      Serial.println(TempNameVar);
      menu_strings[fileCounter] = TempNameVar;
      fileCounter++;
    }

    entry.close();
  }
}

// returns true if error, false if OK
bool chooseInputFile ()
{

  if (readHexFile(name, checkFile))
  {
    return true;  // error, don't attempt to write
  }

  // check file would fit into device memory
  if (highestAddress > currentSignature.flashSize)
  {
    ShowMessage (MSG_FILE_TOO_LARGE_FOR_FLASH);
    return true;
  }

  // check start address makes sense
  if (updateFuses (false))
  {
    return true;
  }

  return false;
}  // end of chooseInputFile

// returns true if OK, false on error
bool writeFlashContents ()
{

  errors = 0;

  if (chooseInputFile ())
    return false;

  // ensure back in programming mode
  if (!startProgramming ())
    return false;

  // now commit to flash
  if (readHexFile(name, writeToFlash))
    return false;

  // verify
  if (readHexFile(name, verifyFlash))
    return false;

  // now fix up fuses so we can boot
  if (errors == 0)
    updateFuses (true);

  return errors == 0;
}  // end of writeFlashContents

void uiStep(void) {
  uiKeyCodeSecond = uiKeyCodeFirst;

  if ( digitalRead(uiKeyUp) == LOW )
    uiKeyCodeFirst = KEY_UP;
  else if ( digitalRead(uiKeyDown) == LOW )
    uiKeyCodeFirst = KEY_DOWN;
  else if ( digitalRead(uiKeyOk) == LOW )
    uiKeyCodeFirst = KEY_OK;
  else if ( digitalRead(uiKeyCancel) == LOW )
    uiKeyCodeFirst = KEY_CANCEL;
  else
    uiKeyCodeFirst = KEY_NONE;

  if ( uiKeyCodeSecond == uiKeyCodeFirst )
    uiKeyCode = uiKeyCodeFirst;
  else
    uiKeyCode = KEY_NONE;
}

void drawMenu(void) {
  uint8_t i, h;
  u8g_uint_t w, d;

  u8g.setFont(u8g_font_6x13);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();

  h = u8g.getFontAscent() - u8g.getFontDescent();
  w = u8g.getWidth();

  for (uint8_t i = 0; i <= fileCounter; i++ ) {
    d = (w - u8g.getStrWidth(menu_strings[i].c_str())) / 2;
    u8g.setDefaultForegroundColor();
    if ( i == menu_current ) {
      u8g.drawBox(0, i * h + 1, w, h);
      u8g.setDefaultBackgroundColor();
    }
    u8g.drawStr(d, i * h, menu_strings[menu_current > 5 ? i + (menu_current - 5) : i].c_str());
  }
}

void updateMenu(void) {
  if ( uiKeyCode != KEY_NONE && last_key_code == uiKeyCode ) {
    return;
  }
  last_key_code = uiKeyCode;

  switch ( uiKeyCode ) {
    case KEY_UP:
      menu_current++;
      if ( menu_current >= fileCounter )
        menu_current = 0;
      menu_redraw_required = 1;
      break;
    case KEY_DOWN:
      if ( menu_current == 0 )
        menu_current = fileCounter;
      menu_current--;
      menu_redraw_required = 1;
      break;
    case KEY_OK:
      //Serial.println(menu_strings[menu_current]);
      flashChip(menu_strings[menu_current]);
      break;
  }
}

void flashChip(String filename) {
  strcpy (name, filename.c_str());   // use fixed name

  if (!startProgramming ())
  {
    ShowMessage (MSG_CANNOT_ENTER_PROGRAMMING_MODE);
    return;
  }  // end of could not enter programming mode

  getSignature ();
  getFuseBytes ();

  // don't have signature? don't proceed
  if (foundSig == -1)
  {
    ShowMessage (MSG_CANNOT_FIND_SIGNATURE);
    return;
  }  // end of no signature

  bool ok = writeFlashContents ();
  stopProgramming ();
  delay (500);

  if (ok) {
    ShowMessage (MSG_FLASHED_OK);
    //forbid buttons click
    pinMode(uiKeyUp, OUTPUT);
  pinMode(uiKeyDown, OUTPUT);
  pinMode(uiKeyOk, OUTPUT);
  pinMode(uiKeyCancel, OUTPUT);
  }
}

//------------------------------------------------------------------------------
//      LOOP
//------------------------------------------------------------------------------

void loop ()
{
  uiStep();                                     // check for key press

  if (  menu_redraw_required != 0 ) {
    u8g.firstPage();
    do  {
      drawMenu();
    } while ( u8g.nextPage() );
    menu_redraw_required = 0;
  }

  updateMenu();                            // update menu bar
}  // end of loop

void serialEvent () {
  while (Serial.available() > 0) {
    String inputString = Serial.readString();
    Serial.println(inputString);
    if (inputString == "reboot") {
      if(DEBUG) Serial.println(F("-----------------Self-Reboot---------------"));
      delay(100);
      reboot();
    }
    else if (inputString.indexOf("load:") > -1) {
      inputString.remove(0, 5);
      if(DEBUG) Serial.println("Recived filename from UART: " + inputString);
      flashChip(inputString);
    }
    // очищаем строку:
    inputString = "";
  }
}

void reboot() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}
