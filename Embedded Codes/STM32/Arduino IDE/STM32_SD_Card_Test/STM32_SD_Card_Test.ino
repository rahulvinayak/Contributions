#include <STM32SD.h>


// If SD card slot has no detect pin then define it as SD_DETECT_NONE
// to ignore it. One other option is to call 'SD.begin()' without parameter.
#ifndef SD_DETECT_PIN
#define SD_DETECT_PIN SD_DETECT_NONE
#endif
HardwareSerial MySerial1(PB7, PB6); // RX, TX
File myFile;

void setup() {
  SD.setDx(PC8, PC9, PC10, PC11);
  SD.setCMD(PD2);
  SD.setCK(PC_12); // using PinName
  // Open serial communications and wait for port to open:
  MySerial1.begin(9600);
 

  MySerial1.print("Initializing SD card...");
  while (!SD.begin(SD_DETECT_PIN)) {
    delay(10);
  }
  MySerial1.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    MySerial1.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    MySerial1.println("done.");
  } else {
    // if the file didn't open, print an error:
    MySerial1.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) {
    MySerial1.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      MySerial1.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    MySerial1.println("error opening test.txt");
  }
  if (!SD.end()) {
    MySerial1.println("Failed to properly end the SD.");
  }
  MySerial1.println("###### End of the SD tests ######");
}

void loop() {
  // nothing happens after setup
}
