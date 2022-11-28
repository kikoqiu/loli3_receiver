#pragma once
#include "Arduino.h"

#if not defined(IBusIBusSerial)
#define IBusIBusSerial IBusSerial
#endif

class IBus {
  private:
    int len;
    int checksum;
  public:
    IBus(int numChannels);
  
    void begin();
    void end();
    void write(unsigned short);
    
};



/*
 * See IbusReader.cs for details about the IBUS protocol
 */

#define COMMAND40 0x40

IBus::IBus(int numChannels) {
  len = 4 + numChannels*2;
}

void IBus::begin() {
  checksum = 0xffff - len - COMMAND40;
  IBusSerial.write(len);
  IBusSerial.write(COMMAND40);
}

void IBus::end() {
  // write checksum
  IBusSerial.write(checksum & 0xff);
  IBusSerial.write(checksum >> 8);
}

/*
 * Write 16bit value in little endian format and update checksum
 */
void IBus::write(unsigned short val) {
  byte b = val & 0xff;
  IBusSerial.write(b);
  checksum -= b;
  b = val >> 8;
  IBusSerial.write(b);
  checksum -= b;
}