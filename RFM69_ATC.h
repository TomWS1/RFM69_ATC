// **********************************************************************************
// Automatic Transmit Power Control class derived from RFM69 library.
// **********************************************************************************
// Copyright Thomas Studwell (2014,2015)
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#ifndef RFM69_ATC_h
#define RFM69_ATC_h
#include <RFM69.h>

class RFM69_ATC: public RFM69 {
  public:
    static volatile byte ACK_RSSI_REQUESTED;  // new flag in CTL byte to request RSSI with ACK (could potentially be merged with ACK_REQUESTED)

    RFM69_ATC(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, uint8_t interruptNum=RF69_IRQ_NUM) :
      RFM69(slaveSelectPin, interruptPin, isRFM69HW, interruptNum) {
    }
    
    bool initialize(uint8_t freqBand, uint8_t ID, uint8_t networkID=1);
    void sendACK(const void* buffer = "", uint8_t bufferSize=0);

    void setHighPower(bool onOFF=true, byte PA_ctl=0x60); //have to call it after initialize for RFM69HW
    void setPowerLevel(uint8_t level); // reduce/increase transmit power level
    int  enableAutoPower(int targetRSSI=-69);   // TWS: New method to enable/disable auto Power control
    int  getAckRSSI(void);                      // TWS: New method to retrieve the ack'd RSSI (if any)
    byte setLNA(byte newReg);                   // TWS: function to control LNA reg for power testing purposes
    void setMode(byte mode);  // TWS: moved from protected to try to build block()/unblock() wrapper

    int  _targetRSSI;     // if non-zero then this is the desired end point RSSI for our transmission
    byte _transmitLevel;  // saved powerLevel in case we do auto power adjustment, this value gets dithered

  protected:
    void interruptHook(uint8_t CTLbyte);
    void sendFrame(byte toAddress, const void* buffer, byte size, bool requestACK=false, bool sendACK=false);  // Need this one to match the RFM69 prototype.
    void sendFrame(byte toAddress, const void* buffer, byte size, bool requestACK, bool sendACK, bool sendRSSI, int lastRSSI);

    int16_t _ackRSSI;         // this contains the RSSI our destination Ack'd back to us (if we enabledAutoPower)
    bool    _powerBoost;      // this controls whether we need to turn on the highpower regs based on the setPowerLevel input
    uint8_t _PA_Reg;          // saved and derived PA control bits so we don't have to spend time reading back from SPI port

    void receiveBegin();
    void setHighPowerRegs(bool onOff);

};

#endif
