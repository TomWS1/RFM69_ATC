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
#include <RFM69_ATC.h>
#include <RFM69.h>   // include the RFM69 library files as well
#include <RFM69registers.h>
#include <SPI.h>

#define  LISTEN_HIGHSPEED  // use 200Kbps for Listen Mode

//#define USE_SERIAL

#ifdef USE_SERIAL
#define PRINT(x) Serial.print(x)
#define PRINTLN(x) Serial.println(x)
#define PRINT2(x,y) Serial.print(x,y)
#define PRINTLN2(x,y) Serial.println(x,y)
#else
#define PRINT(x) 
#define PRINTLN(x) 
#define PRINT2(x,y) 
#define PRINTLN2(x,y) 
#endif
#ifdef USE_SERIAL
  #define CHECKPOINT { PRINT(__FUNCTION__); PRINTLN(__LINE__); }
#else
  #define CHECKPOINT
#endif


volatile byte RFM69_ATC::ACK_RSSI_REQUESTED;  // new type of flag on ACK_REQUEST

//=============================================================================
// initialize() - some extra initialization before calling base class
//=============================================================================
bool RFM69_ATC::initialize(uint8_t freqBand, uint8_t nodeID, uint8_t networkID)
{

  _targetRSSI = 0;        // TWS: default to disabled
  _ackRSSI = 0;           // TWS: no existing response yet...
  ACK_RSSI_REQUESTED = 0; // TWS: init to none
  
  _powerBoost = false;    // TWS: require someone to explicitly turn boost on!
  _transmitLevel = 31;    // TWS: match default value in PA Level register...

#ifdef SPI_HAS_TRANSACTION
  SPI.usingInterrupt(_interruptNum);
#else
  #error "MUST HAVE SPI_HAS_TRANSACTION DEFINED!"
#endif
  writeReg( REG_OPMODE, RF_OPMODE_LISTENABORT | RF_OPMODE_STANDBY );  // make sure Listen mode is aborted
  writeReg( REG_OPMODE, RF_OPMODE_STANDBY );
  writeReg( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY );
  
  bool rc = RFM69::initialize(freqBand, nodeID, networkID);  // use base class to initialize most everything
  PRINT("transmit level after init:"); PRINTLN(_transmitLevel);
  return rc;
}

//=============================================================================
// restoreInit() - use base class initialization, but keep current powerlevel settings and existing RSSI controls
//=============================================================================
void RFM69_ATC::restoreInit(uint8_t freqBand, uint8_t nodeID, uint8_t networkID)
{
  uint8_t 
    saveTransmitLevel = _transmitLevel;

  _ackRSSI = 0;           // TWS: no existing response yet...
  ACK_RSSI_REQUESTED = 0; // TWS: init to none
  writeReg( REG_OPMODE, RF_OPMODE_LISTENABORT | RF_OPMODE_STANDBY );  // make sure Listen mode is aborted
  writeReg( REG_OPMODE, RF_OPMODE_STANDBY );
  writeReg( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY );
  
  RFM69::initialize(freqBand, nodeID, networkID);  // use base class to re-initialize most everything
    
  if (_isRFM69HW)  // set HighPower ONLY if RFM69HW type radio, otherwise use internal PA
    setHighPower(true, 0x20);          // might be overridden if transmitLevel >31...
  setPowerLevel(saveTransmitLevel);  // but restore transmit level to last value
}



//=============================================================================
// select() - replaces base class with one supporting spiTransactions
//=============================================================================
// select the transceiver
void RFM69_ATC::select() {
  // save current SPI settings
  _SPCR = SPCR;
  _SPSR = SPSR;
  _SREG = SREG;
#ifdef SPI_HAS_TRANSACTION
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
#else
  noInterrupts();
  // set RFM69 SPI settings
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
#endif
  digitalWrite(_slaveSelectPin, LOW);
}


//=============================================================================
// unselect() - replaces base class with one supporting spiTransactions
//=============================================================================
// UNselect the transceiver chip
void RFM69_ATC::unselect() {
  digitalWrite(_slaveSelectPin, HIGH);
#ifdef SPI_HAS_TRANSACTION
  SPI.endTransaction();
#else  
  SREG = _SREG;
#endif
  // restore SPI settings to what they were before talking to RFM69
  SPCR = _SPCR;
  SPSR = _SPSR;
}


//=============================================================================
// setMode() - got to set updated transmit power level before switching to TX mode
//=============================================================================
void RFM69_ATC::setMode(uint8_t newMode)
{
  if (newMode == _mode)
    return;

  _powerBoost = (_transmitLevel >= 50);  // this needs to be set before changing mode just in case setHighPowerRegs is called...
  RFM69::setMode(newMode);  // call base class first
    
  if (newMode == RF69_MODE_TX)  // special stuff if switching to TX mode
  {
      if (_targetRSSI) setPowerLevel(_transmitLevel);   // TWS: apply most recent transmit level if auto power
      if (_isRFM69HW) setHighPowerRegs(true);
  }

}

//=============================================================================
// sendAck() - updated to call new sendFrame with additional parameters
//=============================================================================
// should be called immediately after reception in case sender wants ACK
void RFM69_ATC::sendACK(const void* buffer, uint8_t bufferSize) 
{
  uint8_t 
    sender = SENDERID;
  int16_t 
    _RSSI = RSSI; // save payload received RSSI value
  bool 
    sendRSSI = ACK_RSSI_REQUESTED;  
    
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  
  uint32_t 
    now = millis();
    
  while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS) receiveDone();
  
  SENDERID = sender;    // TWS: Restore SenderID after it gets wiped out by receiveDone()
  sendFrame(sender, buffer, bufferSize, false, true, sendRSSI, _RSSI);   // TWS: Special override on sendFrame with extra params
  RSSI = _RSSI; // restore payload RSSI
}


//=============================================================================
// sendFrame() - the basic version is used to match the RFM69 prototype so we can extend it
//=============================================================================
// this sendFrame is generally called by the internal RFM69 functions.  Simply transfer to our modified version.
void RFM69_ATC::sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{
   sendFrame(toAddress, buffer, bufferSize, requestACK, sendACK, false, 0);  // default sendFrame
}

//=============================================================================
// sendFrame() - the new one with additional parameters.  This packages recv'd RSSI with the packet, if required.
//=============================================================================
void RFM69_ATC::sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK, bool sendRSSI, int16_t lastRSSI)
{
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  
  bufferSize += (sendACK && sendRSSI)?2:0;  // if sending ACK_RSSI then increase data size by two
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

  // write to FIFO
  select();
  SPI.transfer(REG_FIFO | 0x80);
  SPI.transfer(bufferSize + 3);
  SPI.transfer(toAddress);
  SPI.transfer(_address);

  // control byte
  if (sendACK) {              // TWS: adding logic to return ACK_RSSI if requested
    SPI.transfer(RFM69_CTL_SENDACK | (sendRSSI?RFM69_CTL_RESERVE1:0));  // TWS  TODO: Replace with EXT1
    if (sendRSSI) {
      SPI.transfer(lastRSSI>>8);    // TWS: send high byte
      SPI.transfer(lastRSSI & 0x00ff);  // TWS: send low byte
      bufferSize -=2;           // account for these two 'data' bytes
    }
  }
  else if (requestACK) {      // TWS: need to add logic to request ackRSSI with ACK
    if (_targetRSSI)          // TWS
      SPI.transfer(RFM69_CTL_REQACK | RFM69_CTL_RESERVE1);     // TWS: ASK for ACK + ASK for RSSI
    else                      // TWS:
     SPI.transfer(RFM69_CTL_REQACK);
  }
  else SPI.transfer(0x00);

  for (uint8_t i = 0; i < bufferSize; i++)
    SPI.transfer(((uint8_t*) buffer)[i]);
  unselect();

  // no need to wait for transmit mode to be ready since its handled by the radio
  setMode(RF69_MODE_TX);
  uint32_t txStart = millis();
  while (digitalRead(_interruptPin) == 0 && millis() - txStart < RF69_TX_LIMIT_MS); // wait for DIO0 to turn HIGH signalling transmission finish
  //while (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady
  setMode(RF69_MODE_STANDBY);
}


//=============================================================================
// receiveDone() - added extra code to base class implementation to completely restore SREG
//=============================================================================
bool RFM69_ATC::receiveDone() {
  uint8_t rd_SREG = SREG;
  noInterrupts();
  // Note: New SPI library prefers to use EIMSK (external interrupt mask) if available 
  // to mask (only) interrupts registered via SPI::usingInterrupt(). It only 
  // falls back to disabling ALL interrupts SREG if EIMSK cannot be used.
  // Hence We cannot assume that methods calls below that call select() unselect() 
  // will result in a call to noInterrupts(). 
  // Thus the code below needs to explicitly do this to be safe.
  if (_mode == RF69_MODE_RX && PAYLOADLEN > 0)
  {
    setMode(RF69_MODE_STANDBY); 
    SREG = rd_SREG;   // enables interrupts
    return true;
  }
  else if (_mode == RF69_MODE_RX) // already in RX no payload yet
  {
    SREG = rd_SREG;
    return false;
  }
  receiveBegin();
  SREG = rd_SREG;
  return false;
}

//=============================================================================
// interruptHook() - gets called by the base class interrupt handler right after the header is fetched.
//=============================================================================
void RFM69_ATC::interruptHook(uint8_t CTLbyte) {

  ACK_RSSI_REQUESTED = CTLbyte & RFM69_CTL_RESERVE1; // TWS: extract the ACK RSSI request bit (could potentially merge with ACK_REQUESTED)
  
  // TWS: now see if this was an ACK with an ACK_RSSI response
  if (ACK_RECEIVED && ACK_RSSI_REQUESTED) {
    // the next two bytes contain the ACK_RSSI (assuming the datalength is valid)
    if (DATALEN >= 2) {
      _ackRSSI = SPI.transfer(0)<<8;  // get high byte
      _ackRSSI |= (SPI.transfer(0) & 0xff);  // merge with low
      DATALEN -= 2;   // and compensate data length accordingly
      // TWS: Now dither transmitLevel value (register update occurs later when transmitting);
      if (_targetRSSI != 0) {
        if (_isRFM69HW) {
          if (_ackRSSI < _targetRSSI && _transmitLevel < 51) _transmitLevel++;
          else if (_ackRSSI > _targetRSSI && _transmitLevel > 32) _transmitLevel--;
        } else {
          if (_ackRSSI < _targetRSSI && _transmitLevel < 31) _transmitLevel++;
          else if (_ackRSSI > _targetRSSI && _transmitLevel > 0) _transmitLevel--;
        }
      }
    }
  }
}


//=============================================================================
//  receiveBegin() - need to clear out our flag before calling base class.
//=============================================================================
void RFM69_ATC::receiveBegin() {
  ACK_RSSI_REQUESTED = 0;
  RFM69::receiveBegin();
}


//=============================================================================
// setPowerLevel() - outright replacement for base class.  Provides finer granularity for RFM69HW.
//=============================================================================
// set output power: 0=min, 31=max (for RFM69W or RFM69CW), 0-31 or 32->51 for RFM69HW (see below)
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
void RFM69_ATC::setPowerLevel(uint8_t powerLevel)
{
  // TWS Update: allow power level selections above 31.  Select appropriate PA based on the value
  _transmitLevel = powerLevel;    // save this for later in case we do auto power control.
  _powerBoost = (powerLevel >= 50);
  
  if (!_isRFM69HW || powerLevel < 32) {     // use original code without change
    _powerLevel = powerLevel;
    writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0xE0) | (_powerLevel > 31 ? 31 : _powerLevel));
  } else {
    // the allowable range of power level value, if >31 is: 32 -> 51, where...
    // 32->47 use PA2 only and sets powerLevel register 0-15,
    // 48->49 uses both PAs, and sets powerLevel register 14-15,
    // 50->51 uses both PAs, sets powerBoost, and sets powerLevel register 14-15.
    if (powerLevel < 48) {
      _powerLevel = powerLevel & 0x0f;  // just use 4 lower bits when in high power mode
      _PA_Reg = 0x20;
    } else {
      _PA_Reg = 0x60;
      if (powerLevel < 50) {
        _powerLevel = powerLevel - 34;  // leaves 14-15
      } else {
        if (powerLevel > 51) 
          powerLevel = 51;  // saturate
        _powerLevel = powerLevel - 36;  // leaves 14-15
      }
    }
    writeReg(REG_OCP, (_PA_Reg==0x60) ? RF_OCP_OFF : RF_OCP_ON);
    writeReg(REG_PALEVEL, _powerLevel | _PA_Reg);
  }
}


//=============================================================================
// setHighPower() - only set High power bits on RFM69HW IFF the power level is set to MAX.  Otherwise it is kept off.
//=============================================================================
void RFM69_ATC::setHighPower(bool onOff, byte PA_ctl) {
  _isRFM69HW = onOff;
    
  writeReg(REG_OCP, (_isRFM69HW && PA_ctl==0x60) ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW) { //turning ON based on module type 
    _powerLevel = readReg(REG_PALEVEL) & 0x1F; // make sure internal value matches reg
    _powerBoost = (PA_ctl == 0x60);
    _PA_Reg = PA_ctl;
    writeReg(REG_PALEVEL, _powerLevel | PA_ctl ); //TWS: enable selected P1 & P2 amplifier stages
  }
  else {
   _PA_Reg = RF_PALEVEL_PA0_ON;        // TWS: save to reflect register value
    writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); //enable P0 only
  }
}

//=============================================================================
// ditto from above.
//=============================================================================
void RFM69_ATC::setHighPowerRegs(bool onOff) {
  if ((0x60 != (readReg(REG_PALEVEL) & 0xe0)) || !_powerBoost)    // TWS, only set to high power if we are using both PAs... and boost range is requested.
    onOff = false;
  writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

//=============================================================================
// enableAutoPower() - call with target RSSI, use 0 to disable (default), any other value with turn on autotransmit control.
//=============================================================================
// TWS: New methods to address autoPower control
int  RFM69_ATC::enableAutoPower(int targetRSSI){    // TWS: New method to enable/disable auto Power control
  _targetRSSI = targetRSSI;         // no logic here, just set the value (if non-zero, then enabled), caller's responsibility to use a reasonable value
}
  

//=============================================================================
// getAckRSSI() - returns the RSSI value ack'd by the far end.
//=============================================================================
int  RFM69_ATC::getAckRSSI(void){                     // TWS: New method to retrieve the ack'd RSSI (if any)
  return (_targetRSSI==0?0:_ackRSSI);
}

//=============================================================================
// setLNA() - used for power level testing.
//=============================================================================
byte RFM69_ATC::setLNA(byte newReg) {  // TWS: New method used to disable LNA AGC for testing purposes
  byte oldReg;
  
  oldReg = readReg(REG_LNA);
  
  writeReg(REG_LNA, ((newReg & 7) | (oldReg & ~7)));   // just control the LNA Gain bits for now
  
  return oldReg;  // return the original value in case we need to restore it
}


//=============================================================================
// static pointer to 'this' needed by irq handler
//=============================================================================
static RFM69_ATC*  pRadio;

//=============================================================================
// irq handler, simply calls listenIrq method so internal methods can be accessed easily
//=============================================================================
static void irq() 
{
  pRadio->listenIrq();
}


//=============================================================================
// listenIrq() - only called by listen irq handler
//=============================================================================
void RFM69_ATC::listenIrq(void)
{
  if( listenReceivedSize != 0 ) return;
  
  noInterrupts();
  
  select();
  SPI.transfer(REG_FIFO & 0x7F);
  uint8_t len = SPI.transfer(0);
  uint8_t tnode = SPI.transfer(0);
  union                     // union to simplify addressing of long and short parts of time offset
  {
    uint32_t l;
    uint8_t  b[4];
  } offset;
  
  offset.l = 0;            // initialize full word to 0 so we only lower bytes
  
  if( len-- > 3 ) {          // include 2 bytes for time slot
    offset.b[0] =  SPI.transfer( 0 );  // and get the time slot
    offset.b[1] =  SPI.transfer( 0 );
    len = len > (listenMaxSize+2) ? listenMaxSize : len-2; // precaution (include the extra two bytes for time slot
    for( uint8_t i = 0; i < len; i++ ) {
      uint8_t b = SPI.transfer( 0 );
      listenBuffer[i] = b;
    }
  }
  
  unselect();
  interrupts();
  
  if( tnode == _address || tnode == RF69_BROADCAST_ADDR )
  {
    listenReceivedSize = len;
    receivedOffset = offset.l;  // save the time slot offset
  }
}


//=============================================================================
// startListening() - switch radio to Listen Mode in prep for sleep until burst
//=============================================================================
void RFM69_ATC::startListening(void* buf, uint8_t size )
{
  pRadio = this;
  
  listenMaxSize = size;
  listenBuffer = (byte*)buf;

  listenReceivedSize = 0;

  detachInterrupt( RF69_IRQ_NUM );
  attachInterrupt( RF69_IRQ_NUM, irq, RISING);
  encrypt( 0 );   // added to ensure encryption is off during listening mode
  setMode( RF69_MODE_STANDBY );
  
  writeReg( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 );

  writeReg( REG_FRFMSB, readReg( REG_FRFMSB) + 1);
  writeReg( REG_FRFLSB, readReg( REG_FRFLSB));      // MUST write to LSB to affect change!
  

#ifdef LISTEN_HIGHSPEED
  writeReg( REG_BITRATEMSB, RF_BITRATEMSB_200000);
  writeReg( REG_BITRATELSB, RF_BITRATELSB_200000);
  writeReg( REG_FDEVMSB, RF_FDEVMSB_100000 );
  writeReg( REG_FDEVLSB, RF_FDEVLSB_100000 );
  writeReg( REG_RXBW, RF_RXBW_DCCFREQ_000 | RF_RXBW_MANT_16 | RF_RXBW_EXP_0 );
#endif
  
  writeReg( REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF );

  writeReg( REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_WHITENING | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON  );
  writeReg( REG_SYNCVALUE1, 0x5A );
  writeReg( REG_SYNCVALUE2, 0x5A );
  writeReg( REG_LISTEN1, RF_LISTEN1_RESOL_RX_64 | RF_LISTEN1_RESOL_IDLE_262000 |
          RF_LISTEN1_CRITERIA_RSSI | RF_LISTEN1_END_10 );
  writeReg( REG_LISTEN2, 11 );
#ifdef LISTEN_HIGHSPEED
  writeReg( REG_LISTEN3, 4 );
#else 
  writeReg( REG_LISTEN3, 6 );
#endif  

  writeReg( REG_RSSITHRESH, 200 );
  writeReg( REG_RXTIMEOUT2, 75 );
  
  writeReg( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY );
  writeReg( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_ON  | RF_OPMODE_STANDBY );
}


//=============================================================================
// listenReceivedBytes() - returns the number of bytes received in listen burst
//=============================================================================
uint8_t RFM69_ATC::listenReceivedBytes(void)
{
  return listenReceivedSize;
}

//=============================================================================
// listenReceivedOffset() - returns the time offset of when this radio saw the burst.
//=============================================================================
uint32_t RFM69_ATC::listenReceivedOffset(void)
{
  return receivedOffset;
}

//=============================================================================
// clearListenBuffer() - empty the listen buffer prior to sleeping.
//=============================================================================
void RFM69_ATC::clearListenBuffer(void)
{
  listenReceivedSize = 0;
  receivedOffset = 0;
}

//=============================================================================
// endListening() - stop the radio from listenMode
//=============================================================================
void RFM69_ATC::endListening( bool reInitialize )
{
  detachInterrupt( RF69_IRQ_NUM );
  writeReg( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTENABORT | RF_OPMODE_STANDBY );
  writeReg( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY );
  writeReg( REG_RXTIMEOUT2, 0 );
  setMode( RF69_MODE_STANDBY );
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

}

//=============================================================================
// sendBurst() - send a burst of packets to a sleeping listening node (or all)
//=============================================================================
void RFM69_ATC::sendBurst( uint8_t targetNode, void* buffer, uint8_t size )
{
  detachInterrupt( RF69_IRQ_NUM );
  encrypt( 0 );
  setMode( RF69_MODE_STANDBY );

  writeReg( REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_WHITENING | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON  );
  writeReg( REG_SYNCVALUE1, 0x5A );
  writeReg( REG_SYNCVALUE2, 0x5A );

  writeReg( REG_FRFMSB, readReg( REG_FRFMSB) + 1);
  writeReg( REG_FRFLSB, readReg( REG_FRFLSB));      // MUST write to LSB to affect change!

#ifdef LISTEN_HIGHSPEED
  writeReg( REG_BITRATEMSB, RF_BITRATEMSB_200000);
  writeReg( REG_BITRATELSB, RF_BITRATELSB_200000);
  writeReg( REG_FDEVMSB, RF_FDEVMSB_100000 );
  writeReg( REG_FDEVLSB, RF_FDEVLSB_100000 );
  writeReg( REG_RXBW, RF_RXBW_DCCFREQ_000 | RF_RXBW_MANT_16 | RF_RXBW_EXP_0 );
#endif  

  
  
  setMode( RF69_MODE_TX );
  
  uint32_t time = millis();
  union                     // union to simplify addressing of long and short parts of time offset
  {
    uint32_t l;
    uint8_t  b[4];
  } offset;
  
  offset.l = 0;            // first offset is naturally 0
    
  while( offset.l < 3000 ) 
  {

    noInterrupts();
    // write to FIFO
    select();
    SPI.transfer(REG_FIFO | 0x80);
    SPI.transfer( size + 3);      // add an extra 2 bytes for time offset of this packet
    SPI.transfer( targetNode );
    SPI.transfer( offset.b[0] );  // send the time offset for this packet
    SPI.transfer( offset.b[1] );

    for (uint8_t i = 0; i < size; i++)
        SPI.transfer(((uint8_t*) buffer)[i]);
    unselect();
    interrupts();
    
    while ((readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY) != 0x00);  // make sure packet is sent before putting more into the FIFO
    offset.l = millis() - time;  // get the 'time' of the next packet
    
  }
  setMode( RF69_MODE_STANDBY );
}


