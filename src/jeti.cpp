#include <stdio.h>
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "uart_jeti_tx.pio.h"
#include "MS5611.h"
#include "SPL06.h"
#include "ms4525.h"
#include "sdp3x.h"
#include "jeti.h"
#include "tools.h"
#include "gps.h"
#include "param.h"
#include <inttypes.h> // used by PRIu32

// Gap of 20ms between 2 frames
// 9700 baud, 9 bits, 2 stops , odd parity
// bit8 = 0 for the synchro byte (0X7E), all other have bit8 = 1
// byte 0 = 0X7F 
// byte 1 = 0xnF (n= any value)
// byte 2 =  bits7...6 = type of frame (00 = txt, 01 = data); bits 5...0 = Length of next bytes includinhg CRC
// byte 3,4,5,6  = device id
// byte 7 = 0X00 (fix)
// for txt:
// byte 8 = id of the description
// byte 9 = bits7..3 = length of name ;  bits 2..0 = length of unit
// next bytes = name + unit in ascii
// CRC ( 1 byte)
// for data
// byte 8 = bits7...4 =  ID of field ; bits 3...0 =  type of data () 
// byte 9... = value; LSB first ; in MSB, bits 7 = sign; bits7..6 = number of decimals 
#ifdef DEBUG
// ************************* Several parameters to help debugging
//#define DEBUGSETNEWDATA
//#define DEBUGFORMATONEVALUE
//#define DEBUGJETIFRAME
//#define DEBUGADSCURRENT
//#define DEBUGSETNEWDATA
//#define DEBUGASERIAL
//#define DEBUGJETI
#endif

extern field fields[];  // list of all telemetry fields and parameters used by Sport
extern MS5611 baro1;
extern SPL06 baro2;
extern MS4525 ms4525;
extern SDP3X sdp3x; 

extern GPS gps;
extern CONFIG config;
extern uint8_t debugTlm;

uint8_t listOfJetiFields[20] ; // list of oXs Field Id to transmit
uint8_t listOfJetiFieldsIdx ; // current fields being handled (for data); there is another idx for text field
uint8_t numberOfJetiFields ; // number of fields to transmit (from 1 ... 15)
// buffer with data to transmit prefilled with some constants
uint8_t jetiData[63] = { 0x7E, 0x9F, 0x40 , 0x11 , 0xA4 ,  0xAD , 0x04, 0x00 };  // 64 = 29 bytes for data + 34 bytes for text ; buffer where Jeti frame is prepared , 7E = header, 0x?F = X frame(? = any 4 bits) , 40 = means data (TXT would be 00) + length in 6 bits - to be filled-, A400 0001 = device id ; 00  = fixed value
uint8_t jetiMaxData ;   // max number of bytes prepared in the buffer to be sent
uint16_t jetiTxBuffer[63] ; // buffer used by dma to provide the data to the pio


jetiState_t jetiState = JETI_IDLE;                  // Holds the state of the PIO/UART.
uint32_t jetiStartReceiving;
//volatile uint8_t oneByteReceived ;           // Keep a flag that is true when a byte has been received
//volatile uint8_t lastByteReceived ;           // Keep the last byte received
//uint8_t prevByteReceived ;
uint8_t  countOfFieldsChecked  ; //   

uint32_t jetiLong ;
uint32_t jetiLat ;

char degreeChar[2] ;

static const bool jetiParityTable256[256] = 
{
#   define P2(n) n, n^1, n^1, n
#   define P4(n) P2(n), P2(n^1), P2(n^1), P2(n)
#   define P6(n) P4(n), P4(n^1), P4(n^1), P4(n)
    P6(0), P6(1), P6(1), P6(0)
};


// one pio and 1 state machines are used to manage the jeti bus 
// the state machine (sm) handle only the TX 
//    We fill a buffer with the data
//    we then start the sm configuring the gpio as output
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    at the end of sending (detected using a IRQ), we stop the state machine and wait
//    we wait about 20 ms to let the jeti receiver send some data that will be discarded


//#define JETI_PIO_TX_PIN 10  // pin being used by the UART pio

PIO jetiPio = pio0;
uint jetiSmTx = 0; // to send the telemetry to jeti RX
uint jetiOffsetTx ; 

// dma channel is used to send Sport telemetry without blocking
int jeti_dma_chan;
dma_channel_config jetiDmaConfig;


// **************** Setup the OutputLib *********************
void setupJeti() {
// set up the DMA but do not yet start it to send data to Sport
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    jeti_dma_chan = dma_claim_unused_channel(true);
    jetiDmaConfig = dma_channel_get_default_config(jeti_dma_chan);
    channel_config_set_read_increment(&jetiDmaConfig, true);
    channel_config_set_write_increment(&jetiDmaConfig, false);
    channel_config_set_dreq(&jetiDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&jetiDmaConfig, DMA_SIZE_16);
    dma_channel_configure(
        jeti_dma_chan,
        &jetiDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &jetiTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    jetiOffsetTx = pio_add_program(jetiPio, &uart_jeti_tx_program);
    uart_jeti_tx_program_init(jetiPio, jetiSmTx, jetiOffsetTx, config.pinTlm, 9700 );  
    degreeChar[0] = 0xB0 ; // for the symbol °, fill with 0xB0 followed by 0x00 as showed in jetiprotocol example and in datasheet (code A02 for european set of char))
    degreeChar[1] = 0x00 ;
    initListOfJetiFields(false) ;  // do not activate all fields but only those with pin defined od installed  
}


void initListOfJetiFields(bool activateAllFields) {  // fill an array with the list of fields (field ID) that are defined -   Note : this list is not yet complete (e.g; GPS, ...)
    listOfJetiFields[0] = OXS_ID ;
    listOfJetiFieldsIdx = 1 ; // 0 is not used for a value, because - for text- it contains the name of the sensor (e.g. openXsensor)
    if (( config.pinVolt[0] != 255) ||  activateAllFields)  {
        listOfJetiFields[listOfJetiFieldsIdx++] = MVOLT ;
    }
    if (( config.pinVolt[1] != 255)  ||  activateAllFields) {
        listOfJetiFields[listOfJetiFieldsIdx++] = CURRENT ;
        listOfJetiFields[listOfJetiFieldsIdx++] = CAPACITY ;
    }
    if (( baro1.baroInstalled || baro2.baroInstalled || baro1.baroInstalled)  ||  activateAllFields) {
        listOfJetiFields[listOfJetiFieldsIdx++] = RELATIVEALT ; 
        listOfJetiFields[listOfJetiFieldsIdx++] = VSPEED ;
    }
    
    if (( config.pinGpsTx != 255 )  ||  activateAllFields) {
        listOfJetiFields[listOfJetiFieldsIdx++] = HEADING ;
        listOfJetiFields[listOfJetiFieldsIdx++] = GROUNDSPEED ;
        listOfJetiFields[listOfJetiFieldsIdx++] = ALTITUDE ; 
    //    listOfJetiFields[listOfJetiFieldsIdx++] = NUMSAT ;
        listOfJetiFields[listOfJetiFieldsIdx++] = LONGITUDE ;
        listOfJetiFields[listOfJetiFieldsIdx++] = LATITUDE ;  
    }
    if (( ms4525.airspeedInstalled || sdp3x.airspeedInstalled)  ||  activateAllFields) {
        listOfJetiFields[listOfJetiFieldsIdx++] = AIRSPEED ; 
    }
    if (( config.pinRpm != 255 )  ||  activateAllFields) {
        listOfJetiFields[listOfJetiFieldsIdx++] = RPM ; 
    }
    if ((config.temperature == 1 || config.temperature == 2)  ||  activateAllFields) {
        listOfJetiFields[listOfJetiFieldsIdx++] = TEMP1 ;
    }
    if ((config.temperature == 2) ||  activateAllFields) {
        listOfJetiFields[listOfJetiFieldsIdx++] = TEMP2 ;
    }
    numberOfJetiFields = listOfJetiFieldsIdx - 1 ;
    listOfJetiFieldsIdx = 1 ;
    
    //printf("list of jeti fields: ");
    //for (uint8_t i = 0; i <= numberOfJetiFields ; i++){
    //    printf(" %2X", listOfJetiFields[i]);
    //}
    //printf("\n");
     
}

bool retrieveFieldIfAvailable(uint8_t fieldId , int32_t * fieldValue , uint8_t * dataType) { // fill fieldValue and dataType for the fieldId when data is available, return true if data is available
  uint8_t GPS_no_fix = ( (gps.GPS_fix_type != 3 ) && (gps.GPS_fix_type != 4 ) ) ; // this flag is true when there is no fix

   switch (fieldId) {
    case RELATIVEALT :
      if ( ! fields[fieldId].available ) return 0 ;
      * fieldValue = int_round(fields[fieldId].value , 10) ; // Altitude converted from cm to dm
      * dataType = JETI22_1D ;
      fields[fieldId].available = false ;
      break ;
    case VSPEED :
         if ( ! fields[fieldId].available ) return 0; 
         * fieldValue = fields[fieldId].value  ;
         * dataType = JETI14_2D ;
         fields[fieldId].available = false ;
        break ;
      case  MVOLT :
          if ( ! fields[fieldId].available ) return 0;
          * fieldValue =  int_round(fields[fieldId].value  , 10) ; 
          * dataType = JETI14_2D ;
          fields[fieldId].available  = false ;
          break ;
      case  CURRENT :
          if ( ! fields[fieldId].available ) return 0;
          * fieldValue =  int_round(fields[fieldId].value  , 10) ; // converted in A with 2 decimals
          * dataType = JETI22_2D ;
          fields[fieldId].available  = false ;
          break ;
      case  CAPACITY :
          if ( ! fields[fieldId].available ) return 0;
          * fieldValue =  fields[fieldId].value ; //  in mAh 
          * dataType = JETI22_0D ;
          fields[fieldId].available  = false ;
          break ;
      case HEADING :
        //if (GPS_no_fix ) return 0 ;
        if ( ! fields[fieldId].available ) return 0; 
        * fieldValue = int_round(fields[fieldId].value  , 100000) ; // convert from degree * 100000 to degree
        * dataType = JETI14_0D ;
        fields[fieldId].available  = false ;
        break ;
      case GROUNDSPEED :
        //if (GPS_no_fix ) return 0 ;
        if ( ! fields[fieldId].available ) return 0; 
        * fieldValue = ((uint32_t) fields[fieldId].value) * 36 /100 ;       // convert from cm/sec to 1/10 of km/h
        * dataType = JETI14_1D ;
        fields[fieldId].available  = false ;
        break ;
      case ALTITUDE : 
        //if (GPS_no_fix ) return 0 ;
        if ( ! fields[fieldId].available ) return 0; 
        * fieldValue  = int_round(fields[fieldId].value , 100) ;                        // convert from cm to m 
        * dataType = JETI14_0D ;
        fields[fieldId].available  = false ;
        break ;
//      case GPS_DISTANCE :
//        if (GPS_no_fix ) return 0 ;
//        * fieldValue  = GPS_distance ;                             // keep in m
//        * dataType = JETI14_0D ;
//        break ;
      case LONGITUDE :      
        // if (GPS_no_fix ) return 0 ;
        if ( ! fields[fieldId].available ) return 0; 
         jetiLong =  formatGpsLongLat ( fields[fieldId].value , true ) ;
         * fieldValue  = jetiLong  ;
         * dataType = JETI_GPS ;
        fields[fieldId].available  = false ; 
         break ;                          
      case LATITUDE :                                           // Still to be added
        // if (GPS_no_fix ) return 0 ;
        if ( ! fields[fieldId].available ) return 0;  
         jetiLat =  formatGpsLongLat (fields[fieldId].value , false ) ;
         * fieldValue  = jetiLat  ;
         * dataType = JETI_GPS ;
        fields[fieldId].available  = false ;  
         break ;
    case AIRSPEED :
         if ( ! fields[fieldId].available ) return 0; 
         * fieldValue = fields[fieldId].value   * 36 / 1000 ; // from cm/s to km/h
         * dataType = JETI14_0D ;
         fields[fieldId].available = false ;
        break ;
    case RPM :
         if ( ! fields[fieldId].available ) return 0; 
         * fieldValue = fields[fieldId].value   * 60 ; // from Hz to RPM
         * dataType = JETI22_0D ;
         fields[fieldId].available = false ;
        break ;
    case TEMP1 :
         if ( ! fields[fieldId].available ) return 0; 
         * fieldValue = fields[fieldId].value ; // degree
         * dataType = JETI14_0D ;
         fields[fieldId].available = false ;
        break ;
    case TEMP2 :
         if ( ! fields[fieldId].available ) return 0; 
         * fieldValue = fields[fieldId].value ; // degree
         * dataType = JETI14_0D ;
         fields[fieldId].available = false ;
        break ;
                                   
   } // end of switch
   return 1 ;
}

uint32_t formatGpsLongLat (int32_t longLat, bool isLong ) { // return the long or latitude in Jeti format
  union   {
    uint32_t valueInteger;
    char    valueBytes[4];
  } gps;
  uint32_t degree ;
  uint16_t minX1000 ;
  uint32_t longLatE7P ;
  longLatE7P = (longLat >= 0 ? longLat : -longLat ) ;
  degree =  (uint16_t) (longLatE7P / 10000000) ; // remove 7 decimal from original value
  minX1000 =  (uint16_t ) ( (longLatE7P - (degree * 10000000)) * 6 / 1000 ) ; // keep 3 decimals to minutes
  gps.valueInteger = 0;
  gps.valueBytes[0]  = minX1000 & 0xFF;
  gps.valueBytes[1]  = ( minX1000 >> 8 ) & 0xFF;
  gps.valueBytes[2]  = degree & 0xFF;                      
  gps.valueBytes[3]  = ( degree >> 8 ) & 0x01;             
  gps.valueBytes[3] |= isLong  ? 0x20 : 0;
  gps.valueBytes[3] |= (longLat < 0) ? 0x40 : 0;
  return gps.valueInteger ;
}

bool tryToAddFieldToJetiBuffer (void) { // return true if a field has been added in the jetiBuffer
      int32_t fieldValue ;
      uint8_t dataType ;
      uint8_t fieldAvailable = 0 ;      
      while (   countOfFieldsChecked  ) {
        if ( retrieveFieldIfAvailable(listOfJetiFields[listOfJetiFieldsIdx] , &fieldValue , &dataType) ) {
          addFieldToJetiBuffer(fieldValue , dataType ) ;
          fieldAvailable = 1;
        } 
        listOfJetiFieldsIdx++ ;
        if (listOfJetiFieldsIdx > numberOfJetiFields ) listOfJetiFieldsIdx = 1;
        countOfFieldsChecked-- ;
        if ( fieldAvailable) return true; 
      } // End While
      return false ; 
}


void addFieldToJetiBuffer(int32_t fieldValue , uint8_t dataType ) {  // dataType has 1 high bits = 0 ,
                                                                     // then 2 bits to say precision (0= 0 digits, ... 2 = 2 digits) and 1 bit = 0
                                                                     // then 4 bits to say the type in Jeti codification     
    uint8_t codifJeti ;
    codifJeti = dataType & 0x0F ;
    switch (codifJeti) {
    case JETI_30 :
      jetiData[jetiMaxData++] = (listOfJetiFieldsIdx << 4) | ( codifJeti ) ; // first 4 bits are the field identifiers ( 1...15) and the last 4 bits the type of data)
      jetiData[jetiMaxData++] = fieldValue & 0xFF ;
      jetiData[jetiMaxData++] = ( fieldValue >> 8 ) & 0xFF ;
      jetiData[jetiMaxData++] = ( fieldValue >> 16 ) & 0xFF ;
      jetiData[jetiMaxData++] = ( ( fieldValue >> 24 ) & 0x1F ) | ((fieldValue < 0) ? 0x80 :0x00 )  ; 
      break ;
    case JETI_22 :
      jetiData[jetiMaxData++] = (listOfJetiFieldsIdx << 4) | (codifJeti) ; // first 4 bits are the field identifiers ( 1...15) and the last 4 bits the type of data)
      jetiData[jetiMaxData++] = fieldValue & 0xFF ;
      jetiData[jetiMaxData++] = ( fieldValue >> 8 ) & 0xFF ;
      jetiData[jetiMaxData++] = ( ( fieldValue >> 16 ) & 0x1F  ) | ((fieldValue < 0) ? 0x80 :0x00 )  ; 
      break ;
    case JETI_14 :
      jetiData[jetiMaxData++] = (listOfJetiFieldsIdx << 4) | ( codifJeti) ; // first 4 bits are the field identifiers ( 1...15) and the last 4 bits the type of data)
      jetiData[jetiMaxData++] = fieldValue & 0xFF ;
      jetiData[jetiMaxData++] = ( ( fieldValue >> 8 ) & 0x1F ) | ((fieldValue < 0) ? 0x80 :0x00 )  ; 
      break ;
    case JETI_GPS :
      jetiData[jetiMaxData++] = (listOfJetiFieldsIdx << 4) | ( codifJeti ) ; // first 4 bits are the field identifiers ( 1...15) and the last 4 bits the type of data)
      jetiData[jetiMaxData++] = fieldValue & 0xFF ;
      jetiData[jetiMaxData++] = ( fieldValue >> 8 ) & 0xFF ;
      jetiData[jetiMaxData++] = ( fieldValue >> 16 ) & 0xFF ;
      jetiData[jetiMaxData++] = ( ( fieldValue >> 24 ) & 0xFF )   ; 
      break ;
    }
    if ( ( codifJeti == 8 ) || ( codifJeti <= 4 ) ) { // when it is a number, then add precision 
         jetiData[jetiMaxData-1] |= dataType & 0x60 ; // add the number of decimals in position 6 & 7 of the last byte
    }
        
}

// Published in "JETI Telemetry Protocol EN V1.06"
//* Jeti EX Protocol: Calculate 8-bit CRC polynomial X^8 + X^2 + X + 1
uint8_t updateJetiCrc (uint8_t crc, uint8_t crc_seed)
{ 
  unsigned char crc_u;
  unsigned char i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for (i=0; i<8; i++)
    crc_u = ( crc_u & 0x80 ) ? 7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

void startJetiTransmit() {
    //#define DEBUG_JETI_TLM
    #ifdef DEBUG_JETI_TLM
      printf("Jeti= ");
      for (uint8_t j = 0 ; j < jetiMaxData ; j++ ){
        printf(" %02X" , jetiData[j]);
      }
      //printf(" t=%d", config.temperature);
      printf("\n");
    #endif
    //#define DEBUG_SHORT_JETI_TLM
    #ifdef DEBUG_SHORT_JETI_TLM
    printf("jj=%2X  %2X \n",jetiData[8],numberOfJetiFields );// print the first byte of data transmitted and the number of fields
    #endif
    // copy from jetiData (8 bits) to jetiTxBuffer (16 bits); we add a bit 9 (0 for only 2 bytes), a bit 10 = odd parity and a bit 11= stopbit
    uint8_t * ptr = (uint8_t *) &jetiData[0] ;
    for (uint8_t i = 0; i< jetiMaxData ; i++){ 
        uint8_t c = *ptr;
        uint8_t f = 1; //flag to be added in bit 8 ; 1 in most cases; 0 for frist byte and 2 others  
        uint8_t p = jetiParityTable256[c]; // this is for even parity
        if ( (i == 0 ) || ( i == ( jetiMaxData - 1 ))  || ( i == ( jetiMaxData -34 )) ) {//force a bit 8 to 0 for the first byte and the 2 text delimiters 
            f = 0 ; 
            p = p ^ 0X01 ; // when bit 8 must be 0, we have to reverse p because jeti uses odd parity 
        } 
        jetiTxBuffer[i] = ( ((uint32_t) c) ) | ( ((uint32_t)f ) << 8) | ( ((uint32_t)p ) << 9) | ((uint32_t) 0x400) ;
        ptr++; 
    }
    // start the state machine
    jeti_uart_tx_program_start(jetiPio, jetiSmTx,  config.pinTlm) ; 
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (jeti_dma_chan, &jetiTxBuffer[0], false);
    dma_channel_set_trans_count (jeti_dma_chan, jetiMaxData, true) ;
}

void mergeLabelUnit( const uint8_t identifier , const char * label, const char * unit )
{
  //printf("ier= %u\n",identifier);
  uint8_t i = 0 ;
  uint8_t k = 0 ;
  jetiMaxData = 10 ;
  jetiData[8] = identifier ;
  while( label[i] != '\0'  )
    jetiData[ jetiMaxData++ ] = label[ i++ ];
  while( unit[k] != '\0'  )
    jetiData[ jetiMaxData++ ] = unit[ k++ ];
  jetiData[9] = (i << 3 ) | k ;
}

void fillJetiBufferWithText() {
  static uint8_t textIdx ;
  textIdx++ ;
  if (textIdx > numberOfJetiFields ) textIdx = 0;
  switch (listOfJetiFields[textIdx]) {
    case OXS_ID:
        mergeLabelUnit( textIdx, "oXs", " "  ) ; 
        break ;
    case RELATIVEALT :
      mergeLabelUnit( textIdx, "Rel. altit", "m"  ) ; 
        break ;
    case VSPEED :
      mergeLabelUnit( textIdx, "Vario", "m/s"  ) ; 
        break ;
    case  MVOLT :
        mergeLabelUnit( textIdx, "Accu. volt", "V"  ) ;
        break ;
    case  CURRENT :
        mergeLabelUnit( textIdx, "Current", "A"  ) ;
        break ;
    case  CAPACITY :
        mergeLabelUnit( textIdx, "Consumption", "mAh"  ) ;
        break ;
      case HEADING :
        mergeLabelUnit( textIdx, "Course", degreeChar  ) ;
        break ;
      case GROUNDSPEED :
        mergeLabelUnit( textIdx, "Speed", "Km/h"  ) ;
        break ;
      case ALTITUDE : 
        mergeLabelUnit( textIdx, "Gps Alt", "m"  ) ;
        break ;
      //case GPS_DISTANCE :
      //  mergeLabelUnit( textIdx, "Distance", "m"  ) ;
      //  break ;
      //case GPS_BEARING :
      //  mergeLabelUnit( textIdx, "Gps Bearing", degreeChar  ) ;
      //  break ;
      case LONGITUDE :                                          // Still to be added 
        mergeLabelUnit( textIdx, "Gps Long", degreeChar  ) ;
        break ;
      case LATITUDE :                                           // Still to be added
        mergeLabelUnit( textIdx, "Gps Lat", degreeChar  ) ;
        break ;
    case AIRSPEED :
        mergeLabelUnit( textIdx, "Airspeed", "Km/h"  ) ;
        break ;
    case RPM :
        mergeLabelUnit( textIdx, "Rpm", "n"  ) ;
        break ;
    case TEMP1 :
        mergeLabelUnit( textIdx, "Temp1", degreeChar  ) ;
        break ;
    case TEMP2 :
        mergeLabelUnit( textIdx, "Temp2", degreeChar  ) ;
        break ;
      
  } // end switch
  jetiData[2] =  ( jetiMaxData - 2 ) ; // update number of bytes that will be in buffer (including crc); keep flag in bit 6/7 to zero because it is text and not data
  
}

void handleJetiTx()    // when there is more than 20mses since sending last frame, this part try to put 2 data in a one frame.
                            // the buffer must contain a header (already filled with fixed data), then DATA or TEXT part , then a CRC,
                            // then a separator (0xFE), 2 * 16 char of text and a trailer (0xFF)
                            // the TEXT part contains or the name of the device (="oXs") or the name and unit of each field
{
  if (config.pinTlm == 255) return ; // skip when Tlm is not foreseen
  static  uint8_t jetiSendDataFrameCount ;
  //printf("Jeti State %x time %" PRIu32 "\n", jetiState , millisRp() -jetiStartReceiving);
  if (jetiState == JETI_SENDING){
      if ( dma_channel_is_busy(jeti_dma_chan) )return ; // skip if the DMA is still sending data
      jetiState = JETI_RECEIVING ;
      jetiStartReceiving = millisRp();
      return;  
  } else if (jetiState == JETI_RECEIVING){
      if ( ( millisRp() -  jetiStartReceiving ) <= 35 ) return; //skip if there is less than 20 msec since end of transmit
                                                 // 35 because dma become free about 15 msec before the end                 
      jetiState = JETI_IDLE;
  }    
  if (jetiState == JETI_IDLE ) {
    if (jetiSendDataFrameCount  >= 5 ) { //Send a text frame instead of a data frame once every 5 data frames 
        jetiSendDataFrameCount = 0 ;
        fillJetiBufferWithText() ;                // fill the buffer (including number of bytes and type but not crc)
    }  else  {  
        //printf(".+. \n");
        jetiSendDataFrameCount++ ;                          // count the number of data frame sent (in order to send a text frame after 5 frames. 
        countOfFieldsChecked = numberOfJetiFields ;             //countOfFieldsChecked is used in order to loop only once thru all data to be sent
        jetiMaxData = 8 ;                                   //  Jeti buffer contains 8 bytes in a header ; this header is already filled.
        if ( tryToAddFieldToJetiBuffer() ) {                    // try to add first field and if OK, continue adding another one.
            tryToAddFieldToJetiBuffer() ;                         // try to add second field 
        } 
        jetiData[2] = 0x40 | ( jetiMaxData - 2 ) ;      // set bit to say that it concerns data (and not text) and update number of bytes that will be in buffer (including crc) 
    } // end test on jetiSendDataFrameCount
    uint8_t crc = 0 ;
    uint8_t c ;
    for(c=2; c<jetiMaxData ; c++) crc = updateJetiCrc (jetiData[c], crc);  // calculate crc starting from position 2 from the buffer
    jetiData[jetiMaxData++] = crc ;                                        // store crc in buffer
//after filling data or text, we fill the buffer with the 2 lines for the display box (this seems to be mandatory by the protocol)        
    jetiData[jetiMaxData++] = 0xFE ;                                       // fill the 2 lines for the display box (with 0xFE and 0xFF as delimiters)         uint8_t carToSend = 0x41 ;
    for ( uint8_t i_jetiBoxText = 0 ; i_jetiBoxText < 16 ; i_jetiBoxText++ ) {
        jetiData[jetiMaxData++] = 0x2E  ;                // here it is a dummy text; it could be changed later on.
        jetiData[jetiMaxData++] = 0x2D  ;
    }
    jetiData[jetiMaxData++] = 0xFF ;
#ifdef DEBUGJETIFRAME                                                          // print the jeti buffer on 4 lines
        //printer->print(F("01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1E 1F 20 21 22 23 24 25 26 27 "));
        //printer->print(F("28 29 2A 2B 2C 2D 2E 2F 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F 40 "));
        //printer->println(F("41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F 50 51 52 53 54 55 56 57 58 59 "));
        //for (int i = 0 ; i<jetiMaxData ; i++ )  // to display the full buffer
        /*
        for (int i = 8 ; i<= 13 ; i++ )           // to display only the first 2 bytes data
        {
           if ( jetiData[i]< 0x10 ) printer->print("0") ;
           printer->print(jetiData[i],HEX); printer->print(" ");
        }
        printer->println("");
        */
        
    for (int i = 0 ; i<jetiMaxData ; i++ )
    {
        if ( jetiData[i] >= 0x20 && jetiData[i] <= 0x7F) {
            printf(%c ,jetiData[i]);
        } else { 
            if ( jetiData[i]< 0x10 ) printf("0") ;
            printf(%X ,jetiData[i]);
        }
    }
    printf("\n \n")                     
#endif        
    startJetiTransmit();  // convert the buffer format, setTx pin as output and start state machine and dma
    jetiState = JETI_SENDING;
    //printf("send Jeti\n");
  }    
}

