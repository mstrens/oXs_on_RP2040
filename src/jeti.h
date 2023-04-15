#pragma once

// list of Jeti type being used
#define JETI_14 0b00000001  // jeti TYpe = 1
#define JETI_22 0b00000100  // jeti TYpe = 4
#define JETI_30 0b00001000  // jeti TYpe = 8

// List of used combination of jeti type of data and number of decimals (in bits 5 and 6)

#define JETI14_0D 0b00000001  // 0 decimal, jeti TYpe = 1
#define JETI14_1D 0b00100001  // 1 decimal, jeti TYpe = 1  
#define JETI14_2D 0b01000001  // 2 decimal, jeti TYpe = 1
#define JETI22_0D 0b00000100  // 0 decimal, jeti TYpe = 1
#define JETI22_1D 0b00100100  // 1 decimal, jeti TYpe = 1
#define JETI22_2D 0b01000100  // 1 decimal, jeti TYpe = 1
#define JETI_GPS  0b00001001  // special GPS format in 4 bytes, type = 9 in decimal

typedef enum  {
      JETI_IDLE =0,
      JETI_SENDING,
      JETI_RECEIVING ,
} jetiState_t ;    

#define OXS_ID 0xFF

/*
//  This is the list of oXs codes for each available measurements
//#define ALTIMETER       1        
#define VERTICAL_SPEED  2        
//#define SENSITIVITY     3        
//#define ALT_OVER_10_SEC 4        // DEFAULTFIELD can NOT be used ; this is the difference of altitude over the last 10 sec (kind of averaging vertical speed)
#define VOLT_1           5        
#define VOLT_2           6        
#define VOLT_3           7        
#define VOLT_4           8        
#define VOLT_5           9        
#define VOLT_6           10       
#define CURRENTMA       11        
#define MILLIAH         12        
#define GPS_COURSE       13       
#define GPS_SPEED        14       
#define GPS_ALTITUDE     15       
#define RPM             16        
#define GPS_DISTANCE       17     
#define GPS_BEARING        18     
//#define SENSITIVITY_2      19     
//#define ALT_OVER_10_SEC_2  20      
#define AIR_SPEED          21      
//#define PRANDTL_COMPENSATION 22     
//#define PPM_VSPEED         23       
//#define PPM                24       
//#define PRANDTL_DTE        25       
//#define TEST_1              26      
//#define TEST_2             27       
//#define TEST_3             28       
//#define VERTICAL_SPEED_A  29
#define REL_ALTIMETER     30
//#define REL_ALTIMETER_2   31
#define CELL_1            32
#define CELL_2            33
#define CELL_3            34
#define CELL_4            35
#define CELL_5            36
#define CELL_6            37
#define CELL_MIN          38
#define CELL_TOT          39
#define ALTIMETER_MAX     40
#define GPS_LONG          41
#define GPS_LAT           42
#define FLOW_ACTUAL       43
#define FLOW_REMAIN       44
#define FLOW_PERCENT      45
#define TEMPERATURE       46
// to do : add alt min, alt max ,  rpm max? , current max (not sure that it is neaded because it can be calculated on TX side
// End of list of type of available measurements
*/
    void setupJeti();
    void initListOfJetiFields(bool activateAllFields);
    bool retrieveFieldIfAvailable(uint8_t fieldId , int32_t * fieldValue , uint8_t * dataType) ; // fill fieldValue and dataType for the fieldId when data is available, return true if data is available
    uint32_t formatGpsLongLat (int32_t longLat, bool isLong ) ; // return the long or latitude in Jeti format
    bool tryToAddFieldToJetiBuffer (void) ; // return true if a field has been added in the jetiBuffer
    void addFieldToJetiBuffer(int32_t fieldValue , uint8_t dataType ) ;
    uint8_t updateJetiCrc (uint8_t crc, uint8_t crc_seed);
    void startJetiTransmit();
    void mergeLabelUnit( const uint8_t identifier , const char * label, const char * unit );
    void fillJetiBufferWithText() ;
    void handleJetiTx();
 
    
   





