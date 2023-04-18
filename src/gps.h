#pragma once


#include "config.h"

// from the UBlox6 document, the largest payout we receive i the NAV-SVINFO and the payload size
// is calculated as 8 + 12*numCh.  numCh in the case of a Glonass receiver is 28.
#define UBLOX_PAYLOAD_SIZE 344 // 344 is the absolute max size 
#define UBLOX_BUFFER_SIZE 100 // but the message that we read should not exceed about 100 bytes (PVT is quite long)

// UBX support
//typedef struct { // not used I think
//    uint8_t preamble1;
//    uint8_t preamble2;
//    uint8_t msg_class;
//    uint8_t msg_id;
//    uint16_t length;
//} ubx_header;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t longitude;                 // in degree with 7 decimals
    int32_t latitude;                  // in degree with 7 decimals 
    int32_t altitude_ellipsoid;        // in mm
    int32_t altitude_msl;              // in mm
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;        // in mm
} __attribute__((__packed__)) ubx_nav_posllh;

typedef struct {
    uint32_t time;              // GPS msToW
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;            // milliseconds
} __attribute__((__packed__)) ubx_nav_status;

typedef struct {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
} __attribute__((__packed__)) ubx_nav_solution;

typedef struct {
    uint32_t time;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t time_acc;
    int32_t nano;
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t additional_flags;
    uint8_t satellites;
    uint32_t longitude;
    uint32_t latitude;
    int32_t height;
    int32_t height_above_sea_level;
    uint32_t hacc; // horizontal accuracy
    uint32_t vacc; // vertical accuracy
    int32_t velocity_noord;
    int32_t velocity_east;
    int32_t velocity_down;
    int32_t gspeed_2d_mm; // speed 2D in mm/sec
    int32_t head_mot_2d;
    uint32_t speed_accuracy;
    uint32_t head_accuracy;
    uint16_t position_DOP;
    uint16_t flag3;
    uint32_t res;
    int32_t  head_velocity;
    int16_t magnetic_declination;
    uint16_t magnetic_accuracy;  
} __attribute__((__packed__)) ubx_nav_pvt;


typedef struct {
    uint32_t time;              // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
} __attribute__((__packed__)) ubx_nav_velned;

typedef struct {
    uint8_t chn;                // Channel number, 255 for SVx not assigned to channel
    uint8_t svid;               // Satellite ID
    uint8_t flags;              // Bitmask
    uint8_t quality;            // Bitfield
    uint8_t cno;                // Carrier to Noise Ratio (Signal Strength) // dbHz, 0-55.
    uint8_t elev;               // Elevation in integer degrees
    int16_t azim;               // Azimuth in integer degrees
    int32_t prRes;              // Pseudo range residual in centimetres
} __attribute__((__packed__)) ubx_nav_svinfo_channel;

typedef struct {
    uint32_t time;              // GPS Millisecond time of week
    uint8_t numCh;              // Number of channels
    uint8_t globalFlags;        // Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
    uint16_t reserved2;         // Reserved
    ubx_nav_svinfo_channel channel[16];         // 16 satellites * 12 byte
} __attribute__((__packed__)) ubx_nav_svinfo;

struct __attribute__((__packed__)) casic_nav_pv_info {
    uint16_t header;              // Header = 0xBA 0XCE
    uint16_t length;              // length of payload 80 = 0X50
    uint16_t identifier;        // Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
    uint32_t runTime;         // 
    uint8_t  posValid;         // 
    uint8_t  velValid;
    uint8_t  system;
    uint8_t  numSV;            // number of satellite in solution
    uint8_t  numSVGPS;
    uint8_t  numSVBDS;
    uint8_t  numSVGLN;
    uint8_t  reserve;
    float    pDop;
    double   lon;      // degree
    double   lat;      // degree
    float    height;   // m
    float    sepGeoid;
    float    hAcc;
    float    vAcc;
    float    veIN;
    float    veIE;
    float    veIU;
    float    speed3D;   // speed m/s
    float    speed2D;   // speed m/s
    float    heading;   // degree
    float    sAcc;
    float    cAcc;
    uint32_t checksum;
} ;  // structure use by casic GPS


// GPS codes that could be used
#define    PREAMBLE1  0xb5
#define    PREAMBLE2  0x62
#define    CLASS_NAV  0x01
#define    CLASS_ACK  0x05
#define    CLASS_CFG  0x06
#define    MSG_ACK_NACK  0x00
#define    MSG_ACK_ACK  0x01
#define    MSG_POSLLH  0x2
#define    MSG_STATUS  0x3
#define    MSG_SOL  0x6 // not supported in ublox10
#define    MSG_PVT  0x7 // not supported in ublox6
#define    MSG_VELNED  0x12
#define    MSG_SVINFO  0x30
#define    MSG_CFG_PRT  0x00
#define    MSG_CFG_RATE  0x08
#define    MSG_CFG_SET_RATE  0x01
#define    MSG_CFG_NAV_SETTINGS  0x24

#define    FIX_NONE  0
#define    FIX_DEAD_RECKONING  1
#define    FIX_2D  2
#define    FIX_3D  3
#define    FIX_GPS_DEAD_RECKONING  4
#define    FIX_TIME  5

#define    NAV_STATUS_FIX_VALID  1

enum {
    GPS_WAIT_END_OF_RESET = 0,
    GPS_M10_IN_RECONFIGURATION,
    GPS_M6_IN_RECONFIGURATION,
    GPS_CONFIGURED,
};


class GPS {
public:
    // **********************
    // GPS data being read
    // **********************
    bool gpsInstalled = false;
    uint8_t gpsState = GPS_WAIT_END_OF_RESET;
    uint16_t initGpsIdx = 0 ; // index of the bytes to be sent to configure the GPS
            
    //int32_t GPS_lon;               // longitude in degree with 7 decimals, (neg for S)
    //bool    GPS_lonAvailable = false ; 
    //int32_t GPS_lat;               // latitude   in degree with 7 decimals, (neg for ?)
    //bool    GPS_latAvailable = false ;

    //int32_t GPS_altitude;              // altitude in mm
    //bool    GPS_altitudeAvailable = false ;
    //uint16_t GPS_speed_3d;                 // speed in cm/s
    bool    GPS_speed_3dAvailable = false;
    uint16_t GPS_speed_2d;                 // speed in cm/s
    bool    GPS_speed_2dAvailable = false ;
    //uint32_t GPS_ground_course ;     // degrees with 5 decimals
    //bool    GPS_ground_courseAvailable = false ;

    //uint8_t GPS_numSat;
    uint8_t GPS_fix_type;
    uint16_t GPS_pdop = 9999;           // Compute GPS quality signal
    uint16_t GPS_packetCount = 0;

    uint32_t gpsDate;               // year(2 last digits = 1 bytes MSB ) + month (1 byte) + day (1 byte) + 0XFF (LSB 1 byte)
    uint32_t gpsTime;               // hour (1 byte MSB ) + min (1 byte) + sec (1 byte) + 0X00 (LSB 1 byte)
    uint32_t prevGpsTime = 0 ;  

    // *********** GPS calculated data
    int16_t GPS_distance ;   // distance from home (first location) in m
    int16_t GPS_heading ;          // heading from home (in Rad)
    int32_t GPS_home_lat ;         // position of home in degre with 7 decimals
    int32_t GPS_home_lon ;         // position of home in degre with 7 decimals
    int32_t GPS_last_lat ;         // last position used for cummulative dist in degre with 7 decimals
    int32_t GPS_last_lon ;         // last position used for cumulative dist in degre with 7 decimals
    int32_t GPS_cumulativeDistCm ;     // cumulative distance in cm
    float GPS_scale ;     // scaling factor to calculate the distance depending on latitude
    int16_t GPS_bearing ;          // bearing from home in degrees

    bool GPS_fix = false ; // true if gps data are available.
    bool new_position = false ;
    bool new_speed = false ;
      

    explicit GPS(void);
    
    void setupGps() ;
    void gpsInitRx();
    void pioRxHandlerIrq();
    void setupGpsUblox() ;
    void handleGpsUblox();
    //void setupGpsCasic() ;
    void readGps();
    void readGpsUblox();
    void readGpsCasic();
    bool parseGps(void) ;
    bool parseGpsUblox(void) ;
    bool parseGpsCasic(void) ;

    int32_t GpsDistanceCm(int32_t deltaLat , int32_t deltaLon);

private:
    uint16_t gpsDataErrors;

    uint8_t _ck_a;// Packet checksum accumulators
    uint8_t _ck_b;// Packet checksum accumulators
    bool _skip_packet = false ;
    uint8_t _step ;
    uint8_t _class;
    uint16_t _payload_length;
    uint16_t _payload_counter;
    uint8_t _msg_id; //used to identify the message type when reading the gps, is used also when buffer is parsed 

    //#if defined( A_LOCATOR_IS_CONNECTED)  && ( A_LOCATOR_IS_CONNECTED == YES)
    //uint32_t GPS_last_fix_millis ; // time when last fix has been received (used by lora locator) 
    //int32_t GPS_last_fix_lon ;     // last lon when a fix has been received
    //int32_t GPS_last_fix_lat ;     // last lat when a fix has been received
    //#endif



};

void uboxChecksum();

