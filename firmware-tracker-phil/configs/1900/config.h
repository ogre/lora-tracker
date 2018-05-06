

#define RADIO_FREQ      FREQ_434_300

#define CALLSIGN_STR    "1900"
#define CALLSIGN_INT    1900

#define BATTV_MUL       81
#define BATTV_DIV       100

#define ENABLE_GPS      //comment out if a GPS is not yet fitted
#define GPS_UBLOX_VERSION   8

//#define UPLINK          //enables/disables uplink after each lora packet
//#define TESTING       //disables the WDT and sets a fake payload name (to prevent being accidently left enabled)
//#define CUTDOWN       //checks the uplinked message when cutdown is needed
#define HABPACK
#define CALLING // Enables Transmission on calling Frequency
#define CALLING_FREQ    FREQ_433_650
#define CALLING_INTERVAL   32 // sentences
#define CALLING_DOWNLINK_FREQ   434300000

//#define CALLING_DOWNLINK_MODE   0 // Mode of main downlink
// **OR**
#define CALLING_DOWNLINK_IMPLICIT   0
#define CALLING_DOWNLINK_ERRORCODING   5
#define CALLING_DOWNLINK_BANDWIDTH   3
#define CALLING_DOWNLINK_SPREADING   10
#define CALLING_DOWNLINK_LDO   1

#define GPS_UPDATE_PERIOD 1000

static const uint8_t sentences_implicit[]  = {0,                };
static const uint8_t sentences_coding[]    = {CODING_4_5,       };
static const uint8_t sentences_spreading[] = {10,               };
static const uint8_t sentences_bandwidth[] = {BANDWIDTH_20_8K,  };