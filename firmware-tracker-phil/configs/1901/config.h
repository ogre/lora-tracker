

#define RADIO_FREQ      FREQ_434_100

#define CALLSIGN_STR    "1901"
#define CALLSIGN_INT    1901

#define BATTV_MUL       81
#define BATTV_DIV       100

#define ENABLE_GPS      //comment out if a GPS is not yet fitted
#define GPS_UBLOX_VERSION   8
#define GPS_UPDATE_PERIOD   1000

//#define GNSS_AID_TIME
#define GNSS_AID_POSITION
// Dartmoor: 50.6628, -4.1816, 400km radius
const uint8_t gnss_aid_position_msg[] = { 0xb5, 0x62, 0x13, 0x40, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xa0, 0x87, 0x32, 0x1e, 0x40, 0xf0, 0x81, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0x62, 0x02, 0x4b, 0x92 };
const uint32_t gnss_aid_position_msg_length = sizeof(gnss_aid_position_msg);
#define GNSS_AID_ALMANAC
#define GNSS_AID_AUXILIARY
//#define GNSS_AID_UBXOFFLINE

//#define UPLINK          //enables/disables uplink after each lora packet
//#define TESTING       //disables the WDT and sets a fake payload name (to prevent being accidently left enabled)
//#define CUTDOWN       //checks the uplinked message when cutdown is needed
//const char cutdown_text[] = "CUTDOWNpassword";
//#define HABPACK
//#define CALLING // Enables Transmission on calling Frequency
//#define CALLING_FREQ    FREQ_433_650
//#define CALLING_INTERVAL   32 // sentences
//#define CALLING_DOWNLINK_FREQ   434300000

//#define CALLING_DOWNLINK_MODE   0 // Mode of main downlink
// **OR**
//#define CALLING_DOWNLINK_IMPLICIT   0
//#define CALLING_DOWNLINK_ERRORCODING   5
//#define CALLING_DOWNLINK_BANDWIDTH   3
//#define CALLING_DOWNLINK_SPREADING   10
//#define CALLING_DOWNLINK_LDO   1

static const uint8_t sentences_implicit[]  = {0,              };
static const uint8_t sentences_coding[]    = {0,              };
static const uint8_t sentences_spreading[] = {0,              };
static const uint8_t sentences_bandwidth[] = {RTTY_SENTENCE,  };

#define TOTAL_SENTENCES 1