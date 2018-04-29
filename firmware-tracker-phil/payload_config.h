

#define RADIO_FREQ  FREQ_434_300

#define CALLSIGN_STR "CRAAG4"

//#define RADIATION
#define ENABLE_GPS      //comment out if a GPS is not yet fitted
#define GPS_UBLOX_VERSION   7

//#define UPLINK          //enables/disables uplink after each lora packet
//#define MULTI_POS     //enables the sending of multiple GPS positions in a packet. Only works with msgpack/lora
//#define TESTING       //disables the WDT and sets a fake payload name (to prevent being accidently left enabled)
//#define CUTDOWN       //checks the uplinked message when cutdown is needed
#define HABPACK

#define GPS_UPDATE_PERIOD 1000

static const uint8_t sentences_coding[]    = {CODING_4_5,       }; //0, 0};
static const uint8_t sentences_spreading[] = {10,               }; //0, 0};
static const uint8_t sentences_bandwidth[] = {BANDWIDTH_20_8K,  }; //RTTY_SENTENCE, RTTY_SENTENCE};

/* MULTI_POS */
// GPS Update Rate should be something like 200ms. Should be a factor of 1000
// Number of GPS positions to collect before starting to send another packet
// MAX_POSITIONS_PER_SENTENCE/GPS_UPDATE_RATE   should ideally be an integer
//#define MAX_POSITIONS_PER_SENTENCE 20 //22    //TODO: ensure output buff is long enough
// memory usage: 3 bytes + 4 (scaling) + 2 (object 62) + 3*2 (describing arrays)