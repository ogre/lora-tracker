#ifndef __GNSS_AID_H__
#define __GNSS_AID_H__

#define GNSS_AID_TIMESTAMP 0

#define GNSS_SEND_AID_ALMANAC() \
_delay_ms(5);

#define GNSS_SEND_AID_AUXILIARY() \
_delay_ms(5);

#define GNSS_SEND_AID_UBXOFFLINE() \
_delay_ms(5);

#endif /* __GNSS_AID_H__ */
