/* configuration data for the AVB stream */
#define NUMBER_OF_STREAMS 3

/* currently we use the locally administered MAAP MAC address base for our Talker stream */

#define MAAP_LOCALLY_ADMINISTRATORED_BASE 0x91e0f000fe00ul

#define OWN_TALKER_MAC_BASE MAAP_LOCALLY_ADMINISTRATORED_BASE

/* Set the MAC address of the i210 */

#define OWN_MAC 0x001b2176bc1cul

/* Note: The following can be drived by looking at the maap announce messages */
/*       of the AVB device using wireshark (set the filter for maap)          */
       
/* The MAC address of the remote AVB device */

#define AVB_DEVICE_SOURCE_MAC 0x0001f2001a21ul

/* The MAAP MAC address used by the AVB device for its Talker stream */

#define AVB_DEVICE_TALKER_MAC_BASE 0x91e0f000fb10ul


