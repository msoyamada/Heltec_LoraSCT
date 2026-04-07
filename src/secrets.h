/*
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
//cbb26990ffbccf7313672a5fa3ec20e3
static const PROGMEM u1_t NWKSKEY[16] = {0xF9, 0xD9, 0xC0, 0xFB, 0xA0, 0x8D, 0x7D, 0x46, 0xC0, 0x30, 0xD7, 0xC8, 0x41, 0x5C, 0x5A, 0x96};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0x8B, 0x03, 0xEB, 0x14, 0xED, 0xEA, 0x42, 0x77, 0xEA, 0x19, 0x58, 0xCB, 0xD2, 0xC4, 0xEE, 0x07};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260DD8FC ; // <-- Change this address for every node!
*/

/*
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
//cbb26990ffbccf7313672a5fa3ec20e3
static const PROGMEM u1_t NWKSKEY[16] = {0x28, 0xF6, 0xFA, 0xA2, 0x63, 0xA8, 0x61, 0x73, 0x95, 0xC3, 0xAA, 0x06, 0x3B, 0x9A, 0x41, 0xCE};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0xFE, 0x93, 0xBF, 0x1D, 0x7A, 0x50, 0x5B, 0x9A, 0x2B, 0xF2, 0x27, 0x0F, 0xD1, 0xAE, 0x62, 0xF2};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260D550F ; // <-- Change this address for every node!
*/
//config for  DHT Lorawan
static const PROGMEM u1_t NWKSKEY[16] = {0x9C, 0x49, 0x24, 0x6B, 0x55, 0xC3, 0x23, 0x1E, 0xDB, 0x36, 0xE4, 0x7C, 0x9B, 0xCA, 0x38, 0x72};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0xB1, 0x20, 0xEC, 0x5F, 0x9B, 0x26, 0x8B, 0xD0, 0xF8, 0x56, 0x38, 0x8B, 0x88, 0xE6, 0x93, 0x61};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260DE74F;