#ifndef SHOCKBURST_H
#define SHOCKBURST_H

#include "nrf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PACKET_BASE_ADDRESS_LENGTH  (4UL)                   //!< Packet base address length field size in bytes
#define PACKET_STATIC_LENGTH        (8UL)                   //!< Packet static length in bytes
#define PACKET_PAYLOAD_MAXSIZE      (PACKET_STATIC_LENGTH)  //!< Packet payload maximum size in bytes

// Error Codes
#define EPERWS 150
#define ECHK 151

enum shockburst_bitrate {
    SHOCKBURST_BITRATE_250KBPS = RADIO_MODE_MODE_Nrf_250Kbit,
    SHOCKBURST_BITRATE_1MBPS = RADIO_MODE_MODE_Nrf_1Mbit,
};

/** @brief Enhanced ShockBurst radio transmission power modes. */
enum shockburst_tx_power {
	/** 4 dBm radio transmit power. */
#if !defined(CONFIG_SOC_NRF5340_CPUNET)
	SHOCKBURST_TX_POWER_4DBM = RADIO_TXPOWER_TXPOWER_Pos4dBm,
#endif
#if defined(CONFIG_SOC_SERIES_NRF52X)
	/** 3 dBm radio transmit power. */
	SHOCKBURST_TX_POWER_3DBM = RADIO_TXPOWER_TXPOWER_Pos3dBm,
#endif
	/** 0 dBm radio transmit power. */
	SHOCKBURST_TX_POWER_0DBM = RADIO_TXPOWER_TXPOWER_0dBm,
	/** -4 dBm radio transmit power. */
	SHOCKBURST_TX_POWER_NEG4DBM = RADIO_TXPOWER_TXPOWER_Neg4dBm,
	/** -8 dBm radio transmit power. */
	SHOCKBURST_TX_POWER_NEG8DBM = RADIO_TXPOWER_TXPOWER_Neg8dBm,
	/** -12 dBm radio transmit power. */
	SHOCKBURST_TX_POWER_NEG12DBM = RADIO_TXPOWER_TXPOWER_Neg12dBm,
	/** -16 dBm radio transmit power. */
	SHOCKBURST_TX_POWER_NEG16DBM = RADIO_TXPOWER_TXPOWER_Neg16dBm,
	/** -20 dBm radio transmit power. */
	SHOCKBURST_TX_POWER_NEG20DBM = RADIO_TXPOWER_TXPOWER_Neg20dBm,
	/** -30 dBm radio transmit power. */
	SHOCKBURST_TX_POWER_NEG30DBM = RADIO_TXPOWER_TXPOWER_Neg30dBm,
	/** -40 dBm radio transmit power. */
	SHOCKBURST_TX_POWER_NEG40DBM = RADIO_TXPOWER_TXPOWER_Neg40dBm
};

enum shockburst_crc {
    SHOCKBURST_CRC_TWO = RADIO_CRCCNF_LEN_Two,
    SHOCKBURST_CRC_ONE = RADIO_CRCCNF_LEN_One,
    SHOCKBURST_CRC_DISABLE = RADIO_CRCCNF_LEN_Disabled
};

enum shockburst_event {
	SHOCKBURST_EVENT_RX_RECEIVED,
};

enum shockburst_radio_state {
    SHOCKBURST_RADIO_STATE_DISABLED = RADIO_STATE_STATE_Disabled,
};

void shockburst_init();

int shockburst_set_frequency(uint32_t freq);

void shockburst_set_bitrate(enum shockburst_bitrate bitrate);

void shockburst_set_tx_power(enum shockburst_tx_power power);

void shockburst_set_base_adress0(uint32_t base0);

void shockburst_set_base_adress1(uint32_t base1);

void shockburst_set_prefixes(uint8_t prefix0, uint8_t prefix1, uint8_t prefix2, uint8_t prefix3, uint8_t prefix4, uint8_t prefix5, uint8_t prefix6, uint8_t prefix7);

void shockburst_set_crc(enum shockburst_crc crc);

enum shockburst_radio_state shockburst_get_radio_state();



/// @brief Sets which address (0-7) should be used for used for TX operations
/// @param addr The address that should be used for TX. 0-7
/// @return Error code or 0 if successful
int shockburst_set_tx_address(uint32_t addr);


/// @brief Bit-sets which addresses should be activated for RX reception
/// @param activations - Bit pattern for which address should be enabled. LSB = Address0
/// @return Error code or 0 if sucessful
int shockburst_set_rx_addresses(uint8_t activations);


// === Blocking API ===

int shockburst_write_tx_payload(uint8_t* payload, uint8_t length);

int shockburst_read_rx_payload(uint8_t* payload);

// ===

int shockburst_set_payload_length(uint32_t length);

int shockburst_set_address_base_length(uint32_t addr_length);

int shockburst_activate_fast_rampup();

int shockburst_deactivate_fast_rampup();

// === Polling API == 

int shockburst_rx_start_listening();

int shockburst_rx_stop_listening();

int shockburst_rx_available();

int shockburst_rx_read(uint8_t* payload);

// ===

// Interrupt API

int shockburst_rx_start_listening_interrupt(void (*handler)(enum shockburst_event));

int shockburst_rx_stop_listening_interrupt();

// ===

#ifdef __cplusplus
}
#endif

#endif