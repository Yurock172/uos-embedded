/*
 * radio.h
 *
 *  Created on: 15 сент. 2019 г.
 *      Author: yurock
 */

#ifndef LORA_RADIO_H_
#define LORA_RADIO_H_


/*!
 * Radio driver supported modems
 */
typedef enum {
  MODEM_FSK = 0,
  MODEM_LORA,
} radio_modems_t;

/*!
 * Radio driver internal state machine states definition
 */

typedef enum {
  RF_IDLE = 0,
  RF_RX_RUNNING,
  RF_TX_RUNNING,
  RF_CAD,
} radio_state_t;

typedef struct _radioif_t radioif_t;

/*!
 * \brief Radio driver callback functions
 */
typedef struct {
  /*!
   * \brief  Tx Done callback prototype.
   */
  void (*tx_done)(radioif_t *radio);
  /*!
   * \brief  Tx Timeout callback prototype.
   */
  void (*tx_timeout)(radioif_t *radio);
  /*!
   * \brief Rx Done callback prototype.
   *
   * \param [IN] payload Received buffer pointer
   * \param [IN] size    Received buffer size
   * \param [IN] rssi    RSSI value computed while receiving the frame [dBm]
   * \param [IN] snr     Raw SNR value given by the radio hardware
   *                     FSK : N/A ( set to 0 )
   *                     LoRa: SNR value in dB
   */
  void (*rx_done)(radioif_t *radio, uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
  /*!
   * \brief  Rx Timeout callback prototype.
   */
  void (*rx_timeout)(radioif_t *radio);
  /*!
   * \brief Rx Error callback prototype.
   */
  void (*rx_error)(radioif_t *radio);
  /*!
   * \brief  FHSS Change Channel callback prototype.
   *
   * \param [IN] currentChannel   Index number of the current channel
   */
  void (*fhss_change_channel)(radioif_t *radio, uint8_t currentChannel);

  /*!
   * \brief CAD Done callback prototype.
   *
   * \param [IN] channelDetected    Channel Activity detected during the CAD
   */
  void (*cad_done)(radioif_t *radio, bool channelActivityDetected);
} radio_events_t;



struct _radioif_t
{
  void (*init)(radioif_t *radio, radio_events_t *events);
  radio_state_t (*get_status)(radioif_t *radio);
  void (*set_modem)(radioif_t *radio, radio_modems_t modem);
  void (*set_channel)(radioif_t *radio, uint32_t freq);
  bool (*is_channel_free)(radioif_t *radio, radio_modems_t modem, uint32_t freq, int16_t rssiThresh);
  uint32_t (*random)(radioif_t *radio);
  void (*set_rx_config)(radioif_t *radio, radio_modems_t modem, uint32_t bandwidth,
      uint32_t datarate, uint8_t coderate, uint32_t bandwidthAfc,
      uint16_t preambleLen, uint16_t symbTimeout, bool fixLen,
      uint8_t payloadLen, bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
      bool iqInverted, bool rxContinuous);
  void (*set_tx_config)(radioif_t *radio, radio_modems_t modem, int8_t power, uint32_t fdev,
      uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
      uint16_t preambleLen, bool fixLen, bool crcOn, bool FreqHopOn,
      uint8_t HopPeriod, bool iqInverted, uint32_t timeout);
  bool (*check_rf_frequency)(radioif_t *radio, uint32_t frequency);
  uint32_t (*time_on_air)(radioif_t *radio, radio_modems_t modem, uint8_t pktLen);
  void (*send)(radioif_t *radio, uint8_t *buffer, uint8_t size);
  void (*sleep)(radioif_t *radio);
  void (*standby)(radioif_t *radio);
  void (*rx)(radioif_t *radio, uint32_t timeout);
  void (*start_cad)(radioif_t *radio);
  int16_t (*rssi)(radioif_t *radio, radio_modems_t modem);
  void (*write)(radioif_t *radio, uint8_t addr, uint8_t data);
  uint8_t (*read)(radioif_t *radio, uint8_t addr);
  void (*write_buffer)(radioif_t *radio, uint8_t addr, uint8_t *buffer, uint8_t size);
  void (*read_buffer)(radioif_t *radio, uint8_t addr, uint8_t *buffer, uint8_t size);
  void (*set_max_payload_length)(radioif_t *radio, radio_modems_t modem, uint8_t max);
};

#define to_radioif(x)   ((radioif_t*)&(x)->radioif)

static inline __attribute__((always_inline))
void radio_init(radioif_t *radio, radio_events_t *events)
{
  radio->init(radio, events);
}

static inline __attribute__((always_inline))
void radio_set_modem(radioif_t *radio, radio_modems_t modem)
{
  radio->set_modem(radio, modem);
}

static inline __attribute__((always_inline))
void radio_set_channel(radioif_t *radio, uint32_t freq)
{
  radio->set_channel(radio, freq);
}

static inline __attribute__((always_inline))
radio_state_t radio_get_status(radioif_t *radio)
{
  return radio->get_status(radio);
}

static inline __attribute__((always_inline))
bool radio_is_channel_free(radioif_t *radio, radio_modems_t modem, uint32_t freq, int16_t rssiThresh)
{
  return radio->is_channel_free(radio, modem, freq, rssiThresh);
}

static inline __attribute__((always_inline))
uint32_t radio_random(radioif_t *radio)
{
  return radio->random(radio);
}

static inline __attribute__((always_inline))
void radio_set_rx_config(radioif_t *radio, radio_modems_t modem, uint32_t bandwidth,
    uint32_t datarate, uint8_t coderate, uint32_t bandwidthAfc,
    uint16_t preambleLen, uint16_t symbTimeout, bool fixLen,
    uint8_t payloadLen, bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
    bool iqInverted, bool rxContinuous)
{
  radio->set_rx_config(radio, modem, bandwidth, datarate, coderate, bandwidthAfc, preambleLen, symbTimeout, fixLen,
      payloadLen, crcOn, FreqHopOn, HopPeriod, iqInverted, rxContinuous);
}

static inline __attribute__((always_inline))
void radio_set_tx_config(radioif_t *radio, radio_modems_t modem, int8_t power, uint32_t fdev,
    uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
    uint16_t preambleLen, bool fixLen, bool crcOn, bool FreqHopOn,
    uint8_t HopPeriod, bool iqInverted, uint32_t timeout)
{
  radio->set_tx_config(radio, modem, power, fdev,
      bandwidth, datarate, coderate,
      preambleLen, fixLen, crcOn, FreqHopOn,
      HopPeriod, iqInverted, timeout);
}

static inline __attribute__((always_inline))
bool radio_check_rf_frequency(radioif_t *radio, uint32_t frequency)
{
  return radio->check_rf_frequency(radio, frequency);
}

static inline __attribute__((always_inline))
uint32_t radio_get_time_on_air(radioif_t *radio, radio_modems_t modem, uint8_t pktLen)
{
  return radio->time_on_air(radio, modem, pktLen);
}

static inline __attribute__((always_inline))
void radio_send(radioif_t *radio, uint8_t *buffer, uint8_t size)
{
  radio->send(radio, buffer, size);
}

static inline __attribute__((always_inline))
void radio_sleep(radioif_t *radio)
{
  radio->sleep(radio);
}

static inline __attribute__((always_inline))
void radio_standby(radioif_t *radio)
{
  radio->standby(radio);
}

static inline __attribute__((always_inline))
void radio_rx(radioif_t *radio, uint32_t timeout)
{
  radio->rx(radio, timeout);
}

static inline __attribute__((always_inline))
void radio_start_cad(radioif_t *radio)
{
  radio->start_cad(radio);
}

static inline __attribute__((always_inline))
int16_t radio_get_rssi(radioif_t *radio, radio_modems_t modem)
{
  return radio->rssi(radio, modem);
}

#endif /* LORA_RADIO_H_ */
