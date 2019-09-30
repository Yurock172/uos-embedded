/*
 * sx1272.h
 *
 *  Created on: 15 сент. 2019 г.
 *      Author: yurock
 */

#ifndef LORA_SX1272_H_
#define LORA_SX1272_H_

#include <runtime/lib.h>
#include <kernel/uos.h>
#include <spi/spi-master-interface.h>
#include <stm32l1/gpio.h>
#include <timer/timer.h>
#include <timer/timeout.h>
#include "radio.h"
#include "sx1272Regs-Fsk.h"
#include "sx1272Regs-LoRa.h"


/*!
 * SX1272 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

#define RX_BUFFER_SIZE                              256

/*!
 * Radio wakeup time from SLEEP mode
 */
#define RADIO_OSC_STARTUP                           1 // [ms]

/*!
 * Radio PLL lock and Mode Ready delay which can vary with the temperature
 */
#define RADIO_SLEEP_TO_RX                           2 // [ms]

/*!
 * Radio complete Wake-up Time with margin for temperature compensation
 */
#define RADIO_WAKEUP_TIME                           ( RADIO_OSC_STARTUP + RADIO_SLEEP_TO_RX )


#define RADIO_INIT_REGISTERS_VALUE                \
{                                                 \
    { MODEM_FSK , REG_LNA                , 0x23 },\
    { MODEM_FSK , REG_RXCONFIG           , 0x1E },\
    { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
    { MODEM_FSK , REG_AFCFEI             , 0x01 },\
    { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
    { MODEM_FSK , REG_OSC                , 0x07 },\
    { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },\
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
    { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
    { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },\
    { MODEM_FSK , REG_IMAGECAL           , 0x02 },\
    { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
    { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
    { MODEM_LORA, REG_LR_DETECTOPTIMIZE  , 0x43 },\
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
}

/*!
 * Radio FSK modem parameters
 */
typedef struct {
  int8_t power;
  uint32_t fdev;
  uint32_t bandwidth;
  uint32_t bandwidth_afc;
  uint32_t datarate;
  uint16_t preamble_len;
  bool fix_len;
  uint8_t payload_len;
  bool crc_on;
  bool iq_inverted;
  bool rx_continuous;
  uint32_t tx_timeout;
} radio_fsk_settings_t;

/*!
 * Radio FSK packet handler state
 */
typedef struct {
  uint8_t preamble_detected;
  uint8_t sync_word_detected;
  int8_t rssi_value;
  int32_t afc_value;
  uint8_t rx_gain;
  uint16_t size;
  uint16_t nb_bytes;
  uint8_t fifo_thresh;
  uint8_t chunk_size;
} radio_fsk_packet_handler_t;

/*!
 * Radio LoRa modem parameters
 */
typedef struct {
  int8_t power;
  uint32_t bandwidth;
  uint32_t datarate;
  bool low_datarate_optimize;
  uint8_t coderate;
  uint16_t preamble_len;
  bool fix_len;
  uint8_t payload_len;
  bool crc_on;
  bool freq_hop_on;
  uint8_t hop_period;
  bool iq_inverted;
  bool rx_continuous;
  uint32_t tx_timeout;
} radio_lora_settings_t;

/*!
 * Radio LoRa packet handler state
 */
typedef struct {
  int8_t snr_value;
  int16_t rssi_value;
  uint8_t size;
} radio_lora_packet_handler_t;

/*!
 * Radio Settings
 */
typedef struct {
  radio_state_t state;
  radio_modems_t modem;
  uint32_t channel;
  radio_fsk_settings_t fsk;
  radio_fsk_packet_handler_t fsk_packet_handler;
  radio_lora_settings_t lora;
  radio_lora_packet_handler_t lora_packet_handler;
} radio_settings_t;




typedef struct sx1272_s {
  radioif_t radioif;
  spimif_t *spi;
  spi_message_t   msg;
  stm32l1_gpio_t reset;
  stm32l1_gpio_t dio0;
  stm32l1_gpio_t dio1;
  stm32l1_gpio_t dio2;
  stm32l1_gpio_t dio3;
  stm32l1_gpio_t dio4;
  stm32l1_gpio_t dio5;
  stm32l1_gpio_t ant_tx;
  stm32l1_gpio_t ant_rx;

  radio_settings_t settings;
  radio_events_t *events;
  timer_t *timer;
  bool radio_is_active;
  timeout_t tx_timeout;
  timeout_t rx_timeout;
  timeout_t rx_timeout_sync_word;
  mutex_t mutex_timeout;
  ARRAY(stack_timeout, 1000);
//  ARRAY(stack_irq, 500);
} sx1272_t;


void lora_init(sx1272_t *lora, spimif_t *s, timer_t *timer, unsigned freq, unsigned mode);


#endif /* LORA_SX1272_H_ */
