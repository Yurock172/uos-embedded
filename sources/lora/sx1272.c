/*
 * sx1272.c
 *
 *  Created on: 15 сент. 2019 г.
 *      Author: yurock
 */

#include "sx1272.h"
#include <timer/timeout.h>
#include <runtime/math.h>

//PORTB
#define RADIO_DIO_0             1
#define RADIO_DIO_1             10
#define RADIO_DIO_2             11
#define RADIO_DIO_3             7
#define RADIO_DIO_4             5
#define RADIO_DIO_5             4

// PORTA
#define RADIO_ANT_SWITCH_TX     4
#define RADIO_RESET             2
// PORTC
#define RADIO_ANT_SWITCH_RX     13

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET                                 -139

typedef struct {
  radio_modems_t Modem;
  uint8_t Addr;
  uint8_t Value;
} radio_registers_t;

const radio_registers_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;


static uint8_t rx_tx_buffer[RX_BUFFER_SIZE];



//////////////////////////////////////////////////////////
void sx1272_init(radioif_t *radio, radio_events_t *events);
radio_state_t sx1272_get_status(radioif_t *radio);
void sx1272_set_modem(radioif_t *radio, radio_modems_t modem);
void sx1272_set_channel(radioif_t *radio, uint32_t freq);
bool sx1272_is_channel_free(radioif_t *radio, radio_modems_t modem, uint32_t freq, int16_t rssiThresh);
uint32_t sx1272_random(radioif_t *radio);
void sx1272_set_rx_config(radioif_t *radio, radio_modems_t modem, uint32_t bandwidth,
    uint32_t datarate, uint8_t coderate, uint32_t bandwidthAfc,
    uint16_t preambleLen, uint16_t symbTimeout, bool fixLen,
    uint8_t payloadLen, bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
    bool iqInverted, bool rxContinuous);
void sx1272_set_tx_config(radioif_t *radio, radio_modems_t modem, int8_t power, uint32_t fdev,
    uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
    uint16_t preambleLen, bool fixLen, bool crcOn, bool FreqHopOn,
    uint8_t HopPeriod, bool iqInverted, uint32_t timeout);
bool sx1272_check_rf_frequency(radioif_t *radio, uint32_t frequency);
uint32_t sx1272_set_time_on_air(radioif_t *radio, radio_modems_t modem, uint8_t pktLen);
void sx1272_send(radioif_t *radio, uint8_t *buffer, uint8_t size);
void sx1272_set_sleep(radioif_t *radio);
void sx1272_set_stby(radioif_t *radio);
void sx1272_set_rx(radioif_t *radio, uint32_t timeout);
void sx1272_start_cad(radioif_t *radio);
int16_t sx1272_read_rssi(radioif_t *radio, radio_modems_t modem);
void sx1272_write(radioif_t *radio, uint8_t addr, uint8_t data);
uint8_t sx1272_read(radioif_t *radio, uint8_t addr);
void sx1272_write_buffer(radioif_t *radio, uint8_t addr, uint8_t *buffer, uint8_t size);
void sx1272_read_buffer(radioif_t *radio, uint8_t addr, uint8_t *buffer, uint8_t size);
void sx1272_set_max_payload_length(radioif_t *radio, radio_modems_t modem, uint8_t max);
//////////////////////////////////////////////////////////


//static void sx1272_write_fifo(sx1272_t *sx, uint8_t *buffer, uint8_t size);
//static void sx1272_read_fifo(sx1272_t *sx, uint8_t *buffer, uint8_t size);
static void SX1272SetOpMode(sx1272_t *sx, uint8_t opMode);
static void SX1272SetAntSwLowPower(sx1272_t *sx, bool status);
static void SX1272AntSwInit(sx1272_t *sx);
static void SX1272AntSwDeInit(sx1272_t *sx);
static void SX1272SetAntSw(sx1272_t *sx, uint8_t rxTx);
static void SX1272IoIrqInit(sx1272_t *sx);
static void sx1272_on_dio0_irq(gpioif_t *pin, void *arg);
static void sx1272_on_dio1_irq(gpioif_t *pin, void *arg);
static void sx1272_on_dio2_irq(gpioif_t *pin, void *arg);
static void sx1272_on_dio3_irq(gpioif_t *pin, void *arg);
static void sx1272_on_dio4_irq(gpioif_t *pin, void *arg);
//static void sx1272_on_dio5_irq(gpioif_t *pin, void *arg);
static uint8_t SX1272GetPaSelect( uint32_t channel );
//static void sx1272_set_modem(sx1272_t *sx, radio_modems_t modem);
//static uint8_t sx1272_get_pa_select(uint32_t channel);
//static void sx1272_set_tx(sx1272_t *sx, uint32_t timeout);
static uint8_t GetFskBandwidthRegValue(uint32_t bandwidth);
void SX1272SetTx(sx1272_t *sx, uint32_t timeout );

//void timeout_hendler(void *arg);

/// functions

static uint8_t SX1272GetPaSelect( uint32_t channel ) {
    return RF_PACONFIG_PASELECT_PABOOST;
}
static void SX1272Reset(sx1272_t *sx) {
// Set RESET pin to 1
  stm32l1_gpio_init(&sx->reset, GPIO_PORT_A, RADIO_RESET, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_50MHZ);
  gpio_set_val(to_gpioif(&sx->reset), 1);
// Wait 1 ms
  mdelay(1);
// Configure RESET as input
  stm32l1_gpio_init(&sx->reset, GPIO_PORT_A, RADIO_RESET, GPIO_FLAGS_INPUT | GPIO_FLAGS_50MHZ);
// Wait 6 ms
  mdelay(6);
}



static void SX1272SetOpMode(sx1272_t *sx, uint8_t opMode) {
  if (opMode == RF_OPMODE_SLEEP) {
    SX1272SetAntSwLowPower(sx, true);
  } else {
    SX1272SetAntSwLowPower(sx, false);
    if (opMode == RF_OPMODE_TRANSMITTER) {
      SX1272SetAntSw(sx, 1);
    } else {
      SX1272SetAntSw(sx, 0);
    }
  }
  sx1272_write(to_radioif(sx), REG_OPMODE, ( sx1272_read(to_radioif(sx), REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

static void SX1272SetAntSwLowPower(sx1272_t *sx, bool status) {
  if (sx->radio_is_active != status) {
    sx->radio_is_active = status;

    if (status == false) {
      SX1272AntSwInit(sx);
    } else {
      SX1272AntSwDeInit(sx);
    }
  }
}

static void SX1272AntSwInit(sx1272_t *sx) {
//  stm32l1_gpio_init(&sx->ant_tx, GPIO_PORT_A, RADIO_ANT_SWITCH_TX, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_PULL_UP | GPIO_FLAGS_50MHZ);
  gpio_set_val(to_gpioif(&sx->ant_tx), 0);
//  stm32l1_gpio_init(&sx->ant_rx, GPIO_PORT_C, RADIO_ANT_SWITCH_RX, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_PULL_UP | GPIO_FLAGS_50MHZ);
  gpio_set_val(to_gpioif(&sx->ant_rx), 1);
}

static void SX1272AntSwDeInit(sx1272_t *sx) {
//  stm32l1_gpio_init(&sx->ant_tx, GPIO_PORT_A, RADIO_ANT_SWITCH_TX, GPIO_FLAGS_ANALOG);
  gpio_set_val(to_gpioif(&sx->ant_tx), 0);
//  stm32l1_gpio_init(&sx->ant_rx, GPIO_PORT_C, RADIO_ANT_SWITCH_RX, GPIO_FLAGS_ANALOG);
  gpio_set_val(to_gpioif(&sx->ant_rx), 1);
}

static void SX1272SetAntSw(sx1272_t *sx, uint8_t rxTx) {
  if (rxTx != 0) { // 1: TX, 0: RX
    gpio_set_val(to_gpioif(&sx->ant_tx), 0);
    gpio_set_val(to_gpioif(&sx->ant_tx), 1);
  } else {
    gpio_set_val(to_gpioif(&sx->ant_rx), 1);
    gpio_set_val(to_gpioif(&sx->ant_rx), 0);
  }
}


void SX1272WriteFifo(sx1272_t *sx,  uint8_t *buffer, uint8_t size ) {
  sx1272_write_buffer(to_radioif(sx), 0, buffer, size );
}

void SX1272ReadFifo(sx1272_t *sx,  uint8_t *buffer, uint8_t size ) {
  sx1272_read_buffer(to_radioif(sx), 0, buffer, size );
}

void SX1272SetTx(sx1272_t *sx, uint32_t timeout ) {
//    TimerSetValue( &TxTimeoutTimer, timeout );
  timeout_set_value(&sx->tx_timeout, timeout);

  switch( sx->settings.modem ) {
  case MODEM_FSK: {
    // DIO0=PacketSent
    // DIO1=FifoEmpty
    // DIO2=FifoFull
    // DIO3=FifoEmpty
    // DIO4=LowBat
    // DIO5=ModeReady
    sx1272_write(to_radioif(sx), REG_DIOMAPPING1, ( sx1272_read(to_radioif(sx), REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
        RF_DIOMAPPING1_DIO1_MASK &
        RF_DIOMAPPING1_DIO2_MASK ) |
        RF_DIOMAPPING1_DIO1_01 );

    sx1272_write(to_radioif(sx), REG_DIOMAPPING2, ( sx1272_read(to_radioif(sx), REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
        RF_DIOMAPPING2_MAP_MASK ) );
    sx->settings.fsk_packet_handler.fifo_thresh = sx1272_read(to_radioif(sx), REG_FIFOTHRESH ) & 0x3F;
  }
    break;
  case MODEM_LORA: {
    if( sx->settings.lora.freq_hop_on == true ) {
      sx1272_write(to_radioif(sx), REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
          RFLR_IRQFLAGS_RXDONE |
          RFLR_IRQFLAGS_PAYLOADCRCERROR |
          RFLR_IRQFLAGS_VALIDHEADER |
          //RFLR_IRQFLAGS_TXDONE |
          RFLR_IRQFLAGS_CADDONE |
          //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
          RFLR_IRQFLAGS_CADDETECTED );

      // DIO0=TxDone, DIO2=FhssChangeChannel
      sx1272_write(to_radioif(sx), REG_DIOMAPPING1, ( sx1272_read(to_radioif(sx), REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
    } else {
      sx1272_write(to_radioif(sx), REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
          RFLR_IRQFLAGS_RXDONE |
          RFLR_IRQFLAGS_PAYLOADCRCERROR |
          RFLR_IRQFLAGS_VALIDHEADER |
          //RFLR_IRQFLAGS_TXDONE |
          RFLR_IRQFLAGS_CADDONE |
          RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
          RFLR_IRQFLAGS_CADDETECTED );

      // DIO0=TxDone
      sx1272_write(to_radioif(sx), REG_DIOMAPPING1, ( sx1272_read(to_radioif(sx), REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
    }
  }
  break;
  }

  sx->settings.state = RF_TX_RUNNING;
//  TimerStart( &TxTimeoutTimer );
  timeout_start(&sx->tx_timeout);
  SX1272SetOpMode(sx, RF_OPMODE_TRANSMITTER );
}

static void SX1272IoIrqInit(sx1272_t *sx) {
  gpio_attach_interrupt(&sx->dio0.gpioif, GPIO_EVENT_RISING_EDGE, sx1272_on_dio0_irq, (void *) sx);
  gpio_attach_interrupt(&sx->dio1.gpioif, GPIO_EVENT_RISING_EDGE, sx1272_on_dio1_irq, (void *) sx);
  gpio_attach_interrupt(&sx->dio2.gpioif, GPIO_EVENT_RISING_EDGE, sx1272_on_dio2_irq, (void *) sx);
  gpio_attach_interrupt(&sx->dio3.gpioif, GPIO_EVENT_RISING_EDGE, sx1272_on_dio3_irq, (void *) sx);
  gpio_attach_interrupt(&sx->dio4.gpioif, GPIO_EVENT_RISING_EDGE, sx1272_on_dio4_irq, (void *) sx);
//  gpio_attach_interrupt(&sx->dio5.gpioif, GPIO_EVENT_RISING_EDGE, sx1272_on_dio5_irq, (void *) sx);
}
/*
void sx1272_io_de_init(sx1272_t *sx) {
//  stm32l1_gpio_init(&lora->DIO0, GPIO_PORT_B, RADIO_DIO_0, GPIO_FLAGS_INPUT);
//  stm32l1_gpio_init(&lora->DIO1, GPIO_PORT_B, RADIO_DIO_1, GPIO_FLAGS_INPUT);
//  stm32l1_gpio_init(&lora->DIO2, GPIO_PORT_B, RADIO_DIO_2, GPIO_FLAGS_INPUT);
//  stm32l1_gpio_init(&lora->DIO3, GPIO_PORT_B, RADIO_DIO_3, GPIO_FLAGS_INPUT);
//  stm32l1_gpio_init(&lora->DIO4, GPIO_PORT_B, RADIO_DIO_4, GPIO_FLAGS_INPUT);
//  stm32l1_gpio_init(&lora->DIO5, GPIO_PORT_B, RADIO_DIO_5, GPIO_FLAGS_INPUT);
}*/

static void sx1272_on_dio0_irq(gpioif_t *pin, void *arg) {
  sx1272_t *sx = (sx1272_t *)arg;
  volatile uint8_t irqFlags = 0;

  switch( sx->settings.state ) {
  case RF_RX_RUNNING:
//    timeout_stop(&sx->tx_timeout);
    // RxDone interrupt
    switch(sx->settings.modem ) {
    case MODEM_FSK:
      if( sx->settings.fsk.crc_on == true ) {
        irqFlags = sx1272_read(to_radioif(sx), REG_IRQFLAGS2 );
        if( ( irqFlags & RF_IRQFLAGS2_CRCOK ) != RF_IRQFLAGS2_CRCOK ) {
          // Clear Irqs
          sx1272_write(to_radioif(sx), REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
              RF_IRQFLAGS1_PREAMBLEDETECT | RF_IRQFLAGS1_SYNCADDRESSMATCH );
          sx1272_write(to_radioif(sx), REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

          timeout_stop(&sx->rx_timeout);

          if( sx->settings.fsk.rx_continuous == false ) {
            timeout_stop(&sx->rx_timeout_sync_word);
            sx->settings.state = RF_IDLE;
          } else {
            // Continuous mode restart Rx chain
            sx1272_write(to_radioif(sx), REG_RXCONFIG, sx1272_read(to_radioif(sx), REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
            timeout_start(&sx->rx_timeout_sync_word);
          }
          if( ( sx->events != NULL ) && ( sx->events->rx_error != NULL ) ) {
            sx->events->rx_error(to_radioif(sx));
          }
          sx->settings.fsk_packet_handler.preamble_detected = false;
          sx->settings.fsk_packet_handler.sync_word_detected = false;
          sx->settings.fsk_packet_handler.nb_bytes = 0;
          sx->settings.fsk_packet_handler.size = 0;
          break;
        }
      }
      // Read received packet size
      if( (sx->settings.fsk_packet_handler.size == 0 ) && (sx->settings.fsk_packet_handler.nb_bytes == 0 ) ) {
        if(sx->settings.fsk.fix_len == false ) {
          SX1272ReadFifo(sx, ( uint8_t* )&sx->settings.fsk_packet_handler.size, 1 );
        } else {
         sx->settings.fsk_packet_handler.size = sx1272_read(to_radioif(sx), REG_PAYLOADLENGTH );
        }
        SX1272ReadFifo(sx, rx_tx_buffer +sx->settings.fsk_packet_handler.nb_bytes, sx->settings.fsk_packet_handler.size -sx->settings.fsk_packet_handler.nb_bytes );
       sx->settings.fsk_packet_handler.nb_bytes += (sx->settings.fsk_packet_handler.size -sx->settings.fsk_packet_handler.nb_bytes );
      } else {
        SX1272ReadFifo(sx, rx_tx_buffer +sx->settings.fsk_packet_handler.nb_bytes,sx->settings.fsk_packet_handler.size -sx->settings.fsk_packet_handler.nb_bytes );
       sx->settings.fsk_packet_handler.nb_bytes += (sx->settings.fsk_packet_handler.size -sx->settings.fsk_packet_handler.nb_bytes );
      }

      timeout_stop(&sx->rx_timeout);
      if(sx->settings.fsk.rx_continuous == false ) {
       sx->settings.state = RF_IDLE;
        timeout_stop(&sx->rx_timeout_sync_word);
      } else {
        // Continuous mode restart Rx chain
        sx1272_write(to_radioif(sx), REG_RXCONFIG, sx1272_read(to_radioif(sx), REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
        timeout_start(&sx->rx_timeout_sync_word);
      }

      if( ( sx->events != NULL ) && ( sx->events->rx_done != NULL ) ) {
        sx->events->rx_done(to_radioif(sx), rx_tx_buffer, sx->settings.fsk_packet_handler.size,sx->settings.fsk_packet_handler.rssi_value, 0 );
      }
     sx->settings.fsk_packet_handler.preamble_detected = false;
     sx->settings.fsk_packet_handler.sync_word_detected = false;
     sx->settings.fsk_packet_handler.nb_bytes = 0;
     sx->settings.fsk_packet_handler.size = 0;
      break;
    case MODEM_LORA: {
      int8_t snr = 0;

      // Clear Irq
      sx1272_write(to_radioif(sx), REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

      irqFlags = sx1272_read(to_radioif(sx), REG_LR_IRQFLAGS );
      if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR ) {
        // Clear Irq
        sx1272_write(to_radioif(sx), REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

        if(sx->settings.lora.rx_continuous == false ) {
         sx->settings.state = RF_IDLE;
        }
        timeout_stop(&sx->rx_timeout);

        if( (sx->events != NULL ) && ( sx->events->rx_error != NULL ) ) {
          sx->events->rx_error(to_radioif(sx));
        }
        break;
      }

     sx->settings.lora_packet_handler.snr_value = sx1272_read(to_radioif(sx), REG_LR_PKTSNRVALUE );
      if(sx->settings.lora_packet_handler.snr_value & 0x80 ) { // The SNR sign bit is 1
        // Invert and divide by 4
        snr = ( ( ~sx->settings.lora_packet_handler.snr_value + 1 ) & 0xFF ) >> 2;
        snr = -snr;
      } else {
        // Divide by 4
        snr = (sx->settings.lora_packet_handler.snr_value & 0xFF ) >> 2;
      }

      int16_t rssi = sx1272_read(to_radioif(sx), REG_LR_PKTRSSIVALUE );
      if( snr < 0 ) {
       sx->settings.lora_packet_handler.rssi_value = RSSI_OFFSET + rssi + ( rssi >> 4 ) +
            snr;
      } else {
       sx->settings.lora_packet_handler.rssi_value = RSSI_OFFSET + rssi + ( rssi >> 4 );
      }

     sx->settings.lora_packet_handler.size = sx1272_read(to_radioif(sx), REG_LR_RXNBBYTES );
      SX1272ReadFifo(sx, rx_tx_buffer, sx->settings.lora_packet_handler.size );

      if(sx->settings.lora.rx_continuous == false ) {
       sx->settings.state = RF_IDLE;
      }
      timeout_stop(&sx->rx_timeout);
      if( ( sx->events != NULL ) && ( sx->events->rx_done != NULL ) ) {
        sx->events->rx_done(to_radioif(sx), rx_tx_buffer, sx->settings.lora_packet_handler.size,
            sx->settings.lora_packet_handler.rssi_value, sx->settings.lora_packet_handler.snr_value );
      }
    }
    break;
    default:
      break;
    }
    break;
  case RF_TX_RUNNING:
    timeout_stop(&sx->tx_timeout);
    // TxDone interrupt
    if( sx->settings.modem == MODEM_LORA) {
      // Clear Irq
      sx1272_write(to_radioif(sx), REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
    }
    sx->settings.state = RF_IDLE;
      if( ( sx->events != NULL ) && ( sx->events->tx_done != NULL ) ) {
        sx->events->tx_done(to_radioif(sx));
      }
    break;
  default:
    break;
  }
}

static void sx1272_on_dio1_irq(gpioif_t *pin, void *arg) {
  sx1272_t *sx = (sx1272_t *)arg;
  switch( sx->settings.state ) {
  case RF_RX_RUNNING:
    switch( sx->settings.modem ) {
    case MODEM_FSK:
      // FifoLevel interrupt
      // Read received packet size
      if( ( sx->settings.fsk_packet_handler.size == 0 ) && ( sx->settings.fsk_packet_handler.nb_bytes == 0 ) ) {
        if( sx->settings.fsk.fix_len == false ) {
          SX1272ReadFifo(sx, ( uint8_t* )&sx->settings.fsk_packet_handler.size, 1 );
        } else {
          sx->settings.fsk_packet_handler.size = sx1272_read(to_radioif(sx), REG_PAYLOADLENGTH );
        }
      }

      if( ( sx->settings.fsk_packet_handler.size - sx->settings.fsk_packet_handler.nb_bytes ) > sx->settings.fsk_packet_handler.fifo_thresh) {
        SX1272ReadFifo(sx, ( rx_tx_buffer + sx->settings.fsk_packet_handler.nb_bytes ), sx->settings.fsk_packet_handler.fifo_thresh);
        sx->settings.fsk_packet_handler.nb_bytes += sx->settings.fsk_packet_handler.fifo_thresh;
      } else {
        SX1272ReadFifo(sx, ( rx_tx_buffer + sx->settings.fsk_packet_handler.nb_bytes ), sx->settings.fsk_packet_handler.size - sx->settings.fsk_packet_handler.nb_bytes);
        sx->settings.fsk_packet_handler.nb_bytes += ( sx->settings.fsk_packet_handler.size - sx->settings.fsk_packet_handler.nb_bytes);
      }
      break;
    case MODEM_LORA:
      // Sync time out
      timeout_stop(&sx->rx_timeout);
      // Clear Irq
      sx1272_write(to_radioif(sx), REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT );

      sx->settings.state = RF_IDLE;
      if( ( sx->events != NULL ) && ( sx->events->rx_timeout != NULL ) ) {
        sx->events->rx_timeout(to_radioif(sx));
      }
      break;
    default:
      break;
    }
    break;
  case RF_TX_RUNNING:
    switch( sx->settings.modem ) {
    case MODEM_FSK:
      // FifoEmpty interrupt
      if( ( sx->settings.fsk_packet_handler.size - sx->settings.fsk_packet_handler.nb_bytes) > sx->settings.fsk_packet_handler.chunk_size) {
        SX1272WriteFifo(sx, ( rx_tx_buffer + sx->settings.fsk_packet_handler.nb_bytes), sx->settings.fsk_packet_handler.chunk_size);
        sx->settings.fsk_packet_handler.nb_bytes += sx->settings.fsk_packet_handler.chunk_size;
      } else {
        // Write the last chunk of data
        SX1272WriteFifo(sx, rx_tx_buffer + sx->settings.fsk_packet_handler.nb_bytes, sx->settings.fsk_packet_handler.size - sx->settings.fsk_packet_handler.nb_bytes );
        sx->settings.fsk_packet_handler.nb_bytes += sx->settings.fsk_packet_handler.size - sx->settings.fsk_packet_handler.nb_bytes;
      }
      break;
    case MODEM_LORA:
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

static void sx1272_on_dio2_irq(gpioif_t *pin, void *arg) {
  sx1272_t *sx = (sx1272_t *)arg;
  switch( sx->settings.state ) {
  case RF_RX_RUNNING:
    switch( sx->settings.modem ) {
    case MODEM_FSK:
      if( ( sx->settings.fsk_packet_handler.preamble_detected == true ) && ( sx->settings.fsk_packet_handler.sync_word_detected == false ) ) {
        timeout_stop(&sx->rx_timeout_sync_word);
        sx->settings.fsk_packet_handler.sync_word_detected = true;
        sx->settings.fsk_packet_handler.rssi_value = -( sx1272_read(to_radioif(sx), REG_RSSIVALUE ) >> 1 );
        sx->settings.fsk_packet_handler.afc_value = ( int32_t )( double )( ( ( uint16_t )sx1272_read(to_radioif(sx), REG_AFCMSB ) << 8 ) |
            ( uint16_t )sx1272_read(to_radioif(sx), REG_AFCLSB ) ) * ( double )FREQ_STEP;
        sx->settings.fsk_packet_handler.rx_gain = ( sx1272_read(to_radioif(sx), REG_LNA ) >> 5 ) & 0x07;
      }
      break;
    case MODEM_LORA:
      if( sx->settings.lora.freq_hop_on == true ) {
        // Clear Irq
        sx1272_write(to_radioif(sx), REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

        if( ( sx->events != NULL ) && ( sx->events->fhss_change_channel != NULL ) ) {
          sx->events->fhss_change_channel(to_radioif(sx), ( sx1272_read(to_radioif(sx), REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
        }
      }
      break;
    default:
      break;
    }
    break;
  case RF_TX_RUNNING:
    switch( sx->settings.modem ) {
    case MODEM_FSK:
      break;
    case MODEM_LORA:
      if( sx->settings.lora.freq_hop_on == true ) {
        // Clear Irq
        sx1272_write(to_radioif(sx), REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

        if( ( sx->events != NULL ) && ( sx->events->fhss_change_channel != NULL ) ) {
          sx->events->fhss_change_channel(to_radioif(sx), ( sx1272_read(to_radioif(sx), REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
        }
      }
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

static void sx1272_on_dio3_irq(gpioif_t *pin, void *arg) {
  sx1272_t *sx = (sx1272_t *)arg;
  switch( sx->settings.modem ) {
  case MODEM_FSK:
    break;
  case MODEM_LORA:
    if( ( sx1272_read(to_radioif(sx), REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED ) {
      // Clear Irq
      sx1272_write(to_radioif(sx), REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
      if( ( sx->events != NULL ) && ( sx->events->cad_done != NULL ) ) {
        sx->events->cad_done(to_radioif(sx), true );
      }
    } else {
      // Clear Irq
      sx1272_write(to_radioif(sx), REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
      if( ( sx->events != NULL ) && ( sx->events->cad_done != NULL ) ) {
        sx->events->cad_done(to_radioif(sx), false );
      }
    }
    break;
  default:
    break;
  }
}

static void sx1272_on_dio4_irq(gpioif_t *pin, void *arg) {
  sx1272_t *sx = (sx1272_t *)arg;
  switch( sx->settings.modem ) {
  case MODEM_FSK:
  {
    if( sx->settings.fsk_packet_handler.preamble_detected == false ) {
      sx->settings.fsk_packet_handler.preamble_detected = true;
    }
  }
  break;
  case MODEM_LORA:
    break;
  default:
    break;
  }
}

//static void sx1272_on_dio5_irq(gpioif_t *pin, void *arg) {
//  sx1272_t *sx = (sx1272_t *)arg;
//  debug_printf("DIO_5 IRQ\r\n");
//  switch (sx->settings.modem) {
//  case MODEM_FSK:
//    break;
//  case MODEM_LORA:
//    break;
//  default:
//    break;
//  }
//}




// Определения для функция работы с радио интерфейсом

//void radio_init(radioif_t *radio, radio_events_t *events);
//radio_state_t radio_get_status(radioif_t *radio);
//void radio_set_modem(radioif_t *radio, radio_modems_t modem);
//void radio_set_channel(radioif_t *radio, uint32_t freq);
//bool radio_is_channel_free(radioif_t *radio, radio_modems_t modem, uint32_t freq, int16_t rssiThresh);
//uint32_t radio_random(radioif_t *radio);
//void radio_set_rx_config(radioif_t *radio, radio_modems_t modem, uint32_t bandwidth,
//    uint32_t datarate, uint8_t coderate, uint32_t bandwidthAfc,
//    uint16_t preambleLen, uint16_t symbTimeout, bool fixLen, uint8_t payloadLen,
//    bool crcOn, bool freqHopOn, uint8_t hopPeriod, bool iqInverted,
//    bool rxContinuous);
//void radio_set_tx_config(radioif_t *radio, radio_modems_t modem, int8_t power, uint32_t fdev,
//    uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
//    uint16_t preambleLen, bool fixLen, bool crcOn, bool freqHopOn,
//    uint8_t hopPeriod, bool iqInverted, uint32_t timeout);
//bool radio_check_rf_frequency(radioif_t *radio, uint32_t frequency);
//uint32_t radio_get_time_on_air(radioif_t *radio, radio_modems_t modem, uint8_t pktLen);
//void radio_send(radioif_t *radio, uint8_t *buffer, uint8_t size);
//void radio_set_sleep(radioif_t *radio);
//void radio_set_stby(radioif_t *radio);
//void radio_set_rx(radioif_t *radio, uint32_t timeout);
//void radio_start_cad(radioif_t *radio);
//int16_t radio_rssi(radioif_t *radio, radio_modems_t modem);


typedef struct {
  uint32_t bandwidth;
  uint8_t RegValue;
} fsk_bandwidth_t;


const fsk_bandwidth_t fskBandwidths[] = {
    { 2600, 0x17 },
    { 3100, 0x0F },
    { 3900, 0x07 },
    { 5200, 0x16 },
    { 6300, 0x0E },
    { 7800, 0x06 },
    { 10400, 0x15 },
    { 12500, 0x0D },
    { 15600, 0x05 },
    { 20800, 0x14 },
    { 25000, 0x0C },
    { 31300, 0x04 },
    { 41700, 0x13 },
    { 50000, 0x0B },
    { 62500, 0x03 },
    { 83333, 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Badwidth
};

static uint8_t GetFskBandwidthRegValue(uint32_t bandwidth) {
  uint8_t i;

  for (i = 0; i < (sizeof(fskBandwidths) / sizeof(fsk_bandwidth_t)) - 1; i++) {
    if ((bandwidth >= fskBandwidths[i].bandwidth)
        && (bandwidth < fskBandwidths[i + 1].bandwidth)) {
      return fskBandwidths[i].RegValue;
    }
  }
  // ERROR: Value not found
  while (1)
    ;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////


//const struct radioif_t radioif = {
//    sx1272_init,
//    sx1272_get_status,
//    sx1272_set_modem,
//    sx1272_set_channel,
//    sx1272_is_channel_free,
//    sx1272_random,
//    sx1272_set_rx_config,
//    sx1272_set_tx_config,
//    sx1272_check_rf_frequency,
//    sx1272_set_time_on_air,
//    sx1272_send,
//    sx1272_set_sleep,
//    sx1272_set_stby,
//    sx1272_set_rx,
//    sx1272_start_cad,
//    sx1272_read_rssi,
//    sx1272_write,
//    sx1272_read,
//    sx1272_write_buffer,
//    sx1272_read_buffer,
//    sx1272_set_max_payload_length
//};


void sx1272_init(radioif_t *radio, radio_events_t *events) {
  uint8_t i;
  sx1272_t *sx = (sx1272_t *)radio;
  sx->events = events;

  SX1272Reset(sx);

  SX1272SetOpMode(sx, RF_OPMODE_SLEEP );

  SX1272IoIrqInit(sx);

  for (i = 0; i < sizeof(RadioRegsInit) / sizeof(radio_registers_t); i++) {
    sx1272_set_modem(radio, RadioRegsInit[i].Modem );
    sx1272_write(radio, RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
  }

  sx1272_set_modem(radio, MODEM_FSK );

  sx->settings.state = RF_IDLE;
}

radio_state_t sx1272_get_status(radioif_t *radio) {
  sx1272_t *sx = (sx1272_t *)radio;
  return sx->settings.state;
}

void sx1272_set_modem(radioif_t *radio, radio_modems_t modem) {
  sx1272_t *sx = (sx1272_t *)radio;
  if (sx->settings.modem == modem) {
    return;
  }

  sx->settings.modem = modem;
  switch (sx->settings.modem) {
  default:
  case MODEM_FSK:
    SX1272SetOpMode(sx, RF_OPMODE_SLEEP);
    sx1272_write(radio, REG_OPMODE, (sx1272_read(radio, REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );

    sx1272_write(radio, REG_DIOMAPPING1, 0x00);
    sx1272_write(radio, REG_DIOMAPPING2, 0x30); // DIO5=ModeReady

    break;
  case MODEM_LORA:
    SX1272SetOpMode(sx, RF_OPMODE_SLEEP);
    sx1272_write(radio, REG_OPMODE,(sx1272_read(radio, REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

    sx1272_write(radio, REG_DIOMAPPING1, 0x00);
    sx1272_write(radio, REG_DIOMAPPING2, 0x00);
    break;
  }
}

void sx1272_set_channel(radioif_t *radio, uint32_t freq) {
  sx1272_t *sx = (sx1272_t *)radio;
  sx->settings.channel = freq;
  freq = (uint32_t) ((double) freq / (double) FREQ_STEP);
  sx1272_write(to_radioif(sx), REG_FRFMSB, (uint8_t) ((freq >> 16) & 0xFF));
  sx1272_write(to_radioif(sx), REG_FRFMID, (uint8_t) ((freq >> 8) & 0xFF));
  sx1272_write(to_radioif(sx), REG_FRFLSB, (uint8_t) (freq & 0xFF));
}

bool sx1272_is_channel_free(radioif_t *radio, radio_modems_t modem, uint32_t freq, int16_t rssiThresh) {
  int16_t rssi = 0;
  sx1272_t *sx = (sx1272_t *)radio;

  sx1272_set_modem(radio, modem );
  sx1272_set_channel(radio, freq );
  SX1272SetOpMode(sx, RF_OPMODE_RECEIVER );
  mdelay(1);

  rssi = sx1272_read_rssi(radio, modem);

  sx1272_set_sleep(radio);
  if( rssi > rssiThresh ) {
    return false;
  }
  return true;
}

uint32_t sx1272_random(radioif_t *radio) {
  sx1272_t *sx = (sx1272_t *)radio;

  uint8_t i;
  uint32_t rnd = 0;

  /*
   * Radio setup for random number generation
   */
  // Set LoRa modem ON
  sx1272_set_modem(radio, MODEM_LORA );

  // Disable LoRa modem interrupts
  sx1272_write(radio, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
      RFLR_IRQFLAGS_RXDONE |
      RFLR_IRQFLAGS_PAYLOADCRCERROR |
      RFLR_IRQFLAGS_VALIDHEADER |
      RFLR_IRQFLAGS_TXDONE |
      RFLR_IRQFLAGS_CADDONE |
      RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
      RFLR_IRQFLAGS_CADDETECTED );

  // Set radio in continuous reception
  SX1272SetOpMode(sx, RF_OPMODE_RECEIVER );

  for( i = 0; i < 32; i++ ) {
    mdelay(1);
    // Unfiltered RSSI value reading. Only takes the LSB value
    rnd |= ( ( uint32_t )sx1272_read(radio, REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
  }

  sx1272_set_sleep(radio);

  return rnd;
}

void sx1272_set_rx_config(radioif_t *radio, radio_modems_t modem, uint32_t bandwidth,
    uint32_t datarate, uint8_t coderate, uint32_t bandwidthAfc,
    uint16_t preambleLen, uint16_t symbTimeout, bool fixLen,
    uint8_t payloadLen, bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
    bool iqInverted, bool rxContinuous) {

  sx1272_t *sx = (sx1272_t *)radio;
  sx1272_set_modem(radio, modem );

  switch( modem ) {
  case MODEM_FSK: {
    sx->settings.fsk.bandwidth = bandwidth;
    sx->settings.fsk.datarate = datarate;
    sx->settings.fsk.bandwidth_afc = bandwidthAfc;
    sx->settings.fsk.fix_len = fixLen;
    sx->settings.fsk.payload_len = payloadLen;
    sx->settings.fsk.crc_on = crcOn;
    sx->settings.fsk.iq_inverted = iqInverted;
    sx->settings.fsk.rx_continuous = rxContinuous;
    sx->settings.fsk.preamble_len = preambleLen;

    datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
    sx1272_write(radio, REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
    sx1272_write(radio, REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

    sx1272_write(radio, REG_RXBW, GetFskBandwidthRegValue( bandwidth ) );
    sx1272_write(radio, REG_AFCBW, GetFskBandwidthRegValue( bandwidthAfc ) );

    sx1272_write(radio, REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
    sx1272_write(radio, REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

    if( fixLen == 1 ) {
      sx1272_write(radio, REG_PAYLOADLENGTH, payloadLen );
    } else {
      sx1272_write(radio, REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum
    }

    sx1272_write(radio, REG_PACKETCONFIG1,
        ( sx1272_read(radio, REG_PACKETCONFIG1 ) &
            RF_PACKETCONFIG1_CRC_MASK &
            RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
            ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
            ( crcOn << 4 ) );
  }
  break;
  case MODEM_LORA: {
    sx->settings.lora.bandwidth = bandwidth;
    sx->settings.lora.datarate = datarate;
    sx->settings.lora.coderate = coderate;
    sx->settings.lora.preamble_len = preambleLen;
    sx->settings.lora.fix_len = fixLen;
    sx->settings.lora.preamble_len = payloadLen;
    sx->settings.lora.crc_on = crcOn;
    sx->settings.lora.freq_hop_on = FreqHopOn;
    sx->settings.lora.hop_period = HopPeriod;
    sx->settings.lora.iq_inverted = iqInverted;
    sx->settings.lora.rx_continuous = rxContinuous;

    if( datarate > 12 ) {
      datarate = 12;
    } else if( datarate < 6 ) {
      datarate = 6;
    }

    if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) ) {
      sx->settings.lora.low_datarate_optimize = 0x01;
    } else {
      sx->settings.lora.low_datarate_optimize = 0x00;
    }

    sx1272_write(radio, REG_LR_MODEMCONFIG1,
        ( sx1272_read(radio, REG_LR_MODEMCONFIG1 ) &
            RFLR_MODEMCONFIG1_BW_MASK &
            RFLR_MODEMCONFIG1_CODINGRATE_MASK &
            RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
            RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
            RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) |
            ( bandwidth << 6 ) | ( coderate << 3 ) |
            ( fixLen << 2 ) | ( crcOn << 1 ) |
            sx->settings.lora.low_datarate_optimize );

    sx1272_write(radio, REG_LR_MODEMCONFIG2,
        ( sx1272_read(radio, REG_LR_MODEMCONFIG2 ) &
            RFLR_MODEMCONFIG2_SF_MASK &
            RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
            ( datarate << 4 ) |
            ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

    sx1272_write(radio, REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

    sx1272_write(radio, REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
    sx1272_write(radio, REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

    if( fixLen == 1 ) {
      sx1272_write(radio, REG_LR_PAYLOADLENGTH, payloadLen );
    }

    if( sx->settings.lora.freq_hop_on == true ) {
      sx1272_write(radio, REG_LR_PLLHOP, ( sx1272_read(radio, REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
      sx1272_write(radio, REG_LR_HOPPERIOD, sx->settings.lora.hop_period );
    }

    if( datarate == 6 ) {
      sx1272_write(radio, REG_LR_DETECTOPTIMIZE,
          ( sx1272_read(radio, REG_LR_DETECTOPTIMIZE ) &
              RFLR_DETECTIONOPTIMIZE_MASK ) |
              RFLR_DETECTIONOPTIMIZE_SF6 );
      sx1272_write(radio, REG_LR_DETECTIONTHRESHOLD,
          RFLR_DETECTIONTHRESH_SF6 );
    } else {
      sx1272_write(radio, REG_LR_DETECTOPTIMIZE,
          ( sx1272_read(radio, REG_LR_DETECTOPTIMIZE ) &
              RFLR_DETECTIONOPTIMIZE_MASK ) |
              RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
      sx1272_write(radio, REG_LR_DETECTIONTHRESHOLD,
          RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
    }
  }
  break;
  }
}

void sx1272_set_tx_config(radioif_t *radio, radio_modems_t modem, int8_t power, uint32_t fdev,
    uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
    uint16_t preambleLen, bool fixLen, bool crcOn, bool FreqHopOn,
    uint8_t HopPeriod, bool iqInverted, uint32_t timeout) {

  sx1272_t *sx = (sx1272_t*) radio;
  uint8_t paConfig = 0;
  uint8_t paDac = 0;

  sx1272_set_modem(radio, modem );

  paConfig = sx1272_read(radio, REG_PACONFIG );
  paDac = sx1272_read(radio, REG_PADAC );

  paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1272GetPaSelect( sx->settings.channel );

  if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST ) {
    if( power > 17 ) {
      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
    } else {
      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
    }
    if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON ) {
      if( power < 5 ) {
        power = 5;
      }
      if( power > 20 ) {
        power = 20;
      }
      paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
    } else {
      if( power < 2 ) {
        power = 2;
      }
      if( power > 17 ) {
        power = 17;
      }
      paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
    }
  } else {
    if( power < -1 ) {
      power = -1;
    }
    if( power > 14 ) {
      power = 14;
    }
    paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
  }
  sx1272_write(radio, REG_PACONFIG, paConfig );
  sx1272_write(radio, REG_PADAC, paDac );

  switch( modem ) {
  case MODEM_FSK: {
    sx->settings.fsk.power = power;
    sx->settings.fsk.fdev = fdev;
    sx->settings.fsk.bandwidth = bandwidth;
    sx->settings.fsk.datarate = datarate;
    sx->settings.fsk.preamble_len = preambleLen;
    sx->settings.fsk.fix_len = fixLen;
    sx->settings.fsk.crc_on = crcOn;
    sx->settings.fsk.iq_inverted = iqInverted;
    sx->settings.fsk.tx_timeout = timeout;

    fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
    sx1272_write(radio, REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
    sx1272_write(radio, REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

    datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
    sx1272_write(radio, REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
    sx1272_write(radio, REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

    sx1272_write(radio, REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
    sx1272_write(radio, REG_PREAMBLELSB, preambleLen & 0xFF );

    sx1272_write(radio, REG_PACKETCONFIG1,
        ( sx1272_read(radio, REG_PACKETCONFIG1 ) & RF_PACKETCONFIG1_CRC_MASK & RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
        ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
        ( crcOn << 4 ) );

  }
  break;
  case MODEM_LORA: {
    sx->settings.lora.power = power;
    sx->settings.lora.bandwidth = bandwidth;
    sx->settings.lora.datarate = datarate;
    sx->settings.lora.coderate = coderate;
    sx->settings.lora.preamble_len = preambleLen;
    sx->settings.lora.fix_len = fixLen;
    sx->settings.lora.freq_hop_on = FreqHopOn;
    sx->settings.lora.hop_period = HopPeriod;
    sx->settings.lora.crc_on = crcOn;
    sx->settings.lora.iq_inverted = iqInverted;
    sx->settings.lora.tx_timeout = timeout;

    if( datarate > 12 ) {
      datarate = 12;
    } else if( datarate < 6 ) {
      datarate = 6;
    }
    if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) ) {
      sx->settings.lora.low_datarate_optimize = 0x01;
    } else {
      sx->settings.lora.low_datarate_optimize = 0x00;
    }

    if( sx->settings.lora.freq_hop_on == true ) {
      sx1272_write(radio, REG_LR_PLLHOP, ( sx1272_read(radio, REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
      sx1272_write(radio, REG_LR_HOPPERIOD, sx->settings.lora.hop_period );
    }

    sx1272_write(radio, REG_LR_MODEMCONFIG1, ( sx1272_read(radio, REG_LR_MODEMCONFIG1 ) &
        RFLR_MODEMCONFIG1_BW_MASK &
        RFLR_MODEMCONFIG1_CODINGRATE_MASK &
        RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
        RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
        RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) |
        ( bandwidth << 6 ) | ( coderate << 3 ) |
        ( fixLen << 2 ) | ( crcOn << 1 ) |
        sx->settings.lora.low_datarate_optimize);

    sx1272_write(radio, REG_LR_MODEMCONFIG2, ( sx1272_read(radio, REG_LR_MODEMCONFIG2 ) &
            RFLR_MODEMCONFIG2_SF_MASK ) | ( datarate << 4 ) );


    sx1272_write(radio, REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
    sx1272_write(radio, REG_LR_PREAMBLELSB, preambleLen & 0xFF );

    if( datarate == 6 ) {
      sx1272_write(radio, REG_LR_DETECTOPTIMIZE,
          (sx1272_read(radio, REG_LR_DETECTOPTIMIZE ) &
              RFLR_DETECTIONOPTIMIZE_MASK ) |
              RFLR_DETECTIONOPTIMIZE_SF6 );
      sx1272_write(radio, REG_LR_DETECTIONTHRESHOLD,
          RFLR_DETECTIONTHRESH_SF6 );
    } else {
      sx1272_write(radio, REG_LR_DETECTOPTIMIZE,
          (sx1272_read(radio, REG_LR_DETECTOPTIMIZE ) &
              RFLR_DETECTIONOPTIMIZE_MASK ) |
              RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
      sx1272_write(radio, REG_LR_DETECTIONTHRESHOLD,
          RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
    }
  }
  break;
  }
}

bool sx1272_check_rf_frequency(radioif_t *radio, uint32_t frequency) {
  return true;
}

uint32_t sx1272_set_time_on_air(radioif_t *radio, radio_modems_t modem, uint8_t pktLen) {
  sx1272_t *sx = (sx1272_t *)radio;
  uint32_t airTime = 0;

  switch( modem ) {
  case MODEM_FSK: {
    airTime = round( ( 8 * ( sx->settings.fsk.preamble_len +
        ( ( sx1272_read(radio, REG_SYNCCONFIG ) & ~RF_SYNCCONFIG_SYNCSIZE_MASK ) + 1 ) +
        ( ( sx->settings.fsk.fix_len == 0x01 ) ? 0.0 : 1.0 ) +
        ( ( ( sx1272_read(radio, REG_PACKETCONFIG1 ) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK ) != 0x00 ) ? 1.0 : 0 ) +
        pktLen +
        ( ( sx->settings.fsk.crc_on == 0x01 ) ? 2.0 : 0 ) ) /
        sx->settings.fsk.datarate ) * 1e3 );
    }
    break;
  case MODEM_LORA: {
    double bw = 0.0;
    switch( sx->settings.lora.bandwidth ) {
    case 0: // 125 kHz
      bw = 125e3;
      break;
    case 1: // 250 kHz
      bw = 250e3;
      break;
    case 2: // 500 kHz
      bw = 500e3;
      break;
    }

    // Symbol rate : time for one symbol (secs)
    double rs = bw / ( 1 << sx->settings.lora.datarate );
    double ts = 1 / rs;
    // time of preamble
    double tPreamble = ( sx->settings.lora.preamble_len + 4.25 ) * ts;
    // Symbol length of payload and time
    double tmp = ceil( ( 8 * pktLen - 4 * sx->settings.lora.datarate +
        28 + 16 * sx->settings.lora.crc_on -
        ( sx->settings.lora.fix_len ? 20 : 0 ) ) /
        ( double )( 4 * sx->settings.lora.datarate -
            ( ( sx->settings.lora.low_datarate_optimize > 0 ) ? 2 : 0 ) ) ) *
                ( sx->settings.lora.coderate + 4 );
    double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
    double tPayload = nPayload * ts;
    // Time on air
    double tOnAir = tPreamble + tPayload;
    // return us secs
    airTime = floor( tOnAir * 1e3 + 0.999 );
  }
  break;
  }
  return airTime;
}

void sx1272_send(radioif_t *radio, uint8_t *buffer, uint8_t size) {
  sx1272_t *sx = (sx1272_t *)radio;
  uint32_t txTimeout = 0;

  switch( sx->settings.modem ) {
  case MODEM_FSK: {
    sx->settings.fsk_packet_handler.nb_bytes = 0;
    sx->settings.fsk_packet_handler.size = size;

    if( sx->settings.fsk.fix_len == false ) {
      SX1272WriteFifo(sx, ( uint8_t* )&size, 1 );
    } else {
      sx1272_write(radio, REG_PAYLOADLENGTH, size );
    }

    if( ( size > 0 ) && ( size <= 64 ) ) {
      sx->settings.fsk_packet_handler.chunk_size = size;
    } else {
      memcpy( rx_tx_buffer, buffer, size );
      sx->settings.fsk_packet_handler.chunk_size = 32;
    }

    // Write payload buffer
    SX1272WriteFifo(sx, buffer, sx->settings.fsk_packet_handler.chunk_size );
    sx->settings.fsk_packet_handler.nb_bytes += sx->settings.fsk_packet_handler.chunk_size;
    txTimeout = sx->settings.fsk.tx_timeout;
  }
    break;
  case MODEM_LORA: {
    if( sx->settings.lora.iq_inverted == true ) {
      sx1272_write(radio, REG_LR_INVERTIQ, ( ( sx1272_read(radio, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
      sx1272_write(radio, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    } else {
      sx1272_write(radio, REG_LR_INVERTIQ, ( ( sx1272_read(radio, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
      sx1272_write(radio, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
    }

    sx->settings.lora_packet_handler.size = size;

    // Initializes the payload size
    sx1272_write(radio, REG_LR_PAYLOADLENGTH, size );

    // Full buffer used for Tx
    sx1272_write(radio, REG_LR_FIFOTXBASEADDR, 0 );
    sx1272_write(radio, REG_LR_FIFOADDRPTR, 0 );

    // FIFO operations can not take place in Sleep mode
    if( ( sx1272_read(radio, REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP ) {
      sx1272_set_stby(radio);
      mdelay(1);
    }
    // Write payload buffer
    SX1272WriteFifo(sx, buffer, size );
    txTimeout = sx->settings.lora.tx_timeout;
  }
  break;
  }

  SX1272SetTx(sx, txTimeout );
}

void sx1272_set_sleep(radioif_t *radio) {
  sx1272_t *sx = (sx1272_t *)radio;
  timeout_stop(&sx->tx_timeout);
  timeout_stop(&sx->rx_timeout);
  SX1272SetOpMode(sx, RF_OPMODE_SLEEP);
  sx->settings.state = RF_IDLE;
}

void sx1272_set_stby(radioif_t *radio) {
  sx1272_t *sx = (sx1272_t *)radio;
  timeout_start(&sx->rx_timeout);
  timeout_start(&sx->tx_timeout);

  SX1272SetOpMode(sx, RF_OPMODE_STANDBY );
  sx->settings.state = RF_IDLE;
}

void sx1272_set_rx(radioif_t *radio, uint32_t timeout) {
  sx1272_t *sx = (sx1272_t *)radio;
  bool rxContinuous = false;

  switch( sx->settings.modem ) {
  case MODEM_FSK:
  {
    rxContinuous = sx->settings.fsk.rx_continuous;

    // DIO0=PayloadReady
    // DIO1=FifoLevel
    // DIO2=SyncAddr
    // DIO3=FifoEmpty
    // DIO4=Preamble
    // DIO5=ModeReady
    sx1272_write(radio, REG_DIOMAPPING1, ( sx1272_read(radio, REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
        RF_DIOMAPPING1_DIO1_MASK &
        RF_DIOMAPPING1_DIO2_MASK ) |
        RF_DIOMAPPING1_DIO0_00 |
        RF_DIOMAPPING1_DIO1_00 |
        RF_DIOMAPPING1_DIO2_11 );

    sx1272_write(radio, REG_DIOMAPPING2, ( sx1272_read(radio, REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
        RF_DIOMAPPING2_MAP_MASK ) |
        RF_DIOMAPPING2_DIO4_11 |
        RF_DIOMAPPING2_MAP_PREAMBLEDETECT );

    sx->settings.fsk_packet_handler.fifo_thresh = sx1272_read(radio, REG_FIFOTHRESH ) & 0x3F;

    sx1272_write(radio, REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );

    sx->settings.fsk_packet_handler.preamble_detected = false;
    sx->settings.fsk_packet_handler.sync_word_detected = false;
    sx->settings.fsk_packet_handler.nb_bytes = 0;
    sx->settings.fsk_packet_handler.size = 0;
  }
    break;
  case MODEM_LORA:
  {
    if( sx->settings.lora.iq_inverted == true ) {
      sx1272_write(radio, REG_LR_INVERTIQ, ( ( sx1272_read(radio, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
      sx1272_write(radio, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    } else {
      sx1272_write(radio, REG_LR_INVERTIQ, ( ( sx1272_read(radio, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
      sx1272_write(radio, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
    }

    rxContinuous = sx->settings.lora.rx_continuous;

    if( sx->settings.lora.freq_hop_on == true ) {
      sx1272_write(radio, REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
          //RFLR_IRQFLAGS_RXDONE |
          //RFLR_IRQFLAGS_PAYLOADCRCERROR |
          RFLR_IRQFLAGS_VALIDHEADER |
          RFLR_IRQFLAGS_TXDONE |
          RFLR_IRQFLAGS_CADDONE |
          //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
          RFLR_IRQFLAGS_CADDETECTED );

      // DIO0=RxDone, DIO2=FhssChangeChannel
      sx1272_write(radio, REG_DIOMAPPING1, ( sx1272_read(radio, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
    } else {
      sx1272_write(radio, REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
          //RFLR_IRQFLAGS_RXDONE |
          //RFLR_IRQFLAGS_PAYLOADCRCERROR |
          RFLR_IRQFLAGS_VALIDHEADER |
          RFLR_IRQFLAGS_TXDONE |
          RFLR_IRQFLAGS_CADDONE |
          RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
          RFLR_IRQFLAGS_CADDETECTED );

      // DIO0=RxDone
      sx1272_write(radio, REG_DIOMAPPING1, ( sx1272_read(radio, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
    }
    sx1272_write(radio, REG_LR_FIFORXBASEADDR, 0 );
    sx1272_write(radio, REG_LR_FIFOADDRPTR, 0 );
  }
  break;
  }

  memset( rx_tx_buffer, 0, ( size_t )RX_BUFFER_SIZE );

  sx->settings.state = RF_RX_RUNNING;
  if( timeout != 0 ) {
//    TimerSetValue( &RxTimeoutTimer, timeout );
//    TimerStart( &RxTimeoutTimer );
    timeout_set_value(&sx->rx_timeout, timeout);
    timeout_start(&sx->rx_timeout);
  }

  if( sx->settings.modem == MODEM_FSK ) {
    SX1272SetOpMode(sx, RF_OPMODE_RECEIVER );

    if( rxContinuous == false ) {
//      TimerSetValue( &RxTimeoutSyncWord, ceil( ( 8.0 * ( sx->settings.fsk.preamble_len +
//          ( ( sx1272_read(radio, REG_SYNCCONFIG ) &
//              ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
//              1.0 ) + 10.0 ) /
//          ( double )sx->settings.fsk.datarate ) * 1e3 ) + 4 );
      timeout_set_value(&sx->rx_timeout_sync_word, ceil( ( 8.0 * ( sx->settings.fsk.preamble_len +
          ( ( sx1272_read(radio, REG_SYNCCONFIG ) &
              ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
              1.0 ) + 10.0 ) /
          ( double )sx->settings.fsk.datarate ) * 1e3 ) + 4 );
//      TimerStart( &RxTimeoutSyncWord );
      timeout_start(&sx->rx_timeout_sync_word);
    }
  } else {
    if( rxContinuous == true ) {
      SX1272SetOpMode(sx, RFLR_OPMODE_RECEIVER );
    } else {
      SX1272SetOpMode(sx, RFLR_OPMODE_RECEIVER_SINGLE );
    }
  }
}

void sx1272_start_cad(radioif_t *radio) {
  sx1272_t *sx = (sx1272_t *)radio;
  switch( sx->settings.modem ) {
  case MODEM_FSK:
  {

  }
  break;
  case MODEM_LORA:
  {
    sx1272_write(radio, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
        RFLR_IRQFLAGS_RXDONE |
        RFLR_IRQFLAGS_PAYLOADCRCERROR |
        RFLR_IRQFLAGS_VALIDHEADER |
        RFLR_IRQFLAGS_TXDONE |
        //RFLR_IRQFLAGS_CADDONE |
        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
        //RFLR_IRQFLAGS_CADDETECTED
    );

    // DIO3=CADDone
    sx1272_write(radio, REG_DIOMAPPING1, ( sx1272_read(radio, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );

    sx->settings.state = RF_CAD;
    SX1272SetOpMode(sx, RFLR_OPMODE_CAD );
  }
    break;
  default:
    break;
  }
}

int16_t sx1272_read_rssi(radioif_t *radio, radio_modems_t modem) {
  int16_t rssi = 0;
  switch( modem ) {
  case MODEM_FSK:
    rssi = -( sx1272_read(radio, REG_RSSIVALUE ) >> 1 );
    break;
  case MODEM_LORA:
    rssi = RSSI_OFFSET + sx1272_read(radio, REG_LR_RSSIVALUE );
    break;
  default:
    rssi = -1;
    break;
  }
  return rssi;
}

void sx1272_write(radioif_t *radio, uint8_t addr, uint8_t data) {
  return sx1272_write_buffer(radio, addr, &data, 1);
}

uint8_t sx1272_read(radioif_t *radio, uint8_t addr) {
  uint8_t data;
  sx1272_read_buffer(radio, addr, &data, 1);
  return data;
}

void sx1272_write_buffer(radioif_t *radio, uint8_t addr, uint8_t *buffer, uint8_t size) {
  sx1272_t *sx = (sx1272_t *)radio;
  sx->msg.tx_data = &addr;
  sx->msg.rx_data = 0;
  sx->msg.word_count = 1;

  if (spim_trx(sx->spi, &sx->msg) != SPI_ERR_OK)
    return;

  sx->msg.tx_data = buffer;
  sx->msg.rx_data = 0;
  sx->msg.word_count = size;

  if (spim_trx(sx->spi, &sx->msg) != SPI_ERR_OK)
    return ;

  return ;
}

void sx1272_read_buffer(radioif_t *radio, uint8_t addr, uint8_t *buffer, uint8_t size) {
  sx1272_t *sx = (sx1272_t *)radio;
  sx->msg.tx_data = &addr;
  sx->msg.rx_data = 0;
  sx->msg.word_count = 1;

  if (spim_trx(sx->spi, &sx->msg) != SPI_ERR_OK)
    return;

  sx->msg.tx_data = 0;
  sx->msg.rx_data = (uint8_t*)buffer;
  sx->msg.word_count = size;

  if (spim_trx(sx->spi, &sx->msg) != SPI_ERR_OK)
    return;
  return;
}

void sx1272_set_max_payload_length(radioif_t *radio, radio_modems_t modem, uint8_t max) {
  sx1272_t *sx = (sx1272_t *)radio;
  sx1272_set_modem(radio,  modem );

  switch( modem ) {
  case MODEM_FSK:
    if( sx->settings.fsk.fix_len == false ) {
      sx1272_write(radio, REG_PAYLOADLENGTH, max );
    }
    break;
  case MODEM_LORA:
    sx1272_write(radio, REG_LR_PAYLOADMAXLENGTH, max );
    break;
  }
}

void timeout_hendler(void *arg) ;

void lora_init(sx1272_t *sx, spimif_t *s, timer_t *timer, unsigned freq, unsigned mode) {
  sx->spi = s;

  radioif_t *r = &sx->radioif;
  sx->events = NULL;
  r->init = sx1272_init;

  //ToDo
  r->get_status = sx1272_get_status;
  r->set_modem = sx1272_set_modem;
  r->set_channel = sx1272_set_channel;
  r->is_channel_free = sx1272_is_channel_free;
  r->random = sx1272_random;
  r->set_rx_config = sx1272_set_rx_config;
  r->set_tx_config = sx1272_set_tx_config;
  r->check_rf_frequency = sx1272_check_rf_frequency;
  r->time_on_air = sx1272_set_time_on_air;
  r->send = sx1272_send;
  r->sleep = sx1272_set_sleep;
  r->standby = sx1272_set_stby;
  r->rx = sx1272_set_rx;
  r->start_cad = sx1272_start_cad;
  r->rssi = sx1272_read_rssi;
  r->write = sx1272_write;
  r->read = sx1272_read;
  r->write_buffer = sx1272_write_buffer;
  r->read_buffer = sx1272_read_buffer;
  r->set_max_payload_length = sx1272_set_max_payload_length;



  sx->timer = timer;

  sx->msg.freq = freq;
  sx->msg.mode = (mode & 0xFF0F) | SPI_MODE_NB_BITS(8);

  stm32l1_gpio_init(&sx->dio0, GPIO_PORT_B, RADIO_DIO_0, GPIO_FLAGS_INPUT | GPIO_FLAGS_PULL_UP);
  stm32l1_gpio_init(&sx->dio1, GPIO_PORT_B, RADIO_DIO_1, GPIO_FLAGS_INPUT | GPIO_FLAGS_PULL_UP);
  stm32l1_gpio_init(&sx->dio2, GPIO_PORT_B, RADIO_DIO_2, GPIO_FLAGS_INPUT | GPIO_FLAGS_PULL_UP);
  stm32l1_gpio_init(&sx->dio3, GPIO_PORT_B, RADIO_DIO_3, GPIO_FLAGS_INPUT | GPIO_FLAGS_PULL_UP);
  stm32l1_gpio_init(&sx->dio4, GPIO_PORT_B, RADIO_DIO_4, GPIO_FLAGS_INPUT | GPIO_FLAGS_PULL_UP);
  stm32l1_gpio_init(&sx->dio5, GPIO_PORT_B, RADIO_DIO_5, GPIO_FLAGS_INPUT | GPIO_FLAGS_PULL_UP);

  stm32l1_gpio_init(&sx->ant_tx, GPIO_PORT_A, RADIO_ANT_SWITCH_TX, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_PULL_UP | GPIO_FLAGS_50MHZ);
  stm32l1_gpio_init(&sx->ant_rx, GPIO_PORT_C, RADIO_ANT_SWITCH_RX, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_PULL_UP | GPIO_FLAGS_50MHZ);

  sx->radio_is_active = false;

  // Initialize driver timeout timers
  timeout_init(&sx->tx_timeout, sx->timer, &sx->mutex_timeout);
  timeout_init(&sx->rx_timeout, sx->timer, &sx->mutex_timeout);
  timeout_init(&sx->rx_timeout_sync_word, sx->timer, &sx->mutex_timeout);

  debug_printf("Try create task\r\n");
  task_create(timeout_hendler, sx, "timeout", 20, sx->stack_timeout, sizeof(sx->stack_timeout));
  debug_printf("Task created\r\n");


}


// Radio interface functions


// сделать задачу для обработки таймаутов
void timeout_hendler(void *arg) {
  sx1272_t *sx = (sx1272_t *)arg;
  radioif_t * radio = &sx->radioif;
//  timer_delay(sx->timer, 100);
  timeout_set_signal(&sx->tx_timeout, (void *)11);
  timeout_set_signal(&sx->rx_timeout, (void *)11);
  timeout_set_signal(&sx->rx_timeout_sync_word, (void *)11);
  debug_printf("Start task timeout hendler\r\n");
  while(1) {
    mutex_lock(&sx->mutex_timeout);
    mutex_wait(&sx->mutex_timeout);
    switch (sx->settings.state) {
    case RF_RX_RUNNING:
      if (sx->settings.modem == MODEM_FSK) {
        sx->settings.fsk_packet_handler.preamble_detected = false;
        sx->settings.fsk_packet_handler.sync_word_detected = false;
        sx->settings.fsk_packet_handler.nb_bytes = 0;
        sx->settings.fsk_packet_handler.size = 0;

        // Clear Irqs
        sx1272_write(radio, REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
            RF_IRQFLAGS1_PREAMBLEDETECT |
            RF_IRQFLAGS1_SYNCADDRESSMATCH);
        sx1272_write(radio, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

        if (sx->settings.fsk.rx_continuous == true) {
          // Continuous mode restart Rx chain
          sx1272_write(radio, REG_RXCONFIG, sx1272_read(radio,  REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
          timeout_start(&sx->rx_timeout_sync_word);
        } else {
          sx->settings.state = RF_IDLE;
          timeout_stop(&sx->rx_timeout_sync_word);
        }
      }
      if ((sx->events != NULL) && (sx->events->rx_timeout != NULL)) {
        sx->events->rx_timeout(radio);
      }
      break;
    case RF_TX_RUNNING:
      sx->settings.state = RF_IDLE;
      if ((sx->events != NULL) && (sx->events->tx_timeout != NULL)) {
        sx->events->tx_timeout(radio);
      }
      break;
    default:
      break;
    }
    mutex_unlock(&sx->mutex_timeout);
  }
}

