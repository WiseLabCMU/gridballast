/**
 * @file lora_module.h
 *
 * @brief Defines the LoRa Module API
 *
 * @author Sharan Turlapati
 */

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQMASK              0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_RX_TIMEOUT_LSB       0x1f
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_LR_HOP_PERIOD        0x24
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_2F                   0x2f
#define REG_30                   0x30
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_36                   0x36
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_3A                   0x3a
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_DIO_MAPPING_2        0X41
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d


//REG_OP_MODE
#define RF_OPMODE_SLEEP         0xF8
#define RF_OPMODE_RANGE         0X7F

// REG_PA_CONFIG
#define RF_PACONFIG_PASELECT_MASK                   0x7F
#define RF_PACONFIG_PASELECT_PABOOST                0x80
#define RF_PACONFIG_PASELECT_RFO                    0x00 // Default
#define RF_PACONFIG_MAX_POWER_MASK                  0x8F
#define RF_PACONFIG_OUTPUTPOWER_MASK                0xF0

//REG_PA_DAC
#define RF_PADAC_20DBM_MASK                         0xF8
#define RF_PADAC_20DBM_ON                           0x07
#define RF_PADAC_20DBM_OFF                          0x04  // Default

/*
 * RegModemConfig1
 */
#define RFLR_MODEMCONFIG1_BW_MASK                   0x0F 
#define RFLR_MODEMCONFIG1_BW_7_81_KHZ               0x00 
#define RFLR_MODEMCONFIG1_BW_10_41_KHZ              0x10 
#define RFLR_MODEMCONFIG1_BW_15_62_KHZ              0x20 
#define RFLR_MODEMCONFIG1_BW_20_83_KHZ              0x30 
#define RFLR_MODEMCONFIG1_BW_31_25_KHZ              0x40 
#define RFLR_MODEMCONFIG1_BW_41_66_KHZ              0x50 
#define RFLR_MODEMCONFIG1_BW_62_50_KHZ              0x60 
#define RFLR_MODEMCONFIG1_BW_125_KHZ                0x70 // Default
#define RFLR_MODEMCONFIG1_BW_250_KHZ                0x80 
#define RFLR_MODEMCONFIG1_BW_500_KHZ                0x90 
                                                    
#define RFLR_MODEMCONFIG1_CODINGRATE_MASK           0xF1 
#define RFLR_MODEMCONFIG1_CODINGRATE_4_5            0x02
#define RFLR_MODEMCONFIG1_CODINGRATE_4_6            0x04 // Default
#define RFLR_MODEMCONFIG1_CODINGRATE_4_7            0x06 
#define RFLR_MODEMCONFIG1_CODINGRATE_4_8            0x08 
                                                    
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK       0xFE 
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_ON         0x01 
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF        0x00 // Default

/*!
 * RegModemConfig2
 */
#define RFLR_MODEMCONFIG2_SF_MASK                   0x0F 
#define RFLR_MODEMCONFIG2_SF_6                      0x60 
#define RFLR_MODEMCONFIG2_SF_7                      0x70 // Default
#define RFLR_MODEMCONFIG2_SF_8                      0x80 
#define RFLR_MODEMCONFIG2_SF_9                      0x90 
#define RFLR_MODEMCONFIG2_SF_10                     0xA0 
#define RFLR_MODEMCONFIG2_SF_11                     0xB0 
#define RFLR_MODEMCONFIG2_SF_12                     0xC0 

#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK     0xF7 
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_ON       0x08 
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF      0x00 

#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK         0xFB 
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON           0x04 
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF          0x00 // Default
 
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK       0xFC 
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB            0x00 // Default

/*!
 * RegSymbTimeoutLsb
 */
#define RFLR_SYMBTIMEOUTLSB_SYMBTIMEOUT             0x64 // Default

/*!
 * RegPreambleLengthMsb
 */
#define RFLR_PREAMBLELENGTHMSB                      0x00 // Default

/*!
 * RegPreambleLengthLsb
 */
#define RFLR_PREAMBLELENGTHLSB                      0x08 // Default

/*!
 * RegPayloadLength
 */
#define RFLR_PAYLOADLENGTH                          0x0E // Default

/*!
 * RegPayloadMaxLength
 */
#define RFLR_PAYLOADMAXLENGTH                       0xFF // Default

/*!
 * RegHopPeriod
 */
#define RFLR_HOPPERIOD_FREQFOPPINGPERIOD            0x00 // Default

/*!
 * RegFifoRxByteAddr (Read Only)
 */

/*!
 * RegModemConfig3
 */
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK  0xF7 
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON    0x08 
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_OFF   0x00 // Default

#define RFLR_MODEMCONFIG3_AGCAUTO_MASK              0xFB 
#define RFLR_MODEMCONFIG3_AGCAUTO_ON                0x04 // Default 
#define RFLR_MODEMCONFIG3_AGCAUTO_OFF               0x00 

//RegPllHop
#define RFLR_PLLHOP_FASTHOP_MASK                    0x7F
#define RFLR_PLLHOP_FASTHOP_ON                      0x80
#define RFLR_PLLHOP_FASTHOP_OFF                     0x00 // Default

/*!
 * RegDetectOptimize
 */
#define RFLR_DETECTIONOPTIMIZE_MASK                 0xF8
#define RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12          0x03 // Default
#define RFLR_DETECTIONOPTIMIZE_SF6                  0x05

/*!
 * RegDetectionThreshold
 */
#define RFLR_DETECTIONTHRESH_SF7_TO_SF12            0x0A // Default
#define RFLR_DETECTIONTHRESH_SF6                    0x0C

/*!
 * RegInvertIQ
 */
#define RFLR_INVERTIQ_RX_MASK                       0xBF
#define RFLR_INVERTIQ_RX_OFF                        0x00
#define RFLR_INVERTIQ_RX_ON                         0x40
#define RFLR_INVERTIQ_TX_MASK                       0xFE
#define RFLR_INVERTIQ_TX_OFF                        0x01
#define RFLR_INVERTIQ_TX_ON                         0x00

/*!
 * RegInvertIQ2
 */
#define RFLR_INVERTIQ2_ON                           0x19
#define RFLR_INVERTIQ2_OFF                          0x1D

/*!
 * RegIrqFlagsMask
 */
#define RFLR_IRQFLAGS_RXTIMEOUT_MASK                0x80 
#define RFLR_IRQFLAGS_RXDONE_MASK                   0x40 
#define RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK          0x20 
#define RFLR_IRQFLAGS_VALIDHEADER_MASK              0x10 
#define RFLR_IRQFLAGS_TXDONE_MASK                   0x08 
#define RFLR_IRQFLAGS_CADDONE_MASK                  0x04 
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK       0x02 
#define RFLR_IRQFLAGS_CADDETECTED_MASK              0x01 

/*!
 * RegIrqFlags
 */
#define RFLR_IRQFLAGS_RXTIMEOUT                     0x80 
#define RFLR_IRQFLAGS_RXDONE                        0x40 
#define RFLR_IRQFLAGS_PAYLOADCRCERROR               0x20 
#define RFLR_IRQFLAGS_VALIDHEADER                   0x10 
#define RFLR_IRQFLAGS_TXDONE                        0x08 
#define RFLR_IRQFLAGS_CADDONE                       0x04 
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL            0x02 
#define RFLR_IRQFLAGS_CADDETECTED                   0x01 

// Additional settings
#define REG_LR_PLLHOP                               0x44
#define REG_LR_TCXO                                 0x4B
#define REG_LR_PADAC                                0x4D
#define REG_LR_FORMERTEMP                           0x5B
#define REG_LR_BITRATEFRAC                          0x5D
#define REG_LR_AGCREF                               0x61
#define REG_LR_AGCTHRESH1                           0x62
#define REG_LR_AGCTHRESH2                           0x63
#define REG_LR_AGCTHRESH3                           0x64
#define REG_LR_PLL                                  0x70

/*!
 * RegDioMapping1
 */
#define RFLR_DIOMAPPING1_DIO0_MASK                  0x3F
#define RFLR_DIOMAPPING1_DIO0_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO0_01                    0x40
#define RFLR_DIOMAPPING1_DIO0_10                    0x80
#define RFLR_DIOMAPPING1_DIO0_11                    0xC0

#define RFLR_DIOMAPPING1_DIO1_MASK                  0xCF
#define RFLR_DIOMAPPING1_DIO1_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO1_01                    0x10
#define RFLR_DIOMAPPING1_DIO1_10                    0x20
#define RFLR_DIOMAPPING1_DIO1_11                    0x30

#define RFLR_DIOMAPPING1_DIO2_MASK                  0xF3
#define RFLR_DIOMAPPING1_DIO2_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO2_01                    0x04
#define RFLR_DIOMAPPING1_DIO2_10                    0x08
#define RFLR_DIOMAPPING1_DIO2_11                    0x0C

#define RFLR_DIOMAPPING1_DIO3_MASK                  0xFC
#define RFLR_DIOMAPPING1_DIO3_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO3_01                    0x01
#define RFLR_DIOMAPPING1_DIO3_10                    0x02
#define RFLR_DIOMAPPING1_DIO3_11                    0x03

/*!
 * RegDioMapping2
 */
#define RFLR_DIOMAPPING2_DIO4_MASK                  0x3F
#define RFLR_DIOMAPPING2_DIO4_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO4_01                    0x40
#define RFLR_DIOMAPPING2_DIO4_10                    0x80
#define RFLR_DIOMAPPING2_DIO4_11                    0xC0

#define RFLR_DIOMAPPING2_DIO5_MASK                  0xCF
#define RFLR_DIOMAPPING2_DIO5_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO5_01                    0x10
#define RFLR_DIOMAPPING2_DIO5_10                    0x20
#define RFLR_DIOMAPPING2_DIO5_11                    0x30

#define RFLR_DIOMAPPING2_MAP_MASK                   0xFE
#define RFLR_DIOMAPPING2_MAP_PREAMBLEDETECT         0x01
#define RFLR_DIOMAPPING2_MAP_RSSI                   0x00  // Default
// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

#define HOPE_RF_SLAVE_SELECT_PIN         21
#define HOPE_RF_RST_PIN                  9    //this pin is the number on the IO expander

#define BYTE_SIZE                        8

#define HOPERF_VERSION                   0x12
#define RF_FREQUENCY                     915000000
#define FREQ_STEP                        61.03515625
#define TX_OUTPUT_POWER                  14

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164.0
#define RSSI_OFFSET_HF                              -157.0

#define RF_MID_BAND_THRESH               525000000

//LORA TX SETTINGS
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_FHSS_ENABLED                           false  
#define LORA_NB_SYMB_HOP                            4     
#define LORA_IQ_INVERSION_ON                        false
#define LORA_CRC_ENABLED                            true

#define LORA_RX_BUFFER_SIZE                         256

#define MCP_DIO0_PIN_NUMBER                         0xA
#define MCP_DIO1_PIN_NUMBER                         0xB
                               
/*
 * Radio driver internal state machine states definition
 */
typedef enum
{
    RF_IDLE = 0,
    RF_RX_RUNNING,
    RF_TX_RUNNING,
    RF_CAD,
}radio_state_t;


/** @brief depth of the lora stack */
#define loraUSStackDepth ((unsigned short) 8192) /* bytes */
/** @brief priority of the lora stack */
#define loraUXPriority (1)

void lora_init();
void rfm95_dio0_irq(uint8_t pin_button, uint8_t val);
void spi_bus_configure();
void spi_device_configure();
void rfm95_send_data (uint8_t data);
void rfm95_receive_data(uint8_t *rx_data);
void rfm95_sleep();
void set_frequency(uint64_t frequency);
void rfm95_set_lora_opmode();
void rfm95_write_register(uint8_t reg, const uint8_t val);
void rfm95_read_register(uint8_t reg, uint8_t *data);
void rfm95_set_tx_power(int8_t power);
void rfm95_set_tx_config(uint32_t f_dev, uint32_t band_width, uint32_t data_rate,
                         uint8_t code_rate, uint16_t preamble_length, bool fix_length, 
                         bool crc_on, bool freq_hop_on, uint8_t hop_period, 
                         bool iq_inverted, uint32_t timeout);
void rfm95_set_rx_config(uint32_t band_width, uint32_t data_rate, uint8_t code_rate,
                          uint32_t bandwidth_afc, uint16_t preamble_length, uint16_t symb_timeout, bool fix_length,
                          uint8_t payload_length, bool crc_on, bool freq_hop_on, uint8_t hop_period,
                          bool iq_inverted, bool rx_continuous);
void rfm95_send(const uint8_t *buffer, size_t size);
void rfm95_receive();
void rfm95_init();
radio_state_t rfm95_get_op_mode();