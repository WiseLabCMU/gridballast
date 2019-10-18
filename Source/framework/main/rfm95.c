#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "Lora_mac/radio/rfm95/rfm95.h"
#include <string.h>
#include "esp_err.h"
#include "freertos/task.h"
#include "Ada_MCP.h" 
#include "driver/gpio.h"
#include "LoraMac_Class_A.h"



spi_bus_config_t spi_bus_config;
spi_device_interface_config_t spi_device_config;
static spi_device_handle_t handle_spi;      // SPI handle.
uint8_t *rx_buffer;
volatile radio_state_t rfm95_current_radio_state = RF_IDLE;
volatile int lora = 0;
uint8_t irq_flags;
extern bool IrqFired;


void spi_bus_configure() {
    spi_bus_config.mosi_io_num = 23;
    spi_bus_config.miso_io_num = 19;
    spi_bus_config.sclk_io_num = 18;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 4096;
}

void spi_device_configure() {
    spi_device_config.address_bits     = 0;
    spi_device_config.command_bits     = 0;
    spi_device_config.dummy_bits       = 0;
    spi_device_config.mode             = 0;
    spi_device_config.duty_cycle_pos   = 0;
    spi_device_config.cs_ena_posttrans = 0;
    spi_device_config.cs_ena_pretrans  = 0;
    spi_device_config.clock_speed_hz   = 10000;
    spi_device_config.spics_io_num     = 21;
    spi_device_config.flags            = 0;
    spi_device_config.queue_size       = 200;
    spi_device_config.pre_cb           = NULL;
    spi_device_config.post_cb          = NULL;
}

/*
 */
void rfm95_send_data (uint8_t data) {
    esp_err_t ret;
    uint8_t tx_data = data;
    spi_transaction_t trans_desc;
    memset(&trans_desc, 0, sizeof(trans_desc)); 
    trans_desc.tx_buffer = (void *)(&tx_data);
    trans_desc.length = BYTE_SIZE * sizeof(uint8_t);
    trans_desc.user = (void *)0;
    ret = spi_device_transmit(handle_spi, &trans_desc);
    assert(ret == ESP_OK);
}

void rfm95_receive_data(uint8_t *rx_data) {
    esp_err_t ret;
    spi_transaction_t trans_desc;
    memset(&trans_desc, 0, sizeof(trans_desc)); 
    trans_desc.rx_buffer = rx_data;
    trans_desc.length = BYTE_SIZE * sizeof(uint8_t);;
    trans_desc.user = (void *)1;
    ret = spi_device_transmit(handle_spi, &trans_desc);
    assert( ret == ESP_OK );
}

void rfm95_read_register(uint8_t reg, uint8_t *data) {
    uint8_t read_register = reg & 0x7f;
    gpio_set_level(HOPE_RF_SLAVE_SELECT_PIN, 0);
    rfm95_send_data(read_register);
    rfm95_receive_data(data);
    gpio_set_level(HOPE_RF_SLAVE_SELECT_PIN, 1);
    //printf("reg %x data %x\n", reg, *data);
}

void rfm95_write_register(uint8_t reg, const uint8_t val) {
    uint8_t write_register = reg | 0x80;
    gpio_set_level(HOPE_RF_SLAVE_SELECT_PIN, 0);
    rfm95_send_data(write_register);
    rfm95_send_data(val);
    gpio_set_level(HOPE_RF_SLAVE_SELECT_PIN, 1);
}

/*
  Puts the Hope RF Lora module to sleep
  Input - none
  Return - none
 */
void rfm95_sleep() {
    //send the register address and data to put it to sleep
    uint8_t sleep;
    rfm95_read_register(REG_OP_MODE, &sleep);
    sleep = (sleep & RF_OPMODE_SLEEP) | MODE_SLEEP;
    rfm95_write_register(REG_OP_MODE, sleep);
}

void rfm95_idle() {
    uint8_t standby;
    rfm95_read_register(REG_OP_MODE, &standby);
    standby = (standby & RF_OPMODE_SLEEP) | MODE_STDBY;
    rfm95_write_register(REG_OP_MODE, standby);
}

void rfm95_tx() {
    uint8_t tx;
    rfm95_read_register(REG_OP_MODE, &tx);
    tx = (tx & RF_OPMODE_SLEEP) | MODE_TX;
    rfm95_write_register(REG_OP_MODE, tx);
}

void rfm95_rx() {
    uint8_t rx;
    rfm95_read_register(REG_OP_MODE, &rx);
    rx = (rx & RF_OPMODE_SLEEP) | MODE_RX_CONTINUOUS;
    rfm95_write_register(REG_OP_MODE, rx);
}

uint8_t get_pa_select(uint64_t channel) {
    //NOTE - this may have to be revisited in case things dont work
    if (channel > RF_MID_BAND_THRESH) {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void set_frequency(uint64_t frequency) {
    uint64_t frf = ((double)frequency) / ((double)FREQ_STEP);
    rfm95_write_register(REG_FRF_MSB, (uint8_t)((frf >> 16) & 0xff));
    rfm95_write_register(REG_FRF_MID, (uint8_t)((frf >> 8) & 0xff));
    rfm95_write_register(REG_FRF_LSB, (uint8_t)(frf & 0xff));

}

void rfm95_set_tx_power(int8_t power) {
    uint8_t pa_config = 0;
    uint8_t pa_dac = 0;
    rfm95_read_register(REG_PA_CONFIG, &pa_config);
    rfm95_read_register(REG_PA_DAC, &pa_dac);
    pa_config = ( pa_config & RF_PACONFIG_PASELECT_MASK ) | get_pa_select(RF_FREQUENCY);
    pa_config = ( pa_config & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if ((pa_config & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST ) {
        if( power > 17 ) {
            pa_dac = ( pa_dac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else {
            pa_dac = ( pa_dac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( pa_dac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON ) {
            if( power < 5 ) {
                power = 5;
            }
            if( power > 20 ) {
                power = 20; 
            }
            pa_config = ( pa_config & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else {
            if( power < 2 ) {
                power = 2;
            }
            if( power > 17 ) {
                power = 17;
            }
            pa_config = ( pa_config & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else {
        if( power < -1 ) {
            power = -1;
        }
        if( power > 14 ) {
            power = 14;
        }
        pa_config = ( pa_config & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    rfm95_write_register( REG_PA_CONFIG, pa_config );
    rfm95_write_register( REG_PA_DAC, pa_dac );

}

bool is_transmitting() {
    uint8_t rx_data;
    rfm95_read_register(REG_IRQ_FLAGS, &rx_data);
    if (rx_data & IRQ_TX_DONE_MASK) {
        return true;
    }
    rfm95_read_register(REG_IRQ_FLAGS, &rx_data);
    if (rx_data & IRQ_TX_DONE_MASK) {
        // clear IRQs
        rfm95_write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }
    return false;
}

void begin_packet() {
    uint8_t rx_data = 0;
    if (is_transmitting()) {
        //allow other tasks to run here 
        return;
    }
    rfm95_idle();
    //we'll set it to the explicit header mode
    rfm95_read_register(REG_MODEM_CONFIG_1, &rx_data);
    rx_data &= 0xfe;
    rfm95_write_register(REG_MODEM_CONFIG_1, rx_data);

    // reset FIFO address and payload length
    rfm95_write_register(REG_FIFO_ADDR_PTR, 0);
    rfm95_write_register(REG_PAYLOAD_LENGTH, 0);

}

void end_packet() {
    uint8_t rx_data = 0;
    // put in TX mode
    rfm95_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    //not sure if async is needed
    vTaskDelay(50 / portTICK_PERIOD_MS);
    while (1) {
        rfm95_read_register(REG_IRQ_FLAGS, &rx_data);
        if (rx_data & IRQ_TX_DONE_MASK) {
            break;
        }
        else {
            vTaskDelay(500/portTICK_PERIOD_MS);
            rx_data = 0;
        }
    }
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

void rfm95_send(const uint8_t *buffer, size_t size) {
    uint8_t current_length;
    size_t i;
    rfm95_sleep();
    if (LORA_IQ_INVERSION_ON) {
        uint8_t invert_iq;
        rfm95_read_register(REG_INVERTIQ, &invert_iq);
        invert_iq = (invert_iq & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON;
        rfm95_write_register(REG_INVERTIQ, invert_iq);
        rfm95_write_register(REG_INVERTIQ2, RFLR_INVERTIQ2_ON);
    }
    else {
        uint8_t invert_iq;
        rfm95_read_register(REG_INVERTIQ, &invert_iq);
        invert_iq = (invert_iq & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF;
        rfm95_write_register(REG_INVERTIQ, invert_iq);
        rfm95_write_register(REG_INVERTIQ2, RFLR_INVERTIQ2_OFF);
    }
    rfm95_read_register(REG_PAYLOAD_LENGTH, &current_length);
    if (current_length + size > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - current_length;
    }
    if (LORA_FHSS_ENABLED) {
        uint8_t irq_mask;
        rfm95_read_register(REG_IRQMASK, &irq_mask);
        irq_mask &= ~(RFLR_IRQFLAGS_TXDONE);
        rfm95_write_register(REG_IRQMASK, irq_mask);
        rfm95_write_register(REG_IRQMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_CADDETECTED);
        uint8_t dio_mapping1;
        rfm95_read_register(REG_DIO_MAPPING_1, &dio_mapping1);
        dio_mapping1 = (dio_mapping1 & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00;
        rfm95_write_register(REG_DIO_MAPPING_1, dio_mapping1);                                       
    }
    else {
        uint8_t irq_mask;
        rfm95_read_register(REG_IRQMASK, &irq_mask);
        irq_mask &= ~(RFLR_IRQFLAGS_TXDONE);
        rfm95_write_register(REG_IRQMASK, irq_mask);
        rfm95_write_register(REG_IRQMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                          RFLR_IRQFLAGS_RXDONE |
                                          RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                          RFLR_IRQFLAGS_VALIDHEADER |
                                          RFLR_IRQFLAGS_CADDONE |
                                          RFLR_IRQFLAGS_CADDETECTED |
                                          RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);
        uint8_t dio_mapping1;
        rfm95_read_register(REG_DIO_MAPPING_1, &dio_mapping1);
        dio_mapping1 = (dio_mapping1 & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01;
        rfm95_write_register(REG_DIO_MAPPING_1, dio_mapping1); 

    }
    rfm95_current_radio_state = RF_TX_RUNNING;
    // update length
    rfm95_write_register(REG_PAYLOAD_LENGTH,  size);
    rfm95_write_register(REG_FIFO_TX_BASE_ADDR, 0);
    rfm95_write_register(REG_FIFO_ADDR_PTR, 0);
    rfm95_idle();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    //start filling the fifo byte by byte
    for (i = 0; i < size; i++) {
        rfm95_write_register(REG_FIFO, buffer[i]);
    }
    rfm95_tx();
}

void rfm95_dio0_irq(uint8_t pin, uint8_t val) {
    uint8_t snr_u = 0, rssi_u = 0, size_rx = 0, i = 0;
    int8_t snr = 0, rssi = 0;
    //printf("IRQ handler\n");
    switch (rfm95_current_radio_state) {
        case RF_RX_RUNNING:
            //rx done
            //clear signal on expander
            IrqFired = true;
            if (val != 1) {
                assert(0);
            }
            //clear irq on rfm chip
            rfm95_read_register(REG_IRQ_FLAGS, &irq_flags);
            rfm95_write_register( REG_IRQ_FLAGS, RFLR_IRQFLAGS_RXDONE );
            if((irq_flags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR ) {
                rfm95_write_register(REG_IRQ_FLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);
                rfm95_current_radio_state = RF_IDLE;
                //TODO - detach the timer implemented
                //other error handling to be done here
            }
            rfm95_read_register(REG_PKT_SNR_VALUE, &snr_u);
            snr = (int8_t)snr_u;
            if (snr & 0x80) {
                // Invert and divide by 4
                snr = ((~snr + 1) & 0xff) >> 2;
                snr = -snr;
            }
            else {
                snr = (snr & 0xff) >> 2;
            }
            rfm95_read_register(REG_PKT_RSSI_VALUE, &rssi_u);
            rssi = (int8_t)rssi_u;
            if (snr < 0) {
                if (RF_FREQUENCY > RF_MID_BAND_THRESH) {
                    rssi = RSSI_OFFSET_HF + rssi + (rssi >> 4) + snr;
                }
                else {
                    rssi = RSSI_OFFSET_LF + rssi + (rssi >> 4) + snr;
                }
            }
            else {
                if (RF_FREQUENCY > RF_MID_BAND_THRESH) {
                    rssi = RSSI_OFFSET_HF + rssi + (rssi >> 4);
                }
                else {
                    rssi = RSSI_OFFSET_LF + rssi + (rssi >> 4);
                }
            }
            rfm95_read_register(REG_RX_NB_BYTES, &size_rx);
            printf("size %x\n", size_rx);
            //read fifo
            for (i = 0; i < 32; i++) {
                rfm95_read_register(REG_FIFO, &rx_buffer[i]);
                printf("%c\n", rx_buffer[i]);
            }
            //printf("\n");
            memset(rx_buffer, 0 , LORA_RX_BUFFER_SIZE);
            //TODO - more clean up???
            rfm95_current_radio_state = RF_IDLE;
            rfm95_idle();
            break;
        case RF_TX_RUNNING:
            //printf("TX interrupt fired\n");
            IrqFired = true;
            rfm95_write_register( REG_IRQ_FLAGS, RFLR_IRQFLAGS_TXDONE );
            rfm95_current_radio_state = RF_IDLE;
            rfm95_idle();
            break;
        default:
            rfm95_idle();
            break;
    }
}

void rfm95_receive() {
    if (LORA_IQ_INVERSION_ON) {
        uint8_t invert_iq;
        rfm95_read_register(REG_INVERTIQ, &invert_iq);
        invert_iq = (invert_iq & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF;
        rfm95_write_register(REG_INVERTIQ, invert_iq);
        rfm95_write_register(REG_INVERTIQ2, RFLR_INVERTIQ2_ON);
    }
    else {
        uint8_t invert_iq;
        rfm95_read_register(REG_INVERTIQ, &invert_iq);
        invert_iq = (invert_iq & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF;
        rfm95_write_register(REG_INVERTIQ, invert_iq);
        rfm95_write_register(REG_INVERTIQ2, RFLR_INVERTIQ2_OFF);
    }
    printf("sha-1\n");
    // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
    if ((LORA_BANDWIDTH + 7) < 9) {
        uint8_t optimize_detect;
        rfm95_read_register(REG_DETECTION_OPTIMIZE, &optimize_detect);
        optimize_detect &= 0x7f;
        rfm95_write_register(REG_DETECTION_OPTIMIZE, optimize_detect);
        rfm95_write_register(REG_30, 0x00);
        switch((LORA_BANDWIDTH + 7)) {
            case 0: // 7.8 kHz
                rfm95_write_register( REG_2F, 0x48 );
                set_frequency(RF_FREQUENCY + 7.81e3 );
                break;
            case 1: // 10.4 kHz
                rfm95_write_register( REG_2F, 0x44 );
                set_frequency(RF_FREQUENCY + 10.42e3 );
                break;
            case 2: // 15.6 kHz
                rfm95_write_register( REG_2F, 0x44 );
                set_frequency(RF_FREQUENCY + 15.62e3 );
                break;
            case 3: // 20.8 kHz
                rfm95_write_register( REG_2F, 0x44 );
                set_frequency(RF_FREQUENCY + 20.83e3 );
                break;
            case 4: // 31.2 kHz
                rfm95_write_register( REG_2F, 0x44 );
                set_frequency(RF_FREQUENCY + 31.25e3 );
                break;
            case 5: // 41.4 kHz
                rfm95_write_register( REG_2F, 0x44 );
                set_frequency(RF_FREQUENCY + 41.67e3 );
                break;
            case 6: // 62.5 kHz
                rfm95_write_register( REG_2F, 0x40 );
                break;
            case 7: // 125 kHz
                rfm95_write_register( REG_2F, 0x40 );
                break;
            case 8: // 250 kHz
                rfm95_write_register( REG_2F, 0x40 );
                break;
        }
    }
    else {
        uint8_t optimize_detect;
        rfm95_read_register(REG_DETECTION_OPTIMIZE, &optimize_detect);
        optimize_detect |= 0x80;
        rfm95_write_register(REG_DETECTION_OPTIMIZE, optimize_detect); 
    }
    printf("sha-2\n");
    if (LORA_FHSS_ENABLED) {
        uint8_t irq_mask;
        rfm95_read_register(REG_IRQMASK, &irq_mask);
        irq_mask &= ~(RFLR_IRQFLAGS_RXDONE);
        rfm95_write_register(REG_IRQMASK, irq_mask);
        rfm95_write_register(REG_IRQMASK, RFLR_IRQFLAGS_VALIDHEADER |
                                           RFLR_IRQFLAGS_TXDONE |
                                           RFLR_IRQFLAGS_CADDONE |
                                           RFLR_IRQFLAGS_CADDETECTED );
        uint8_t dio_mapping1;
        rfm95_read_register(REG_DIO_MAPPING_1, &dio_mapping1);
        dio_mapping1 = (dio_mapping1 & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00;
        rfm95_write_register(REG_DIO_MAPPING_1, dio_mapping1);                                       
    }
    else {
        uint8_t irq_mask;
        rfm95_read_register(REG_IRQMASK, &irq_mask);
        irq_mask &= ~(RFLR_IRQFLAGS_RXDONE);
        rfm95_write_register(REG_IRQMASK, irq_mask);
        rfm95_write_register(REG_IRQMASK, RFLR_IRQFLAGS_VALIDHEADER |
                                          RFLR_IRQFLAGS_TXDONE |
                                          RFLR_IRQFLAGS_CADDONE |
                                          RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                          RFLR_IRQFLAGS_CADDETECTED );
        uint8_t dio_mapping1;
        rfm95_read_register(REG_DIO_MAPPING_1, &dio_mapping1);
        dio_mapping1 = (dio_mapping1 & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00;
        rfm95_write_register(REG_DIO_MAPPING_1, dio_mapping1); 

    }
    printf("sha-3\n");
    rfm95_write_register( REG_FIFO_RX_BASE_ADDR, 128);
    rfm95_write_register( REG_FIFO_ADDR_PTR, 128 );
    //change state to indicate we are receiving
    rfm95_current_radio_state = RF_RX_RUNNING;
    printf("sha-4\n");
    //TODO - placeholder for being able to receive continously
    rfm95_rx();
    printf("sha-5\n");
    //while(!lora);
    //lora = 0;
    /* uint8_t size_rx, i = 0;
    rfm95_read_register(REG_RX_NB_BYTES, &size_rx);
    printf("size %x\n", size_rx);
    //read fifo
    for (i = 0; i < 4; i++) {
        rfm95_read_registe
        r(REG_FIFO, &rx_buffer[i]);
        printf("%c\n", rx_buffer[i]);
    }
    memset(rx_buffer, 0 , LORA_RX_BUFFER_SIZE);
    //TODO - more clean up???
    rfm95_current_radio_state = RF_IDLE;
    rfm95_idle();*/
}

void rfm95_set_tx_config(uint32_t f_dev, uint32_t band_width, uint32_t data_rate,
                         uint8_t code_rate, uint16_t preamble_length, bool fix_length, 
                         bool crc_on, bool freq_hop_on, uint8_t hop_period, 
                         bool iq_inverted, uint32_t timeout) {
    bool low_data_rate_optimize;
    uint8_t modem_config_1, modem_config_2, modem_config_3;
    rfm95_sleep();

    if (band_width > 2) {
        assert(0);
    }
    band_width += 7;
    if (data_rate > 12) {
        data_rate = 12;
    }
    else if (data_rate < 6) {
        data_rate = 6;
    }
    if( ( ( band_width == 7 ) && ( ( data_rate == 11 ) || ( data_rate == 12 ) ) ) ||
        ( ( band_width == 8 ) && ( data_rate == 12 ) ) ) {
        low_data_rate_optimize = true;
    }
    else {
        low_data_rate_optimize = false;
    }
    if (freq_hop_on) {
        uint8_t freq_hop_reg;
        rfm95_read_register(REG_LR_PLLHOP, &freq_hop_reg);
        freq_hop_reg  = (freq_hop_reg & RFLR_PLLHOP_FASTHOP_MASK) | (RFLR_PLLHOP_FASTHOP_ON);
        rfm95_write_register(REG_LR_PLLHOP, freq_hop_reg);
        rfm95_write_register(REG_LR_HOP_PERIOD, hop_period);
    }
    rfm95_read_register(REG_MODEM_CONFIG_1, &modem_config_1);
    modem_config_1 = (modem_config_1 & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK)
                     | (band_width << 4) | (code_rate << 1) | fix_length;
    rfm95_write_register(REG_MODEM_CONFIG_1, modem_config_1);

    rfm95_read_register(REG_MODEM_CONFIG_2, &modem_config_2);
    modem_config_2 = (modem_config_2 & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK )
                     | (data_rate << 4) | (crc_on << 2);
    rfm95_write_register(REG_MODEM_CONFIG_2, modem_config_2);              

    rfm95_read_register(REG_MODEM_CONFIG_2, &modem_config_3);
    modem_config_3 = (modem_config_3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) | low_data_rate_optimize;
    rfm95_write_register(REG_MODEM_CONFIG_2, modem_config_3);  
    
    rfm95_write_register( REG_PREAMBLE_MSB, ( preamble_length >> 8 ) & 0x00FF );
    rfm95_write_register( REG_PREAMBLE_LSB, preamble_length & 0xFF );

    if (data_rate == 6) {
        uint8_t optimize_detect;
        rfm95_read_register(REG_DETECTION_OPTIMIZE, &optimize_detect);
        optimize_detect = (optimize_detect & RFLR_DETECTIONOPTIMIZE_MASK) | RFLR_DETECTIONOPTIMIZE_SF6;
        rfm95_write_register(REG_DETECTION_OPTIMIZE, optimize_detect);
        rfm95_write_register(REG_DETECTION_THRESHOLD, RFLR_DETECTIONTHRESH_SF6);
    }
    else {
        uint8_t optimize_detect;
        rfm95_read_register(REG_DETECTION_OPTIMIZE, &optimize_detect);
        optimize_detect = (optimize_detect & RFLR_DETECTIONOPTIMIZE_MASK) | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12;
        rfm95_write_register(REG_DETECTION_OPTIMIZE, optimize_detect);
        rfm95_write_register(REG_DETECTION_THRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
    }
}

void rfm95_set_lora_opmode() {
    //set rfm95 to operate in Lora mode
    uint8_t opmode;
    rfm95_read_register(REG_OP_MODE, &opmode);
    opmode = (opmode & RF_OPMODE_RANGE) | MODE_LONG_RANGE_MODE;
    rfm95_write_register(REG_OP_MODE, opmode);
    rfm95_write_register(REG_DIO_MAPPING_1, 0x00);
    rfm95_write_register(REG_DIO_MAPPING_2, 0x00);
}

radio_state_t rfm95_get_op_mode() {
    return rfm95_current_radio_state;
}
void rfm95_set_rx_config(uint32_t band_width, uint32_t data_rate, uint8_t code_rate,
                          uint32_t bandwidth_afc, uint16_t preamble_length, uint16_t symb_timeout, bool fix_length,
                          uint8_t payload_length, bool crc_on, bool freq_hop_on, uint8_t hop_period,
                          bool iq_inverted, bool rx_continuous) {
    rfm95_sleep();
    bool low_data_rate_optimize;
    uint8_t modem_config_1, modem_config_2, modem_config_3;

    if (band_width > 2) {
        assert(0);
    }
    band_width += 7;
    if (data_rate > 12) {
        data_rate = 12;
    }
    else if (data_rate < 6) {
        data_rate = 6;
    }
    if( ( ( band_width == 7 ) && ( ( data_rate == 11 ) || ( data_rate == 12 ) ) ) ||
        ( ( band_width == 8 ) && ( data_rate == 12 ) ) ) {
        low_data_rate_optimize = true;
    }
    else {
        low_data_rate_optimize = false;
    }
    rfm95_read_register(REG_MODEM_CONFIG_1, &modem_config_1);
    modem_config_1 = (modem_config_1 & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK)
                     | (band_width << 4) | (code_rate << 1) | fix_length;
    rfm95_write_register(REG_MODEM_CONFIG_1, modem_config_1);

    rfm95_read_register(REG_MODEM_CONFIG_2, &modem_config_2);
    modem_config_2 = (modem_config_2 & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK)
                     | (data_rate << 4) | (crc_on << 2) | ( ( symb_timeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    rfm95_write_register(REG_MODEM_CONFIG_2, modem_config_2);

    rfm95_read_register(REG_MODEM_CONFIG_3, &modem_config_3);
    modem_config_3 = (modem_config_3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) | (low_data_rate_optimize << 3);
    rfm95_write_register(REG_MODEM_CONFIG_3, modem_config_3);  

    rfm95_write_register( REG_RX_TIMEOUT_LSB, ( uint8_t )( symb_timeout & 0xFF ));
    rfm95_write_register( REG_PREAMBLE_MSB, ( uint8_t )( ( preamble_length >> 8 ) & 0xFF ));
    rfm95_write_register( REG_PREAMBLE_LSB, ( uint8_t )( preamble_length & 0xFF ));

    if (fix_length) {
        rfm95_write_register(REG_PAYLOAD_LENGTH, payload_length);
    }
    if (freq_hop_on) {
        uint8_t freq_hop_reg;
        rfm95_read_register(REG_LR_PLLHOP, &freq_hop_reg);
        freq_hop_reg  = (freq_hop_reg & RFLR_PLLHOP_FASTHOP_MASK) | (RFLR_PLLHOP_FASTHOP_ON);
        rfm95_write_register(REG_LR_PLLHOP, freq_hop_reg);
        rfm95_write_register(REG_LR_HOP_PERIOD, hop_period);
    }

    if(( band_width == 9 ) && ( RF_FREQUENCY > RF_MID_BAND_THRESH ) ) {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        rfm95_write_register( REG_36, 0x02 );
        rfm95_write_register( REG_3A, 0x64 );
    }
    else if( band_width == 9 ) {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        rfm95_write_register( REG_36, 0x02 );
        rfm95_write_register( REG_3A, 0x7F );
    }
    else {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        rfm95_write_register( REG_36, 0x03 );
    }
    if (data_rate == 6) {
        uint8_t optimize_detect;
        rfm95_read_register(REG_DETECTION_OPTIMIZE, &optimize_detect);
        optimize_detect = (optimize_detect & RFLR_DETECTIONOPTIMIZE_MASK) | RFLR_DETECTIONOPTIMIZE_SF6;
        rfm95_write_register(REG_DETECTION_OPTIMIZE, optimize_detect);
        rfm95_write_register(REG_DETECTION_THRESHOLD, RFLR_DETECTIONTHRESH_SF6);
    }
    else {
        uint8_t optimize_detect;
        rfm95_read_register(REG_DETECTION_OPTIMIZE, &optimize_detect);
        optimize_detect = (optimize_detect & RFLR_DETECTIONOPTIMIZE_MASK) | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12;
        rfm95_write_register(REG_DETECTION_OPTIMIZE, optimize_detect);
        rfm95_write_register(REG_DETECTION_THRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
    }

}

void rfm95_isr_handler(uint8_t pin, uint8_t val) {
    if (pin == MCP_DIO0_PIN_NUMBER) {
        rfm95_dio0_irq(pin, val);
    }
    else if (pin == MCP_DIO1_PIN_NUMBER) {
    }
    else {
        assert(0);
    }
}

void lora_task_old() {
    while (1) {
        //uint8_t rx_data = 0;
        //lets try transmitting a PHY message for now
        uint8_t data[4] = {'P', 'I', 'N', 'G'};
        //begin_packet();
        rfm95_send(data, 4);
        while (rfm95_current_radio_state == RF_TX_RUNNING) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            taskYIELD();
        }
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        rfm95_receive();
        while (rfm95_current_radio_state == RF_RX_RUNNING) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            taskYIELD();
        }
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        //rfm95_dio0_irq(MCP_DIO0_PIN_NUMBER, 1);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        //rfm95_send(data, 4);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        //end_packet();
    }
}

void rfm95_init() {
    //initialize the HOPE RF module
    //configuration settings used from https://github.com/sandeepmistry/arduino-LoRa
    uint8_t rx_data = 0;
    spi_bus_configure();
    spi_device_configure();
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_bus_config, 1));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &spi_device_config, &handle_spi));

    //pull the slave select bit high
    gpio_pad_select_gpio(HOPE_RF_SLAVE_SELECT_PIN);
    gpio_set_direction(HOPE_RF_SLAVE_SELECT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(HOPE_RF_SLAVE_SELECT_PIN, 1);

    //reset the HOPE RF module
    pinMode(HOPE_RF_RST_PIN, GPIO_MODE_OUTPUT);
    digitalWrite(HOPE_RF_RST_PIN,0);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    digitalWrite(HOPE_RF_RST_PIN,1);
  

    rfm95_read_register(REG_VERSION, &rx_data);
    if (rx_data != HOPERF_VERSION) {
        //sanity checker, make sure version matches, also ensures SPI communication is working
        assert(0);
    }

    //in order to modify any register settings in rfm95, we need to put it to sleep mode
    //refer data sheet
    rfm95_sleep();
    
    //placeholder for changing tx frequency
    set_frequency(RF_FREQUENCY);
    
    rfm95_set_lora_opmode();
    rfm95_set_tx_power(TX_OUTPUT_POWER);

    rfm95_set_tx_config(0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, LORA_CRC_ENABLED, 
                        LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP, LORA_IQ_INVERSION_ON, 2000 );

    rfm95_set_rx_config(LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                        LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, LORA_CRC_ENABLED, LORA_FHSS_ENABLED, 
                        LORA_NB_SYMB_HOP, LORA_IQ_INVERSION_ON, true);

    //set the pointers of the FIFO buffer to 0, so that we use the full buffer
    rfm95_write_register(REG_FIFO_TX_BASE_ADDR, 0);
    rfm95_write_register(REG_FIFO_RX_BASE_ADDR, 0);

    rx_buffer = malloc(LORA_RX_BUFFER_SIZE);
/* 
    //set LF frequency boost
    rfm95_read_register(REG_LNA, &rx_data);
    rfm95_write_register(REG_LNA, (rx_data | 0x3));
    rfm95_read_register(REG_LNA, &rx_data);

    //set autoAGC
    rfm95_write_register(REG_MODEM_CONFIG_3, 0x04);
*/
    //placeholder for boosting transmission power
    rfm95_idle();
    rfm95_current_radio_state = RF_IDLE;

    //setup the interrupts in the IO expander
    pinMode(0xA,GPIO_MODE_INPUT);
    setupInterruptPin(0xA,GPIO_INTR_HIGH_LEVEL);

    pinMode(0xB, GPIO_MODE_INPUT);
    setupInterruptPin(0xB,GPIO_INTR_HIGH_LEVEL);

    xTaskCreate(lora_task_old, "lora_task", 8192, NULL, 9, NULL);
    
}