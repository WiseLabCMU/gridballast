#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "lora_module.h"
#include <string.h>
#include "esp_err.h"
#include "freertos/task.h"
#include "Ada_MCP.h" 
#include "driver/gpio.h"


spi_bus_config_t spi_bus_config;
spi_device_interface_config_t spi_device_config;
static spi_device_handle_t handle_spi;      // SPI handle.

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
    printf("reg %x data %x\n", reg, *data);
}

void rfm95_write_register(uint8_t reg, const uint8_t val) {
    uint8_t write_register = reg | 0x80;
    gpio_set_level(HOPE_RF_SLAVE_SELECT_PIN, 0);
    rfm95_send_data(write_register);
    rfm95_send_data(val);
    gpio_set_level(HOPE_RF_SLAVE_SELECT_PIN, 1);
    /* uint8_t check;
    rfm95_read_register(reg, &check);
    printf("reg %x val %x check %x\n", reg, val, check);
    assert(check == val);*/
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
uint8_t get_pa_select(uint64_t channel) {
    //NOTE - this may have to be revisited in case things dont work
    if (channel > RF_MID_BAND_THRESH) {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else {
        return RF_PACONFIG_PASELECT_RFO;
    }
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

void rfm95_set_tx_config(uint32_t f_dev, uint32_t band_width, uint32_t data_rate,
                         uint8_t code_rate, uint16_t preamble_length, bool fix_length, 
                         bool crc_on, bool freq_hop_on, uint8_t hop_period, 
                         bool iq_inverted, uint32_t timeout) {
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
    if (freq_hop_on) {
        uint8_t freq_hop_reg;
        rfm95_read_register(REG_LR_PLLHOP, &freq_hop_reg);
        freq_hop_reg  = (freq_hop_reg & RFLR_PLLHOP_FASTHOP_MASK) | (RFLR_PLLHOP_FASTHOP_ON);
        rfm95_write_register(REG_LR_PLLHOP, freq_hop_reg);
        rfm95_write_register(REG_LR_HOP_PERIOD, hop_period);
    }
    rfm95_read_register(REG_MODEM_CONFIG_1, &modem_config_1);
    modem_config_1 = (modem_config_1 & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK &RFLR_MODEMCONFIG1_CODINGRATE_MASK)
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

void lora_task() {
    while (1) {
        //uint8_t rx_data = 0;
        //lets try transmitting a PHY message for now
        uint8_t data[4] = {'P', 'I', 'N', 'G'};
        //begin_packet();
        rfm95_send(data, 4);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //end_packet();
    }
}

void set_frequency(uint64_t frequency) {
    uint64_t frf = ((double)frequency) / ((double)FREQ_STEP);
    rfm95_write_register(REG_FRF_MSB, (uint8_t)((frf >> 16) & 0xff));
    rfm95_write_register(REG_FRF_MID, (uint8_t)((frf >> 8) & 0xff));
    rfm95_write_register(REG_FRF_LSB, (uint8_t)(frf & 0xff));

}

void lora_init() {
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
    
    //set rfm95 to operate in Lora mode
    rfm95_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE);
    rfm95_write_register(REG_DIO_MAPPING_1, 0x00);
    rfm95_write_register(REG_DIO_MAPPING_2, 0x00);

    rfm95_set_tx_power(TX_OUTPUT_POWER);

    rfm95_set_tx_config(0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, LORA_CRC_ENABLED, 
                        LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP, LORA_IQ_INVERSION_ON, 2000 );

    //set the pointers of the FIFO buffer to 0, so that we use the full buffer
    rfm95_write_register(REG_FIFO_TX_BASE_ADDR, 0);
    rfm95_write_register(REG_FIFO_RX_BASE_ADDR, 0);

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
    xTaskCreate(lora_task, "lora_task", 8192, NULL, 9, NULL);
    
}
