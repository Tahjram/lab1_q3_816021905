#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"


static const char *TAG = "main";



#define I2C_MASTER_SCL_IO                   2                /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO                   0               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM                      I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE           0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE           0                /*!< I2C master do not need buffer */

//ADS1115 Addresses
#define ADS1115_CONV                        0x0             //Conversion Register
#define ADS1115_CONF                        0x1             //Configuration Register
#define ADS1115_LO                          0x2             //Low Threshold Register
#define ADS1115_HI                          0x3             //High Threshold Register
//Slave Addresses for ADDR pins
#define ADS1115_GND                         0x48
#define ADS1115_VDD                         0x49
#define ADS1115_SDA                         0x4A
#define ADS1115_SCL                         0x4B

#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

// Struct for Config Register
typedef struct CONFIG
{
    uint8_t OS;                                             //Operational Staus 15
    uint8_t MUX;                                            //Mux Configuration 14:12
    uint8_t PGA;                                            //Gain Amplifier Configuration 11:9
    uint8_t MODE;                                           //Operating Mode 8
    uint8_t DR;                                             //Data rate 7:5
    uint8_t COMP_MODE;                                      //Comparator Mode 4
    uint8_t COMP_POL;                                       //Comparator Polarity 3
    uint8_t COMP_LAT;                                       //Latching Comparator 2
    uint8_t COMP_QUE;                                       //Comparator Queue 1:0
    uint16_t CONF;                                          //16-bit Configuration Line
}ADS1115_CONF_REG;


static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}


static esp_err_t i2c_master_ads1115_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_GND << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


static esp_err_t i2c_master_ads1115_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_GND << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_GND << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t data_write(i2c_port_t i2c_num, uint8_t reg_address, uint16_t data)
{
    int ret;
    uint8_t bytes[2];
    bytes[0] = (data >> 8) & 0xFF;
    bytes[1] = (data >> 0) & 0xFF;

    ret = i2c_master_ads1115_write(i2c_num, reg_address, bytes, 2);
    return ret;
}

static esp_err_t data_read(i2c_port_t i2c_num, uint8_t reg_address, uint16_t *data)
{
    int ret;
    uint8_t bytes[2];

    ret = i2c_master_ads1115_read(i2c_num, reg_address, bytes, 2);
    *data = (bytes[0] << 8) | bytes[1];
    return ret;
}

static esp_err_t i2c_master_ads1115_init(i2c_port_t i2c_num)
{
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_master_init();

    ADS1115_CONF_REG conf;
    conf.OS = 0x00;        // NULL
    conf.MUX = 0x04;       // AINp = AIN0 and AINn = GND
    conf.PGA = 0x01;       // FS = 4.096 V
    conf.MODE = 0x00;      // Continuous-Conversion Mode
    conf.DR = 0x04;        // 128SPS
    conf.COMP_MODE = 0x00; // Traditional Comparator
    conf.COMP_POL = 0x00;  // Active Low
    conf.COMP_LAT = 0x00;  // Non-latching Comparator
    conf.COMP_QUE = 0x02;  // Assert After Four Conversions

    uint16_t data;
    data = (conf.OS << 3) | conf.MUX;
    data = (data << 3) | conf.PGA;
    data = (data << 1) | conf.MODE;
    data = (data << 3) | conf.DR;
    data = (data << 1) | conf.COMP_MODE;
    data = (data << 1) | conf.COMP_POL;
    data = (data << 1) | conf.COMP_LAT;
    data = (data << 2) | conf.COMP_QUE;
    conf.CONF = data;

    // Output Configuration Bits
    ESP_LOGI(TAG, "Configuration Bits: %d\n", conf.CONF);

    // Writing to CONFIG Register
    ESP_ERROR_CHECK(data_write(i2c_num, ADS1115_CONF, conf.CONF));

    return ESP_OK;
}

static void i2c_task(void *arg)
{
    uint16_t data;
    double voltage;
    esp_err_t check;

    
    i2c_master_ads1115_init(I2C_MASTER_NUM);
    
    while(1)
    {
        check = data_read(I2C_MASTER_NUM,ADS1115_CONV,&data);
        if(check == ESP_OK)
        {
            ESP_LOGI(TAG,"Read Successful\n");
            voltage = (double)data * 1.25e-4;
            ESP_LOGI(TAG,"Voltage = %d.%d V\n",(uint16_t)voltage,(uint16_t)(voltage*100)%100);
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }

    }

    ESP_LOGI(TAG,"Exiting\n");
    i2c_driver_delete(I2C_MASTER_NUM);
}

void app_main(void)
{
    //start i2c task
    xTaskCreate(i2c_task, "i2c_task", 2048, NULL, 10, NULL);
}