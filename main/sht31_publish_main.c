#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

static const char *TAG = "sht31-publish";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define SHT31_I2CADDR                   ( 0x44 )
#define SHT31_MEAS_HIGHREP              ( 0x2400 )
#define SHT31_READSTATUS                ( 0xF32D )
#define SHT31_CLEARSTATUS               ( 0x3041 )
#define SHT31_SOFTRESET                 ( 0x30A2 )
#define SHT31_STATUS_DATA_CRC_ERROR     ( 0x0001 )
#define SHT31_STATUS_COMMAND_ERROR      ( 0x0002 )
#define SHT31_STATUS_RESET_DETECTED     ( 0x0010 )
#define SHT31_STATUS_TEMPERATURE_ALERT  ( 0x0400 )
#define SHT31_STATUS_HUMIDITY_ALERT     ( 0x0800 )
#define SHT31_STATUS_HEATER_ACTIVE      ( 0x2000 )
#define SHT31_STATUS_ALERT_PENDING      ( 0x8000 )
#define ACK_CHECK_EN                    ( 0x1 )
#define ACK_CHECK_DIS                   ( 0x0 )
#define ACK_VAL                         ( 0x0 )
#define NACK_VAL                        ( 0x1 )


SemaphoreHandle_t print_mux = NULL;

static esp_err_t sht31_write_cmd( i2c_port_t i2c_num, uint16_t command )
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT31_I2CADDR << 1 | 0x00), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command >> 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command & 0xFF, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t sht31_read_temperature_humidity( i2c_port_t i2c_num, uint8_t *data )
{
    int ret = sht31_write_cmd( i2c_num, SHT31_MEAS_HIGHREP );
    ESP_LOGW(TAG, "Write Command Result: %s", esp_err_to_name(ret));
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(50 / portTICK_RATE_MS);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT31_I2CADDR << 1 | 0x01), ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data[0], ACK_VAL);
    i2c_master_read_byte(cmd, &data[1], ACK_VAL);
    i2c_master_read_byte(cmd, &data[2], ACK_VAL);
    i2c_master_read_byte(cmd, &data[3], ACK_VAL);
    i2c_master_read_byte(cmd, &data[4], ACK_VAL);
    i2c_master_read_byte(cmd, &data[5], NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t sht31_get_temperature_humidity( i2c_port_t i2c_num, float *temp, float *hum )
{
    int ret;
    uint8_t data[6];

    ret = sht31_read_temperature_humidity( i2c_num, &data[0] );
    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t r_temp = (data[0] << 8) | data[1];
    uint16_t r_hum = (data[3] << 8) | data[4];

    *temp =  175.0 * (float)r_temp / 0xFFFF - 45.0;
    *hum = 100.0 * (float)r_hum / 0xFFFF;

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


static void sht31_task(void *arg)
{
    int ret;
    uint32_t task_idx = (uint32_t)arg;
    int cnt = 0;
    float temp = 0;
    float hum = 0;
    
    while(1)
    {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        ret = sht31_get_temperature_humidity(I2C_MASTER_NUM, &temp, &hum);
        
        if( ret == ESP_ERR_TIMEOUT )
        {
            ESP_LOGE(TAG, "I2C Timeout");
        }
        else if( ret == ESP_OK )
        {
            ESP_LOGI(TAG, "Temp: %f, Hum: %f", temp, hum);
        }
        else
        {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
        
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void app_main()
{
    // print_mux = xSemaphoreCreateMutex();
    // ESP_ERROR_CHECK(i2c_slave_init());
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(sht31_task, "sht31_task", 1024 * 2, (void *)0, 10, NULL);
}
