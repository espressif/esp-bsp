/*
 * mpu60xx.c
 *
 *  Created on: 15-Oct-2024
 *      Author: rohan
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "mpu60xx.h"




//MPU60XX registers
#define MPU60xx_ADDR                0x68    // MPU60XX I2C address

#define MPU60xx_DEVICE_ID_REG       0X75    // Device ID Register

#define MPU60xx_USER_CTRL_REG       0X6A    // User Control Register
#define MPU60xx_PWR_MGMT1_REG       0X6B    // Power Management Registers 1
#define MPU60xx_PWR_MGMT2_REG       0X6C    // Power Management Registers 2

#define MPU60xx_SMPRT_DIV           0X19    // Sample Rate Divider
#define MPU60xx_CFG_REG             0X1A    // Configuration Register
#define MPU60xx_GYRO_CFG_REG        0X1B    // Gyroscope Configuration Register
#define MPU60xx_ACCEL_CFG_REG       0X1C    // Accelerometer Configuration Register

#define MPU60xx_MOT_THR             0x1F    // Motion detection threshold bits [7:0]
#define MPU60xx_MOT_DUR             0x20    // Duration counter, threshold for motion int. 1 kHz rate, LSB = 1 ms


#define MPU60xx_INT_PIN_CONFIG      0X37    // Interrupt/Bypass Setting Register
#define MPU60xx_INT_EN_REG          0X38    // Interrupt Enable Register
#define MPU60xx_INT_STATUS          0x3A     // Interrupt status register

#define MPU60xx_ACCEL_XOUTH_REG     0X3B    // Accelerometer X-axis High Byte Register
#define MPU60xx_ACCEL_XOUTL_REG     0X3C    // Accelerometer X-axis Low Byte Register
#define MPU60xx_ACCEL_YOUTH_REG     0X3D    // Accelerometer Y-axis High Byte Register
#define MPU60xx_ACCEL_YOUTL_REG     0X3E    // Accelerometer Y-axis Low Byte Register
#define MPU60xx_ACCEL_ZOUTH_REG     0X3F    // Accelerometer Z-axis High Byte Register
#define MPU60xx_ACCEL_ZOUTL_REG     0X40    // Accelerometer Z-axis Low Byte Register

#define MPU60xx_TEMP_OUTH_REG       0X41    // Temperature Value High Byte Register
#define MPU60xx_TEMP_OUTL_REG       0X42    // Temperature Value Low Byte Register

#define MPU60xx_GYRO_XOUTH_REG      0X43    // Gyroscope X-axis High Byte Register
#define MPU60xx_GYRO_XOUTL_REG      0X44    // Gyroscope X-axis Low Byte Register
#define MPU60xx_GYRO_YOUTH_REG      0X45    // Gyroscope Y-axis High Byte Register
#define MPU60xx_GYRO_YOUTL_REG      0X46    // Gyroscope Y-axis Low Byte Register
#define MPU60xx_GYRO_ZOUTH_REG      0X47    // Gyroscope Z-axis High Byte Register
#define MPU60xx_GYRO_ZOUTL_REG      0X48    // Gyroscope Z-axis Low Byte Register
#define MPU60xx_SIGNAL_PATH_RESET   0x68    //Signal path reset register


static const char *TAG = "MPU60xx"; // tag used in logs

static const float_t s_g_value = 9.80665; // standard acceleration of gravity, multiplier to convert accelerometer reading to m/s 


/**
 * @brief Write command to MPU60xx. 
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] reg MPU60xx register address to write to.
 *
 * @param[in] data Data bytes to send to MPU60xx.
 *
 * @param[in] length Size, in bytes, of data.
 *
 * @return  
 *      - ESP_OK: gyrometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log  
 *
 * @note only values defined in "mpu60xx_gyro_range_t" are valid
 */
static esp_err_t mpu60xx_write_register(mpu60xx_handle_t mpu_handle, uint8_t reg,const uint8_t* data, size_t length);

/**
 * @brief Read data from MPU60xx 
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] data_addr MPU60xx register address to read from 
 *
 * @param[out] data Data buffer store values read from the MPU60xx.
 *
 * @param[in] length Size, in bytes, of data.
 *
 * @return  
 *      - ESP_OK: gyrometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log  
 *
 * @note only values defined in "mpu60xx_gyro_range_t" are valid
 */
static esp_err_t mpu60xx_read_register(mpu60xx_handle_t mpu_handle, const uint8_t data_addr,  uint8_t *data, size_t length);

/**
 * @brief Get accelerometer resolution
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[out] range resolution read from accelerometer
 *
 * @return  
 *      - ESP_OK: gyrometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log  
 */
static esp_err_t getAccelerometerRange(mpu60xx_handle_t mpu_handle,mpu60xx_accel_range_t * range );

/**
 * @brief Get gyrometer resolution
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[out] range resolution read from gyrometer
 *
 * @return  
 *      - ESP_OK: gyrometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log  
 */
static esp_err_t getGyrometerRange(mpu60xx_handle_t mpu_handle,mpu60xx_gyro_range_t * range );

/**
 * @brief Enable digital low pass filter on MPU60xx
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] b_width low pass filter bandwidth options
 *   
 * @return  
 *      - ESP_OK: successfully enabled dlpf with provided b_width
 *      - other error codes : from the underlying i2c driver, check log  
 */
static esp_err_t en_DLPF( mpu60xx_handle_t mpu_handle, mpu60xx_lowpass_t b_width);

/**
 * @brief Enable digital high pass filter on MPU60xx
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] b_width high pass filter bandwidth options
 *   
 * @return  
 *      - ESP_OK: successfully enabled dhpf with provided b_width
 *      - other error codes : from the underlying i2c driver, check log  
 */
static esp_err_t en_DHPF( mpu60xx_handle_t mpu_handle, mpu60xx_highpass_t bandwidth);

/**
 * @brief Register ISR to handle interrupt from MPU60xx
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] interrupt_configuration configuration passed through "mpu60xx_intrpt_config_t"
 *   
 * @return  
 *      - ESP_OK: successfully registered ISR
 *      - other error codes : from the underlying i2c driver or ISR mechanism, check log  
 */
static esp_err_t register_isr(mpu60xx_handle_t * sensor, mpu60xx_intrpt_config_t* interrupt_configuration);




static esp_err_t mpu60xx_write_register(mpu60xx_handle_t mpu_handle, uint8_t reg,const uint8_t* data, size_t length)
{
    esp_err_t ret =i2c_master_probe(*mpu_handle.bus_handle, MPU60xx_ADDR, 50);
    if (ret == ESP_OK) {
         ret=i2c_master_transmit(*mpu_handle.dev_handle, data, length, 50);
    }
    else
        ESP_LOGE(TAG, "mpu60xx not detected on i2c");
    
    return ret;
}

static esp_err_t mpu60xx_read_register(mpu60xx_handle_t mpu_handle, const uint8_t data_addr,  uint8_t *data, size_t length)
{
    esp_err_t ret =i2c_master_probe(*mpu_handle.bus_handle, MPU60xx_ADDR, 50);
    if (ret == ESP_OK) {
         ret = i2c_master_transmit_receive(*mpu_handle.dev_handle, (uint8_t*)&data_addr, 1, data, length,50);
    }
    else
        ESP_LOGE(TAG, "mpu60xx not detected on i2c");
        
    return ret;
}

esp_err_t mpu60xx_setAccelerometerRange(mpu60xx_handle_t mpu_handle, mpu60xx_accel_range_t range)
{		
		uint8_t cmd[2];
		cmd[0]=MPU60xx_ACCEL_CFG_REG;
		cmd[1]=range;
		esp_err_t  ret = mpu60xx_write_register(mpu_handle,MPU60xx_ADDR,cmd, 2);
		return ret;
}

static esp_err_t getAccelerometerRange(mpu60xx_handle_t mpu_handle,mpu60xx_accel_range_t * range )
{
	uint8_t reg_value;	
	
	esp_err_t  ret = mpu60xx_read_register(mpu_handle,MPU60xx_ACCEL_CFG_REG, &reg_value, 1);
	
 	if (ret == ESP_OK) {
		*range = reg_value & 0x18;			 
	}
	return ret;
}

esp_err_t mpu60xx_setGyrometerRange(mpu60xx_handle_t mpu_handle, mpu60xx_gyro_range_t range)
{
		uint8_t cmd[2];
		cmd[0]=MPU60xx_GYRO_CFG_REG;
		cmd[1]=range;
		esp_err_t  ret = mpu60xx_write_register(mpu_handle,MPU60xx_ADDR,cmd, 2);
		return ret;
}

static esp_err_t getGyrometerRange(mpu60xx_handle_t mpu_handle,mpu60xx_gyro_range_t * range )
{	
	uint8_t reg_value;	
	
	esp_err_t  ret = mpu60xx_read_register(mpu_handle,MPU60xx_GYRO_CFG_REG, &reg_value, 1);
	
 	if (ret == ESP_OK) {
		*range = reg_value & 0x18;			 
	}  
	return ret; 
}

esp_err_t mpu60xx_setSampleRate ( mpu60xx_handle_t mpu_handle, uint32_t sample_rate, mpu60xx_lowpass_t dlpf_bw)
{
	uint8_t cmd[2],smpl_rt_div=0;
	uint32_t gy_out_rate = 8000;
	esp_err_t ret;
	
	if (sample_rate<5) sample_rate =5; //minimum valid value =4.94
	
	if (dlpf_bw!=MPU60XX_BAND_260_HZ) {
		if (sample_rate>1000) sample_rate =1000; // max when DLPF used
		gy_out_rate = 1000;
	}
	else if (sample_rate>8000) sample_rate =8000;// max when DLPF not used
	
	
	ret = en_DLPF(mpu_handle, sample_rate);
	if (ret !=ESP_OK)
			return ret;
	
	smpl_rt_div = (uint8_t) ( gy_out_rate/ sample_rate) -1;
	cmd[0] =MPU60xx_SMPRT_DIV;
	cmd[1]= smpl_rt_div;
	ret = mpu60xx_write_register(mpu_handle,MPU60xx_ADDR,cmd,2);
	return ret;
}

static esp_err_t en_DLPF( mpu60xx_handle_t mpu_handle, mpu60xx_lowpass_t b_width)
{
    uint8_t reg_value, cmd[2];
    esp_err_t ret;
						
	ret= mpu60xx_read_register(mpu_handle, MPU60xx_CFG_REG,&reg_value,1);
	if (ret !=ESP_OK) {
        ESP_LOGE(TAG,"unable to read MPU60xx_CFG_REG  register");
        return ret;
	}
		
    reg_value = reg_value &  ((uint8_t)(~7));
    reg_value |= b_width;
	
    cmd[0] = MPU60xx_CFG_REG;
    cmd[1] = reg_value;
    ret= mpu60xx_write_register(mpu_handle,MPU60xx_ADDR,cmd, 2);

	if (ret !=ESP_OK)
		ESP_LOGE(TAG,"unable to write to MPU60xx_CFG_REG  register");
	return ret;
	
}

static esp_err_t en_DHPF( mpu60xx_handle_t mpu_handle, mpu60xx_highpass_t bandwidth)
{
	esp_err_t ret;
	uint8_t reg_value,cmd[2];
		ret= mpu60xx_read_register(mpu_handle, MPU60xx_ACCEL_CFG_REG,&reg_value,1);
	if (ret !=ESP_OK) {
		ESP_LOGE(TAG,"unable to read MPU60xx_CFG_REG  register");
		return ret;
	}
		
	reg_value = reg_value &  ((uint8_t)(~7));
	reg_value |= bandwidth;
	
	cmd[0] = MPU60xx_ACCEL_CFG_REG;
	cmd[1] = reg_value;
	ret= mpu60xx_write_register(mpu_handle,MPU60xx_ADDR,cmd, 2);

	if (ret !=ESP_OK)
		ESP_LOGE(TAG,"unable to write to MPU60xx_CFG_REG  register");
	
	return ret;
}


esp_err_t mpu60xx_en_MotionDetection(mpu60xx_handle_t * mpu_handle,mpu60xx_motion_detect_config_t *mdc, mpu60xx_intrpt_config_t* interrupt_configuration)
{
	uint8_t  cmd[2];
	esp_err_t ret;
		
	cmd[0] =MPU60xx_MOT_THR;
	cmd[1]= mdc->motion_threshold;
	ret = mpu60xx_write_register(*mpu_handle,MPU60xx_ADDR,cmd,2);
	
	
	cmd[0] =MPU60xx_MOT_DUR;
	cmd[1]= mdc->motion_duration;
	ret = mpu60xx_write_register(*mpu_handle,MPU60xx_ADDR,cmd,2);
	
	en_DHPF(*mpu_handle,mdc->dhpf_bw);
	mpu60xx_enMotionDetectInterrupt(mpu_handle, true);
	
	if (NULL != interrupt_configuration) {
	    if (GPIO_IS_VALID_GPIO(interrupt_configuration->interrupt_pin)) {
	        // Set GPIO connected to mpu60xx INT pin only when user configures interrupts.
	       // mpu60xx_dev_t *sensor_device = (mpu60xx_dev_t *) sensor;
	       // sensor_device->int_pin = interrupt_configuration->interrupt_pin;
	    } else {
	        ret = ESP_ERR_INVALID_ARG;
	        return ret;
	    }
	
	    uint8_t int_pin_cfg = 0x00;
	
	    ret = mpu60xx_read_register(*mpu_handle, MPU60xx_INT_PIN_CONFIG, &int_pin_cfg, 1);
	
	    if (ESP_OK != ret) {
	        return ret;
	    }
	    
	    int_pin_cfg &= (uint8_t)~(BIT4 | BIT5 | BIT6 | BIT7);
	
	    if (MPU60XX_INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
	        int_pin_cfg |= BIT7;
	    }
	
	    if (MPU60XX_INTERRUPT_PIN_OPEN_DRAIN == interrupt_configuration->pin_mode) {
	        int_pin_cfg |= BIT6;
	    }
	
	    if (MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED == interrupt_configuration->interrupt_latch) {
	        int_pin_cfg |= BIT5;
	    }
	
	    if (MPU60XX_INTERRUPT_CLEAR_ON_ANY_READ == interrupt_configuration->interrupt_clear_behavior) {
	        int_pin_cfg |= BIT4;
	    }
		
		cmd[0] = MPU60xx_INT_PIN_CONFIG;
		cmd[1] = int_pin_cfg;
	    ret = mpu60xx_write_register(*mpu_handle, MPU60xx_ADDR, cmd, 2);
			
		 gpio_int_type_t gpio_intr_type;
		 uint8_t pull_up=0, pull_down=0;
	
	    if (MPU60XX_INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
	        gpio_intr_type = GPIO_INTR_NEGEDGE;
	        pull_up=1;
	    } else {
	        gpio_intr_type = GPIO_INTR_POSEDGE;
	        pull_down=1;
	    }
	
	    gpio_config_t int_gpio_config = {
	        .mode = GPIO_MODE_INPUT,
	        .intr_type = gpio_intr_type,
	        .pin_bit_mask = (BIT0 << interrupt_configuration->interrupt_pin),
	        .pull_up_en = pull_up,
	        .pull_down_en=pull_down,
	    };
	
	    ret = gpio_config(&int_gpio_config);
	    
	    ret= register_isr(mpu_handle,interrupt_configuration);
	}
	
	return ret;
}


esp_err_t mpu60xx_enMotionDetectInterrupt(mpu60xx_handle_t * mpu_handle, bool active)
{
	uint8_t reg_value, cmd[2];
	
	esp_err_t ret= mpu60xx_read_register(*mpu_handle, MPU60xx_INT_EN_REG,&reg_value,1);
	if (ret !=ESP_OK) {
		ESP_LOGE(TAG,"unable to read MPU60xx_INT_EN_REG  register");
		return ret;
	}
	if (active)
		reg_value |= (BIT6);
	else
		reg_value = reg_value &  ((uint8_t)(~BIT6));
	
	cmd[0] = MPU60xx_INT_EN_REG;
	cmd[1] = reg_value;
	ret= mpu60xx_write_register(*mpu_handle,MPU60xx_ADDR,cmd, 2);

	if (ret !=ESP_OK)
		ESP_LOGE(TAG,"unable to write to MPU60xx_INT_EN_REG  register");
	
return ret;
}

static esp_err_t register_isr(mpu60xx_handle_t * sensor, mpu60xx_intrpt_config_t* interrupt_configuration)
{
    esp_err_t ret;
    
    gpio_install_isr_service(0);
    ret = gpio_isr_handler_add(
              interrupt_configuration->interrupt_pin,
              ((gpio_isr_t) * (interrupt_configuration->isr)),
              NULL
            );

    if ( ret != ESP_OK ) {
        return ret;
    }
    return ret;
}

esp_err_t mpu60xx_getMotionInterruptStatus(mpu60xx_handle_t mpu_handle, bool * status)
{
	uint8_t reg_value;
	esp_err_t ret;
	ret= mpu60xx_read_register(mpu_handle, MPU60xx_INT_STATUS,&reg_value,1);
	if (ret == ESP_OK) {
        reg_value = reg_value &  ((uint8_t)(64));
        if (reg_value)
            *status = true;
        else
            *status = false;
	}
	else
        ESP_LOGE(TAG,"unable to read MPU60xx_INT_EN_REG register");
    return ret;

}

esp_err_t mpu60xx_read_sensor_raw (mpu60xx_handle_t mpu_handle, mpu60xx_reading_raw_t * sensor_raw_read)
{
    uint8_t data[14];
    esp_err_t ret = mpu60xx_read_register(mpu_handle,MPU60xx_ACCEL_XOUTH_REG, data, 14);
    if ( ret == ESP_OK) {
        sensor_raw_read->accX_raw = ((uint16_t)data[0] << 8) | data[1];
        sensor_raw_read->accY_raw = ((uint16_t)data[2] << 8) | data[3];
        sensor_raw_read->accZ_raw = ((uint16_t)data[4] << 8) | data[5];
        
        sensor_raw_read->temp_raw = ((uint16_t)data[6] << 8) | data[7];
        
        
        sensor_raw_read->gyroX_raw  = ((uint16_t)data[8] << 8) | data[9];
        sensor_raw_read->gyroY_raw  = ((uint16_t)data[10] << 8) | data[11];
        sensor_raw_read->gyroZ_raw  = ((uint16_t)data[12] << 8) | data[13];  
    }
    else
        ESP_LOGE(TAG, "Failed to read from MPU60xx");
    
    return ret;   
}


esp_err_t mpu60xx_read_sensor(mpu60xx_handle_t mpu_handle, mpu60xx_reading_t * sensor_t)
{	
    mpu60xx_reading_raw_t  sensor_raw_read;
 	esp_err_t ret = mpu60xx_read_sensor_raw(mpu_handle,&sensor_raw_read);
    
    if ( ret == ESP_OK) {	
		sensor_t->temperature = 36.53 + ( (int16_t)sensor_raw_read.temp_raw )/340.0;
        
        mpu60xx_accel_range_t accel_range;
        ret= getAccelerometerRange(mpu_handle, &accel_range); 
        if (ret==ESP_OK) {
			 float accel_scale = 1;
			// select appropriate scale based on range    
			switch (accel_range) {
			case MPU60XX_RANGE_2_G: accel_scale = 16384;
			break;
			
			case MPU60XX_RANGE_4_G: accel_scale = 8192;
			break;
			
			case MPU60XX_RANGE_8_G: accel_scale = 4096;
			break;
			
			case MPU60XX_RANGE_16_G: accel_scale = 2048;
			break;
			}
			
			// raw values provide results in terms of 'g'(standard acceleration of gravity),
			// multiply with 's_g_value' to convert into 'm/s^2'
  			sensor_t->accX = s_g_value*((float_t)  sensor_raw_read.accX_raw) / accel_scale;
  			sensor_t->accY = s_g_value*((float_t) sensor_raw_read.accY_raw) / accel_scale;
  			sensor_t->accZ = s_g_value*((float_t) sensor_raw_read.accZ_raw) / accel_scale;
		}
        else {	
            ESP_LOGE(TAG, "Failed to get Accelerometer Range from MPU60xx");
			return ret;
		}
        
        
        mpu60xx_gyro_range_t gyro_range;
        ret= getGyrometerRange(mpu_handle, &gyro_range); 
        if (ret==ESP_OK) {
            float gyro_scale = 1;
			
			// select appropriate scale based on range      
            switch (gyro_range) {
			case MPU60XX_RANGE_250_DEG: gyro_scale = 131;
			break;
			
			case MPU60XX_RANGE_500_DEG: gyro_scale = 65.5;
			break;
			
			case MPU60XX_RANGE_1000_DEG: gyro_scale = 32.8;
			break;
			
			case MPU60XX_RANGE_2000_DEG: gyro_scale = 16.4;
			break;
			}
			
			// setup range dependant scaling
  			sensor_t->gyroX = ((float_t) sensor_raw_read.gyroX_raw) / gyro_scale;
  			sensor_t->gyroY = ((float_t) sensor_raw_read.gyroY_raw) / gyro_scale;
  			sensor_t->gyroZ = ((float_t) sensor_raw_read.gyroZ_raw) / gyro_scale;
		}
        else {
            ESP_LOGE(TAG, "Failed to get Gyrometer Range from MPU60xx");
			return ret;
		}
    }
    else
        ESP_LOGE(TAG, "Failed to read from MPU60xx");
    
    return ret;
}


esp_err_t mpu60xx_init(mpu60xx_init_config_t config_handle, mpu60xx_handle_t mpu_handle)
{
	esp_err_t ret ;
	
	ret= i2c_master_probe(*mpu_handle.bus_handle, MPU60xx_ADDR, 50);
	
	if (ret!=ESP_OK) {
		ESP_LOGE(TAG, "mpu60xx not detected on i2c");
		return ret;
	} 
	
	ESP_LOGI(TAG, "mpu60xx is available\n");

	i2c_device_config_t mpu_60xx_config = { 
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address = MPU60xx_ADDR,
	    .scl_speed_hz = config_handle.scl_speed_hz,
	    };

	ESP_LOGI(TAG, "adding mpu60xx as device to bus\n");

	ESP_ERROR_CHECK(i2c_master_bus_add_device(*mpu_handle.bus_handle, &mpu_60xx_config, mpu_handle.dev_handle));
	
	ESP_LOGI(TAG, "device added to bus\n");

	uint8_t cmd[2];
	
	//reset MPU60xx
	cmd[0] =MPU60xx_PWR_MGMT1_REG;
	cmd[1]= 0x80;
	ESP_ERROR_CHECK(mpu60xx_write_register(mpu_handle, MPU60xx_ADDR,cmd,2));
	vTaskDelay(100/portTICK_PERIOD_MS);
	
	if(config_handle.temp_sensor == true){ //enable temperature sensor, default is disabled
            cmd[0] =MPU60xx_PWR_MGMT1_REG;
            cmd[1]= 0x00;
            ESP_ERROR_CHECK(mpu60xx_write_register(mpu_handle,MPU60xx_ADDR,cmd,2));
    	}

	
	mpu60xx_setGyrometerRange(mpu_handle, config_handle.gyro_res);
	
	mpu60xx_setAccelerometerRange(mpu_handle, config_handle.accel_res);
	
	mpu60xx_setSampleRate(mpu_handle, config_handle.sample_rate, config_handle.dlpf_bw);
	
	cmd[0] =MPU60xx_INT_EN_REG;
	cmd[1]= 0x00;
	mpu60xx_write_register(mpu_handle,MPU60xx_ADDR,cmd,2);
	
	cmd[0] =MPU60xx_USER_CTRL_REG;
	cmd[1]= 0x00;
	mpu60xx_write_register(mpu_handle,MPU60xx_ADDR,cmd,2);
	
	ESP_LOGI(TAG, "mpu60xx initialization complete\n");
	return ESP_OK;
}
