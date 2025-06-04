idf_component_register( 
			SRCS "srcs/mpu60xx.c" "srcs/mpu60xx_regs_bits.c" "srcs/mpu60xx_interrupts.c" "./private/mpu60xx_pvt.c"      # list the source files of this component
    			
    			INCLUDE_DIRS "include"       # optional, add here public include directories
    			
    			PRIV_INCLUDE_DIRS "./private/include"  # optional, add here private include directories
    			
    			REQUIRES "esp_driver_i2c" "esp_driver_gpio"           # optional, list the public requirements (component names)
    			
    			PRIV_REQUIRES esp_driver_gptimer       # optional, list the private requirements
    		       )
