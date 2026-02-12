idf_component_register(SRCS "cytrons.c" 
			    "encoders.c" 
			    "limit_switches.c" 
			    "uart.c" 
			    "current_sensors.c" 
			    "servos.c" 
                    INCLUDE_DIRS "include"
		    REQUIRES driver)
