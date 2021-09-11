/* 1o - Reference Libraries for code development */
// Adds references for library call (printf/scanf)
#include <stdio.h>
// Adds references for SDK settings
#include "sdkconfig.h"
// Adds FreeRTOS resources for treat management (using delay)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Adds resources for ESP32 system information references  
#include "esp_system.h"
// Adds resources to view flash information (SPI)
#include "esp_spi_flash.h"
// Adds  resources to manipulate input and output signals
#include "driver/gpio.h"

/* 2o - Defines for data reference to be used in program */
// Understands "define" as "nickname" - define nickname original
#define TRUE 1
#define LED_BOARD GPIO_NUM_2

/* 3o - Global variables (avoid if possible, but use with care) */
// -> uint32_t => unsigned int 32 bits - integer variable, unsigned, with 32 bits dimension
uint32_t counter = 0;

/* 4o - Functions prototype presents in code (when not used .h a part*/

/* 5o - Method and treat implementation */


void app_main(void ) // Uses the Application CPU
{
    // Shows message on terminal
    printf("Initialing Warm Up ESP32... \n");

    // Get information of our chip:
    // We create a type to store information
    esp_chip_info_t chip_info;
    // We charge information
    esp_chip_info(&chip_info);

    printf("ESP32 - %s with %d CPU Cores - WiFi %s %s \n", // %s -> string / %d-> integer
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" :  "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
            
        printf("Silicon Revision: %d \n", chip_info.revision);

        printf("Flash: %d MB %s \n", 
                (spi_flash_get_chip_size() / (1024*1024)),
                (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
        
        // Lets config the LED (GPIO 2)
        gpio_reset_pin(LED_BOARD);
        gpio_set_direction(LED_BOARD, GPIO_MODE_OUTPUT);

        // All that was done until here will be performed a only one time when the microcontroller start

        // Continuous perform loop - it will be perform up to we use break, or turn off chip 

    while(TRUE)
    {
        printf("------------------------");
        printf("Counter: %d \n", counter);

        // Turn off led?
        printf("Turning off led...");
        gpio_set_level(LED_BOARD, 0);
        vTaskDelay(1000/ portTICK_PERIOD_MS); // wait 1000 ms 

        // Turn on led?
        printf("Turning on led...");
        gpio_set_level(LED_BOARD, 1);
        vTaskDelay(1000/ portTICK_PERIOD_MS); // wait 1000 ms

        counter++; // Increments "counter" each... 2 seconds! - counter++ -> counter = counter+1 
        if(counter == 4294967295)
        {
            counter = 0;
        }
    }
}