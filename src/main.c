/* 1o - Reference Libraries for code development */
// Adds references for library call (printf/scanf)
#include <stdio.h>
#include <stdbool.h>
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
//#define TRUE 1
#define LED_BOARD   GPIO_NUM_2
#define BUTTON_1    GPIO_NUM_22
#define CONTROL_LED GPIO_NUM_21

#define TIMER_DIVIDER (16)                              // hardware timer clock divider
#define TIMER_SCALE   (TIMER_BASE_CLK / TIMER_DIVIDER)  // converts counter value in seconds

/* 3o - Global variables (avoid if possible, but use with care) */
// -> uint32_t => unsigned int 32 bits - integer variable, unsigned, with 32 bits dimension
uint32_t counter = 0;

typedef struct  example_timer_info_t {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

typedef struct example_timer_event_t {
    example_timer_info_t info;
    uint64_t timer_count_value;
} example_timer_event_t;


/* 4o - Functions prototype presents in code (when not used .h a part*/

/* 5o - Method and treat implementation */

static void IRAM_ATTR gpio_isr_handler(void *arg) // IRAN_ATTR -> this function is allocated in RAM
{
    // verify if "BUTTON_1" was an interrupt source
    // 1 - How do it develop debounce for interrupt?
    if(BUTTON_1 == (uint32_t) arg) // typecast for type uint32_t - integer used in gpio definition
    {
        if(gpio_get_level((uint32_t) arg) == 0)
        {
            counter++;
        }
    }
}

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
/*      
        "Discrete Method -> of GPIO setting"
        gpio_reset_pin(LED_BOARD);
        gpio_set_direction(LED_BOARD, GPIO_MODE_OUTPUT); */

        // All that was done until here will be performed a only one time when the microcontroller start

        // Continuous perform loop - it will be perform up to we use break, or turn off chip 


//      Pin Setting Method using "gpio_config_t"
//      Firstly we set button
/*         gpio_config_t button_config = {
            .intr_type = GPIO_INTR_DISABLE, // No Interrupts
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << BUTTON_1), // ULL -> Unsigned Long Long
            .pull_down_en = false, // Enables pull-down
            .pull_up_en   = true  // Disable pull-up 
        }; */

        gpio_config_t button_config = {
            .intr_type = GPIO_INTR_NEGEDGE, // Interrupts in falling edge 1 -> 0
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << BUTTON_1), // ULL -> Unsigned Long Long
            .pull_down_en = false, // Enables pull-down
            .pull_up_en   = true  // Disable pull-up 
        };

//      GPIO Setting:
        gpio_config(&button_config);

//      Setting Led
        gpio_config_t led_config = {
            .intr_type = GPIO_INTR_DISABLE, // No Interrupts
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << LED_BOARD) | (1ULL << CONTROL_LED), // ULL -> Unsigned Long Long
            .pull_up_en   = false  // Disable pull-up 
        };
//      LED Setting:
        gpio_config(&led_config);

        gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); // Setting a interrupt treat routine of low priority
        gpio_isr_handler_add(BUTTON_1, gpio_isr_handler, (void*) BUTTON_1);

        //bool previous_button_1 = false;
        bool led_state = false; 

        while(true)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            printf("Counter Value: %d\n", counter);
            gpio_set_level(LED_BOARD, led_state);

            led_state = !led_state; // 0 -> 1 , 1 -> 0, 0 -> 1 ...
        }

/*         while(true)
        {
            vTaskDelay(30 / portTICK_PERIOD_MS);
          if((previous_Button_1 == true) && (gpio_get_level(BUTTON_1) == false))
            {
                counter++;
            }

            printf("Counter Value: %d\n", counter);
            
            //previous_Button_1 = gpio_get_level(BUTTON_1);
        }*/

/*     while(TRUE)
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
    } */
}