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
// Adding timer driver library of Espressif for ESP32 
#include "driver/timer.h"

/* 2o - Defines for data reference to be used in program */
// Understands "define" as "nickname" - define nickname original
//#define TRUE 1
#define LED_BOARD   GPIO_NUM_2
#define BUTTON_1    GPIO_NUM_22
#define CONTROL_LED GPIO_NUM_21

#define TIMER_DIVIDER (16)                              // hardware timer clock divider
#define TIMER_SCALE   (TIMER_BASE_CLK / TIMER_DIVIDER)  // converts counter value in seconds
// timer_base_clk is 80 Mhz by default 

/* 3o - Global variables (avoid if possible, but use with care) */
// -> uint32_t => unsigned int 32 bits - integer variable, unsigned, with 32 bits dimension
uint32_t counter = 0;
bool control_led = false;

// Data reference structure of timer
typedef struct  exemplo_timer_info_t {
    int timer_group;
    int timer_idx;
    int alarm_interval; // 
    bool auto_reload;
} exemplo_timer_info_t;

// Reference structure for timer events ("optional")
typedef struct exemplo_timer_event_t {
    exemplo_timer_info_t info;
    uint64_t timer_counter_value;
} exemplo_timer_event_t;

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

/*
* A simpler helper function to print the raw timer counter value
* and the counter value converted to seconds
*
*/

static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\r\n", (uint32_t)(counter_value >> 32),
                                      (uint32_t)(counter_value));

    printf("Time : %.8f s\r\n", (double) counter_value / TIMER_SCALE);
}

// Callback ofr time interrupt treatment
static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    exemplo_timer_info_t *info = (exemplo_timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    // Load time event values = we can use this later! Per hour is just reference
    exemplo_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval, 
        .timer_counter_value = timer_counter_value
    };

    // If we don't have auto_reload set
    if(!info->auto_reload)
    {
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    // Blink led in interrupt occurrence -- ACTION  in certain time (or amount of events) desired
    gpio_set_direction(CONTROL_LED, control_led);
    control_led = !control_led; // toggle led state to the next times

    /* Now just send the event data back to the main program task */
    // xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

/**
 * @brief Initialize selected timer of timer group
 * 
 * @param group Timer Group number, index from 0
 * @param auto_reload whether auto_reload on alarm event
 * @param timer_interval_sec interval of alarm 
*/

static void example_tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB

    timer_init(group, timer, &config);

    /*
        Timer's counter will initially start from values below.
        Also, if auto_reload is set, this value will be automatically reload on alarm 
    */
   timer_set_counter_value(group, timer, 0);

   /* Configure the alarm value and the interrupt on alarm. */
   timer_set_alarm_value(group, timer, timer_interval_sec * TIMER_SCALE);
   timer_enable_intr(group, timer);

   exemplo_timer_info_t *timer_info = calloc(1, sizeof(exemplo_timer_info_t));
   timer_info->timer_group = group;
   timer_info->timer_idx = timer;
   timer_info->auto_reload = auto_reload;
   timer_info->alarm_interval = timer_interval_sec;
   timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);

   timer_start(group, timer);
}

// code main  execution  routine of timer
void app_main() // Uses the Application CPU
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

        example_tg_timer_init(TIMER_GROUP_0, TIMER_0, true, 3);

        /*
        timer_config_t config = {
            .divider = TIMER_DIVIDER,      // scale factor - divider by 16
            .counter_dir = TIMER_COUNT_UP, // timer as rising counter
            .counter_en = TIMER_PAUSE,     // timer starts stop
            .alarm_en = TIMER_ALARM_EN,    // timer as alarm (a.k.a interrupts)
            .auto_reload = true            // we want auto-reload ! For while
        };
        timer_init(TIMER_GROUP_0, TIMER_0, &config);

        uint32_t intervalo_em_segundos = 5;

        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0); // starts counting in 0
        timer_enable_intr(TIMER_GROUP_0, TIMER_0);          // enables timer interrupt

        // calloc - allocates memory for a elements vector and initializes everything in 0
        esp_timer_info_t *timer_info = calloc(1, sizeof(esp_timer_event_t));
        timer_info->timer_group = TIMER_GROUP_0;
        timer_info->timer_idx = TIMER_0;
        timer_info->auto_reload = true;

        // 5 seconds to start alarms and interrup generation
        timer_info->alarm_interval = intervalo_em_segundos;
        timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, timer_info, 0);

        // At moment EVERYTHING was perfumary and setup ritual
        // From this exactly moment, TIMER starts to count independent of CPU, and generate interrupts so on
        // As configured
        timer_start(TIMER_GROUP_0, TIMER_0);

        //bool previous_button_1 = false;
        */
        bool led_state = false; 

        while(true)
        {
            exemplo_timer_event_t evt;
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