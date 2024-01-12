#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"

//define the GPIO for the button function
#define RIGHT_BOTTOM_BUTTON (GPIO_NUM_38)
#define LEFT_BOTTOM_BUTTON (GPIO_NUM_36)
#define LEFT_TOP_BUTTON (GPIO_NUM_16)
#define RIGHT_TOP_BUTTON (GPIO_NUM_17)
#define BOTTOM_BUTTON (GPIO_NUM_35)
#define TOP_BUTTON (GPIO_NUM_21)
#define LEFT_BUTTON (GPIO_NUM_18)
#define RIGHT_BUTTON (GPIO_NUM_33)
#define CENTER_BUTTON (GPIO_NUM_34)

static const char *TAG = "Program";

//interrupt defines
#define ESP_INR_FLAG_DEFAULT 0

//GPIO masks
#define INPUT_PIN_SEL  ((1ULL<<RIGHT_BOTTOM_BUTTON) | (1ULL<<LEFT_BOTTOM_BUTTON) | (1ULL<<LEFT_TOP_BUTTON) | (1ULL<<RIGHT_TOP_BUTTON) | (1ULL<<BOTTOM_BUTTON) | (1ULL<<TOP_BUTTON) | (1ULL<<LEFT_BUTTON) | (1ULL<<RIGHT_BUTTON) | (1ULL<<CENTER_BUTTON))

//queue define
static QueueHandle_t gpio_evt_queue = NULL;


/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD) ),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE) )
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 0, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}

/***** Interrupt and queue handling *****/

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


/******* Functions *******/

void send_keycode(uint16_t input_keycode) {
    ESP_LOGI(TAG, "Sending Keyboard report");
    uint8_t keycode[6] = {input_keycode};
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
    vTaskDelay(pdMS_TO_TICKS(50));
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);

}

/******** Tasks ********/

static void gpio_task (void *arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, pdMS_TO_TICKS(100))) {
            //debouncing delay
            vTaskDelay(pdMS_TO_TICKS(10));
            if (gpio_get_level(io_num) == 0) {
                ESP_LOGI(TAG, "Button pressed");
                if (io_num == LEFT_BOTTOM_BUTTON) send_keycode(HID_KEY_F13);
                if (io_num == BOTTOM_BUTTON) send_keycode(HID_KEY_F14);
                if (io_num == RIGHT_BOTTOM_BUTTON) send_keycode(HID_KEY_F15);
                if (io_num == LEFT_BUTTON) send_keycode(HID_KEY_F17);
                if (io_num == CENTER_BUTTON) send_keycode(HID_KEY_F18);
                if (io_num == RIGHT_BUTTON) send_keycode(HID_KEY_F19);
                if (io_num == LEFT_TOP_BUTTON) send_keycode(HID_KEY_F20);
                if (io_num == TOP_BUTTON) send_keycode(HID_KEY_F21);
                if (io_num == RIGHT_TOP_BUTTON) send_keycode(HID_KEY_F22);
                xQueueReset(gpio_evt_queue);
            }
        }
    }

}




void app_main(void)
{
    // Initialize buttons that will trigger HID reports
    const gpio_config_t io_config = {
        .pin_bit_mask = INPUT_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
        .pull_up_en = true,
        .pull_down_en = false,
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));

    // Create a queue to handle button press events from ISR
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Start tasks
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    // Install ISR service with default configuration
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INR_FLAG_DEFAULT));

    // Attach the interrupt service routine
    ESP_ERROR_CHECK(gpio_isr_handler_add(LEFT_BOTTOM_BUTTON, gpio_isr_handler, (void*) LEFT_BOTTOM_BUTTON));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BOTTOM_BUTTON, gpio_isr_handler, (void*) BOTTOM_BUTTON));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RIGHT_BOTTOM_BUTTON, gpio_isr_handler, (void*) RIGHT_BOTTOM_BUTTON));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LEFT_BUTTON, gpio_isr_handler, (void*) LEFT_BUTTON));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CENTER_BUTTON, gpio_isr_handler, (void*) CENTER_BUTTON));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RIGHT_BUTTON, gpio_isr_handler, (void*) RIGHT_BUTTON));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LEFT_TOP_BUTTON, gpio_isr_handler, (void*) LEFT_TOP_BUTTON));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TOP_BUTTON, gpio_isr_handler, (void*) TOP_BUTTON));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RIGHT_TOP_BUTTON, gpio_isr_handler, (void*) RIGHT_TOP_BUTTON));

    /***** Config USB *****/
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}