#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define LED_ROJO   33
#define LED_VERDE  25
#define LED_AZUL   26

#define DELAY_ROJO_MS   1000
#define DELAY_VERDE_MS  2000
#define DELAY_AZUL_MS   4000

#define STACK_TAMANIO   (1024 * 2)

const char *LOG_TAG = "SecuenciaLED";

void configurarLeds(void);
void tareaSecuenciaLeds(void *param);

void app_main(void)
{
    configurarLeds();

    xTaskCreate(tareaSecuenciaLeds,
                "tareaSecuencia",
                STACK_TAMANIO,
                NULL,
                1,
                NULL);
}

void configurarLeds(void)
{
    int pines[] = {LED_ROJO, LED_VERDE, LED_AZUL};
    for (int i = 0; i < 3; i++)
    {
        gpio_reset_pin(pines[i]);
        gpio_set_direction(pines[i], GPIO_MODE_OUTPUT);
    }
}

void tareaSecuenciaLeds(void *param)
{
    while (1)
    {
        // LED ROJO
        ESP_LOGI(LOG_TAG, "LED Rojo encendido");
        gpio_set_level(LED_ROJO, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_ROJO_MS));
        gpio_set_level(LED_ROJO, 0);
        vTaskDelay(pdMS_TO_TICKS(200)); // Pequeña pausa

        // LED VERDE
        ESP_LOGW(LOG_TAG, "LED Verde encendido");
        gpio_set_level(LED_VERDE, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_VERDE_MS));
        gpio_set_level(LED_VERDE, 0);
        vTaskDelay(pdMS_TO_TICKS(200));

        // LED AZUL
        ESP_LOGE(LOG_TAG, "LED Azul encendido");
        gpio_set_level(LED_AZUL, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_AZUL_MS));
        gpio_set_level(LED_AZUL, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
