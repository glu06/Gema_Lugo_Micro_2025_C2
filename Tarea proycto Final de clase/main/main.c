#include <stdio.h>
#include <string.h>
#include <strings.h>  
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "mqtt_client.h"

// ============ CONFIG ============
#define WIFI_SSID   "TU_SSID"
#define WIFI_PASS   "TU_PASSWORD"
#define MQTT_URI    "mqtt://broker.hivemq.com:1883"

#define TOPIC_BASE  "esp32/final"
#define TOPIC_STATE TOPIC_BASE "/state"   // JSON de estado/errores
#define TOPIC_TELE  TOPIC_BASE "/tele"    // Telemetría periódica
#define TOPIC_CMD   TOPIC_BASE "/cmd"     // Comandos

// Pines (ajustalos)
#define BTN_PP_GPIO         GPIO_NUM_4     // Botón con PULLUP (activo en 0)
#define LIM_OPEN_GPIO       GPIO_NUM_33    // Final de carrera "abierta"
#define LIM_CLOSE_GPIO      GPIO_NUM_32    // Final de carrera "cerrada"
#define MOTOR_OPEN_GPIO     GPIO_NUM_27    // Señal abrir
#define MOTOR_CLOSE_GPIO    GPIO_NUM_19    // Señal cerrar
#define LAMP_GPIO           GPIO_NUM_2     // Lámpara (LED)
#define BUZZER_LED_GPIO     GPIO_NUM_26    // Buzzer simulado (LED)
#define SENSOR_ACTIVE_LEVEL 1              // nivel activo de límites

// Temporizaciones
#define TICK_MS             100            // Timer FreeRTOS 100 ms
#define BLINK_OPEN_MS       500            // 0.5 s (cuando abre)
#define BLINK_CLOSE_MS      250            // 0.25 s (cuando cierra)
#define MOVE_TIMEOUT_MS     12000          // 12 s por seguridad
// =================================

static const char *TAG = "FINAL";

// ---- util tiempo -> ticks (base 100ms) ----
#define MS_TO_TICKS(ms)     ((ms) / TICK_MS)

// ---- GPIO helpers ----
static inline bool lim_open_active(void){  return gpio_get_level(LIM_OPEN_GPIO)  == SENSOR_ACTIVE_LEVEL; }
static inline bool lim_close_active(void){ return gpio_get_level(LIM_CLOSE_GPIO) == SENSOR_ACTIVE_LEVEL; }
static inline void motor_stop(void){ gpio_set_level(MOTOR_OPEN_GPIO,0); gpio_set_level(MOTOR_CLOSE_GPIO,0); }
static inline void motor_open_dir(void){ gpio_set_level(MOTOR_CLOSE_GPIO,0); gpio_set_level(MOTOR_OPEN_GPIO,1); }
static inline void motor_close_dir(void){ gpio_set_level(MOTOR_OPEN_GPIO,0);  gpio_set_level(MOTOR_CLOSE_GPIO,1); }
static inline void lamp_set(bool on){ gpio_set_level(LAMP_GPIO, on?1:0); }
static inline void buzzer_led(bool on){ gpio_set_level(BUZZER_LED_GPIO, on?1:0); }

// ---- Boton con anti-rebote (100ms base) ----
static bool btn_edge_pressed(void){
    static uint8_t stable=0; static int last_raw=1, deb=1, last_deb=1;
    int raw = gpio_get_level(BTN_PP_GPIO);
    if(raw==last_raw) { if(stable<2) stable++; } else { stable=0; last_raw=raw; }
    if(stable>=2){ deb=raw; stable=0; }
    bool edge = (last_deb==1 && deb==0);
    last_deb = deb;
    return edge;
}

// ---- Estados y errores ----
typedef enum { ST_UNKNOWN=0, ST_CLOSED, ST_OPENING, ST_OPEN, ST_CLOSING, ST_ERROR } pp_state_t;
typedef enum { ERR_NONE=0, ERR_LIMIT_CONFLICT=1, ERR_TIMEOUT_OPEN=2, ERR_TIMEOUT_CLOSE=3 } pp_error_t;

static volatile pp_state_t s_state = ST_UNKNOWN;
static volatile pp_error_t s_error = ERR_NONE;

// ---- MQTT ----
static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool s_mqtt_connected = false;

// ---- Tick (cola desde timer) ----
static QueueHandle_t s_tickq;

// ---- Publicacion de estado ----
static void publish_state(const char *reason){
    if(!s_mqtt_connected) return;
    char json[192];
    const char *st = (s_state==ST_UNKNOWN)?"UNKNOWN":(s_state==ST_CLOSED)?"CLOSED":
                     (s_state==ST_OPENING)?"OPENING":(s_state==ST_OPEN)?"OPEN":
                     (s_state==ST_CLOSING)?"CLOSING":"ERROR";
    snprintf(json,sizeof(json),
        "{\"state\":\"%s\",\"error\":%d,\"lim_open\":%d,\"lim_close\":%d,\"reason\":\"%s\"}",
        st, s_error, lim_open_active(), lim_close_active(), reason?reason:"");
    esp_mqtt_client_publish(s_mqtt, TOPIC_STATE, json, 0, 1, 0);
}

// ---- MQTT handler ----
static void mqtt_ev(void *h, esp_event_base_t base, int32_t id, void *data){
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;
    switch((esp_mqtt_event_id_t)id){
        case MQTT_EVENT_CONNECTED:
            s_mqtt_connected = true;
            esp_mqtt_client_subscribe(s_mqtt, TOPIC_CMD, 1);
#if CONFIG_MQTT_PROTOCOL_5
            // Publicación con propiedad v5 (opcional)
            esp_mqtt5_property_list_t *props = esp_mqtt5_property_list_init();
            if(props){
                esp_mqtt5_user_property_item_t up = { .key="origin", .value="esp32-final" };
                esp_mqtt5_property_list_add_user_property(props,&up);
                esp_mqtt_client_publish_with_properties(s_mqtt, TOPIC_STATE, "{\"boot\":\"online\"}", 0, 1, 0, props);
                esp_mqtt5_property_list_destroy(props);
            } else
#endif
            {
                esp_mqtt_client_publish(s_mqtt, TOPIC_STATE, "{\"boot\":\"online\"}", 0, 1, 0);
            }
            break;
        case MQTT_EVENT_DISCONNECTED: s_mqtt_connected=false; break;
        case MQTT_EVENT_DATA:{
            char topic[96]={0}, payload[96]={0};
            int tlen = e->topic_len < 95 ? e->topic_len : 95;
            int dlen = e->data_len  < 95 ? e->data_len  : 95;
            memcpy(topic,e->topic,tlen); topic[tlen]=0;
            memcpy(payload,e->data,dlen); payload[dlen]=0;

            if(strcmp(topic,TOPIC_CMD)==0){
                // Comandos: open | close | stop | pp | lamp:on/off | buzzer:on/off | reset | get
                if(strcasecmp(payload,"open")==0){
                    if(s_state==ST_CLOSED || s_state==ST_UNKNOWN){ s_state=ST_OPENING; publish_state("cmd:open"); }
                }else if(strcasecmp(payload,"close")==0){
                    if(s_state==ST_OPEN || s_state==ST_UNKNOWN){ s_state=ST_CLOSING; publish_state("cmd:close"); }
                }else if(strcasecmp(payload,"stop")==0){
                    motor_stop(); s_state=ST_UNKNOWN; publish_state("cmd:stop");
                }else if(strcasecmp(payload,"pp")==0){
                    // simula pulsación del botón PP
                    if(s_state==ST_CLOSED) { s_state=ST_OPENING; }
                    else if(s_state==ST_OPEN){ s_state=ST_CLOSING; }
                    else if(s_state==ST_UNKNOWN){ s_state=ST_CLOSING; }
                    publish_state("cmd:pp");
                }else if(strcasecmp(payload,"lamp:on")==0){
                    lamp_set(true);
                }else if(strcasecmp(payload,"lamp:off")==0){
                    lamp_set(false);
                }else if(strcasecmp(payload,"buzzer:on")==0){
                    buzzer_led(true);
                }else if(strcasecmp(payload,"buzzer:off")==0){
                    buzzer_led(false);
                }else if(strcasecmp(payload,"reset")==0){
                    s_error=ERR_NONE; s_state=ST_UNKNOWN; motor_stop(); lamp_set(false); buzzer_led(false); publish_state("cmd:reset");
                }else if(strcasecmp(payload,"get")==0){
                    publish_state("cmd:get");
                }
            }
            break;
        }
        default: break;
    }
}

// ---- WiFi/MQTT start ----
static void wifi_start(void){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t w = {0};
    strlcpy((char*)w.sta.ssid, WIFI_SSID, sizeof(w.sta.ssid));
    strlcpy((char*)w.sta.password, WIFI_PASS, sizeof(w.sta.password));
    w.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &w));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void mqtt_start(void){
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.client_id = "esp32-final",
        .network.disable_auto_reconnect = false,
        .network.reconnect_timeout_ms = 5000,
        .session.protocol_ver = MQTT_PROTOCOL_V_5, // si no activas v5, el driver usa 3.1.1
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_ev, NULL);
    esp_mqtt_client_start(s_mqtt);
}

// ---- Tarea FSM (tick por cola) ----
static void fsm_task(void *arg){
    // Estado inicial por límites
    if(lim_open_active() && lim_close_active()){ s_state=ST_ERROR; s_error=ERR_LIMIT_CONFLICT; }
    else if(lim_open_active()) s_state=ST_OPEN;
    else if(lim_close_active()) s_state=ST_CLOSED;
    else s_state=ST_UNKNOWN;
    publish_state("boot");

    uint32_t t_blink=0, t_timeout=0, tele_cnt=0;
    bool lamp_level=false, buzz_level=false;
    uint8_t tick;
    while(1){
        if(xQueueReceive(s_tickq,&tick,portMAX_DELAY)!=pdTRUE) continue;

        // botón físico PP
        if(btn_edge_pressed()){
            if(s_state==ST_CLOSED) s_state=ST_OPENING;
            else if(s_state==ST_OPEN) s_state=ST_CLOSING;
            else if(s_state==ST_UNKNOWN) s_state=ST_CLOSING;
            publish_state("btn");
        }

        // Telemetría cada 1 s
        if(++tele_cnt >= MS_TO_TICKS(1000)){ tele_cnt=0;
            if(s_mqtt_connected){
                char tele[96];
                snprintf(tele,sizeof(tele),"{\"lim_open\":%d,\"lim_close\":%d}", lim_open_active(), lim_close_active());
                esp_mqtt_client_publish(s_mqtt, TOPIC_TELE, tele, 0, 0, 0);
            }
        }

        // FSM
        switch(s_state){
            case ST_OPENING:
                if(lim_open_active() && lim_close_active()){ s_error=ERR_LIMIT_CONFLICT; s_state=ST_ERROR; motor_stop(); publish_state("err:limits"); break; }
                motor_open_dir();
                t_timeout++;
                if(++t_blink >= MS_TO_TICKS(BLINK_OPEN_MS)){ t_blink=0; lamp_level=!lamp_level; lamp_set(lamp_level); }
                if(lim_open_active()){ motor_stop(); s_state=ST_OPEN; t_timeout=0; lamp_set(true); publish_state("opened"); }
                else if(t_timeout >= MS_TO_TICKS(MOVE_TIMEOUT_MS)){ motor_stop(); s_error=ERR_TIMEOUT_OPEN; s_state=ST_ERROR; publish_state("err:timeout_open"); }
                break;

            case ST_CLOSING:
                if(lim_open_active() && lim_close_active()){ s_error=ERR_LIMIT_CONFLICT; s_state=ST_ERROR; motor_stop(); publish_state("err:limits"); break; }
                motor_close_dir();
                t_timeout++;
                if(++t_blink >= MS_TO_TICKS(BLINK_CLOSE_MS)){ t_blink=0; lamp_level=!lamp_level; lamp_set(lamp_level); }
                if(lim_close_active()){ motor_stop(); s_state=ST_CLOSED; t_timeout=0; lamp_set(false); publish_state("closed"); }
                else if(t_timeout >= MS_TO_TICKS(MOVE_TIMEOUT_MS)){ motor_stop(); s_error=ERR_TIMEOUT_CLOSE; s_state=ST_ERROR; publish_state("err:timeout_close"); }
                break;

            case ST_OPEN:
                motor_stop(); lamp_set(true); t_blink=0; t_timeout=0; break;
            case ST_CLOSED:
                motor_stop(); lamp_set(false); t_blink=0; t_timeout=0; break;
            case ST_UNKNOWN:
                motor_stop(); lamp_set(false); t_blink=0; t_timeout=0; break;

            case ST_ERROR:
                motor_stop();
                // parpadeo buzzer LED 100ms para señalar error (y lámpara 0.5s)
                if(++t_blink >= MS_TO_TICKS(500)){ t_blink=0; lamp_level=!lamp_level; lamp_set(lamp_level); }
                buzz_level = !buzz_level; buzzer_led(buzz_level);
                break;
        }
    }
}

// ---- Timer FreeRTOS 100ms ----
static TimerHandle_t s_timer;
static void timer_cb(TimerHandle_t x){
    uint8_t one=1;
    if(s_tickq) xQueueSend(s_tickq,&one,0);
}

void app_main(void){
    ESP_ERROR_CHECK(nvs_flash_init());

    // GPIO dirección
    gpio_config_t out = {
        .pin_bit_mask = (1ULL<<MOTOR_OPEN_GPIO)|(1ULL<<MOTOR_CLOSE_GPIO)|
                        (1ULL<<LAMP_GPIO)|(1ULL<<BUZZER_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT, .pull_up_en=0, .pull_down_en=0, .intr_type=GPIO_INTR_DISABLE
    };
    gpio_config(&out);
    motor_stop(); lamp_set(false); buzzer_led(false);

    gpio_config_t in_btn = { .pin_bit_mask=1ULL<<BTN_PP_GPIO, .mode=GPIO_MODE_INPUT,
                             .pull_up_en=1, .pull_down_en=0, .intr_type=GPIO_INTR_DISABLE };
    gpio_config(&in_btn);
    gpio_config_t in_lim = { .pin_bit_mask=(1ULL<<LIM_OPEN_GPIO)|(1ULL<<LIM_CLOSE_GPIO),
                             .mode=GPIO_MODE_INPUT, .pull_up_en=1, .pull_down_en=0, .intr_type=GPIO_INTR_DISABLE };
    gpio_config(&in_lim);

    // WiFi + MQTT
    wifi_start();
    mqtt_start();

    // Cola y timer 100ms
    s_tickq = xQueueCreate(32,sizeof(uint8_t));
    s_timer = xTimerCreate("tick100", pdMS_TO_TICKS(TICK_MS), pdTRUE, NULL, timer_cb);
    xTimerStart(s_timer, 0);

    // FSM
    xTaskCreate(fsm_task, "fsm", 4096, NULL, 6, NULL);

    ESP_LOGI(TAG, "Proyecto final listo. Topics: %s (state), %s (tele), %s (cmd).", TOPIC_STATE, TOPIC_TELE, TOPIC_CMD);
}
