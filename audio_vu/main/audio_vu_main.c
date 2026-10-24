/*
 * audio_vu — Clase 5 (Sistemas Embebidos, UCU 2026)
 *
 * Base de la App 5. El micrófono de la ESP-LyraT-8311A entra por el códec
 * ES8311, viaja por I2S con DMA hasta la SRAM y se convierte en un nivel en
 * decibelios que se pinta en la TFT, en el LED RGB y en la consola.
 *
 *   micro ──▶ ES8311 ──I2S/DMA──▶ [captura p8] ──cola──▶ [proceso p6] ──┐
 *              (I2C: control)      RMS, pico,          balística         │ mutex
 *                                  recortes           VU / PPM           ▼
 *                                                                 ┌────────────┐
 *   [tactil p9] ──notificación────────────────────────────────────▶│  estado    │
 *    cola de la ISR del táctil                                     │ compartido │
 *                                                                 └────────────┘
 *                                                                        │ mutex
 *   [interfaz p3] ◀──────────────────────────────────────────────────────┘
 *    TFT + LED RGB + panel por consola
 *
 *   [timer software] ──▶ informe de tareas y de audio cada 10 s
 *
 * Lo que hay que ver:
 *   - el audio NO se puede retrasar: el DMA sigue llenando descriptores mires
 *     o no, y lo que no se recoge a tiempo se pierde. La tarea `captura` no
 *     hace más que leer, medir y encolar;
 *   - todo lo lento (formatear, dibujar, escribir por la UART) ocurre en la
 *     tarea de menor prioridad, y NUNCA con el mutex tomado;
 *   - el decibelio se calcula en enteros: el Xtensa LX7 no tiene FPU;
 *   - la pantalla se refresca por trozos, no entera: dibujar 320x240 en cada
 *     bloque costaría más que capturarlo.
 *
 * Hardware (ver README para los microinterruptores):
 *   - Kaluga-1 v1.2. En la v1.3 cambia un pin: ver I2S_DIN más abajo.
 *   - ESP-LyraT-8311A en el Extension Header  (micrófono + códec ES8311).
 *   - ESP-LyraP-TouchA en el Touch FPC Connector (botones).
 *   - ESP-LyraP-LCD32 en el conector de LCD    (opcional: -DPANTALLA=1 o 2).
 *   - LED RGB en GPIO45, con el JUMPER RGB colocado.
 *   - Target: esp32s2.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/touch_sens.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "pantalla.h"

/* ======================================================================== */
/*  Configuración                                                            */
/* ======================================================================== */

/* --- Audio: ESP-LyraT-8311A sobre Kaluga-1 --------------------------------
 *
 * Los pines son los mismos en las dos revisiones del kit, menos I2S_DIN, la
 * línea por la que el códec entrega el micrófono:
 *
 *      Kaluga-1 v1.2   GPIO_NUM_46
 *      Kaluga-1 v1.3   GPIO_NUM_34
 *
 * La revisión está impresa en la placa principal. Con el pin de la otra
 * revisión el programa arranca y no da ningún error, pero no recibe audio:
 * el nivel se queda fijo en -90 dBFS.
 *
 * En la v1.2 ese GPIO46 es además un pin de arranque: el ESP32-S2 solo entra
 * en modo de descarga si lo encuentra en bajo. Con la placa de audio montada
 * el códec lo deja en alto y el flasheo falla con "Wrong boot mode detected".
 * Espressif indica flashear con la placa de audio desmontada. */
#define I2S_PUERTO              I2S_NUM_0
#define I2S_MCLK                GPIO_NUM_35
#define I2S_BCLK                GPIO_NUM_18
#define I2S_WS                  GPIO_NUM_17
#define I2S_DOUT                GPIO_NUM_12   /* ESP -> códec (altavoz)     */
#define I2S_DIN                 GPIO_NUM_46   /* códec -> ESP (micrófono)   */
#define I2C_PUERTO              I2C_NUM_0
#define I2C_SCL                 GPIO_NUM_7
#define I2C_SDA                 GPIO_NUM_8
#define PA_ENABLE               GPIO_NUM_10   /* amplificador del altavoz   */

/* 16 kHz es el ritmo de la voz: cubre hasta 8 kHz, que es donde vive casi
 * toda la inteligibilidad. Duplicarlo duplica exactamente todo: bytes por
 * segundo, tamaño de los buffers y trabajo de la CPU. */
#define FRECUENCIA_HZ           16000
#define MCLK_MULTIPLO           384           /* el ES8311 lo pide así      */

/* 512 tramas = 32 ms de audio por bloque. Es el compromiso central del
 * ejemplo: bloques cortos refrescan más y responden antes, pero multiplican
 * los despertares de la tarea y los cambios de contexto. */
#define BLOQUE_TRAMAS           512
#define CANALES                 2             /* ranuras I2S; el ES8311 es mono */
#define BLOQUE_BYTES            (BLOQUE_TRAMAS * CANALES * (int)sizeof(int16_t))

#define GANANCIA_MIN_DB         0
#define GANANCIA_MAX_DB         36
#define GANANCIA_PASO_DB        6
#define GANANCIA_INICIAL_DB     24

/* --- LED RGB ------------------------------------------------------------- */
#define RGB_LED_GPIO            45
#define RGB_LED_COUNT           1

/* --- Prioridades: por PLAZO, no por importancia (rate-monotonic) ---------
 *
 *   tactil    responde a un dedo:   ~100 ms y se nota
 *   captura   plazo DURO de 32 ms:  si llega tarde, el DMA pisa el bloque
 *   proceso   el plazo de captura
 *   interfaz  refresco visual:      si se retrasa 100 ms, nadie lo ve
 *
 * `captura` va por debajo de `tactil` a propósito: la tarea del táctil hace
 * cuatro cosas y se bloquea; nunca puede robarle 32 ms a nadie. */
#define PRIO_TACTIL             9
#define PRIO_CAPTURA            8
#define PRIO_PROCESO            6
#define PRIO_INTERFAZ           3

#define PILA_TACTIL             4096          /* habla con el códec por I2C */
#define PILA_CAPTURA            4096
#define PILA_PROCESO            3072
#define PILA_INTERFAZ           4096

#define COLA_NIVELES_LARGO      8
#define COLA_TOQUES_LARGO       16
#define INTERFAZ_PERIODO_MS     100           /* plazo sin audio y ritmo de la consola */
#define INFORME_PERIODO_MS      10000

/* --- Calibración del táctil (igual que en las Clases 2, 3 y 4) ----------- */
#define BARRIDOS_CALIBRACION    3
#define UMBRAL_RATIO            0.02f
#define UMBRAL_INICIAL          2000

static const char *TAG = "audio_vu";

/* ======================================================================== */
/*  Aritmética entera:  el decibelio sin coma flotante                       */
/* ======================================================================== */

/* Raíz cuadrada entera por Newton. Se usa para el valor eficaz de cada
 * bloque.
 *
 * Partiendo de r = x, la sucesión baja hasta la parte entera de la raíz. Se
 * para cuando deja de bajar, no cuando deja de cambiar: si x + 1 es un
 * cuadrado perfecto (3, 8, 15, 24...) la iteración entera no se estabiliza,
 * alterna para siempre entre la raíz y el entero siguiente. */
static uint32_t raiz(uint64_t x)
{
    if (x == 0) {
        return 0;
    }
    uint64_t r = x;
    while (1) {
        const uint64_t sig = (r + x / r) / 2;
        if (sig >= r) {
            return (uint32_t)r;
        }
        r = sig;
    }
}

/* log2(x) en Q8: parte entera por la posición del bit más alto, fracción por
 * interpolación lineal de la mantisa. Error de unas centésimas de bit. */
static int32_t log2_q8(uint32_t x)
{
    if (x == 0) {
        return 0;
    }
    const int entero = 31 - __builtin_clz(x);
    const uint32_t mantisa = (entero >= 16) ? (x >> (entero - 16)) : (x << (16 - entero));
    return (int32_t)(entero * 256) + (int32_t)((mantisa - 65536) >> 8);
}

/* Nivel en dB respecto de un fondo de escala, en Q8.
 *
 *      dB = 20·log10(x) = 20·log10(2)·log2(x) = 6,0206·log2(x)
 *
 * y 6,0206 en Q8 es 1541. Eso es TODO lo que hay detrás de un decibelio: un
 * logaritmo y un factor de escala. El suelo se corta en -90 dB porque por
 * debajo solo hay ruido de cuantización del propio convertidor. */
#define DB_SUELO_Q8   (-90 * 256)

static int32_t db_q8(uint32_t valor, uint32_t fondo)
{
    if (valor == 0 || fondo == 0) {
        return DB_SUELO_Q8;
    }
    const int32_t rel = log2_q8(valor) - log2_q8(fondo);
    const int32_t db = (int32_t)(((int64_t)rel * 1541) / 256);
    return (db < DB_SUELO_Q8) ? DB_SUELO_Q8 : db;
}

static void fmt_q8(char *dst, size_t n, int32_t q8)
{
    const char *signo = (q8 < 0) ? "-" : "";
    const int32_t a = (q8 < 0) ? -q8 : q8;
    snprintf(dst, n, "%s%" PRId32 ".%01" PRId32, signo, a / 256, ((a % 256) * 10) / 256);
}

/* ======================================================================== */
/*  Tipos                                                                    */
/* ======================================================================== */

/* Lo que `captura` mide de un bloque y entrega a `proceso`. Viaja POR COPIA:
 * 24 bytes en la cola en vez de los 2048 bytes del bloque de audio. Copiar el
 * audio entero por la cola sería el error de diseño clásico.
 *
 * El orden de los campos cuenta: en Xtensa un int64_t se alinea a 8 bytes.
 * Con la marca de tiempo entre los campos pequeños el compilador mete 4 bytes
 * de relleno delante y otros 4 al final, y la estructura mide 32. */
typedef struct {
    int64_t  marca_us;        /* el campo de 8 bytes, primero              */
    uint32_t rms;             /* eficaz de la parte alterna, 0..32767      */
    uint32_t pico;            /* máximo absoluto del bloque                */
    uint32_t medida_us;       /* lo que costó medir el bloque              */
    uint16_t recortes;        /* muestras contra el fondo de escala        */
    uint16_t muestras;
} nivel_t;

_Static_assert(sizeof(nivel_t) == 24, "nivel_t ya no mide 24 bytes: revisar el orden de los campos");

typedef struct {
    int     canal;
    int64_t marca_us;
} toque_t;

/* Balística: cómo sube y baja la aguja. No es cosmética, es la diferencia
 * entre un VU y un PPM, y cambia lo que el instrumento sirve para medir. */
typedef enum {
    MODO_VU = 0,       /* lento en los dos sentidos: mide sensación sonora */
    MODO_PPM,          /* ataque instantáneo, caída lenta: caza los picos  */
    MODO_CRUDO,        /* sin suavizar: se ve el ruido de la medida        */
    MODO_N
} modo_t;

static const char *NOMBRE_MODO[MODO_N] = { "VU", "PPM", "CRUDO" };

typedef struct {
    int32_t  db_q8;           /* nivel mostrado, ya con balística          */
    int32_t  pico_db_q8;      /* retención de pico                         */
    uint32_t recortes;        /* acumulado desde el último RECORD          */
    uint32_t bloques;
    uint32_t perdidos;        /* bloques que no cupieron en la cola        */
    int32_t  ganancia_db;
    modo_t   modo;
} estado_t;

/* ======================================================================== */
/*  Objetos del kernel y periféricos                                         */
/* ======================================================================== */

static QueueHandle_t     s_cola_niveles;
static QueueHandle_t     s_cola_toques;
static SemaphoreHandle_t s_mutex_estado;
static TimerHandle_t     s_timer_informe;
static TaskHandle_t      s_tarea_interfaz;

static estado_t s_estado = {
    .db_q8 = DB_SUELO_Q8,
    .pico_db_q8 = DB_SUELO_Q8,
    .ganancia_db = GANANCIA_INICIAL_DB,
};

static i2s_chan_handle_t      s_i2s_rx;
static i2s_chan_handle_t      s_i2s_tx;       /* en marcha y en silencio: App 5 */
static esp_codec_dev_handle_t s_codec;
static led_strip_handle_t     s_led;
static touch_sensor_handle_t  s_touch;

/* Diagnóstico. Cada contador lo escribe una sola tarea. */
static uint32_t s_medida_max_us;     /* peor caso de medir un bloque        */
static uint32_t s_dibujo_max_us;     /* peor caso de refrescar la pantalla  */
static uint32_t s_lecturas_cortas;   /* i2s_channel_read devolvió de menos  */

/* --- Botones táctiles ----------------------------------------------------
 *
 * Con la placa de audio montada NO se usan PHOTO (T6) ni NETWORK (T11): el
 * GPIO6 es el BT_ADC de la 8311A y la retroiluminación de la pantalla, y T11
 * es el chip select de la pantalla. */
typedef struct {
    int canal;
    const char *nombre;
} boton_t;

static const boton_t BOTONES[] = {
    { 2, "PLAY"     },   /* cambia la balística: VU -> PPM -> CRUDO       */
    { 1, "VOL_UP"   },   /* sube la ganancia del micrófono                */
    { 3, "VOL_DOWN" },   /* la baja                                       */
    { 5, "RECORD"   },   /* reinicia el pico retenido y los recortes      */
};
#define BOTONES_N   (sizeof(BOTONES) / sizeof(BOTONES[0]))

static touch_channel_handle_t s_canales[BOTONES_N];

/* ======================================================================== */
/*  LED RGB                                                                  */
/* ======================================================================== */

static void led(uint8_t r, uint8_t g, uint8_t b)
{
    ESP_ERROR_CHECK(led_strip_set_pixel(s_led, 0, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(s_led));
}

static void configurar_led(void)
{
    const led_strip_config_t strip = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = RGB_LED_COUNT,
    };
    const led_strip_rmt_config_t rmt = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip, &rmt, &s_led));
    ESP_ERROR_CHECK(led_strip_clear(s_led));
}

/* ======================================================================== */
/*  I2S y códec ES8311                                                       */
/* ======================================================================== */

/* El I2S mueve las muestras; el I2C configura el códec. Son dos buses
 * distintos con dos trabajos distintos, y confundirlos es el malentendido más
 * común al empezar con audio: por el I2S no se manda "sube el volumen". */
static void configurar_i2s(void)
{
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PUERTO, I2S_ROLE_MASTER);
    /* Con auto_clear, un tx al que nadie escribe manda ceros: silencio. */
    chan.auto_clear = true;
    /* Se crean los dos canales aunque hoy solo se lea de rx. En el ESP32-S2
     * tx y rx comparten BCLK y WS, y los genera el canal que se inicializa
     * PRIMERO; el otro queda de esclavo interno. Tiene que ser el tx: es lo
     * que da por hecho esp_codec_dev, y así la App 5 reproduce escribiendo en
     * un canal que ya está en marcha, sin tocar esta función. */
    ESP_ERROR_CHECK(i2s_new_channel(&chan, &s_i2s_tx, &s_i2s_rx));

    i2s_std_config_t std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(FRECUENCIA_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                       I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_DOUT,
            .din = I2S_DIN,
        },
    };
    /* Configurado así, el ES8311 no se conforma con BCLK: saca todos sus
     * relojes internos de un reloj maestro que tiene que ser un múltiplo
     * exacto de la frecuencia de muestreo, y el mismo que se le dice a él. */
    std.clk_cfg.mclk_multiple = MCLK_MULTIPLO;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_tx, &std));   /* maestro */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_rx, &std));
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_tx));
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_rx));
}

static void configurar_codec(void)
{
    i2c_master_bus_handle_t bus = NULL;
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PUERTO,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    /* Los constructores de esp_codec_dev toman punteros no constantes: se
     * quedan con la configuración, no la copian. Marcarla const aquí sería
     * mentir sobre el contrato. */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_PUERTO,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = bus,
    };
    const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_ERROR_CHECK(ctrl != NULL ? ESP_OK : ESP_ERR_NOT_FOUND);

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_PUERTO,
        .rx_handle = s_i2s_rx,
        .tx_handle = s_i2s_tx,
    };
    const audio_codec_data_if_t *datos = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_ERROR_CHECK(datos != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    const audio_codec_gpio_if_t *gpio = audio_codec_new_gpio();
    ESP_ERROR_CHECK(gpio != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    es8311_codec_cfg_t es_cfg = {
        .ctrl_if = ctrl,
        .gpio_if = gpio,
        /* Solo ADC: hoy nada más se graba. La App 5 lo cambia a
         * ESP_CODEC_DEV_WORK_MODE_BOTH para poder reproducir. */
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_ADC,
        .master_mode = false,        /* manda el ESP32-S2, obedece el códec */
        .use_mclk = true,
        .pa_pin = PA_ENABLE,
        .pa_reverted = false,
        .hw_gain = {
            .pa_voltage = 5.0,
            .codec_dac_voltage = 3.3,
        },
        .mclk_div = MCLK_MULTIPLO,
    };
    const audio_codec_if_t *es8311 = es8311_codec_new(&es_cfg);
    ESP_ERROR_CHECK(es8311 != NULL ? ESP_OK : ESP_ERR_NOT_FOUND);

    esp_codec_dev_cfg_t dev = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = es8311,
        .data_if = datos,
    };
    s_codec = esp_codec_dev_new(&dev);
    ESP_ERROR_CHECK(s_codec != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    esp_codec_dev_sample_info_t muestreo = {
        .bits_per_sample = 16,
        .channel = CANALES,
        .channel_mask = 0x03,
        .sample_rate = FRECUENCIA_HZ,
        /* Al abrir el dispositivo, esp_codec_dev RECONFIGURA el reloj del I2S
         * con lo que diga esta estructura. Sin este campo usaría el múltiplo
         * por defecto, 256, y pisaría el 384 de configurar_i2s(): saldrían
         * 4,096 MHz por MCLK mientras el ES8311 hace sus cuentas para 6,144.
         * Arranca, no da error y el audio sale mal. */
        .mclk_multiple = MCLK_MULTIPLO,
    };
    ESP_ERROR_CHECK(esp_codec_dev_open(s_codec, &muestreo)
                        == ESP_CODEC_DEV_OK ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(esp_codec_dev_set_in_gain(s_codec, (float)GANANCIA_INICIAL_DB)
                        == ESP_CODEC_DEV_OK ? ESP_OK : ESP_FAIL);
}

/* ======================================================================== */
/*  ISR del táctil                                                           */
/* ======================================================================== */

/* Contexto de interrupción: SOLO encola. El patrón de la Clase 2, ya
 * automático a estas alturas del curso. */
static bool IRAM_ATTR al_tocar(touch_sensor_handle_t sensor,
                               const touch_active_event_data_t *ev,
                               void *ctx)
{
    BaseType_t hp_task_woken = pdFALSE;
    const toque_t t = { .canal = ev->chan_id, .marca_us = esp_timer_get_time() };

    xQueueSendFromISR(s_cola_toques, &t, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

/* ======================================================================== */
/*  TAREA 1 — captura:  el único plazo duro del programa                     */
/* ======================================================================== */

/* Prioridad 8. Se bloquea en i2s_channel_read() hasta que el DMA tiene un
 * bloque completo, lo mide y encola el RESULTADO. No formatea, no dibuja y no
 * imprime: cada microsegundo que pasa aquí es un microsegundo que el DMA está
 * llenando el siguiente descriptor sin nadie que lo recoja. */
static void tarea_captura(void *arg)
{
    /* El buffer de audio vive en el heap, no en la pila: 2 KB en la pila de
     * una tarea es la mitad de lo que tiene. */
    int16_t *bloque = heap_caps_malloc(BLOQUE_BYTES, MALLOC_CAP_DEFAULT);
    ESP_ERROR_CHECK(bloque != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_LOGI(TAG, "[captura] %d tramas por bloque = %d ms a %d Hz",
             BLOQUE_TRAMAS, (BLOQUE_TRAMAS * 1000) / FRECUENCIA_HZ, FRECUENCIA_HZ);

    while (1) {
        size_t leidos = 0;
        if (i2s_channel_read(s_i2s_rx, bloque, BLOQUE_BYTES, &leidos, 1000) != ESP_OK) {
            ESP_LOGW(TAG, "[captura] lectura I2S fallida");
            continue;
        }
        const int64_t t0 = esp_timer_get_time();
        const int tramas = (int)(leidos / (CANALES * sizeof(int16_t)));
        if (tramas < BLOQUE_TRAMAS) {
            s_lecturas_cortas++;
        }

        /* El ES8311 es MONO: su único convertidor sale en la ranura izquierda
         * de la trama estéreo. En la derecha esp_codec_dev deja la referencia
         * del DAC, es decir, lo que se esté reproduciendo: hoy, silencio. Se
         * toma una de cada dos muestras; contar también la otra ranura
         * falsearía el nivel. */
        int32_t suma = 0;
        for (int i = 0; i < tramas; i++) {
            suma += bloque[i * CANALES];
        }
        const int32_t continua = (tramas > 0) ? (suma / tramas) : 0;

        /* Eficaz de la parte alterna y pico, en la misma pasada. La continua
         * se resta ANTES de elevar al cuadrado: un micrófono con offset daría
         * un nivel alto en silencio absoluto. */
        int64_t suma2 = 0;
        uint32_t pico = 0;
        uint16_t recortes = 0;

        for (int i = 0; i < tramas; i++) {
            const int32_t s = bloque[i * CANALES] - continua;
            suma2 += (int64_t)s * s;
            const uint32_t abs_s = (uint32_t)((s < 0) ? -s : s);
            if (abs_s > pico) {
                pico = abs_s;
            }
            /* Recorte: la muestra tocó el tope del convertidor. Lo que se
             * perdió ahí NO se recupera subiendo o bajando nada después. */
            if (abs_s >= 32000) {
                recortes++;
            }
        }

        const nivel_t n = {
            .rms = (tramas > 0) ? raiz((uint64_t)(suma2 / tramas)) : 0,
            .pico = pico,
            .recortes = recortes,
            .muestras = (uint16_t)tramas,
            .marca_us = t0,
            .medida_us = (uint32_t)(esp_timer_get_time() - t0),
        };
        if (n.medida_us > s_medida_max_us) {
            s_medida_max_us = n.medida_us;
        }

        /* Sin bloquearse: si `proceso` se ha quedado atrás, se tira la medida
         * y se cuenta. Bloquear aquí retrasaría la siguiente lectura de I2S y
         * la pérdida sería de AUDIO, que es mucho peor que perder una medida
         * de nivel. La política se elige, y se justifica. */
        if (xQueueSend(s_cola_niveles, &n, 0) != pdTRUE) {
            xSemaphoreTake(s_mutex_estado, portMAX_DELAY);
            s_estado.perdidos++;
            xSemaphoreGive(s_mutex_estado);
        }
    }
}

/* ======================================================================== */
/*  TAREA 2 — proceso:  la balística del instrumento                         */
/* ======================================================================== */

/* Prioridad 6. Convierte el eficaz en decibelios y le aplica la balística:
 * cuánto tarda la aguja en subir (ataque) y en bajar (caída). Un VU real
 * integra unos 300 ms; un PPM sube al instante y cae despacio. */
static void tarea_proceso(void *arg)
{
    nivel_t n;

    while (xQueueReceive(s_cola_niveles, &n, portMAX_DELAY) == pdTRUE) {
        const int32_t db = db_q8(n.rms, 32768);
        const int32_t db_pico = db_q8(n.pico, 32768);

        xSemaphoreTake(s_mutex_estado, portMAX_DELAY);
        modo_t modo = s_estado.modo;
        int32_t actual = s_estado.db_q8;
        xSemaphoreGive(s_mutex_estado);

        /* Filtro de un polo, en enteros: nuevo = actual + (objetivo-actual)/k.
         * Cuanto mayor k, más lento. Con bloques de 32 ms, k=8 son ~250 ms de
         * constante de tiempo, que es justo la balística de un VU. */
        switch (modo) {
        case MODO_VU:
            actual += (db - actual) / 8;
            break;
        case MODO_PPM:
            /* Ataque instantáneo, caída lenta: así no se escapa ningún pico,
             * que es exactamente para lo que sirve un medidor de picos. */
            actual = (db > actual) ? db : actual + (db - actual) / 24;
            break;
        case MODO_CRUDO:
        default:
            actual = db;
            break;
        }

        xSemaphoreTake(s_mutex_estado, portMAX_DELAY);
        s_estado.db_q8 = actual;
        if (db_pico > s_estado.pico_db_q8) {
            s_estado.pico_db_q8 = db_pico;
        }
        s_estado.recortes += n.recortes;
        s_estado.bloques++;
        xSemaphoreGive(s_mutex_estado);

        /* Un emisor, un receptor: la notificación directa es lo más barato
         * del kernel y no necesita ningún objeto. */
        xTaskNotifyGive(s_tarea_interfaz);
    }
}

/* ======================================================================== */
/*  TAREA 3 — tactil                                                         */
/* ======================================================================== */

static void aplicar_ganancia(int32_t db)
{
    /* La API del códec pide un float. Es la única coma flotante del programa
     * y está en el borde, donde no cuesta nada: la conversión ocurre una vez
     * por pulsación, no una vez por muestra. */
    if (esp_codec_dev_set_in_gain(s_codec, (float)db) != ESP_CODEC_DEV_OK) {
        ESP_LOGW(TAG, "[tactil] el codec rechazo la ganancia %" PRId32 " dB", db);
    }
}

static void tarea_tactil(void *arg)
{
    toque_t ev;

    while (xQueueReceive(s_cola_toques, &ev, portMAX_DELAY) == pdTRUE) {
        const int64_t latencia = esp_timer_get_time() - ev.marca_us;
        const char *nombre = "?";
        for (size_t i = 0; i < BOTONES_N; i++) {
            if (BOTONES[i].canal == ev.canal) {
                nombre = BOTONES[i].nombre;
            }
        }

        int32_t ganancia_nueva = -1;

        xSemaphoreTake(s_mutex_estado, portMAX_DELAY);
        if (ev.canal == BOTONES[0].canal) {                    /* PLAY */
            s_estado.modo = (s_estado.modo + 1) % MODO_N;
        } else if (ev.canal == BOTONES[1].canal) {             /* VOL_UP */
            if (s_estado.ganancia_db < GANANCIA_MAX_DB) {
                s_estado.ganancia_db += GANANCIA_PASO_DB;
                ganancia_nueva = s_estado.ganancia_db;
            }
        } else if (ev.canal == BOTONES[2].canal) {             /* VOL_DOWN */
            if (s_estado.ganancia_db > GANANCIA_MIN_DB) {
                s_estado.ganancia_db -= GANANCIA_PASO_DB;
                ganancia_nueva = s_estado.ganancia_db;
            }
        } else if (ev.canal == BOTONES[3].canal) {             /* RECORD */
            s_estado.pico_db_q8 = DB_SUELO_Q8;
            s_estado.recortes = 0;
        }
        const estado_t e = s_estado;
        xSemaphoreGive(s_mutex_estado);

        /* Fuera del mutex: hablar por I2C con el códec tarda cientos de
         * microsegundos y no puede bloquear a las tareas de audio. */
        if (ganancia_nueva >= 0) {
            aplicar_ganancia(ganancia_nueva);
        }

        ESP_LOGI(TAG, "[tactil] %s (T%d) -> modo %s  ganancia %" PRId32 " dB "
                      "| latencia ISR->tarea %" PRId64 " us",
                 nombre, ev.canal, NOMBRE_MODO[e.modo], e.ganancia_db, latencia);
    }
}

/* ======================================================================== */
/*  TAREA 4 — interfaz:  pantalla, LED y consola                             */
/* ======================================================================== */

/* La escala del instrumento: de -60 dBFS (casi silencio) a 0 dBFS (tope). */
#define ESCALA_MIN_DB     (-60)
#define ESCALA_MAX_DB     0

#define BARRA_X           16
#define BARRA_Y           110
#define BARRA_ANCHO       288
#define BARRA_ALTO        54
#define MARCA_ANCHO       3             /* la marca de pico retenido */

static uint16_t COLOR_FONDO, COLOR_TINTA, COLOR_TENUE;
static uint16_t COLOR_VERDE, COLOR_AMBAR, COLOR_ROJO, COLOR_HUECO;

/* Traduce dB a píxeles de barra, recortando por los dos extremos. */
static int db_a_pixeles(int32_t db_q8)
{
    const int32_t min_q8 = ESCALA_MIN_DB * 256;
    if (db_q8 <= min_q8) {
        return 0;
    }
    if (db_q8 >= ESCALA_MAX_DB * 256) {
        return BARRA_ANCHO;
    }
    return (int)(((int64_t)(db_q8 - min_q8) * BARRA_ANCHO)
                 / ((ESCALA_MAX_DB - ESCALA_MIN_DB) * 256));
}

static uint16_t color_por_nivel(int32_t db_q8)
{
    if (db_q8 >= -3 * 256)  { return COLOR_ROJO; }
    if (db_q8 >= -12 * 256) { return COLOR_AMBAR; }
    return COLOR_VERDE;
}

/* Lo que no cambia nunca se dibuja UNA vez. Repintar el fondo, el título y la
 * escala en cada refresco costaría 150 KB de SPI treinta veces por segundo:
 * más ancho de banda que el propio audio. */
static void dibujar_fondo_fijo(void)
{
    pantalla_limpiar(COLOR_FONDO);
    pantalla_texto(16, 14, "VU  ES8311  16 KHZ", 2, COLOR_TINTA, COLOR_FONDO);
    pantalla_rect(BARRA_X - 2, BARRA_Y - 2, BARRA_ANCHO + 4, BARRA_ALTO + 4, COLOR_TENUE);
    pantalla_rect(BARRA_X, BARRA_Y, BARRA_ANCHO, BARRA_ALTO, COLOR_HUECO);

    /* Marcas de escala cada 12 dB, con su número debajo. */
    for (int db = ESCALA_MIN_DB; db <= ESCALA_MAX_DB; db += 12) {
        const int x = BARRA_X + db_a_pixeles(db * 256);
        pantalla_rect(x, BARRA_Y + BARRA_ALTO + 4, 2, 6, COLOR_TENUE);
        char etiqueta[8];
        snprintf(etiqueta, sizeof(etiqueta), "%d", db);
        pantalla_texto(x - pantalla_ancho_texto(etiqueta, 1) / 2,
                       BARRA_Y + BARRA_ALTO + 12, etiqueta, 1,
                       COLOR_TENUE, COLOR_FONDO);
    }
}

/* Devuelve a la franja [x0, x1) de la barra el color que le toca: relleno por
 * debajo de `ancho`, hueco por encima. Es lo que queda al quitar la marca de
 * pico de donde estaba. */
static void restaurar_franja(int x0, int x1, int ancho, uint16_t color)
{
    const int corte = (ancho < x0) ? x0 : (ancho > x1) ? x1 : ancho;

    pantalla_rect(BARRA_X + x0, BARRA_Y, corte - x0, BARRA_ALTO, color);
    pantalla_rect(BARRA_X + corte, BARRA_Y, x1 - corte, BARRA_ALTO, COLOR_HUECO);
}

/* Refresco incremental: solo se manda a la pantalla lo que ha cambiado desde
 * el refresco anterior. El estado anterior son las cinco variables `static`:
 * ese es todo el coste de no repintar 150 KB en cada bloque. */
static void dibujar_nivel(const estado_t *e)
{
    static int ancho_previo = 0;          /* el fondo fijo deja la barra vacía */
    static int marca_previa = -1;
    static uint16_t color_previo;
    static char texto_previo[24] = "";
    static char pie_previo[40] = "";

    const int64_t t0 = esp_timer_get_time();
    const int ancho = db_a_pixeles(e->db_q8);
    const uint16_t color = color_por_nivel(e->db_q8);
    const bool cambio_de_zona = (color != color_previo);

    /* La barra: lo que crece se pinta y lo que mengua se borra. Si además ha
     * cambiado de zona (de verde a ámbar, por ejemplo), hay que repintar
     * también lo que ya estaba. */
    const int desde = cambio_de_zona ? 0 : ancho_previo;
    if (ancho > desde) {
        pantalla_rect(BARRA_X + desde, BARRA_Y, ancho - desde, BARRA_ALTO, color);
    }
    if (ancho < ancho_previo) {
        pantalla_rect(BARRA_X + ancho, BARRA_Y, ancho_previo - ancho, BARRA_ALTO,
                      COLOR_HUECO);
    }

    /* La marca de pico: tres píxeles, siempre dentro de la barra. Si se ha
     * movido, antes se restaura lo que tapaba. Se repinta en cada refresco
     * porque la barra puede haberle pasado por encima. */
    int marca = db_a_pixeles(e->pico_db_q8) - 1;
    if (marca > BARRA_ANCHO - MARCA_ANCHO) {
        marca = BARRA_ANCHO - MARCA_ANCHO;
    }
    if (marca_previa >= 0 && marca != marca_previa) {
        restaurar_franja(marca_previa, marca_previa + MARCA_ANCHO, ancho, color);
    }
    if (marca >= 0) {
        pantalla_rect(BARRA_X + marca, BARRA_Y, MARCA_ANCHO, BARRA_ALTO, COLOR_TINTA);
    }
    marca_previa = marca;

    /* El número grande solo se redibuja cuando cambia el texto o su color. Es
     * de ancho fijo, nueve caracteres: cada carácter se pinta con su fondo, así
     * que el texto nuevo tapa entero al anterior y no hay que borrar antes. */
    char db[16], linea[24];
    fmt_q8(db, sizeof(db), e->db_q8);
    snprintf(linea, sizeof(linea), "%6s DB", db);
    if (cambio_de_zona || strcmp(linea, texto_previo) != 0) {
        pantalla_texto(16, 48, linea, 4, color, COLOR_FONDO);
        snprintf(texto_previo, sizeof(texto_previo), "%s", linea);
    }

    /* El pie, igual: ancho fijo, y solo cuando cambia. 25 caracteres de 12
     * píxeles son todo lo que cabe en la línea. */
    char pie[40];
    snprintf(pie, sizeof(pie), "%-5s  GAIN %2" PRId32 "  CLIP %-4" PRIu32,
             NOMBRE_MODO[e->modo], e->ganancia_db,
             (e->recortes > 9999) ? 9999 : e->recortes);
    if (strcmp(pie, pie_previo) != 0) {
        pantalla_texto(16, 208, pie, 2, COLOR_TENUE, COLOR_FONDO);
        snprintf(pie_previo, sizeof(pie_previo), "%s", pie);
    }

    ancho_previo = ancho;
    color_previo = color;

    /* pantalla_rect() y pantalla_texto() no vuelven hasta que el SPI termina,
     * así que esto es lo que la pantalla ha tardado de verdad. */
    const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    if (us > s_dibujo_max_us) {
        s_dibujo_max_us = us;
    }
}

/* Barra de texto para el monitor serie: el mismo instrumento, sin pantalla. */
static void barra_de_texto(char *dst, size_t n, int32_t db_q8)
{
    const size_t ancho = 32;
    const int llenos = (int)((size_t)db_a_pixeles(db_q8) * ancho / BARRA_ANCHO);

    size_t i = 0;
    dst[i++] = '[';
    for (size_t k = 0; k < ancho && i + 2 < n; k++) {
        dst[i++] = ((int)k < llenos) ? '#' : ' ';
    }
    dst[i++] = ']';
    dst[i] = '\0';
}

/* Prioridad 3, la más baja: dibujar es lo único que puede esperar. Despierta
 * por notificación (hay nivel nuevo) o por plazo, lo que llegue antes. */
static void tarea_interfaz(void *arg)
{
    char db[16], pico[16], barra[40];
    int64_t ultima_linea_us = 0;

    if (pantalla_disponible()) {
        dibujar_fondo_fijo();
    }

    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(INTERFAZ_PERIODO_MS));

        /* Copia bajo mutex, trabajo lento fuera. Retener el mutex mientras se
         * habla con el SPI de la pantalla bloquearía a la tarea de audio. */
        xSemaphoreTake(s_mutex_estado, portMAX_DELAY);
        const estado_t e = s_estado;
        xSemaphoreGive(s_mutex_estado);

        if (e.bloques == 0) {
            continue;
        }

        /* El LED como semáforo de nivel: verde bien, ámbar cerca del tope,
         * rojo recortando. Es la misma información que la barra, para quien
         * mira la placa y no la pantalla. */
        if (e.db_q8 >= -3 * 256) {
            led(40, 0, 0);
        } else if (e.db_q8 >= -12 * 256) {
            led(30, 18, 0);
        } else if (e.db_q8 >= -45 * 256) {
            led(0, 26, 0);
        } else {
            led(0, 0, 10);
        }

        if (pantalla_disponible()) {
            dibujar_nivel(&e);
        }

        /* La pantalla y el LED siguen al audio bloque a bloque. La consola
         * no: treinta líneas por segundo no las lee nadie, y a 115200 baudios
         * se llevarían un tercio de la UART. */
        const int64_t ahora = esp_timer_get_time();
        if (ahora - ultima_linea_us < INTERFAZ_PERIODO_MS * 1000) {
            continue;
        }
        ultima_linea_us = ahora;

        fmt_q8(db, sizeof(db), e.db_q8);
        fmt_q8(pico, sizeof(pico), e.pico_db_q8);
        barra_de_texto(barra, sizeof(barra), e.db_q8);
        ESP_LOGI(TAG, "[interfaz] %-6s dBFS %s pico %-6s | modo %-5s gan %" PRId32
                      " dB  recortes %" PRIu32,
                 db, barra, pico, NOMBRE_MODO[e.modo], e.ganancia_db, e.recortes);
    }
}

/* ======================================================================== */
/*  Informe periódico                                                        */
/* ======================================================================== */

static const char *nombre_estado(eTaskState s)
{
    switch (s) {
    case eRunning:   return "EJECUTANDO";
    case eReady:     return "LISTA";
    case eBlocked:   return "BLOQUEADA";
    case eSuspended: return "SUSPENDIDA";
    case eDeleted:   return "BORRADA";
    default:         return "?";
    }
}

static void informe(TimerHandle_t t)
{
    static const char *TAREAS[] = { "tactil", "captura", "proceso", "interfaz" };

    xSemaphoreTake(s_mutex_estado, portMAX_DELAY);
    const estado_t e = s_estado;
    xSemaphoreGive(s_mutex_estado);

    ESP_LOGI(TAG, "--- Informe (cada %d s) ---------------------------------",
             INFORME_PERIODO_MS / 1000);
    ESP_LOGI(TAG, "  %-10s %-4s %-11s %-8s", "tarea", "prio", "estado", "pila libre");
    for (size_t i = 0; i < sizeof(TAREAS) / sizeof(TAREAS[0]); i++) {
        const TaskHandle_t h = xTaskGetHandle(TAREAS[i]);
        if (h == NULL) {
            continue;
        }
        ESP_LOGI(TAG, "  %-10s %-4u %-11s %" PRIu32 " B", TAREAS[i],
                 (unsigned)uxTaskPriorityGet(h), nombre_estado(eTaskGetState(h)),
                 (uint32_t)uxTaskGetStackHighWaterMark(h));
    }

    /* El presupuesto de tiempo: cuánto del bloque de 32 ms se gasta midiendo.
     * Si se acercara al 100 %, no habría CPU para nada más y empezarían a
     * perderse muestras. */
    const uint32_t bloque_us = (BLOQUE_TRAMAS * 1000000u) / FRECUENCIA_HZ;
    ESP_LOGI(TAG, "  audio: %d Hz x 16 bits mono = %d B/s  ·  bloque %" PRIu32 " us",
             FRECUENCIA_HZ, FRECUENCIA_HZ * 2, bloque_us);
    ESP_LOGI(TAG, "  medir un bloque: %" PRIu32 " us como mucho (%" PRIu32 " %% del bloque)",
             s_medida_max_us, (s_medida_max_us * 100) / bloque_us);
    ESP_LOGI(TAG, "  refrescar la pantalla: %" PRIu32 " us como mucho  (%s)",
             s_dibujo_max_us, pantalla_controlador());
    ESP_LOGI(TAG, "  bloques %" PRIu32 "  perdidos %" PRIu32 "  lecturas cortas %" PRIu32
                  "  cola %u/%d",
             e.bloques, e.perdidos, s_lecturas_cortas,
             (unsigned)uxQueueMessagesWaiting(s_cola_niveles), COLA_NIVELES_LARGO);
    /* Con la PSRAM habilitada, esp_get_free_heap_size() suma las dos
     * memorias y no dice nada de la que escasea. Se informan por separado. */
    ESP_LOGI(TAG, "  SRAM interna libre: %" PRIu32 " B  (minimo historico %" PRIu32 " B)",
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "  PSRAM libre: %" PRIu32 " B",
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

/* ======================================================================== */
/*  Táctil:  configuración y calibración                                     */
/* ======================================================================== */

static touch_channel_config_t config_canal_por_defecto(void)
{
    touch_channel_config_t cfg = {
        .active_thresh = { UMBRAL_INICIAL },
        .charge_speed = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
    };
    return cfg;
}

static void calibrar_umbrales(void)
{
    ESP_LOGI(TAG, "Calibrando... NO toques la placa durante el arranque");

    ESP_ERROR_CHECK(touch_sensor_enable(s_touch));
    for (int i = 0; i < BARRIDOS_CALIBRACION; i++) {
        ESP_ERROR_CHECK(touch_sensor_trigger_oneshot_scanning(s_touch, 2000));
    }
    ESP_ERROR_CHECK(touch_sensor_disable(s_touch));

    for (size_t i = 0; i < BOTONES_N; i++) {
        uint32_t benchmark[TOUCH_SAMPLE_CFG_NUM] = { 0 };
        ESP_ERROR_CHECK(touch_channel_read_data(s_canales[i],
                                                TOUCH_CHAN_DATA_TYPE_BENCHMARK,
                                                benchmark));
        touch_channel_config_t cfg = config_canal_por_defecto();
        cfg.active_thresh[0] = (uint32_t)(benchmark[0] * UMBRAL_RATIO);
        ESP_ERROR_CHECK(touch_sensor_reconfig_channel(s_canales[i], &cfg));

        ESP_LOGI(TAG, "  %-8s T%-2d  benchmark=%" PRIu32 "  umbral=%" PRIu32,
                 BOTONES[i].nombre, BOTONES[i].canal, benchmark[0],
                 cfg.active_thresh[0]);
    }
}

static void configurar_tactil(void)
{
    touch_sensor_sample_config_t muestreo[TOUCH_SAMPLE_CFG_NUM] = {
        TOUCH_SENSOR_V2_DEFAULT_SAMPLE_CONFIG(500, TOUCH_VOLT_LIM_L_0V5,
                                                   TOUCH_VOLT_LIM_H_2V2),
    };
    touch_sensor_config_t sens_cfg =
        TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(TOUCH_SAMPLE_CFG_NUM, muestreo);
    ESP_ERROR_CHECK(touch_sensor_new_controller(&sens_cfg, &s_touch));

    touch_channel_config_t chan_cfg = config_canal_por_defecto();
    for (size_t i = 0; i < BOTONES_N; i++) {
        ESP_ERROR_CHECK(touch_sensor_new_channel(s_touch, BOTONES[i].canal,
                                                 &chan_cfg, &s_canales[i]));
    }

    touch_sensor_filter_config_t filtro = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    ESP_ERROR_CHECK(touch_sensor_config_filter(s_touch, &filtro));

    calibrar_umbrales();

    touch_event_callbacks_t callbacks = { .on_active = al_tocar };
    ESP_ERROR_CHECK(touch_sensor_register_callbacks(s_touch, &callbacks, NULL));

    ESP_ERROR_CHECK(touch_sensor_enable(s_touch));
    ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(s_touch));
}

/* ======================================================================== */
/*  main                                                                     */
/* ======================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "=== audio_vu: microfono, codec ES8311 y VU-metro ===");
    ESP_LOGI(TAG, "PLAY = balistica VU/PPM/CRUDO | VOL_UP y VOL_DOWN = ganancia | "
                  "RECORD = reinicia pico");

    COLOR_FONDO = pantalla_color(10, 16, 30);
    COLOR_TINTA = pantalla_color(230, 236, 245);
    COLOR_TENUE = pantalla_color(110, 128, 155);
    COLOR_HUECO = pantalla_color(26, 34, 52);
    COLOR_VERDE = pantalla_color(56, 190, 110);
    COLOR_AMBAR = pantalla_color(230, 170, 40);
    COLOR_ROJO  = pantalla_color(226, 70, 60);

    configurar_led();
    led(30, 16, 0);                       /* ámbar: arrancando */

    if (pantalla_iniciar()) {
        ESP_LOGI(TAG, "pantalla %s activa", pantalla_controlador());
    } else {
        ESP_LOGI(TAG, "sin pantalla (%s): el VU sale por consola y por el LED",
                 pantalla_controlador());
    }

    /* Objetos del kernel ANTES que las tareas y que la interrupción. */
    s_cola_niveles = xQueueCreate(COLA_NIVELES_LARGO, sizeof(nivel_t));
    s_cola_toques  = xQueueCreate(COLA_TOQUES_LARGO, sizeof(toque_t));
    s_mutex_estado = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK((s_cola_niveles && s_cola_toques && s_mutex_estado)
                        ? ESP_OK : ESP_ERR_NO_MEM);

    configurar_i2s();
    configurar_codec();
    configurar_tactil();

    ESP_LOGI(TAG, "codec listo: %d Hz, 16 bits, ganancia de entrada %d dB",
             FRECUENCIA_HZ, GANANCIA_INICIAL_DB);

    /* `interfaz` se crea la primera porque `proceso` le manda notificaciones,
     * y notificar a un handle NULL es un assert y un reinicio. Las cuatro
     * tareas tienen más prioridad que app_main (1): cada una empieza a
     * ejecutarse en cuanto se crea, antes de que app_main pase a la línea
     * siguiente. Y `captura` no espera al primer bloque, porque el DMA está en
     * marcha desde que se abrió el códec y ya tiene audio acumulado. En
     * `multitarea` (Clase 4) este orden lo ponía un semáforo de arranque;
     * aquí no lo hay, y lo pone la secuencia de xTaskCreate(). */
    xTaskCreate(tarea_interfaz, "interfaz", PILA_INTERFAZ, NULL, PRIO_INTERFAZ,
                &s_tarea_interfaz);
    xTaskCreate(tarea_tactil,   "tactil",   PILA_TACTIL,   NULL, PRIO_TACTIL,   NULL);
    xTaskCreate(tarea_captura,  "captura",  PILA_CAPTURA,  NULL, PRIO_CAPTURA,  NULL);
    xTaskCreate(tarea_proceso,  "proceso",  PILA_PROCESO,  NULL, PRIO_PROCESO,  NULL);

    s_timer_informe = xTimerCreate("informe", pdMS_TO_TICKS(INFORME_PERIODO_MS),
                                   pdTRUE, NULL, informe);
    ESP_ERROR_CHECK(s_timer_informe != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTimerStart(s_timer_informe, pdMS_TO_TICKS(100)) == pdPASS
                        ? ESP_OK : ESP_FAIL);

    ESP_LOGI(TAG, "Arranque completo. app_main termina; el planificador sigue.");
}
