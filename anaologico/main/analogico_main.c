/*
 * analogico — Clase 5 (Sistemas Embebidos, UCU 2026)
 *
 * El camino analógico completo, medido por la propia placa y SIN NINGUNA
 * placa de extensión ni un solo cable: en el ESP32-S2 el DAC_1 y el ADC2_CH6
 * son el MISMO pad, GPIO17. El chip se excita a sí mismo y se escucha.
 *
 *      ┌───────────┐   GPIO17    ┌───────────┐
 *      │  DAC_1    │────────●────│ ADC2_CH6  │
 *      │  8 bits   │   (un solo  │  13 / 12  │
 *      └───────────┘    pad)     └───────────┘
 *
 * Dos partes, seleccionables al compilar:
 *
 *   idf.py -DPARTE=1 build flash monitor    el EJE VERTICAL: cuantización
 *   idf.py -DPARTE=2 build flash monitor    el EJE HORIZONTAL: muestreo
 *
 * PARTE 1 — cuánto vale un número
 *   Barre los 256 códigos del DAC y lee cada uno con el ADC. Imprime la curva
 *   de transferencia real, el error de escala y de offset, el ruido de fondo
 *   y cuántos bits del ADC son de verdad. Todo con dac_oneshot y adc_oneshot:
 *   ni un DMA.
 *
 * PARTE 2 — cuándo vale ese número
 *   Genera un coseno con el generador hardware del DAC y lo muestrea con el
 *   ADC por DMA a distintas frecuencias. Mide la frecuencia recuperada y
 *   enseña el PLEGADO (aliasing) en cuanto la señal pasa de fs/2.
 *
 * Por qué el generador de coseno y no dac_continuous: en el ESP32-S2 el DMA
 * del DAC y el DMA del ADC son EL MISMO periférico (SPI3). No pueden estar
 * los dos abiertos a la vez. El generador de coseno es hardware puro dentro
 * del RTC y no gasta DMA, así que deja el SPI3 libre para el ADC.
 *
 * Que el ADC pueda leer la salida del DAC depende de una opción de
 * configuración que viene al revés por defecto: ver sdkconfig.defaults y la
 * nota en configurar_adc_oneshot().
 *
 * Ni un solo float en todo el programa: el Xtensa LX7 del ESP32-S2 no tiene
 * FPU. Ver clase-2/detalles/detalle-punto-fijo-y-float.md.
 *
 * Hardware:
 *   - ESP32-S2-Kaluga-1 sola: ninguna placa de extensión, ningún jumper.
 *   - Con la ESP-LyraT-8311A montada, GPIO17 es el LRCK del códec: quítala
 *     para esta práctica (ver README).
 *   - Target: esp32s2.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/dac_cosine.h"
#include "driver/dac_oneshot.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#ifndef PARTE
#define PARTE 1
#endif

static const char *TAG = "analogico";

/* --- El pad compartido ---------------------------------------------------
 *
 * En el ESP32-S2 cada pad RTC puede llevar varias funciones analógicas a la
 * vez porque todas cuelgan del mismo multiplexor del RTC:
 *
 *      GPIO17  =  DAC_1  =  ADC2_CH6
 *      GPIO18  =  DAC_2  =  ADC2_CH7
 *
 * Eso es lo que permite este ejemplo sin cables. El precio es una opción de
 * configuración que hay que cambiar: ver la nota en configurar_adc_oneshot(). */
#define PAD_GPIO                17
#define ADC_UNIDAD              ADC_UNIT_2
#define ADC_CANAL               ADC_CHANNEL_6
#define DAC_CANAL               DAC_CHAN_0

/* Atenuación de 12 dB, la mayor que hay. Según el datasheet del ESP32-S2
 * (tabla 5-6), el rango efectivo de medida con esta atenuación es 0 a 2300 mV
 * y por encima de 2450 mV la exactitud empeora.
 *
 * Es decir: el ADC NO llega a los 3,3 V que sí alcanza el DAC. La curva de la
 * PARTE 1 se aplana por arriba, y eso no es un fallo del programa: es que los
 * dos convertidores del mismo chip no comparten fondo de escala. Saberlo
 * antes de elegir el divisor de tensión de un sensor ahorra una tarde. */
#define ADC_ATENUACION          ADC_ATTEN_DB_12
#define ADC_FONDO_MV            2300      /* rango efectivo, no 3300 */

/* El MISMO ADC da 13 bits gobernado por el RTC (oneshot) y 12 gobernado por
 * el controlador digital (DMA). No es una errata: son dos caminos distintos
 * dentro del chip. */
#define BITS_ONESHOT            ADC_BITWIDTH_13
#define BITS_DMA                ADC_BITWIDTH_12

#define BOTON_BOOT              GPIO_NUM_0

/* ======================================================================== */
/*  Aritmética entera                                                        */
/* ======================================================================== */

/* Raíz cuadrada entera por Newton. Se usa para la desviación típica.
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

/* log2(x) en Q8 (256 = 1,0). Parte entera por la posición del bit más alto,
 * parte fraccionaria por interpolación lineal de la mantisa. El error es de
 * unas centésimas de bit, de sobra para lo que se usa aquí. */
static int32_t log2_q8(uint32_t x)
{
    if (x == 0) {
        return 0;
    }
    int entero = 31 - __builtin_clz(x);
    /* Mantisa normalizada a Q16 dentro de [1,0 ; 2,0) */
    uint32_t mantisa = (entero >= 16) ? (x >> (entero - 16)) : (x << (16 - entero));
    uint32_t frac = mantisa - 65536;                 /* 0 .. 65535 */
    return (int32_t)(entero * 256) + (int32_t)(frac >> 8);
}

/* Imprime un valor en Q8 como "n,dd" sin usar coma flotante. */
static void fmt_q8(char *dst, size_t n, int32_t q8)
{
    const char *signo = (q8 < 0) ? "-" : "";
    const int32_t a = (q8 < 0) ? -q8 : q8;
    snprintf(dst, n, "%s%" PRId32 ",%02" PRId32, signo, a / 256, ((a % 256) * 100) / 256);
}

/* ======================================================================== */
/*  Objetos comunes                                                          */
/* ======================================================================== */

static adc_cali_handle_t s_cali;      /* traduce cuentas crudas a milivoltios */

/* La calibración NO es un lujo: sin ella una cuenta cruda no significa nada.
 * Los coeficientes salen de la eFuse que Espressif graba en fábrica midiendo
 * ese chip concreto. Si el chip no los trae, el driver avisa y sigue con el
 * valor típico, que puede errar un 5 %. */
static bool configurar_calibracion(adc_bitwidth_t bits)
{
    adc_cali_line_fitting_config_t cfg = {
        .unit_id = ADC_UNIDAD,
        .atten = ADC_ATENUACION,
        .bitwidth = bits,
    };
    const esp_err_t err = adc_cali_create_scheme_line_fitting(&cfg, &s_cali);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sin calibracion de fabrica (%s): los mV son orientativos",
                 esp_err_to_name(err));
        s_cali = NULL;
        return false;
    }
    return true;
}

static int a_milivoltios(int crudo)
{
    int mv = 0;
    if (s_cali != NULL && adc_cali_raw_to_voltage(s_cali, crudo, &mv) == ESP_OK) {
        return mv;
    }
    return 0;
}

/* Espera a que se pulse BOOT, o a que venza el plazo. Devuelve true si hubo
 * pulsación. Sondeo puro y duro: aquí no hay plazos que cumplir y no merece
 * la pena una interrupción. */
static bool esperar_boot(uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += 50) {
        if (gpio_get_level(BOTON_BOOT) == 0) {
            while (gpio_get_level(BOTON_BOOT) == 0) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

static void configurar_boton(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BOTON_BOOT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

/* ======================================================================== */
#if PARTE == 1
/* ======================================================================== */
/*  PARTE 1 — el eje vertical: cuantización, escala y ruido                  */
/* ======================================================================== */

#define CODIGOS_PASO            16     /* de 256 códigos, se imprimen 17     */
#define LECTURAS_POR_CODIGO     32
#define LECTURAS_RUIDO          1024
#define CODIGO_RUIDO            128    /* medio recorrido: ~1,65 V           */
#define PROMEDIO_N              16     /* sobremuestreo del bloque final     */

static adc_oneshot_unit_handle_t s_adc;
static dac_oneshot_handle_t      s_dac;

/* EL ADC APAGA EL DAC, Y NO AVISA.
 *
 * Con la configuración por defecto de ESP-IDF, cada adc_oneshot_read() sobre
 * un canal que comparte pad con un DAC empieza por apagar ese DAC: dentro de
 * adc_oneshot_hal_setup(), s_disable_dac() llama a dac_ll_power_down(). Tiene
 * sentido si el pad lleva un sensor externo, porque un DAC encendido por
 * descuido falsearía la medida, y es lo contrario de lo que se busca aquí.
 *
 * Lo gobierna CONFIG_ADC_DISABLE_DAC_OUTPUT, que este ejemplo pone a `n` en
 * sdkconfig.defaults. Con el valor por defecto el programa compila igual y
 * arranca igual, pero mide un pad que nadie excita: la primera lectura apaga
 * el DAC y dac_oneshot_output_voltage() ya no lo vuelve a encender, porque
 * solo cambia el código de salida.
 *
 * Es el tipo de dependencia que no da error: se encuentra leyendo el driver. */
static void configurar_adc_oneshot(void)
{
    const adc_oneshot_unit_init_cfg_t unidad = {
        .unit_id = ADC_UNIDAD,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unidad, &s_adc));

    const adc_oneshot_chan_cfg_t canal = {
        .atten = ADC_ATENUACION,
        .bitwidth = BITS_ONESHOT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CANAL, &canal));
}

static void configurar_dac_oneshot(void)
{
    const dac_oneshot_config_t cfg = { .chan_id = DAC_CANAL };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&cfg, &s_dac));
}

/* Media de n lecturas del ADC. Devuelve también el mínimo y el máximo, que
 * es lo que enseña de verdad cuánto se mueve la medida. */
static int leer_media(int n, int *min, int *max)
{
    int64_t suma = 0;
    int lo = INT32_MAX, hi = 0;

    for (int i = 0; i < n; i++) {
        int crudo = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc, ADC_CANAL, &crudo));
        suma += crudo;
        if (crudo < lo) { lo = crudo; }
        if (crudo > hi) { hi = crudo; }
    }
    if (min != NULL) { *min = lo; }
    if (max != NULL) { *max = hi; }
    return (int)(suma / n);
}

static void curva_de_transferencia(void)
{
    ESP_LOGI(TAG, "--- Curva de transferencia  DAC(8 bits) -> pad -> ADC(13 bits) ---");
    ESP_LOGI(TAG, "  %-7s %-10s %-10s %-10s %s", "codigo", "V ideal", "ADC crudo",
             "V medido", "error");

    int32_t err_max = 0;

    for (int codigo = 0; codigo <= 255; codigo += CODIGOS_PASO) {
        ESP_ERROR_CHECK(dac_oneshot_output_voltage(s_dac, (uint8_t)codigo));
        /* El pad tiene capacidad parásita y el buffer del DAC no es
         * instantáneo: sin esta pausa se mide el valor ANTERIOR. */
        vTaskDelay(pdMS_TO_TICKS(5));

        const int crudo = leer_media(LECTURAS_POR_CODIGO, NULL, NULL);
        const int mv_ideal = (codigo * 3300) / 255;
        const int mv_real = a_milivoltios(crudo);
        const int32_t error = mv_real - mv_ideal;

        /* El error solo se contabiliza dentro del rango efectivo del ADC: por
         * encima de 2300 mV la medida está fuera de especificación y meterla
         * en la estadística sería mentirse. */
        if (mv_ideal <= ADC_FONDO_MV) {
            if (error > err_max) { err_max = error; }
            if (-error > err_max) { err_max = -error; }
        }

        ESP_LOGI(TAG, "  %-7d %d,%03d V    %-10d %d,%03d V    %+" PRId32 " mV %s",
                 codigo, mv_ideal / 1000, mv_ideal % 1000, crudo,
                 mv_real / 1000, mv_real % 1000, error,
                 (mv_ideal > ADC_FONDO_MV) ? " <-- fuera del rango del ADC" : "");
    }

    /* El último código: 255 no cae en el barrido de 16 en 16. */
    ESP_ERROR_CHECK(dac_oneshot_output_voltage(s_dac, 255));
    vTaskDelay(pdMS_TO_TICKS(5));
    const int crudo = leer_media(LECTURAS_POR_CODIGO, NULL, NULL);
    ESP_LOGI(TAG, "  %-7d %d,%03d V    %-10d %d,%03d V", 255, 3300 / 1000, 3300 % 1000,
             crudo, a_milivoltios(crudo) / 1000, a_milivoltios(crudo) % 1000);

    const int lsb_dac = (3300 * 10 + 128) / 256;      /* en décimas de mV */
    ESP_LOGI(TAG, "  1 LSB del DAC = 3300/256 = %d,%d mV  ·  1 LSB del ADC = %d/8192 = %d uV",
             lsb_dac / 10, lsb_dac % 10, ADC_FONDO_MV, (ADC_FONDO_MV * 1000) / 8192);
    ESP_LOGI(TAG, "  error maximo dentro del rango efectivo (0 a %d mV): %+" PRId32 " mV",
             ADC_FONDO_MV, err_max);
    ESP_LOGI(TAG, "  el error no es 'un fallo': es la suma del offset del DAC, su no");
    ESP_LOGI(TAG, "  linealidad, la carga del ADC sobre el pad y la propia calibracion");
}

/* Desviación típica de n muestras, en décimas de LSB, a partir de su suma y
 * de su suma de cuadrados:
 *
 *      n² · var  =  n · Σx²  −  (Σx)²
 *
 * La resta se hace con las sumas, antes de dividir. Dividir primero trunca la
 * media, y al elevarla al cuadrado ese resto se multiplica por 2·media: con
 * el pad en ~5000 cuentas, 0,1 LSB perdidos en la media suman ~1000 LSB² a
 * una varianza que vale unos 20. En int64 no hay desbordamiento: con 13 bits
 * y 1024 muestras el término mayor, ya multiplicado por 100, queda en 7·10^15. */
static uint32_t sigma_decimas(int64_t suma, int64_t suma2, int64_t n)
{
    const int64_t num = n * suma2 - suma * suma;
    if (num <= 0) {
        return 0;
    }
    return raiz(((uint64_t)num * 100) / (uint64_t)(n * n));
}

static void ruido_y_bits_efectivos(void)
{
    ESP_LOGI(TAG, "--- Ruido de fondo y bits efectivos (codigo %d, %d lecturas) ---",
             CODIGO_RUIDO, LECTURAS_RUIDO);

    ESP_ERROR_CHECK(dac_oneshot_output_voltage(s_dac, CODIGO_RUIDO));
    vTaskDelay(pdMS_TO_TICKS(20));

    int64_t suma = 0, suma2 = 0;
    int lo = INT32_MAX, hi = 0;

    for (int i = 0; i < LECTURAS_RUIDO; i++) {
        int crudo = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc, ADC_CANAL, &crudo));
        suma += crudo;
        suma2 += (int64_t)crudo * crudo;
        if (crudo < lo) { lo = crudo; }
        if (crudo > hi) { hi = crudo; }
    }

    const int32_t media = (int32_t)(suma / LECTURAS_RUIDO);
    const uint32_t sigma = sigma_decimas(suma, suma2, LECTURAS_RUIDO);

    /* Resolución libre de ruido: los bits que quedan cuando se descuenta el
     * ruido pico a pico, que se toma como 6,6·sigma (definición habitual).
     *      bits = log2( fondo de escala / (6,6 · sigma) )
     * Con sigma en décimas de LSB el cociente es fondo·100 / (66·sigma). Si
     * sigma sale 0, o tan pequeña que la cuenta pasa de 13, el ruido está por
     * debajo de lo que se puede medir y se cortan los bits al máximo del
     * convertidor. */
    const uint32_t fondo = 8191;
    int32_t bits = 13 * 256;
    if (sigma != 0) {
        bits = log2_q8(fondo * 100) - log2_q8(sigma * 66);
        if (bits > 13 * 256) {
            bits = 13 * 256;
        }
    }
    char buf[16];
    fmt_q8(buf, sizeof(buf), bits);

    ESP_LOGI(TAG, "  media %" PRId32 "  min %d  max %d  recorrido %d LSB  sigma %" PRIu32
                  ",%" PRIu32 " LSB",
             media, lo, hi, hi - lo, sigma / 10, sigma % 10);
    ESP_LOGI(TAG, "  resolucion libre de ruido: ~%s bits de los 13 nominales", buf);

    /* Promediar N lecturas divide sigma por raiz(N): un bit por cada
     * cuadruplicación. Es la forma más barata de comprar resolución, y se
     * paga en tiempo. */
    const int n2 = LECTURAS_RUIDO / PROMEDIO_N;
    int64_t suma_p = 0, suma2_p = 0;
    for (int i = 0; i < n2; i++) {
        const int p = leer_media(PROMEDIO_N, NULL, NULL);
        suma_p += p;
        suma2_p += (int64_t)p * p;
    }
    const uint32_t sigma_p = sigma_decimas(suma_p, suma2_p, n2);
    const uint32_t teoria = sigma / raiz(PROMEDIO_N);
    ESP_LOGI(TAG, "  promediando %d lecturas: sigma %" PRIu32 ",%" PRIu32 " LSB "
                  "(la teoria dice sigma/raiz(%d) = %" PRIu32 ",%" PRIu32 ")",
             PROMEDIO_N, sigma_p / 10, sigma_p % 10, PROMEDIO_N, teoria / 10, teoria % 10);
}

static void medir_tiempo_de_conversion(void)
{
    const int n = 1000;
    const int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < n; i++) {
        int crudo = 0;
        (void)adc_oneshot_read(s_adc, ADC_CANAL, &crudo);
    }
    const int64_t us = esp_timer_get_time() - t0;

    ESP_LOGI(TAG, "--- Cuanto cuesta una lectura ---");
    ESP_LOGI(TAG, "  %d lecturas oneshot en %" PRId64 " us  ->  %" PRId64 " us cada una",
             n, us, us / n);
    ESP_LOGI(TAG, "  a ese ritmo el techo son ~%" PRId64 " muestras/s por sondeo;",
             1000000 / (us / n));
    ESP_LOGI(TAG, "  el ADC por DMA de la PARTE 2 llega a 83 333 sin tocar la CPU");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== analogico PARTE 1: el eje vertical (cuantizacion) ===");
    ESP_LOGI(TAG, "DAC_1 y ADC2_CH6 son el mismo pad, GPIO%d: no hace falta ningun cable",
             PAD_GPIO);

#if CONFIG_ADC_DISABLE_DAC_OUTPUT
    /* El driver no avisa; el ejemplo sí. Aparece si el sdkconfig se generó
     * antes de que sdkconfig.defaults trajera la opción: borrarlo y recompilar. */
    ESP_LOGW(TAG, "CONFIG_ADC_DISABLE_DAC_OUTPUT=y: la primera lectura del ADC apagara el");
    ESP_LOGW(TAG, "DAC y se medira un pad sin excitar. Ver sdkconfig.defaults y el README");
#endif

    configurar_boton();
    configurar_adc_oneshot();
    configurar_dac_oneshot();

    if (configurar_calibracion(BITS_ONESHOT)) {
        ESP_LOGI(TAG, "calibracion de fabrica cargada: las cuentas se traducen a mV");
    }

    while (1) {
        curva_de_transferencia();
        ruido_y_bits_efectivos();
        medir_tiempo_de_conversion();
        ESP_LOGI(TAG, "--- pulsa BOOT para repetir (o espera 20 s) ---");
        esperar_boot(20000);
    }
}

/* ======================================================================== */
#elif PARTE == 2
/* ======================================================================== */
/*  PARTE 2 — el eje horizontal: muestreo, Nyquist y plegado                 */
/* ======================================================================== */

#define MUESTRAS_BLOQUE         4096       /* por medida                     */
#define FS_REFERENCIA           80000      /* fs alta: mide la f "de verdad" */
#define FS_ENSAYO_A             20000      /* pliega por encima de 10 kHz    */
#define FS_ENSAYO_B             8000       /* pliega por encima de  4 kHz    */

/* Las frecuencias del barrido, pensadas para fs = 20 000: por debajo y por
 * encima de 10 kHz, en parejas que deben plegarse al mismo sitio.
 *    9 500 y 10 500  ->  9 500      12 000 y 8 000  ->  8 000 */
static const uint32_t BARRIDO_HZ[] = {
    500, 1000, 2000, 4000, 6000, 8000, 9500, 10500, 12000, 14000, 16000, 18000, 19500
};
#define BARRIDO_N   (sizeof(BARRIDO_HZ) / sizeof(BARRIDO_HZ[0]))

static adc_continuous_handle_t s_adc;
static dac_cosine_handle_t     s_cos;
static adc_continuous_data_t  *s_muestras;

/* El generador de coseno del DAC es hardware dentro del RTC: una tabla de
 * senos y un acumulador de fase. No gasta CPU ni DMA, y por eso deja libre el
 * SPI3 que necesita el ADC. Lo malo: lo reloja el RC_FAST, un oscilador RC
 * interno de ~8 MHz con varios por ciento de error y sensible a temperatura.
 * La frecuencia que sale NO es exactamente la que se pide, y eso es
 * precisamente lo que el ejemplo mide. */
static void arrancar_coseno(uint32_t hz)
{
    if (s_cos != NULL) {
        ESP_ERROR_CHECK(dac_cosine_stop(s_cos));
        ESP_ERROR_CHECK(dac_cosine_del_channel(s_cos));
        s_cos = NULL;
    }
    const dac_cosine_config_t cfg = {
        .chan_id = DAC_CANAL,
        .freq_hz = hz,
        .clk_src = DAC_COSINE_CLK_SRC_DEFAULT,
        /* Un cuarto de amplitud: la onda queda centrada en ~1,65 V con picos
         * de ~1,2 a ~2,1 V, holgadamente dentro del rango efectivo del ADC
         * (0 a 2300 mV). A media amplitud el pico rozaría los 2,5 V, donde el
         * datasheet ya advierte de que la exactitud empeora, y a amplitud
         * completa recortaría contra los raíles de alimentación.
         *
         * Se paga con nivel: la señal sale 12 dB por debajo del fondo de
         * escala DEL DAC. La columna `nivel` la refiere al fondo del ADC, que
         * es menor, así que marcará algo menos de esos 12 dB. */
        .atten = DAC_COSINE_ATTEN_DB_12,
        .phase = DAC_COSINE_PHASE_0,
        .offset = 0,
        .flags.force_set_freq = true,
    };
    ESP_ERROR_CHECK(dac_cosine_new_channel(&cfg, &s_cos));
    ESP_ERROR_CHECK(dac_cosine_start(s_cos));
}

/* (Re)configura el ADC por DMA a la frecuencia de muestreo pedida. El handle
 * se crea una sola vez; lo que cambia entre medidas es la configuración.
 *
 * Aquí no hace falta la opción de la PARTE 1: el apagado del DAC vive en el
 * camino oneshot del driver, y el modo continuo no pasa por él. El coseno
 * sigue saliendo mientras el ADC se reconfigura. */
static void configurar_adc_dma(uint32_t fs)
{
    if (s_adc != NULL) {
        ESP_ERROR_CHECK(adc_continuous_stop(s_adc));
    } else {
        const adc_continuous_handle_cfg_t handle_cfg = {
            /* El pool interno amortigua: mientras la tarea formatea una línea
             * de log, el DMA sigue escribiendo. Si se llena, se pierden
             * muestras — y perderlas en silencio sería lo peor de todo. */
            .max_store_buf_size = MUESTRAS_BLOQUE * SOC_ADC_DIGI_RESULT_BYTES * 4,
            .conv_frame_size = MUESTRAS_BLOQUE * SOC_ADC_DIGI_RESULT_BYTES,
        };
        ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &s_adc));
    }

    adc_digi_pattern_config_t patron = {
        .atten = ADC_ATENUACION,
        .channel = ADC_CANAL,
        .unit = ADC_UNIDAD,
        .bit_width = BITS_DMA,
    };
    const adc_continuous_config_t cfg = {
        .pattern_num = 1,
        .adc_pattern = &patron,
        .sample_freq_hz = fs,
        .conv_mode = ADC_CONV_SINGLE_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_continuous_config(s_adc, &cfg));
    ESP_ERROR_CHECK(adc_continuous_start(s_adc));
}

/* Estadísticas de un bloque. Todo entero: media (la componente continua),
 * pico a pico, valor eficaz de la parte alterna y frecuencia por cruces por
 * cero con histéresis. */
typedef struct {
    uint32_t n;
    int32_t  media;
    int32_t  pico_a_pico;
    uint32_t rms;
    uint32_t hz;
} bloque_t;

static bool medir_bloque(uint32_t fs, bloque_t *out)
{
    uint32_t n = 0;
    if (adc_continuous_read_parse(s_adc, s_muestras, MUESTRAS_BLOQUE, &n, 1000) != ESP_OK
        || n < 64) {
        return false;
    }

    int64_t suma = 0;
    int32_t lo = INT32_MAX, hi = INT32_MIN;
    uint32_t validas = 0;

    for (uint32_t i = 0; i < n; i++) {
        if (!s_muestras[i].valid) {
            continue;
        }
        const int32_t v = (int32_t)s_muestras[i].raw_data;
        suma += v;
        if (v < lo) { lo = v; }
        if (v > hi) { hi = v; }
        validas++;
    }
    if (validas < 64) {
        return false;
    }

    const int32_t media = (int32_t)(suma / validas);

    /* Valor eficaz de la parte alterna: se resta la continua ANTES de elevar
     * al cuadrado. Sin ese paso lo que sale es el eficaz total, dominado por
     * los 1,65 V de reposo, y no dice nada de la señal. */
    int64_t suma2 = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (!s_muestras[i].valid) {
            continue;
        }
        const int32_t d = (int32_t)s_muestras[i].raw_data - media;
        suma2 += (int64_t)d * d;
    }

    /* Cruces por cero de la señal centrada, con histéresis de un octavo del
     * pico: sin histéresis el ruido cruza decenas de veces por ciclo y la
     * frecuencia estimada se dispara. */
    const int32_t histeresis = (hi - lo) / 8;
    int estado = 0;                 /* -1 abajo, +1 arriba, 0 sin decidir */
    uint32_t cruces = 0;

    for (uint32_t i = 0; i < n; i++) {
        if (!s_muestras[i].valid) {
            continue;
        }
        const int32_t d = (int32_t)s_muestras[i].raw_data - media;
        if (d > histeresis && estado <= 0) {
            if (estado != 0) { cruces++; }
            estado = 1;
        } else if (d < -histeresis && estado >= 0) {
            if (estado != 0) { cruces++; }
            estado = -1;
        }
    }

    /* Dos cruces por ciclo:  f = cruces · fs / (2 · N) */
    out->n = validas;
    out->media = media;
    out->pico_a_pico = hi - lo;
    out->rms = raiz((uint64_t)(suma2 / validas));
    out->hz = (uint32_t)(((uint64_t)cruces * fs) / (2ULL * validas));
    return true;
}

/* Nivel en decibelios respecto de un fondo de escala, en Q8.
 *
 *      dB = 20·log10(x) = 20·log10(2)·log2(x) = 6,0206·log2(x)
 *
 * y 6,0206 en Q8 es 1541. Es exactamente la cuenta que hará el VU-metro del
 * ejemplo `audio_vu`, aquí en pequeño: el decibelio no es más que un
 * logaritmo con un factor de escala delante. */
static int32_t db_q8(uint32_t valor, uint32_t fondo)
{
    if (valor == 0 || fondo == 0) {
        return -99 * 256;
    }
    const int32_t rel = log2_q8(valor) - log2_q8(fondo);
    return (int32_t)(((int64_t)rel * 1541) / 256);
}

/* La frecuencia que se debería ver al muestrear f_real a fs: se pliega el
 * espectro sobre fs/2 tantas veces como haga falta. */
static uint32_t plegada(uint32_t f_real, uint32_t fs)
{
    uint32_t f = f_real % fs;
    if (f > fs / 2) {
        f = fs - f;
    }
    return f;
}

static void barrido(uint32_t fs)
{
    ESP_LOGI(TAG, "--- Barrido con fs = %" PRIu32 " Hz   (Nyquist = %" PRIu32 " Hz) ---",
             fs, fs / 2);
    ESP_LOGI(TAG, "  %-9s %-9s %-9s %-9s %-8s %-9s %s",
             "f pedida", "f real", "f medida", "f teorica", "pk-pk", "nivel",
             "veredicto");

    for (size_t i = 0; i < BARRIDO_N; i++) {
        bloque_t ref = { 0 }, ens = { 0 };

        /* 1) A fs alta la señal cabe de sobra: esto es la frecuencia REAL que
         *    entrega el generador, con el error del oscilador RC incluido. */
        configurar_adc_dma(FS_REFERENCIA);
        arrancar_coseno(BARRIDO_HZ[i]);
        vTaskDelay(pdMS_TO_TICKS(20));

        (void)medir_bloque(FS_REFERENCIA, &ref);          /* descarta el primero */
        if (!medir_bloque(FS_REFERENCIA, &ref)) {
            ESP_LOGW(TAG, "  %-9" PRIu32 " sin lectura de referencia", BARRIDO_HZ[i]);
            continue;
        }

        /* 2) A fs de ensayo se ve lo que el sistema recupera de verdad. */
        configurar_adc_dma(fs);
        vTaskDelay(pdMS_TO_TICKS(20));

        (void)medir_bloque(fs, &ens);
        if (!medir_bloque(fs, &ens)) {
            ESP_LOGW(TAG, "  %-9" PRIu32 " sin lectura de ensayo", BARRIDO_HZ[i]);
            continue;
        }

        const uint32_t teorica = plegada(ref.hz, fs);
        /* Margen del 8 %: entre la estimación por cruces y la deriva del RC
         * no tiene sentido pedir coincidencia exacta. */
        const uint32_t margen = (teorica / 12) + 60;
        const bool coincide = (ens.hz + margen >= teorica) && (teorica + margen >= ens.hz);
        const bool pliega = ref.hz > fs / 2;

        /* Pico a pico en milivoltios de verdad: la calibración traduce las dos
         * puntas de la onda y se restan. Una diferencia de cuentas NO se
         * puede convertir multiplicando por un factor, porque la recta de
         * calibración tiene también término independiente. */
        const int mv_pp = a_milivoltios(ens.media + ens.pico_a_pico / 2)
                        - a_milivoltios(ens.media - ens.pico_a_pico / 2);

        /* Nivel respecto del fondo de escala. Para una senoide que ocupa todo
         * el recorrido de 12 bits, el eficaz es 4095/(2·raiz(2)) = 1448. */
        char nivel[16];
        fmt_q8(nivel, sizeof(nivel), db_q8(ens.rms, 1448));

        ESP_LOGI(TAG, "  %-9" PRIu32 " %-9" PRIu32 " %-9" PRIu32 " %-9" PRIu32
                      " %-4d mV  %-6s dB %s%s",
                 BARRIDO_HZ[i], ref.hz, ens.hz, teorica, mv_pp, nivel,
                 coincide ? "ok" : "REVISAR", pliega ? "  <-- PLEGADA" : "");
    }

    ESP_LOGI(TAG, "  por debajo de %" PRIu32 " Hz la f medida sigue a la real;",
             fs / 2);
    ESP_LOGI(TAG, "  por encima BAJA en vez de subir: eso es el plegado, y ningun");
    ESP_LOGI(TAG, "  filtro digital posterior puede deshacerlo");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== analogico PARTE 2: el eje horizontal (muestreo) ===");
    ESP_LOGI(TAG, "coseno hardware en DAC_1 (GPIO%d) muestreado por el ADC2 del mismo pad",
             PAD_GPIO);
    ESP_LOGI(TAG, "el DMA del DAC y el del ADC son el mismo SPI3: por eso el coseno");
    ESP_LOGI(TAG, "se genera con el generador del RTC, que no gasta DMA");

    configurar_boton();
    (void)configurar_calibracion(BITS_DMA);

    s_muestras = calloc(MUESTRAS_BLOQUE, sizeof(adc_continuous_data_t));
    ESP_ERROR_CHECK(s_muestras != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    while (1) {
        barrido(FS_ENSAYO_A);
        barrido(FS_ENSAYO_B);
        ESP_LOGI(TAG, "--- pulsa BOOT para repetir (o espera 20 s) ---");
        esperar_boot(20000);
    }
}

#else
#error "PARTE debe ser 1 (cuantizacion) o 2 (muestreo).  idf.py -DPARTE=2 build"
#endif
