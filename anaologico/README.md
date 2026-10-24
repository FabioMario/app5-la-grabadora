| Supported Targets | ESP32-S2 |
| ----------------- | -------- |

# analogico — el camino analógico, medido por la propia placa (Clase 5)

El **ADC** y el **DAC** internos del ESP32-S2, con la placa **sola**: sin ninguna placa de extensión, sin jumpers y **sin un solo cable**. En el ESP32-S2 el `DAC_1` y el `ADC2_CH6` son el **mismo pad, GPIO17**, así que el chip puede excitarse a sí mismo y escucharse.

```
      ┌───────────┐   GPIO17    ┌───────────┐
      │   DAC_1   │────────●────│ ADC2_CH6  │
      │  8 bits   │  (un solo   │  13 / 12  │
      └───────────┘    pad)     └───────────┘
```

El ejemplo tiene **dos partes**, que se eligen al compilar, porque son dos preguntas distintas sobre el mismo convertidor:

```bash
idf.py -DPARTE=1 build flash monitor      # el eje VERTICAL: cuánto vale un número
idf.py -DPARTE=2 build flash monitor      # el eje HORIZONTAL: cuándo vale ese número
```

## PARTE 1 — cuantización, escala, ruido

Barre los 256 códigos del DAC y lee cada uno con el ADC. Imprime la curva de transferencia real, el error frente al valor ideal, el ruido de fondo y **cuántos de los 13 bits son de verdad**. Todo con `dac_oneshot` y `adc_oneshot`: ni un DMA por ningún lado.

Lo que se aprende:

| Observación | Por qué pasa |
|---|---|
| La curva **se aplana** por encima de ~2,3 V | El ADC del ESP32-S2 con 12 dB de atenuación mide **0 a 2300 mV**, no 3,3 V (datasheet, tabla 5-6). El DAC sí llega a 3,3 V: **los dos convertidores del mismo chip no comparten fondo de escala** |
| Hay **error incluso en el centro** de la escala | Es la suma del offset del DAC, su no linealidad, la carga que el ADC pone sobre el pad y la propia calibración |
| La `sigma` del ruido vale **varios LSB** | El datasheet admite hasta ±7 LSB de no linealidad diferencial: los 13 bits nominales **no son 13 bits útiles** |
| Promediar 16 lecturas divide la `sigma` **por 4** | El ruido baja con la raíz del número de muestras: dos bits más por cada dieciséis lecturas, pagados en tiempo |

## PARTE 2 — muestreo, Nyquist y plegado

Genera un coseno con el **generador hardware del DAC** y lo muestrea con el **ADC por DMA** a distintas frecuencias. Para cada frecuencia hace **dos** medidas: una a 80 kHz, que da la frecuencia *real* que está saliendo, y otra a la frecuencia de ensayo, que da la que el sistema *recupera*. Cuando la segunda deja de seguir a la primera, eso es el plegado.

```
I (5312) analogico: --- Barrido con fs = 20000 Hz   (Nyquist = 10000 Hz) ---
I (5313) analogico:   f pedida  f real    f medida  f teorica pk-pk    nivel     veredicto
I (5460) analogico:   9500      9472      9463      9472      812  mV  -9.6   dB ok
I (5610) analogico:   10500     10488     9512      9512      810  mV  -9.6   dB ok
I (5761) analogico:   12000     11974     8026      8026      808  mV  -9.7   dB ok  <-- PLEGADA
I (5915) analogico:   18000     17952     2048      2048      804  mV  -9.8   dB ok  <-- PLEGADA
```

> Las parejas son lo importante: **9500 y 10500 miden lo mismo**, y **12000 mide lo que mediría 8000**. Una vez muestreada, la señal de 12 kHz y la de 8 kHz son **el mismo juego de números**: no hay filtro digital, ni transformada, ni algoritmo que las separe después.

### Por qué el generador de coseno y no `dac_continuous`

Porque en el ESP32-S2 **el DMA del DAC y el DMA del ADC son el mismo periférico**: los dos reclaman el **SPI3**.

```c
/* esp_driver_dac/esp32s2/dac_dma.c */   #define DAC_DMA_PERIPH_SPI_HOST  SPI3_HOST
/* esp_adc/esp32s2/adc_dma.c        */   #define ADC_DMA_SPI_HOST         SPI3_HOST
```

El segundo que intente reservarlo falla. El **generador de coseno** (`dac_cosine`) es hardware puro dentro del RTC —una tabla de senos y un acumulador de fase—, no gasta DMA y deja el SPI3 entero para el ADC. Es la razón por la que este ejemplo puede hacer lo que hace.

El precio: al generador lo reloja el **RC_FAST**, un oscilador RC interno de unos 8 MHz con varios por ciento de error. La frecuencia que sale **no** es exactamente la que se pide, y por eso el programa la mide en vez de creérsela. Un oscilador RC no es un cristal, y eso se nota en cuanto algo depende de la frecuencia absoluta.

## El ADC apaga el DAC, y no avisa

Con la configuración por defecto de ESP-IDF, cada `adc_oneshot_read()` sobre un canal que comparte pad con un DAC **empieza por apagar ese DAC**. Está en el HAL del ADC, dentro de `adc_oneshot_hal_setup()`:

```c
/* esp_hal_ana_conv/adc_oneshot_hal.c */
if (hal->disable_dac_output) {
    s_disable_dac(hal, chan);        /* ADC2_CH6  ->  dac_ll_power_down(DAC_CHAN_0) */
}
```

Tiene sentido cuando el pad lleva un sensor externo: un DAC encendido por descuido falsearía la medida. Para medir el propio DAC es lo contrario de lo que hace falta, y por eso el `sdkconfig.defaults` de este ejemplo trae:

```
CONFIG_ADC_DISABLE_DAC_OUTPUT=n
```

La ayuda de la opción lo dice tal cual: *«Disable this option so as to measure the output of DAC by internal ADC, for test usage»*. Sin esa línea el programa compila, arranca y mide un pad que nadie excita: la primera lectura apaga el DAC, y `dac_oneshot_output_voltage()` no lo vuelve a encender porque solo cambia el código de salida. Sin ningún mensaje de error del driver, sin ningún `assert`, y con números que parecen los de un sensor mal conectado.

> El apagado vive en el camino *oneshot*. El modo continuo de la PARTE 2 no pasa por él: ahí el coseno sigue saliendo con la opción en cualquier valor.

> `sdkconfig.defaults` solo se aplica al **crear** el `sdkconfig`. Si ya tenías uno de antes, bórralo (o `idf.py fullclean`) y vuelve a compilar; el ejemplo avisa por consola al arrancar si la opción sigue activa.

## Hardware

- **ESP32-S2-Kaluga-1** y nada más. Sin placas de extensión, sin jumper RGB, sin cables.
- Si tienes la **ESP-LyraT-8311A** montada, **quítala** para esta práctica: el GPIO17 es el LRCK del códec y el GPIO18 su SCLK. No rompe nada, pero el pad queda cargado con la entrada del ES8311 y las medidas salen desplazadas.
- Target ESP-IDF: **`esp32s2`**.

## Estructura

```
analogico/
├── CMakeLists.txt
├── sdkconfig.defaults        Watchdog de IDLE, canario de pila y ADC_DISABLE_DAC_OUTPUT
├── main/
│   ├── CMakeLists.txt        Define PARTE=n
│   └── analogico_main.c
└── README.md   (este archivo)
```

## Cómo compilar, flashear y monitorear

```bash
idf.py set-target esp32s2
idf.py -DPARTE=1 build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

- Sustituye `/dev/cu.usbserial-XXXX` por tu puerto (`ls /dev/cu.*` en macOS).
- Conecta el cable al puerto **UART** de la placa.
- Salir del monitor: `Ctrl + ]`.
- **Pulsa BOOT** para repetir la medida sin esperar los 20 segundos.

> Al cambiar de `PARTE` hay que **volver a compilar**: es una decisión de tiempo de compilación, no de ejecución. Es el mismo mecanismo que el `-DARQ=n` del ejemplo `arquitecturas` de la Clase 4.

## Salida esperada (PARTE 1)

```
I (301) analogico: === analogico PARTE 1: el eje vertical (cuantizacion) ===
I (302) analogico: DAC_1 y ADC2_CH6 son el mismo pad, GPIO17: no hace falta ningun cable
I (315) analogico: calibracion de fabrica cargada: las cuentas se traducen a mV
I (318) analogico: --- Curva de transferencia  DAC(8 bits) -> pad -> ADC(13 bits) ---
I (319) analogico:   codigo  V ideal    ADC crudo  V medido   error
I (325) analogico:   0       0,000 V    118        0,033 V    +33 mV
I (331) analogico:   64      0,827 V    2921       0,820 V    -7 mV
I (337) analogico:   128     1,655 V    5872       1,648 V    -7 mV
I (343) analogico:   192     2,482 V    8191       2,300 V    -182 mV  <-- fuera del rango del ADC
I (349) analogico:   255     3,300 V    8191       2,300 V
I (351) analogico:   1 LSB del DAC = 3300/256 = 12,9 mV  ·  1 LSB del ADC = 2300/8192 = 280 uV
I (352) analogico:   error maximo dentro del rango efectivo (0 a 2300 mV): +33 mV
I (360) analogico: --- Ruido de fondo y bits efectivos (codigo 128, 1024 lecturas) ---
I (712) analogico:   media 5872  min 5841  max 5903  recorrido 62 LSB  sigma 9,4 LSB
I (713) analogico:   resolucion libre de ruido: ~6,94 bits de los 13 nominales
I (982) analogico:   promediando 16 lecturas: sigma 2,4 LSB (la teoria dice sigma/raiz(16) = 2,3)
I (990) analogico: --- Cuanto cuesta una lectura ---
I (1041) analogico:   1000 lecturas oneshot en 50231 us  ->  50 us cada una
I (1042) analogico:   a ese ritmo el techo son ~20000 muestras/s por sondeo;
I (1043) analogico:   el ADC por DMA de la PARTE 2 llega a 83 333 sin tocar la CPU
```

> Las dos salidas de este README son **orientativas**: enseñan el formato y lo que hay que buscar. Los números concretos **cambian de una placa a otra**, porque dependen de la calibración grabada en la eFuse de tu chip; lo que no cambia es la forma de la curva.

## Cosas para probar

| Experimento | Qué observar |
|---|---|
| Bajar `ADC_ATENUACION` a `ADC_ATTEN_DB_0` | El rango efectivo cae a 0–700 mV: la curva se aplana casi enseguida |
| Subir `LECTURAS_POR_CODIGO` a 256 | La curva se alisa; el barrido tarda ocho veces más. Ese es el compromiso |
| Quitar `CONFIG_ADC_DISABLE_DAC_OUTPUT=n` de `sdkconfig.defaults`, borrar `sdkconfig` y recompilar la PARTE 1 | El ADC apaga el DAC en la primera lectura y la curva deja de seguir al código. El driver **no avisa de nada**; el aviso que ves es del ejemplo |
| `FS_ENSAYO_B` a `1000` en la PARTE 2 | Todo el barrido queda por encima de Nyquist y se pliega por debajo de 500 Hz: ninguna medida se parece a la frecuencia real |
| Comparar `f real` entre un arranque en frío y otro tras diez minutos | El RC_FAST deriva con la temperatura, y la frecuencia con él |
| Cambiar `MUESTRAS_BLOQUE` a 512 en la PARTE 2 | La estimación de frecuencia pierde resolución: `fs / (2·N)` |

## Solución de problemas

| Síntoma | Solución |
|---|---|
| En la PARTE 1 la curva no sigue al código | El ADC está apagando el DAC. En `sdkconfig` tiene que aparecer `# CONFIG_ADC_DISABLE_DAC_OUTPUT is not set`; si no, borra `sdkconfig` y recompila |
| `sin calibracion de fabrica` | Tu chip no trae los coeficientes en eFuse; los mV son orientativos, las cuentas crudas siguen valiendo |
| La curva se aplana antes de 2,3 V | Hay algo cargando el pad: quita la placa de audio |
| `f medida` no se parece a nada | El `pk-pk` es pequeño: la señal no está llegando. Revisa que la PARTE 2 esté compilada (`-DPARTE=2`) |
| `Failed to connect` al flashear | Secuencia **BOOT + RST** (modo bootloader) |
| El watchdog de tareas se queja | La PARTE 2 se queda dentro de `adc_continuous_read_parse()`; `sdkconfig.defaults` ya desactiva esa comprobación |

## Referencias

- [../../detalle-muestreo-y-plegado.md](../../detalle-muestreo-y-plegado.md)
- [../../detalles/adc-y-dac-en-el-esp32s2.md](../../detalles/adc-y-dac-en-el-esp32s2.md)
- [../../../clase-2/detalles/detalle-punto-fijo-y-float.md](../../../clase-2/detalles/detalle-punto-fijo-y-float.md)
- [../../../clase-4/detalles/freertos-en-esp-idf.md](../../../clase-4/detalles/freertos-en-esp-idf.md)
- Espressif, *ESP32-S2 Series Datasheet*, §5.5 *ADC Characteristics* y tabla 5-6 *ADC Calibration Results*.
- Espressif, *ESP-IDF Programming Guide — Analog to Digital Converter* y *Digital to Analog Converter*.
- Guía de conexión e instalación: [../../../bibliografia/Guia-Kaluga-1-macOS-ESP-IDF-VSCode.md](../../../bibliografia/Guia-Kaluga-1-macOS-ESP-IDF-VSCode.md)
