| Supported Targets | ESP32-S2 |
| ----------------- | -------- |

# audio_vu — micrófono, códec ES8311 y VU-metro (Clase 5)

Firmware base de la **App 5**. El micrófono de la **ESP-LyraT-8311A** entra por el códec **ES8311**, viaja por **I2S con DMA** hasta la SRAM y se convierte en un nivel en decibelios que se pinta en la **TFT**, en el **LED RGB** y en la consola.

```
   micro ──▶ ES8311 ──I2S/DMA──▶ [captura p8] ──cola──▶ [proceso p6] ──┐
              (I2C: control)      RMS, pico,          balística        │ mutex
                                  recortes           VU / PPM          ▼
                                                                ┌────────────┐
   [tactil p9] ──notificación──────────────────────────────────▶│  estado    │
    cola de la ISR del táctil                                   │ compartido │
                                                                └────────────┘
                                                                       │ mutex
   [interfaz p3] ◀─────────────────────────────────────────────────────┘
    TFT + LED RGB + panel por consola

   [timer software] ──▶ informe de tareas y de audio cada 10 s
```

Es la arquitectura de la Clase 4 aplicada a un problema con un **plazo real**. Lo que hay que ver funcionando:

- el audio **no espera**: el DMA sigue llenando descriptores mires o no, y lo que no se recoge a tiempo se pierde. La tarea `captura` no hace más que leer, medir y encolar;
- todo lo lento —formatear, dibujar, escribir por la UART— ocurre en la tarea de **menor** prioridad, y **nunca** con el mutex tomado;
- el **decibelio se calcula en enteros**: el Xtensa LX7 no tiene FPU;
- la pantalla se refresca **por trozos**: repintar 320×240 cuesta 30 ms de SPI, casi lo que dura un bloque de audio.

## Hardware

| Placa                           | Dónde va                                                        | Para qué                                                                        |
| ------------------------------- | ---------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| **ESP-LyraT-8311A**       | Extension Header                                                 | Micrófono, códec ES8311 y amplificador                                         |
| **ESP-LyraP-TouchA**      | Touch FPC Connector, cable de 20 pines                           | Los cuatro botones                                                               |
| **ESP-LyraP-LCD32**       | Encima de la placa de audio, en su Extension Header              | La pantalla —**opcional**, ver abajo                                      |
| **Jumper RGB**            | Junto al LED                                                     | Sin él, el LED de GPIO45 no recibe señal                                       |
| **Altavoz o auriculares** | Conector de 2 pines o salida de auriculares de la placa de audio | Solo para la**App 5**: la ESP-LyraT-8311A trae micrófono, pero no altavoz |

### La revisión del kit

El ejemplo está ajustado y probado en un **kit v1.2**: placa principal v1.2, ESP-LyraT-8311A v1.2 y ESP-LyraP-LCD32 v1.1. La revisión está impresa en cada placa. Entre el kit v1.2 y el v1.3 cambian tres cosas que este ejemplo toca:

|                                                        | Kit v1.2                                                             | Kit v1.3                                        |
| ------------------------------------------------------ | -------------------------------------------------------------------- | ----------------------------------------------- |
| Dato del micrófono:`I2S_DIN` en `audio_vu_main.c` | **GPIO46**, el valor que trae el ejemplo                       | **GPIO34**: hay que cambiarlo a mano      |
| Flashear con la placa de audio montada                 | **No se puede**: ver «Cómo compilar, flashear y monitorear» | Se puede                                        |
| Retroiluminación de la pantalla                       | La enciende el programa con el**GPIO6 en bajo**                | Encendida por hardware; el GPIO6 no la gobierna |

> Con el pin de la otra revisión el programa arranca y **no da ningún error**, pero no recibe audio: el nivel se queda fijo en −90.0 dBFS. La columna del kit v1.3 sale de la documentación de Espressif: el ejemplo no se ha probado en uno.

### Los microinterruptores, esta vez no van todos en OFF

Hasta la Clase 4 la instrucción era «**T1–T14 en OFF**», porque solo estaba la placa táctil. Con la de audio y la de LCD montadas eso **ya no vale**: en OFF esos GPIO quedan colgados del conector táctil, y aquí hacen falta para otras cosas.

| Microinterruptor         |   Posición   | Por qué                                                                                                          |
| ------------------------ | :-----------: | ----------------------------------------------------------------------------------------------------------------- |
| **T1, T2, T3, T5** | **OFF** | Son los cuatro botones que usa el ejemplo: VOL_UP, PLAY, VOL_DOWN y RECORD                                        |
| **T7, T8**         | **ON** | GPIO7 y GPIO8 son el**I2C** del ES8311                                                                      |
| **T10, T12**       | **ON** | GPIO10 es el PA del altavoz y GPIO12 el DOUT de I2S                                                               |
| **T9, T11, T13**   | **ON** | GPIO9, 11 y 13 son MOSI, CS y DC de la**pantalla**                                                          |
| **T6**             | **ON** | Por el GPIO6 se enciende la**retroiluminación** de la pantalla; es también el BT_ADC de la placa de audio |
| **T4, T14**        | **ON** | No se usan                                                                                                        |

> La regla, en una línea: **en OFF solo los canales táctiles que de verdad usas**; todos los demás, en ON.

Cada microinterruptor une su GPIO con el conector de extensión, que es donde van montadas la placa de audio y la pantalla. El GPIO llega siempre al conector del táctil; en ON llega además a las placas de encima. Por eso, si T7 y T8 están en OFF el códec no responde por I2C y el programa se reinicia una y otra vez.

### Los otros interruptores del kit

| Dónde                            | Interruptores                                                        | Posición                                                                                                                                                                                    |
| --------------------------------- | -------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ESP-LyraP-LCD32 v1.1, por detrás | *Control Switch*: BLCT_H, BLCT_L, CS_H, CS_L, RST_H y RST_L        | **Todos en OFF**. En ON solo añaden una resistencia de 10 kΩ a 3,3 V o a masa en esas líneas, y el programa ya las maneja. BLCT_L en ON no basta para encender la retroiluminación |
| ESP-LyraP-LCD32 v1.1, por detrás | *Touch Screen Switch*: T_MISO, T_MOSI, T_SCK, T_CS, T_IRQ y T_BUSY | **Todos en OFF**. Son para un panel táctil que el kit no trae                                                                                                                         |
| Placa principal                   | Grupo JTAG                                                           | No afecta al ejemplo                                                                                                                                                                         |
| ESP-LyraT-8311A                   | No tiene microinterruptores, solo seis pulsadores                    | —                                                                                                                                                                                           |

### Los conflictos de pines, y cómo los esquiva este ejemplo

El ESP32-S2 tiene los pines multiplexados y las cuatro placas de extensión se pisan entre sí. El *User Guide* del kit trae la tabla; esto es lo que aplica aquí:

| Conflicto                                                                                             | Solución adoptada                                                                                                                                                                                            |
| ----------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **T6 (PHOTO)** ↔ retroiluminación del LCD y BT_ADC de la placa de audio, los tres en el GPIO6 | No se inicializa T6                                                                                                                                                                                           |
| **T11 (NETWORK)** ↔ *chip select* de la pantalla                                             | No se inicializa T11                                                                                                                                                                                          |
| **GPIO6**: retroiluminación del LCD ↔ BT_ADC                                                  | `pantalla.c` pone el GPIO6 en **bajo**, que es lo que enciende la ESP-LyraP-LCD32 v1.1. Mientras esté en bajo, los seis botones de la placa de audio no se pueden leer; el ejemplo usa los táctiles |
| **GPIO46**: dato del micrófono ↔ pin de arranque del ESP32-S2, solo en el kit v1.2            | Se flashea con la placa de audio desmontada                                                                                                                                                                   |
| El**SPI3** es el único DMA del ADC y del DAC                                                   | La pantalla va por**SPI2** y lo deja libre, aunque este ejemplo no lo use                                                                                                                               |

## Con pantalla y sin pantalla

```bash
idf.py -DPANTALLA=0 build    # sin ESP-LyraP-LCD32: consola + LED
idf.py -DPANTALLA=1 build    # con LCD32, driver del ST7789: lo probado
idf.py -DPANTALLA=2 build    # con LCD32, driver del ILI9341
```

Con `PANTALLA=0` todas las funciones de `pantalla.c` se compilan **vacías** y el programa que las llama no necesita ni un `#ifdef`. Es la forma barata de que el mismo código funcione con y sin la placa montada, y de que quien no la tenga pueda seguir la clase entera.

> Según la guía de Espressif, la **ESP-LyraP-LCD32 v1.1** (kit v1.2) lleva un ST7789V, y la **v1.2** (kit v1.3) puede traer un ST7789V o un ILI9341 según la serie del panel. Lo probado en una placa es `PANTALLA=1` con una LCD32 v1.1. `PANTALLA=2` compila, pero **no se ha probado**.

> **Desde VS Code**, el botón *Build* no pasa `-DPANTALLA`: compila con el valor que haya guardado en la carpeta `build/`, y si no hay ninguno, con `PANTALLA=0`. Ejecuta una vez `idf.py -DPANTALLA=1 build` en la terminal de ESP-IDF, que se abre desde la paleta de comandos con «ESP-IDF: Open ESP-IDF Terminal»; en una terminal normal `idf.py` no existe. El valor queda guardado hasta que se borre `build/`, por ejemplo con un *Full Clean*. Si el informe de la consola dice `(sin pantalla)`, se compiló con `PANTALLA=0`.

### Cómo debe verse la pantalla

Fondo azul muy oscuro, casi negro. Arriba, el título en blanco. Debajo, el nivel en dBFS en grande y la barra, los dos del mismo color: **verde** hasta −12 dBFS, **ámbar** hasta −3 y **rojo** por encima. Abajo, en gris, el modo, la ganancia y los recortes. La imagen va apaisada y el texto se lee de izquierda a derecha.

Si el fondo sale claro y las letras oscuras, la imagen está **en negativo**; si el texto se lee al revés, está **en espejo**. Las dos cosas dependen del panel y se corrigen en `pantalla.c`: ver «Solución de problemas».

## Las cuatro tareas

| Tarea              | Prio | Espera en             | Trabajo                                | Por qué esa prioridad                                   |
| ------------------ | :--: | --------------------- | -------------------------------------- | -------------------------------------------------------- |
| **tactil**   |  9  | Cola de la ISR        | Balística, ganancia, reinicio de pico | Responde a una persona, y hace cuatro cosas y se bloquea |
| **captura**  |  8  | `i2s_channel_read`  | RMS, pico, recortes de cada bloque     | **El único plazo duro**: 32 ms y el DMA no espera |
| **proceso**  |  6  | Cola de niveles       | dBFS y balística del instrumento      | Consume lo que produce`captura`                        |
| **interfaz** |  3  | Notificación o plazo | TFT, LED y panel por consola           | Si se retrasa 100 ms, nadie lo nota                      |

> `captura` va **por debajo** de `tactil` a propósito. Parece contradictorio —el audio es lo urgente— pero la tarea del táctil solo cambia cuatro variables y se vuelve a bloquear: nunca puede robarle 32 ms a nadie. Poner el audio arriba del todo no lo haría más rápido, y dejaría los botones sin atender durante los picos de carga.

## Mapa de botones

| Serigrafía        | Canal | Acción                                                |
| ------------------ | :---: | ------------------------------------------------------ |
| **PLAY**     |  T2  | Cambia la**balística**: VU → PPM → CRUDO      |
| **VOL_UP**   |  T1  | Sube la ganancia del micrófono, de 6 en 6 dB hasta 36 |
| **VOL_DOWN** |  T3  | La baja hasta 0                                        |
| **RECORD**   |  T5  | Reinicia el pico retenido y el contador de recortes    |

### Las tres balísticas, y por qué no son cosmética

| Modo            | Ataque                 | Caída          | Para qué sirve                                                            |
| --------------- | ---------------------- | --------------- | -------------------------------------------------------------------------- |
| **VU**    | lento (~250 ms)        | lento           | Mide**sensación sonora**: es lo que hace un vúmetro de aguja       |
| **PPM**   | **instantáneo** | lento (~750 ms) | Caza**picos**: es lo que hace falta para no recortar al grabar       |
| **CRUDO** | ninguno                | ninguno         | Enseña el ruido de la propia medida, que es el que los otros dos esconden |

> Un VU y un PPM **miden cosas distintas** con los mismos números. Con el modo CRUDO delante se entiende de golpe por qué existe la balística: sin filtrar, la aguja es ilegible.

## Colores del LED

| Color                 | Nivel                   | Significa                          |
| --------------------- | ----------------------- | ---------------------------------- |
| **Ámbar fijo** | —                      | Arrancando y calibrando el táctil |
| **Azul tenue**  | por debajo de −45 dBFS | Silencio                           |
| **Verde**       | −45 a −12 dBFS        | Zona de trabajo                    |
| **Ámbar**      | −12 a −3 dBFS         | Cerca del tope                     |
| **Rojo**        | por encima de −3 dBFS  | Al borde del recorte               |

## Estructura

```
audio_vu/
├── CMakeLists.txt
├── sdkconfig.defaults        Tick de 1 ms, estadísticas de tareas, canario, PSRAM
├── main/
│   ├── CMakeLists.txt        Define PANTALLA=n
│   ├── idf_component.yml     led_strip, esp_codec_dev y esp_lcd_ili9341
│   ├── audio_vu_main.c       Audio, tareas y VU
│   ├── pantalla.h            Rectángulos y texto; vacío si PANTALLA=0
│   └── pantalla.c
└── README.md   (este archivo)
```

`pantalla.c` está **aparte** por dos razones: para que la App 5 lo reutilice sin tocarlo, y porque mezclar el dibujo con el audio en un solo archivo haría ilegibles los dos.

## Cómo compilar, flashear y monitorear

```bash
idf.py set-target esp32s2
idf.py -DPANTALLA=1 build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

- Sustituye `/dev/cu.usbserial-XXXX` por tu puerto (`ls /dev/cu.*` en macOS).
- Conecta el cable al puerto **UART** de la placa.
- Salir del monitor: `Ctrl + ]`.

> ⚠️ **Kit v1.2: se flashea con la placa de audio desmontada.** El GPIO46, por donde llega el micrófono, es también un pin de arranque: el ESP32-S2 solo entra en modo de descarga si lo encuentra en bajo, y con la placa de audio montada queda en alto. `esptool` termina con `Failed to connect to ESP32-S2: Wrong boot mode detected (0x4)`. Mantener pulsado BOOT no basta, porque ese botón solo baja el GPIO0. Se desmonta la placa de audio, con la pantalla que lleva encima, se flashea, se vuelven a montar y se enciende. Mientras la placa de audio no esté, el programa no encuentra el códec y se reinicia una y otra vez: es lo esperado.

> ⚠️ **No toques la placa durante el arranque.** Si el `benchmark` del táctil se mide con el dedo apoyado, ese botón no se activará hasta el siguiente reset.

## Salida esperada

```
I (301) audio_vu: === audio_vu: microfono, codec ES8311 y VU-metro ===
I (302) audio_vu: PLAY = balistica VU/PPM/CRUDO | VOL_UP y VOL_DOWN = ganancia | RECORD = reinicia pico
I (338) pantalla: panel ST7789 listo, 320x240, SPI2 a 40 MHz
I (340) audio_vu: pantalla ST7789 activa
I (395) audio_vu: Calibrando... NO toques la placa durante el arranque
I (412) audio_vu:   PLAY     T2   benchmark=40887  umbral=817
I (413) audio_vu:   VOL_UP   T1   benchmark=39954  umbral=799
I (414) audio_vu:   VOL_DOWN T3   benchmark=40233  umbral=804
I (415) audio_vu:   RECORD   T5   benchmark=41102  umbral=822
I (418) audio_vu: codec listo: 16000 Hz, 16 bits, ganancia de entrada 24 dB
I (420) audio_vu: [captura] 512 tramas por bloque = 32 ms a 16000 Hz
I (421) audio_vu: Arranque completo. app_main termina; el planificador sigue.
I (523) audio_vu: [interfaz] -58.2  dBFS [                                ] pico -54.1  | modo VU    gan 24 dB  recortes 0
I (626) audio_vu: [interfaz] -31.7  dBFS [############                    ] pico -24.8  | modo VU    gan 24 dB  recortes 0
I (3204) audio_vu: [tactil] PLAY (T2) -> modo PPM  ganancia 24 dB | latencia ISR->tarea 41 us
I (10362) audio_vu: --- Informe (cada 10 s) ---------------------------------
I (10363) audio_vu:   tarea      prio estado      pila libre
I (10364) audio_vu:   tactil     9    BLOQUEADA   3288 B
I (10365) audio_vu:   captura    8    BLOQUEADA   3012 B
I (10366) audio_vu:   proceso    6    BLOQUEADA   2408 B
I (10367) audio_vu:   interfaz   3    BLOQUEADA   2760 B
I (10368) audio_vu:   audio: 16000 Hz x 16 bits mono = 32000 B/s  ·  bloque 32000 us
I (10369) audio_vu:   medir un bloque: 731 us como mucho (2 % del bloque)
I (10370) audio_vu:   refrescar la pantalla: 9412 us como mucho  (ST7789)
I (10371) audio_vu:   bloques 312  perdidos 0  lecturas cortas 0  cola 0/8
I (10372) audio_vu:   SRAM interna libre: 151208 B  (minimo historico 148964 B)
I (10373) audio_vu:   PSRAM libre: 2093596 B
```

> Esta salida es **orientativa**: enseña el formato y lo que hay que buscar. Los tiempos, las pilas y la memoria libre de tu placa serán otros, y entre las líneas del ejemplo aparecen también las de `esp_codec_dev` y del driver de I2S.

**Las cuatro tareas aparecen BLOQUEADAS**, igual que en la Clase 4, y el presupuesto de tiempo es lo nuevo: **medir un bloque cuesta el 2 % del bloque**. Ese porcentaje es el número que dice si el sistema tiene margen. Si se acercara al 100 %, no habría CPU para nada más y empezarían a perderse muestras.

## Qué mecanismo se usa dónde, y por qué

| Mecanismo                   | Dónde                       | Por qué ese y no otro                                                                  |
| --------------------------- | ---------------------------- | --------------------------------------------------------------------------------------- |
| **DMA de I2S**        | Códec → SRAM               | Mover 32 000 B/s con la CPU sería absurdo                                              |
| **Cola**              | `captura → proceso`       | Transporta la**medida** (24 bytes), no el audio (2048 bytes)                      |
| **Cola**              | ISR del táctil →`tactil` | La ISR entrega trabajo a una tarea: patrón diferido                                    |
| **Mutex**             | Acceso a`s_estado`         | Tres tareas escriben y leen la misma estructura                                         |
| **Notificación**     | `proceso → interfaz`      | Un emisor, un receptor: lo más barato del kernel                                       |
| **Semáforo binario** | ISR del SPI →`interfaz`   | Esperar a que el DMA termine de mandar un trozo antes de reutilizar el buffer de dibujo |
| **Timer software**    | Informe cada 10 s            | Periódico y ligero, sin gastar una tarea                                               |

> Por la cola viaja el **resultado**, no el bloque. Copiar 2048 bytes por la cola en cada bloque multiplicaría por 85 el tráfico y no aportaría nada: el consumidor solo necesita cuatro números. Además, la copia de una cola se hace dentro de una sección crítica.

## Detalles de diseño que conviene mirar en el código

- **La cola no bloquea al productor.** `xQueueSend(..., 0)` prefiere perder una **medida** a retrasar la siguiente lectura de I2S, porque lo que se perdería entonces sería **audio**. La política se elige, y el ejemplo cuenta lo que descarta.
- **El decibelio, en enteros.** `dB = 20·log10(x) = 6,0206·log2(x)`, y 6,0206 en Q8 es 1541. `log2_q8()` saca la parte entera de la posición del bit más alto y la fracción interpolando la mantisa. No hay ni una coma flotante en el camino del audio.
- **La continua se resta antes de elevar al cuadrado.** Sin ese paso, una entrada con offset marcaría nivel alto en silencio absoluto. El ES8311 ya cancela la continua con un filtro digital propio; el programa la resta igualmente porque no debe depender de cómo esté configurado el códec.
- **Se procesa una ranura de cada dos.** El ES8311 es **mono** y entrega su único convertidor en la ranura izquierda de la trama estéreo. En la derecha `esp_codec_dev` deja la **referencia del DAC**, es decir, lo que se esté reproduciendo: hoy, silencio. Contar las dos ranuras falsearía el nivel.
- **La pantalla se refresca por diferencias.** Solo se pinta el trozo de barra que cambió, la marca de pico que se movió y el número y el pie **cuando el texto cambia**. El fondo, el título y la escala se dibujan **una vez**.
- **Se espera al DMA antes de reutilizar el buffer de dibujo.** `esp_lcd_panel_draw_bitmap()` encola la transferencia y vuelve; `pantalla.c` se bloquea en un semáforo que da la interrupción del SPI. Sin esa espera, el carácter siguiente se compondría sobre un buffer que el DMA todavía está leyendo.
- **El múltiplo de MCLK se declara en tres sitios, y tiene que ser el mismo.** En el I2S (`mclk_multiple`), en el ES8311 (`mclk_div`) y en la estructura con la que se abre el dispositivo: `esp_codec_dev` **reconfigura** el reloj del I2S al abrirlo, y sin ese campo vuelve a 256 mientras el códec sigue haciendo cuentas para 384. No da ningún error.
- **El canal `tx` se inicializa primero y queda en marcha.** En el ESP32-S2, `tx` y `rx` comparten BCLK y WS, y los genera el que se inicializa primero. Con `auto_clear` manda silencio hasta que la App 5 le escriba algo.
- **El orden de los campos de `nivel_t` está elegido.** Con el `int64_t` delante la estructura mide 24 bytes; con la marca de tiempo entre los campos pequeños, el relleno la deja en 32. Un `_Static_assert` lo vigila.
- **El buffer de audio vive en el heap.** 2 KB en la pila de una tarea es la mitad de lo que tiene.

## Por qué el `sdkconfig.defaults`

| Opción                                          | Para qué                                                                                                    |
| ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------ |
| `CONFIG_FREERTOS_HZ=1000`                      | Un bloque son 32 ms: con un tick de 10 ms el retraso mínimo sería un tercio del bloque                     |
| `CONFIG_FREERTOS_USE_TRACE_FACILITY=y`         | Habilita el informe de estado y pila de cada tarea                                                           |
| `CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY=y` | Avisa del desbordamiento en vez de reiniciar sin explicación                                                |
| `CONFIG_SPIRAM=y`                              | No hace falta para el VU,**sí** para la App 5: 10 s de audio son 320 KB y no caben en la SRAM interna |

## Cosas para probar

| Experimento                                            | Qué observar                                                                                                                                                                                            |
| ------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Modo**CRUDO** y hablar                           | El número salta varios dB entre bloques: eso es lo que la balística esconde                                                                                                                            |
| Subir la ganancia a 36 dB y dar una palmada            | Aparecen**recortes**: información destruida que no se recupera bajando el volumen después                                                                                                        |
| `BLOQUE_TRAMAS` a 128                                | El VU responde cuatro veces más rápido y la tarea despierta cuatro veces más                                                                                                                          |
| `BLOQUE_TRAMAS` a 4096                               | 256 ms por bloque: el VU se vuelve inútil aunque el audio siga entrando bien                                                                                                                            |
| Poner`PRIO_INTERFAZ` a 9                             | Dibujar manda: sube el máximo de «medir un bloque», porque la interfaz interrumpe a`captura` en mitad de la medida. El audio no se pierde mientras el retraso quepa en los 90 ms de holgura del DMA |
| Meter un`ESP_LOGI` por muestra dentro de `captura` | El porcentaje del bloque se dispara y el audio empieza a perderse                                                                                                                                        |
| Quitar la resta de la continua                         | Con este códec apenas cambia: el ES8311 ya la cancela en digital. Con un convertidor sin ese filtro, como el ADC interno, el nivel en silencio subiría decenas de dB                                   |
| Quitar la espera de`enviar()` en `pantalla.c`      | Los caracteres pueden salir mezclados: el DMA sigue leyendo el buffer mientras se compone el siguiente                                                                                                   |
| Tapar el micrófono con el dedo                        | El nivel baja, pero no llega a −90: lo que queda es el ruido propio de la cadena                                                                                                                        |

## Solución de problemas

| Síntoma                                                                | Solución                                                                                                                              |
| ----------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| `es8311_codec_new` falla o no hay I2C                                 | Microinterruptores**T7 y T8 en ON**; la placa de audio bien asentada en el Extension Header                                      |
| El nivel está fijo en**−90.0 dBFS**, sin moverse ni una décima | No llega ningún dato del códec:`I2S_DIN` no es el de tu kit. GPIO46 en el v1.2, GPIO34 en el v1.3. El programa no da ningún error |
| El nivel se mueve, pero muy poco                                        | Ganancia a 0 dB: sube con VOL_UP                                                                                                       |
| Nivel alto y constante en silencio                                      | La continua no se está restando, o el micrófono está saturado: baja la ganancia                                                     |
| Ningún botón responde                                                 | **T1, T2, T3 y T5 en OFF**; cable FPC bien insertado                                                                             |
| Un canal siempre activo                                                 | Se calibró con el dedo encima: reiniciar sin tocar                                                                                    |
| La pantalla no hace nada y el informe dice`(sin pantalla)`            | Se compiló con`PANTALLA=0`: ver «Con pantalla y sin pantalla»                                                                     |
| Pantalla apagada, pero el log dice`panel ST7789 listo`                | La retroiluminación no enciende: microinterruptor**T6 en ON**                                                                   |
| Pantalla encendida, pero en blanco                                      | Prueba el otro controlador:`-DPANTALLA=2`                                                                                            |
| Colores en negativo: fondo claro y letras oscuras                       | Cambia el argumento de`esp_lcd_panel_invert_color()` en `pantalla.c`                                                               |
| Rojo y azul intercambiados                                              | Cambia`LCD_RGB_ELEMENT_ORDER_BGR` por `LCD_RGB_ELEMENT_ORDER_RGB` en `pantalla.c`                                                |
| Imagen en espejo, o derecha pero cabeza abajo                           | Los dos argumentos de`esp_lcd_panel_mirror()` en `pantalla.c`: en espejo, cambia uno; cabeza abajo, cambia los dos                 |
| `A stack overflow in task X`                                          | Sube la pila de esa tarea; mira su`pila libre` en el informe                                                                         |
| `no se pudo abrir el SPI2`                                            | Algo más reclamó el bus antes que la pantalla                                                                                        |
| `Failed to connect` con `Wrong boot mode detected` al flashear      | Kit v1.2: flashea con la placa de audio desmontada, ver «Cómo compilar, flashear y monitorear»                                      |
| `Failed to connect` al flashear, sin ese mensaje                      | Secuencia**BOOT + RST** (modo bootloader)                                                                                        |

## Referencias

- [../../detalle-decibelio-y-punto-fijo.md](../../detalle-decibelio-y-punto-fijo.md)
- [../../detalle-muestreo-y-plegado.md](../../detalle-muestreo-y-plegado.md)
- [../../detalles/i2s-codec-y-pantalla.md](../../detalles/i2s-codec-y-pantalla.md)
- [../../detalles/adc-y-dac-en-el-esp32s2.md](../../detalles/adc-y-dac-en-el-esp32s2.md)
- [../../../clase-4/detalle-cola-semaforo-o-mutex.md](../../../clase-4/detalle-cola-semaforo-o-mutex.md)
- [../../../clase-2/detalles/touch-capacitivo-esp32s2.md](../../../clase-2/detalles/touch-capacitivo-esp32s2.md)
- Espressif, *ESP-IDF Programming Guide — Inter-IC Sound (I2S)* y *LCD*.
- Espressif, *ESP32-S2-Kaluga-1 Kit User Guide*, «Compatibility of Extension Boards» y «Hardware Revision Details».
- Everest Semiconductor, *ES8311 Low Power Mono Audio Codec — Datasheet*.
- Guía de conexión e instalación: [../../../bibliografia/Guia-Kaluga-1-macOS-ESP-IDF-VSCode.md](../../../bibliografia/Guia-Kaluga-1-macOS-ESP-IDF-VSCode.md)
