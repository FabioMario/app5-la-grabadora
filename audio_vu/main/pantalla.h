/*
 * pantalla — dibujo mínimo sobre la TFT del ESP-LyraP-LCD32 (Clase 5)
 *
 * Un envoltorio muy delgado sobre esp_lcd: rectángulos y texto, que es todo
 * lo que necesita un VU-metro. No hay librería gráfica, no hay LVGL y no hay
 * buffer de pantalla completo: se dibuja SOLO lo que cambia.
 *
 * Compilado con PANTALLA=0 todas las funciones son nulas y el programa que
 * las llama no necesita ni un #ifdef. Es la forma barata de que el mismo
 * código funcione con y sin la placa de LCD montada.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* La ESP-LyraP-LCD32 es un panel de 240x320. Lo usamos apaisado. */
#define PANTALLA_ANCHO      320
#define PANTALLA_ALTO       240

/* Color de 16 bits, 5 rojo + 6 verde + 5 azul.
 *
 * Los bytes van INVERTIDOS a propósito: el panel espera cada palabra de 16
 * bits con el byte más significativo primero, y el ESP32-S2 es little endian.
 * Invertir aquí, una vez, sale más barato que pedirle al driver que lo haga
 * en cada transferencia — y funciona igual con ST7789 que con ILI9341. */
static inline uint16_t pantalla_color(uint8_t r, uint8_t g, uint8_t b)
{
    const uint16_t c = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8));
}

/* Arranca el panel. Devuelve false si no hay pantalla compilada o si el
 * panel no responde: el llamante sigue funcionando por consola. */
bool pantalla_iniciar(void);

bool pantalla_disponible(void);

/* Nombre del controlador compilado, para el log de arranque. */
const char *pantalla_controlador(void);

void pantalla_limpiar(uint16_t color);

/* Rectángulo relleno. Recorta solo contra los bordes de la pantalla. */
void pantalla_rect(int x, int y, int ancho, int alto, uint16_t color);

/* Texto en una fuente de 5x7 escalada por un entero. Solo dígitos, mayúsculas
 * y unos pocos signos: lo que cabe en 40 glifos y hace falta aquí. */
void pantalla_texto(int x, int y, const char *texto, int escala,
                    uint16_t frente, uint16_t fondo);

/* Ancho en píxeles que ocupará ese texto, para poder centrarlo. */
int pantalla_ancho_texto(const char *texto, int escala);
