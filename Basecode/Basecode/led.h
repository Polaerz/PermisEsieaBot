#pragma once

#include "settings.h"

typedef struct LED
{
    /// @brief Valeur renvoyée par pigpio_start().
    int m_pi;

    /// @brief Identifiant du gpio du bouton.
    int m_gpio;

    /// @brief Accumulateur permettant de mesurer la durée d'un demi-clignotement.
    uint32_t m_accu;

    /// @brief Temps du dernier appel à la fonction update().
    uint32_t m_prevUpdateTick;

    /// @brief Durée d'un demi-clignotement.
    uint32_t m_cycleTime;

    /// @brief Nombre de demi-clignotements.
    int m_cycleCount;

    /// @brief Indice du demi-clignotemet courant.
    int m_cycleIndex;
} LED;

/// @brief Initialise une LED.
/// @param self la LED.
/// @param pi valeur renvoyée par pigpio_start().
/// @param gpio identifiant du gpio de la LED.
void LED_init(LED *self, int pi, int gpio);

/// @brief Supprime une LED.
/// @param self la LED.
void LED_quit(LED *self);

/// @brief Met à jour une LED.
/// @param self la LED.
void LED_update(LED *self);

/// @brief Fait clignoter une LED.
/// @param self la LED.
/// @param count le nombre de clignotements.
/// @param cycleTime la durée d'un clignotement
///        (la première moitié la LED est alumée, la seconde elle est éteinte).
void LED_blink(LED *self, int count, float cycleTime);

/// @brief Indique si une LED clignote.
/// @param self la LED.
/// @return true si la LED clignote, false sinon.
bool LED_isBlinking(LED *self);
