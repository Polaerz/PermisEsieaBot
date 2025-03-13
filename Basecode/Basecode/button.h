#pragma once

#include "settings.h"

/// @brief Structure représentant un bouton poussoir.
typedef struct Button
{
    /// @brief Valeur renvoyée par pigpio_start().
    int m_pi;

    /// @brief Identifiant du gpio du bouton.
    int m_gpio;

    /// @brief Identifiant du callback associé au bouton.
    int m_callbackID;

    /// @brief Booléen indiquant si le bouton vient d'être pressé.
    bool m_isPressed;

    /// @brief Nombre de callbacks validés par la fonction update(). 
    int m_prevCbCount;

    /// @brief Nombre courant de callbacks.
    int m_currCbCount;
} Button;

/// @brief Initialise un bouton.
/// @param self le bouton.
/// @param pi valeur renvoyée par pigpio_start().
/// @param gpio identifiant du gpio du bouton.
void Button_init(Button *self, int pi, int gpio);

/// @brief Supprime un bouton.
/// @param self le bouton.
void Button_quit(Button *self);

/// @brief Met à jour un bouton. Le bouton doit être activé avec start().
/// @param self le bouton.
void Button_update(Button *self);

/// @brief Indique si un bouton vient d'être pressé.
/// @param self le bouton.
/// @return true si le bouton vient d'être pressé, false sinon.
bool Button_isPressed(Button *self);
