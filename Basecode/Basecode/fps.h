#pragma once

#include "settings.h"

/// @brief Structure représentant un compteur de FPS.
typedef struct FPS
{
    /// @brief Valeur renvoyée par pigpio_start().
    int m_pi;

    /// @brief Nombre d'appels à la fonction update() depuis le dernier affichage.
    int m_count;
    
    /// @brief Accumulateur permettant de mesurer le temps écoulé.
    uint32_t m_accu;

    /// @brief Temps du dernier appel à la fonction update().
    uint32_t m_prevUpdateTick;
} FPS;

/// @brief Initialise un compteur de FPS.
/// @param self le compteur de FPS.
/// @param pi valeur renvoyée par pigpio_start().
void FPS_init(FPS *self, int pi);

/// @brief Supprime un compteur de FPS.
/// @param self le compteur de FPS.
void FPS_quit(FPS *self);

/// @brief Met à jour un compteur de FPS. Le compteur doit être activé avec start().
/// @param self le compteur de FPS.
void FPS_update(FPS *self);
