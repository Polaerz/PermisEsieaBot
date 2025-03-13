#pragma once

#include "settings.h"

// Distance maximale mesurée par le capteur.
#define SENSOR_MAX_DISTANCE_CM 400.f

// Intervalle de temps entre deux mesures.
#define SENSOR_STEP_TIME 60000

// Intervalle de temps maximum entre deux mesure si aucun signal "echo" n'est reçu.
#define SENSOR_MAX_STEP_TIME 100000

/// @brief Structure représentant un capteur de distance à ultrasons.
typedef struct UltrasonicSensor
{
    /// @brief Valeur renvoyée par pigpio_start().
    int m_pi;
    
    /// @brief Identifiant du gpio du signal "trigger".
    int m_gpioTrig;

    /// @brief Identifiant du gpio du signal "echo".
    int m_gpioEcho;
    
    /// @brief Identifiant du callback associé au capteur.
    int m_callbackID;

    /// @brief Booléen indiquant si le signal "echo" a commencé.
    bool m_echoStarted;

    /// @brief Booléen indiquant si le signal "echo" s'est terminé.
    bool m_echoFinished;

    /// @brief Booléen indiquant si la mesure de la distance vient d'être effectuée.
    bool m_hasNewDistance;

    /// @brief Dernière distance en centimètres mesurée par le capteur.
    /// Vaut SENSOR_MAX_DISTANCE_CM si aucun obstacle n'est détecté.
    float m_distance;

    /// @brief Accumulateur permettant d'assurer au moins SENSOR_STEP_TIME microsecondes entre chaque mesure.
    uint32_t m_accu;

    /// @brief Temps du dernier appel à la fonction update().
    uint32_t m_prevUpdateTick;

    /// @brief Temps du début du signal "echo".
    uint32_t m_echoTick;

    /// @brief Temps du début du signal "trigger".
    uint32_t m_trigTick;
} UltrasonicSensor;

/// @brief Initialise un capteur de distance.
/// @param self le capteur de distance.
/// @param pi valeur renvoyée par pigpio_start().
/// @param gpioTrig identifiant du gpio du signal "trigger".
/// @param gpioEcho identifiant du gpio du signal "echo".
void UltrasonicSensor_init(UltrasonicSensor *self, int pi, int gpioTrig, int gpioEcho);

/// @brief Supprime un capteur de distance.
/// @param self le capteur de distance.
void UltrasonicSensor_quit(UltrasonicSensor *self);

/// @brief Met à jour un capteur de distance.
/// @param self le capteur de distance.
void UltrasonicSensor_update(UltrasonicSensor *self);

/// @brief Indique si une nouvelle distance vient d'être mesurée par un capteur.
/// @param self le capteur de distance.
/// @return true si une distance vient d'être mesurée, false sinon.
bool UltrasonicSensor_hasNewDistance(UltrasonicSensor *self);

/// @brief Renvoie la dernière distance mesurée par un capteur.
/// @param self le capteur de distance.
/// @return La dernière distance mesurée par un capteur.
float UlrasonicSensor_getDistance(UltrasonicSensor *self);
