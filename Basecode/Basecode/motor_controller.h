#pragma once

#include "settings.h"

// Vitesse minimale acceptée par un moteur asservi. 
#define MIN_SPEED 10.f

/// @brief Structure représentant un moteur asservi.
typedef struct MotorController
{
    /// @brief Valeur renvoyée par pigpio_start().
    int m_pi;

    /// @brief Identifiant du gpio du capteur sur la roue codeuse.
    int m_gpioControl;

    /// @brief Identifiant du gpio du moteur dans le sens avant.
    int m_gpioForward;

    /// @brief Identifiant du gpio du moteur dans le sens arrière.
    int m_gpioBackward;

    /// @brief Identifiant du gpio du moteur courant (soit égal au sens avant, soit au sens arrière).
    int m_gpioMotor;

    /// @brief Identifiant du callback associé au capteur sur la roue codeuse.
    int m_callbackID;

    /// @brief Nombre courant de callbacks.
    int m_cbCount;

    /// @brief Nombre de callbacks a effectuer avant d'utiliser le régulateur PI (Proportionnel Intégral).
    int m_controllerCbCount;

    /// @brief Gain proportionnel du régulateur.
    float m_kp;

    /// @brief Gain d'intégration du régulateur.
    float m_ki;

    /// @brief Valeur de l'intégrale du régulateur.
    float m_integral;

    /// @brief Vitesse cible du moteur en pas par seconde.
    float m_targetSpeed;

    /// @brief Dernière vitesse mesurée par la roue codeuse en pas par seconde.
    float m_speed;
    
    /// @brief Valeur courante envoyée au PWM du moteur.
    int m_power;
    
    /// @brief Valeur de démarrage du moteur.
    /// Cette valeur est envoyée au PWM pendant les 4 premiers pas mesurés par
    /// le capteur sur la roue codeuse à chaque redémarrage du moteur.
    int m_startPower;

    /// @brief Temps du dernier appel au callback.
    uint32_t m_prevCbTick;

    /// @brief Temps du dernier appel à la fonction update().
    uint32_t m_prevUpdateTick;

    /// @brief Nombre d'erreurs du capteur sur la roue codeuse.
    int m_errorCount;

    /// @brief Durée pendant laquelle la valeur envoyée au PWN est saturée (égale à 255).
    float m_saturationTime;
} MotorController;

/// @brief Initialise un moteur asservi.
/// @param self le moteur.
/// @param pi valeur renvoyée par pigpio_start().
/// @param gpioForward identifiant du gpio du moteur dans le sens avant.
/// @param gpioBackward identifiant du gpio du moteur dans le sens arrière.
/// @param gpioControl identifiant du gpio du capteur sur la roue codeuse.
void MotorController_init(MotorController *self, int pi, int gpioForward, int gpioBackward, int gpioControl);

/// @brief Supprime un moteur asservi.
/// @param self le moteur.
void MotorController_quit(MotorController *self);

/// @brief Met à jour un moteur.
/// @param self le moteur.
void MotorController_update(MotorController *self);

/// @brief Arrête un moteur.
/// @param self le moteur.
void MotorController_stop(MotorController *self);

/// @brief Définit la vitesse cible du moteur.
/// @param self le moteur.
/// @param speed la vitesse cible (en pas par seconde).
void MotorController_setTargetSpeed(MotorController *self, float speed);

/// @brief Définit les paramètres du régulateur PI du moteur.
/// @param self le moteur.
/// @param kp le gain proportionnel.
/// @param ki le gain d'intégration.
void MotorController_setController(MotorController *self, float kp, float ki);

/// @brief Valeur de démarrage du moteur.
/// Cette valeur est envoyée au PWM pendant les 4 premiers pas mesurés par le
/// capteur sur la roue codeuse à chaque redémarrage du moteur.
/// @param self le moteur.
/// @param startPower la valeur de démarrage du moteur.
void MotorController_setStartPower(MotorController *self, int startPower);

/// @brief Définit le sens du moteur (avant ou arrière).
/// @param self le moteur.
/// @param goBackward booléen indiquant si le moteur doit tourner dans le sens arrière.
void MotorController_setBackward(MotorController *self, bool goBackward);

/// @brief Renvoie la vitesse du moteur.
/// @param self le moteur.
/// @return La vitesse du moteur.
/// La valeur renvoyée dépend de l'état du moteur :
/// - si le moteur est à l'arrêt, la fonction renvoie 0.f ;
/// - si le moteur démarre, la fonction renvoie la vitesse cible ;
/// - sinon, la fonction renvoie la dernière vitesse mesurée par le capteur sur la roue codeuse.
float MotorController_getSpeed(MotorController *self);

/// @brief Renvoie la distance parcourue par le moteur.
/// @param self le moteur.
/// @return La distance parcourue par le moteur.
/// Cette valeur est calculée à partir du nombre de pas mesurés par le capteur sur la roue codeuse.
/// Elle ne prend pas en compte le sens du moteur ni les éventuels dérapages de la roue.
float MotorController_getDistance(MotorController *self);
