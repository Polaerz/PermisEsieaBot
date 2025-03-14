<div align="center">
<img src="https://cdn.discordapp.com/attachments/1212852209491116113/1349860905789161553/Logo_Esiea_-_blanc_fond_bleu_1.png?ex=67d4a36f&is=67d351ef&hm=7eab9224bede781da7d8cef37ad0484f6e27d8397b9b143322e5b84533456c3c&" width="300" alt="Concord Logo">
</div>

# Permis Esiea Bot - Classe 10 Internationale
Pour mieux comprendre ces codes, veuillez lire attentivement ce fichier, qui offre une vue d'ensemble du ML Challenge.

## 🚨 Ce ne sont pas les codes finaux !!
En effet, le développement de ce bot n'est pas encore terminé, et des améliorations sont à venir.

## ⚠️ A propos
Vous trouverez ici tous les programmes utilisés pour le développement du bot. Certains fichiers sont en double, tandis que d'autres servent uniquement au débogage. Il est impératif d'utiliser les fichiers présents dans le dossier [`Basecode/`].

## 💾 Rappels

Les informations ci-dessous servent de rappel pour se connecter à *l'ESIEABOT* ainsi que pour utiliser certaines commandes et données pouvant être utiles lors des différents tests à effectuer.

#### **Connection :**

```console
ssh pi@10.42.0.1
```
#### **Mise à jour du bot :**

*MAJ système*
```console
sudo apt update
sudo apt upgrade
```
*MAJ manette* 
```console
sudo apt install joystick
```
#### **Tests de mise à jour :**

*Test système* 
```console
/esieabot/available/official/add-on-board-test.py
```
*Test manette* 
```console
jstest /dev/input/js0
```

#### **Le Raspberry PI :**
Le Raspberry Pi fonctionne grâce à ce que l'on appelle le [`GPIO`] (General Purpose Input/Output).
La communication entre le GPIO et l'utilisateur s'effectue via `pigpiod_if2`, une bibliothèque de fonctions directement intégrée au Raspberry Pi.

Documentation Raspberry Pi -> [Here](https://www.raspberrypi.com/documentation/)

# 📄Fichiers .c

|  Fichiers                                                                                                      | Description                                 |
|----------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| [button.c](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/button.c)                     | Gestion du bonton sur l'add-board
| [fps.c](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/fps.c)                           | Gestion des Frames per second
| [input.c](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/input.c)                       | Gestion des inputs de la manettes 
| [led.c](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/led.c)                           | Gestion de la led
| [motor_controller.c](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/motor_controller.c) | Gestion des moteurs
| [tools.c](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/tools.c)                       | Outils
| [ultrasonic_sensor.c](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/ultrasonic_sensor.c) | Gestion des capteurs

## Button.c
1. `void` Button_init(Button *self, int pi, int gpio);

```c
void Button_init(Button *self, int pi, int gpio)
{
    assert(self);
    assert(pi >= 0);
    assert(0 <= gpio && gpio < 32);

    memset(self, 0, sizeof(*self));
    self->m_pi = pi;
    self->m_gpio = gpio;

    set_mode(self->m_pi, self->m_gpio, PI_INPUT);
    set_pull_up_down(self->m_pi, self->m_gpio, PI_PUD_DOWN);

    self->m_callbackID = callback_ex(
        self->m_pi, self->m_gpio, RISING_EDGE,
        _Button_cb, self);
    assert(self->m_callbackID >= 0 && "The callback ID must be >= 0");
}
```
## Initialisation des variables du bouton
**Assert** 
```console
 assert(self);
 assert(pi >= 0);
 assert(0 <= gpio && gpio < 32);
```
Les ``asserts`` sont utilisés pour vérifier que les conditions nécessaires à l'exécution du programme sont remplies. Si ce n'est pas le cas, le programme s'arrête automatiquement.

**memset**
```console
memset(self, 0, sizeof(*self));
```
La fonction ``memset`` permet de remplir une zone mémoire avec une valeur spécifique dans notre cas nos init. 

**CallBack**
```console
  self->m_callbackID = callback_ex(
        self->m_pi, self->m_gpio, RISING_EDGE,
        _Button_cb, self);
    assert(self->m_callbackID >= 0 && "The callback ID must be >= 0");
```
Une ``callback`` est une fonction passée en argument à une autre fonction, qui sera ensuite exécutée automatiquement lorsqu'un événement spécifique se produit. Dans notre cas appuyer sur le bouton.

2. ``void`` Button_quit(Button *self);

```c
void Button_quit(Button *self)
{
    assert(self && self->m_pi >= 0 && "The MotorController must be initialized");    
    assert(self->m_callbackID >= 0);

    int exitStatus = callback_cancel(self->m_callbackID);
    assert(exitStatus == 0);

    memset(self, 0, sizeof(*self));
    self->m_pi = -1;
}
```
**Quitter proprement le code**

Quitter proprement son code est essentiel pour éviter d'éventuelles erreurs lors d'une future utilisation.

3. ``void`` Button_update(Button *self);
  
```c
void Button_update(Button *self)
{
    assert(self && self->m_pi >= 0 && "The Button must be initialized");

    if(self->m_prevCbCount < self->m_currCbCount){
        self->m_prevCbCount = self->m_currCbCount;
        self->m_isPressed = true;
    }else{
        self->m_isPressed = false;
    }
}

```
**Mettre à jour l'état du bouton**

Ici, on vérifie en permanence si le bouton est appuyé ou non.

4. ``bool`` Button_isPressed(Button *self);

```c
bool Button_isPressed(Button *self);
{
    assert(self && self->m_pi >= 0 && "The Button must be initialized");

    return self -> m_isPressed;
}
```
**Etat bouton**

Cette fonction booléenne permet de déterminer si le bouton est pressé ou non.

5.``static`` void _Button_cb(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);

```c
static void _Button_cb(
    int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
    Button *self = (Button *)user;

    printf("I am pressed");
    self->m_currCbCount+=1;
}
```
**Callback du bouton**

Un callback qui s'exécute lorsqu'un bouton est pressé et qui convertit user en un pointeur Button * pour accéder aux données du bouton. Affiche "I am pressed". Incrémente m_currCbCount pour compter le nombre de pressions.

## FPS.c

1. ``void`` FPS_init(FPS *self, int pi);
   
```c
void FPS_init(FPS *self, int pi)
{
    assert(self);
    assert(pi >= 0);
    memset(self, 0, sizeof(*self));

    self->m_pi = pi;
    self->m_prevUpdateTick = get_current_tick(self->m_pi);
}
```
**Initialisation des varibles**

2. ``void`` FPS_quit(FPS *self);

```c
void FPS_quit(FPS *self)
{
    assert(self && self->m_pi >= 0 && "The FPS must be initialized");
    memset(self, 0, sizeof(*self));
    self->m_pi = -1;
}
```

**Quitter proprement le code**

3. ``void`` FPS_update(FPS *self);

```c
void FPS_update(FPS *self)
{
    assert(self && self->m_pi >= 0 && "The FPS must be initialized");

    self->m_count+=1;
    self->m_accu += get_current_tick(self->m_pi) - self->m_prevUpdateTick;

    if(self->m_accu >= 5000000) {
        printf("fps : %d\n", self->m_count/5);
        self->m_count = 0;
        self->m_accu %= 5000000;
    }
    
    self->m_prevUpdateTick = get_current_tick(self->m_pi);
}
```
**Mise à jour des FPS**

```console
 self->m_accu %= 5000000;
```

On utilise un modulo car m_accu est une durée cela veut dire que l'on ne peut pas l'ré-initialiser à zéro n'importe comment il faut utiliser un modulo.

## Led.c

1. ``void`` LED_init(LED *self, int pi, int gpio);

```c
void LED_init(LED *self, int pi, int gpio)
{
    assert(self);
    assert(pi >= 0);
    assert(0 <= gpio && gpio < 32);
    memset(self, 0, sizeof(*self));

    self->m_pi = pi;
    self->m_gpio = gpio;
    self->m_cycleTime = 1000000;
    self->m_prevUpdateTick = get_current_tick(self->m_pi);
    
    set_mode(self->m_pi, self->m_gpio, PI_OUTPUT);
}
```
**Initilisation de la fonction** 

``m_cycletime`` en micro-seconde

2.``void`` LED_quit(LED *self);

```c
void LED_quit(LED *self)
{
    assert(self && self->m_pi >= 0 && "The LED must be initialized");

    gpio_write(self -> m_pi, self ->  m_gpio, PI_LOW);
    
    memset(self, 0, sizeof(*self));
    self->m_pi = -1;
}
```
**Quitter le code proprement**

3.``void`` LED_update(LED *self);

```c
void LED_update(LED *self)
{
    assert(self && self->m_pi >= 0 && "The LED must be initialized");

    if(LED_isBlinking(self))
    {
        self->m_accu += get_current_tick(self->m_pi) - (self->m_prevUpdateTick);

        if(self->m_accu >= self->m_cycleTime)
        {
            if(self-> m_cycleIndex % 2 == 0)
            {
                 gpio_write(self -> m_pi, self -> m_gpio, PI_LOW);
            }
            else
            {
                 gpio_write(self -> m_pi, self -> m_gpio, PI_HIGH);
            }
            self->m_cycleIndex++;
            if(self-> m_cycleIndex == self ->  m_cycleCount){
                gpio_write(self -> m_pi, self -> m_gpio, PI_LOW);   
            }
        }
        self->m_accu=self->m_accu%self->m_cycleTime;
        self->m_prevUpdateTick = get_current_tick(self->m_pi);    
    }
}
```
**isBlinking**

```console
if(LED_isBlinking(self))
```

Cette condition sert à lancer la mise à jour uniquement lorsque la LED est allumée.

**Modulo 2**

```console
 if(self-> m_cycleIndex % 2 == 0)
```

Cette condition permet de différencier lorsque la LED doit être éteinte ou allumée.

4.``void`` LED_blink(LED *self, int count, float cycleTime);

```c
void LED_blink(LED *self, int count, float cycleTime)
{
    assert(self && self->m_pi >= 0 && "The LED must be initialized");

    self->m_accu = 0;
    self->m_cycleCount = count*2;
    self->m_cycleTime = (cycleTime*1000000)/2;
    self->m_cycleIndex = 0;
    
    gpio_write(self -> m_pi, self -> m_gpio, PI_HIGH);
    self->m_prevUpdateTick = get_current_tick(self->m_pi); 
}
```
**Lancer le clignotement de la LED**

5.``bool`` LED_isBlinking(LED *self);

```c
bool LED_isBlinking(LED *self)
{
    assert(self && self->m_pi >= 0 && "The LED must be initialized");

    // TODO : vérifier si les cycles sont terminés
    if(self ->  m_cycleIndex < self ->  m_cycleCount)
    {
        return true;
    
    }
    return false;
}
```
**Test pour vérifier si la LED clignote**

## Ultrasonic_Sensor.c

1.``void`` UltrasonicSensor_init(UltrasonicSensor *self, int pi, int gpioTrig, int gpioEcho);

```c
void UltrasonicSensor_init(
    UltrasonicSensor *self, int pi, int gpioTrig, int gpioEcho)
{
    assert(self);
    assert(pi >= 0);
    assert(0 <= gpioTrig && gpioTrig < 32);
    assert(0 <= gpioEcho && gpioEcho < 32);
    memset(self, 0, sizeof(*self));

    uint32_t currTick = get_current_tick(self->m_pi);

    self->m_pi = pi;
    self->m_gpioTrig = gpioTrig;
    self->m_gpioEcho = gpioEcho;
    self->m_distance = SENSOR_MAX_DISTANCE_CM;
    self->m_accu = SENSOR_MAX_STEP_TIME + 1;
    self->m_prevUpdateTick = currTick;

    set_mode(pi, gpioTrig, PI_OUTPUT);
    set_mode(pi, gpioEcho, PI_INPUT);

    self->m_callbackID = callback_ex(
        self->m_pi, self->m_gpioEcho, EITHER_EDGE,
        _UltrasonicSensor_cbEcho, self
    );
    assert(self->m_callbackID >= 0);
}
```
**Initalisation des capteurs**

2. ``void`` UltrasonicSensor_quit(UltrasonicSensor *self);

```c
void UltrasonicSensor_quit(UltrasonicSensor *self)
{
    assert(self && self->m_pi >= 0 && "The UltrasonicSensor must be initialized");
    gpio_write(self->m_pi, self->m_gpioTrig, PI_LOW);
    
    assert(self->m_callbackID >= 0);
    int exitStatus = callback_cancel(self->m_callbackID);
    assert(exitStatus == 0);

    memset(self, 0, sizeof(*self));
    self->m_pi = -1;
}
```
**Fermer le code proprement**

3.``void`` UltrasonicSensor_triggerOnce(UltrasonicSensor *self);

```c
void UltrasonicSensor_triggerOnce(UltrasonicSensor *self)
{
    assert(self && self->m_pi >= 0 && "The UltrasonicSensor must be initialized");

    self->m_accu = 0;
    self->m_echoTick = 0;
    self->m_trigTick = get_current_tick(self->m_pi);
    self->m_echoStarted = false;
    self->m_echoFinished = false;
    

    gpio_trigger(self->m_pi, self->m_gpioTrig, 11, PI_HIGH);

    self->m_prevUpdateTick = get_current_tick(self->m_pi);
}
```
**Déclenche une impulsion**

4.``void`` UltrasonicSensor_update(UltrasonicSensor *self);

```c
void UltrasonicSensor_update(UltrasonicSensor *self)
{
    assert(self && self->m_pi >= 0 && "The UltrasonicSensor must be initialized");
    self->m_accu += get_current_tick(self->m_pi) - (self->m_prevUpdateTick);
    self->m_hasNewDistance = false;


    if(self->m_accu >= SENSOR_MAX_STEP_TIME){
        //self->m_accu=self->m_accu%self->m_cycleTime;
        self->m_distance = SENSOR_MAX_DISTANCE_CM;
        self->m_hasNewDistance = true;
        UltrasonicSensor_triggerOnce(self);
        //renvoyer un courent
    }else if(self->m_accu >= SENSOR_STEP_TIME && self->m_echoFinished){
        self->m_hasNewDistance = true;
        UltrasonicSensor_triggerOnce(self);
    }
    
    self->m_prevUpdateTick = get_current_tick(self->m_pi);
}
```

**Mise à jour des capteurs**

5.``bool`` UltrasonicSensor_hasNewDistance(UltrasonicSensor *self);

```c
bool UltrasonicSensor_hasNewDistance(UltrasonicSensor *self)
{
    assert(self && self->m_pi >= 0 && "The UltrasonicSensor must be initialized");

    return self->m_hasNewDistance;
}
```
**Recois un nouvelle distance**

6. ``float`` UlrasonicSensor_getDistance(UltrasonicSensor *self);

```c
float UlrasonicSensor_getDistance(UltrasonicSensor *self)
{
    assert(self && self->m_pi >= 0 && "The UltrasonicSensor must be initialized");
    
    return self->m_distance;
}
```
**Revoi la distance capter**

7.``static`` void _UltrasonicSensor_cb1Echo(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);

```c
static void _UltrasonicSensor_cbEcho(
    int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
    UltrasonicSensor *self = (UltrasonicSensor *)user;

    if(level == PI_HIGH && self->m_echoStarted == false){
        self->m_echoStarted = true;
        self->m_echoTick = tick;    //echo = time
        self->m_hasNewDistance = false;
    }else{
        self->m_distance = (tick - self->m_echoTick)*0.017015f;
        self->m_echoFinished = true;           //temps - echo
        self->m_hasNewDistance = true;
    }    
}
```
**Callback**

## motor_controller.c

1.``void`` MotorController_init(MotorController *self, int pi, int gpioForward, int gpioBackward, int gpioControl);

```c
void MotorController_init(MotorController *self, int pi, int gpioForward, int gpioBackward, int gpioControl)
{
    assert(self);
    assert(pi >= 0);
    assert(0 <= gpioForward && gpioForward < 32);
    assert(0 <= gpioBackward && gpioBackward < 32);
    assert(0 <= gpioControl && gpioControl < 32);
    memset(self, 0, sizeof(*self));

    uint32_t currTick = get_current_tick(self->m_pi);

    self->m_pi = pi;
    self->m_gpioForward = gpioForward;
    self->m_gpioBackward = gpioBackward;
    self->m_gpioControl = gpioControl;
    self->m_gpioMotor = gpioForward;
    self->m_callbackID = -1;

    self->m_kp = 4.f;
    self->m_ki = 4.f;
    
    self->m_cbCount = 0;
    self->m_prevCbTick = currTick;
    self->m_prevUpdateTick = currTick;
    // On laisse passer 4 fentes avant de commencer la régulation
    self->m_controllerCbCount = self->m_cbCount + 4;

    set_mode(self->m_pi, self->m_gpioForward, PI_OUTPUT);
    set_mode(self->m_pi, self->m_gpioBackward, PI_OUTPUT);
    set_mode(self->m_pi, self->m_gpioControl, PI_INPUT);
    set_pull_up_down(self->m_pi, self->m_gpioControl, PI_PUD_DOWN);

    self->m_callbackID = callback_ex(
        self->m_pi, self->m_gpioControl, RISING_EDGE,
        _MotorController_cb, self
    );
    assert(self->m_callbackID >= 0);
}
```
**Initialisation des moteurs**

2. ``void`` MotorController_quit(MotorController *self);

```c
void MotorController_quit(MotorController *self)
{
    assert(self && self->m_pi >= 0 && "The MotorController must be initialized");
    gpio_write(self->m_pi, self->m_gpioForward, PI_LOW);
    gpio_write(self->m_pi, self->m_gpioBackward, PI_LOW);
    
    assert(self->m_callbackID >= 0);
    int exitStatus = callback_cancel(self->m_callbackID);
    assert(exitStatus == 0);

    memset(self, 0, sizeof(*self));
    self->m_pi = -1;
}
```
**Quitter le code proprement**

3.``void`` MotorController_setTargetSpeed(MotorController *self, float speed);

```c
void MotorController_setTargetSpeed(MotorController *self, float speed)
{
    assert(self && self->m_pi >= 0 && "The MotorController must be initialized");
    
    if (speed < MIN_SPEED)
    {
        MotorController_stop(self);
    }
    else
    {
        if (self->m_targetSpeed < MIN_SPEED)
        {
            // Le moteur est actuelement arrêté
            // Il faut le redémarrer
            self->m_controllerCbCount = self->m_cbCount + 4;
            self->m_integral = self->m_startPower;
            self->m_power = self->m_startPower;
        }
        self->m_targetSpeed = speed;
    }
}
```
**Initialiser une vitesse cible à atteindre**

4.``void`` MotorController_setController(MotorController *self, float kp, float ki);

```c
void MotorController_setController(MotorController *self, float kp, float ki)
{
    assert(self && self->m_pi >= 0 && "The MotorController must be initialized");
    self->m_kp = kp;
    self->m_ki = ki;
}
```

**Initialiser les variables Kp et Ki**

* Kp corresponds à la constante proportionelle
* Ki corresponds à la constante intégrale

5. ``void`` MotorController_setStartPower(MotorController *self, int startPower);

```c
void MotorController_setStartPower(MotorController *self, int startPower)
{
    assert(self && self->m_pi >= 0 && "The MotorController must be initialized");
    self->m_startPower = startPower;
}
```
**Initialisation de la puissance moteur**

6.``void`` MotorController_setBackward(MotorController *self, bool goBackward);

```c
void MotorController_setBackward(MotorController *self, bool goBackward)
{
    assert(self && self->m_pi >= 0 && "The MotorController must be initialized");
    
    if (goBackward)
    {
        if(self->m_gpioMotor != self->m_gpioBackward){
           MotorController_stop(self);
            self->m_gpioMotor = self->m_gpioBackward; 
        }
        
    }else{
        if(self->m_gpioMotor != self->m_gpioForward){
            MotorController_stop(self);
            self->m_gpioMotor = self->m_gpioForward;
        }
        
    }
}
```
**Marche arrière**

7. .``void`` MotorController_stop(MotorController *self);

```c
void MotorController_stop(MotorController *self)
{
    assert(self && self->m_pi >= 0 && "The MotorController must be initialized");
    gpio_write(self->m_pi, self->m_gpioForward, PI_LOW);
    gpio_write(self->m_pi, self->m_gpioBackward, PI_LOW);
    self->m_targetSpeed = 0.f;
}
```
**Quitter proprement le code**

8. .``void`` MotorController_update(MotorController *self);

```c
void MotorController_update(MotorController *self)
{
    assert(self && self->m_pi >= 0 && "The MotorController must be initialized");
    
    if(self->m_targetSpeed == 0){
       return; 
    }
        if(self->m_controllerCbCount > self->m_cbCount){
        set_PWM_dutycycle(self->m_pi, self->m_gpioMotor, self->m_startPower);
    }else{
        float e = (self->m_targetSpeed - self->m_speed);
        float dt = (get_current_tick(self->m_pi)-self->m_prevUpdateTick)/1000000.f;
        self->m_prevUpdateTick = get_current_tick(self->m_pi);
        
        self->m_integral += self->m_ki*(e * dt);

        self->m_power = self->m_kp*e + self->m_integral;

        if (self->m_power >= 255)
        {
            self->m_saturationTime+=dt;
        }

        set_PWM_dutycycle(self->m_pi, self->m_gpioMotor, self->m_power);
    }
}
```
**Mise à jour moteur**

9.``float`` MotorController_getSpeed(MotorController *self);

```c
float MotorController_getSpeed(MotorController *self)
{
    if (self->m_cbCount < self->m_controllerCbCount) return self->m_targetSpeed;
    return self->m_speed;
}
```
**Vitesse actuelle**

10. ``float`` MotorController_getDistance(MotorController *self);

```c
float MotorController_getDistance(MotorController *self)
{
    return (float)(self->m_cbCount) * 18.8495556f / 20.f;
}
```
**Retourne la distance parcourue par la roue depuis le lancement**

11.``static`` void _MotorController_cb(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);

```c
static void _MotorController_cb(
    int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
    MotorController *self = (MotorController *)user;

    if (tick - self->m_prevCbTick < 10)
    {
        // Erreur de callback
        return;
    }

    float speed = 1000000.f / (float)(tick - self->m_prevCbTick);
    if (speed >= 120.f)
    {
        // Erreur de mesure
        self->m_errorCount++;
        return;
    }

    self->m_speed = speed;
    self->m_prevCbTick = tick;
    self->m_cbCount++;
}
```
**Callback**

# 📚 Fichier.h

|  Fichiers                                                                                                        | Description                                 |
|------------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| [button.h](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/button.h)                       | Librarie du bonton sur l'add-board          |
| [fps.h](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/fps.h)                             | Librarie des Frames per second              |
| [input.h](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/input.h)                        | Librarie des inputs de la manettes          |
| [led.h](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/led.h)                             | Librarie de la led                          |
| [motor_controller.h](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/motor_controller.h)   | Librarie des moteurs                        |
| [tools.h](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/tools.h)                         | Librarie des Outils                         |
| [ultrasonic_sensor.h](https://github.com/Polaerz/PermisEsieaBot/blob/main/Basecode/Basecode/ultrasonic_sensor.h) | Librarie des capteurs                       |











