<div align="center">
<img src="https://cdn.discordapp.com/attachments/1212852209491116113/1349860905789161553/Logo_Esiea_-_blanc_fond_bleu_1.png?ex=67d4a36f&is=67d351ef&hm=7eab9224bede781da7d8cef37ad0484f6e27d8397b9b143322e5b84533456c3c&" width="300" alt="Concord Logo">
</div>

# Permis Esiea Bot - Classe 10 Internationale
Pour mieux comprendre ces codes, veuillez lire attentivement ce fichier, qui offre une vue d'ensemble du ML Challenge.

## 🚨 Se ne sont pas les codes finaux !!
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

    // TODO : initialiser correctement les champs de la structure pour lancer un clignotement
    self->m_accu = 0;
    self->m_cycleCount = count*2;
    self->m_cycleTime = (cycleTime*1000000)/2;
    self->m_cycleIndex = 0;
    // TODO : allumer la led (mode PI_HIGH)
    
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

## 

