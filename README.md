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

2. ``void`` Button_quit(Button *self)

```c
void Button_quit(Button *self)
{
    // Vérification des paramètres de la fonction
    assert(self && self->m_pi >= 0 && "The MotorController must be initialized");    
    assert(self->m_callbackID >= 0);

    // Annulation du callback
    int exitStatus = callback_cancel(self->m_callbackID);
    assert(exitStatus == 0);

    // Réinitialisation des champs de l'objet
    memset(self, 0, sizeof(*self));
    self->m_pi = -1;
    //
}
```
   
