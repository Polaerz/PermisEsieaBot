#include "ultrasonic_sensor.h"

static void _UltrasonicSensor_cbEcho(
    int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);

static void UltrasonicSensor_triggerOnce(UltrasonicSensor *self);

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

void UltrasonicSensor_triggerOnce(UltrasonicSensor *self)
{
    assert(self && self->m_pi >= 0 && "The UltrasonicSensor must be initialized");

    // TODO : Réinitialisation des paramètres de mesure
    self->m_accu = 0;
    self->m_echoTick = 0;
    self->m_trigTick = get_current_tick(self->m_pi);
    self->m_echoStarted = false;
    self->m_echoFinished = false;
    

    // TODO : Déclencher la mesure (envoyer impulsion de 11us sur l'entrée Trig)

    gpio_trigger(self->m_pi, self->m_gpioTrig, 11, PI_HIGH);

    self->m_prevUpdateTick = get_current_tick(self->m_pi);
}

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
    // TODO : Gérer le temps avant la prochaine mesure
    
    // TODO : Si besoin, déclencher une mesure

    self->m_prevUpdateTick = get_current_tick(self->m_pi);
}

bool UltrasonicSensor_hasNewDistance(UltrasonicSensor *self)
{
    assert(self && self->m_pi >= 0 && "The UltrasonicSensor must be initialized");
    
    // TODO : Vérifier si on a reçu une nouvelle distance
    return self->m_hasNewDistance;
}

float UlrasonicSensor_getDistance(UltrasonicSensor *self)
{
    assert(self && self->m_pi >= 0 && "The UltrasonicSensor must be initialized");
    
    // TODO : Retourner la distance
    return self->m_distance;
}

static void _UltrasonicSensor_cbEcho(
    int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
    //cast
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

    // TODO : Gérer l'echo reçu par le capteur (début et fin de signal)
    
}
