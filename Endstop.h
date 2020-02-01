#ifndef ENDSTOP_H
#define ENDSTOP_H

typedef struct Endstop
{

    int pin;
    int state;
    int trigState;

} Endstop;

int
Endstop_init(Endstop *es,
             const int pin,
             const int trigState)
{
    if (es == NULL)
    {
        return -1;
    }

    es->pin       = pin;
    es->trigState = trigState;

    pinMode(es->pin, INPUT_PULLUP);

    return 0;
};

int
Endstop_read(Endstop *es)
{
    if (es == NULL)
    {
        return 0;
    }

    es->state = digitalRead(es->pin);

    return (es->state == es->trigState);
};


#endif // ENDSTOP_H
