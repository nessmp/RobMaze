#ifndef ULTRASONICO_H
#define ULTRASONICO_H

#include "Arduino.h"

class Ultrasonico
{
    public:
        /** Default constructor */
        Ultrasonico();
        Ultrasonico(int iTrigger, int iEcho);
        /** Default destructor */
        virtual ~Ultrasonico();
        int distance();

    protected:

    private:
        int iTrigger;
        int iEcho;
};

#endif // ULTRASONICO_H
