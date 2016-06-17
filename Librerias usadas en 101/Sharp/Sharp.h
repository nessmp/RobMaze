#ifndef SHARP_H
#define SHARP_H

#include <Arduino.h>

class Sharp
{
    public:
        /** Default constructor */
        Sharp();
        Sharp(int irPin, int maxDistance);
        /** Default destructor */
        virtual ~Sharp();
        int distance();
        int cm();

    protected:

    private:
        int irPin
        int maxDistance
};

#endif // SHARP_H
