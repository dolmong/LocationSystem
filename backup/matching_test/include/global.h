#ifndef GLOBAL_H
#define GLOBAL_H

typedef struct _InterestPoint
{
        float   _x;
        float   _y;
        float   _scale;
        int     _Lap;
        float   _majordiretion;
        float   _IpDescriptor[64];
        float   _dx;
        float   _dy;
        int     _filter;
        int     _chkOverlap;
        float   _height;
        float   _vx;
        float   _vy;
        float   _dis;
} Interestpoint;

#endif
