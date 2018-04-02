#pragma once

#include <Laser.hpp>

class RPLaser : public Laser  
{
    
public:
    
    RPLaser() : 
      Laser(360, 10, 360, 0, 0, 0) { }
    
};

