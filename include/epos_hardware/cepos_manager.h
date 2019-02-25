#include "epos_hardware/cepos.h"
#include <boost/ptr_container/ptr_map.hpp>

typedef std::map<std::string, CEpos* > MapMotor;

class CEposManager{
private:
    MapMotor m_motormap;
    //MapMotor m_motors;
    //std::vector<std::string> m_motor_names;
    //std::vector<CEpos*> m_motors; 

public:
    CEposManager(std::vector<EposParameter>);

    MapMotor GetMotors();
    MapMotor* GetMotorsPtr();

    bool init();
    void read();
    void write();
    
};
