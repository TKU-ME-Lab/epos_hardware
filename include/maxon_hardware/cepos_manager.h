#include "maxon_hardware/cepos.h"
#include <boost/ptr_container/ptr_map.hpp>

typedef std::map<std::string, boost::shared_ptr<CEpos> > MapMotor;

class CEposManager{
private:
    MapMotor m_motors;
    //MapMotor m_motors;

public:
    CEposManager(std::vector<EposParameter>);

    MapMotor GetMotors();
    MapMotor* GetMotorsPtr() {return &m_motors;}

    bool init();
    void read();
    void write();
    
};
