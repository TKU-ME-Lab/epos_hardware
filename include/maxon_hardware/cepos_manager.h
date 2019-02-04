#include "maxon_hardware/cepos.h"

class CEposManager{
private:
    std::map<std::string, boost::shared_ptr<CEpos> > m_motors;



public:
    CEposManager(std::vector<EposParameter>);

    std::map<std::string, boost::shared_ptr<CEpos> > GetMotors();

};
