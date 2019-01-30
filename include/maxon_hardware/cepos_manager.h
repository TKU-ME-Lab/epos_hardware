#include "maxon_hardware/cepos.h"

struct EposParameter{
    std::string motor_name;
    std::string actuator;
    std::string protocol;
    std::string interface;
    uint64_t nodeid;
    std::string serial_number;
    std::string mode;
    bool clear_fault;
    bool is_sub_device;
    std::string master_device;
};


class CEposManager{
private:
    std::map<std::string, boost::shared_ptr<CEpos> > m_motors;



public:
    CEposManager(std::vector<EposParameter>);

    std::map<std::string, boost::shared_ptr<CEpos> > GetMotors();

};
