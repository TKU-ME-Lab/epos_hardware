#include "maxon_hardware/Definitions.h"
#include "maxon_hardware/util.h"

class CEpos {
    typedef enum{
        PROFILE_POSITION_MODE = 1,
        PROFILE_VELOCITY_MODE = 3
    } OperationMode;
    
    typedef void* HANDLE;

private:
    HANDLE m_keyhandle;
    std::string m_motor_name;
    std::string m_device_name;
    unsigned long m_serial_number;
    unsigned int m_nodeid;
    OperationMode m_OperationMode;

    double m_position;
    double m_velocity;
    double m_current;
    double m_effort;

    bool m_has_init;

    double m_position_cmd;
    double m_velocity_cmd;

    int m_max_profile_velocity;
    
    bool m_halt_velocity;

    double m_torque_constant;
    double m_nominal_current;
    double m_max_current;

    unsigned short m_statusword;

public:
    CEpos(const std::string actuator, const std::string protocol, const std::string interface, const int id, 
          const std::string serial_number, const std::string mode, const bool clear_fault);
    ~CEpos();

    bool init();
    void write();
    void read();

    std::string motor_name() {return m_motor_name;}
    std::string device_name() {return m_device_name;}

    double* GetPosition();
    double* GetVelocity();
    double* GetEffort();
};


