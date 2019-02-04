#include "maxon_hardware/Definitions.h"
#include "maxon_hardware/util.h"

typedef struct{
    std::string motor_name;
    std::string actuator;
    std::string protocol;
    std::string interface;
    uint16_t nodeid;
    std::string serial_number;
    std::string mode;
    bool clear_fault;
    bool is_sub_device;
    std::string master_device;
}EposParameter;

class CEpos {
    typedef enum{
        PROFILE_POSITION_MODE = 1,
        PROFILE_VELOCITY_MODE = 3
    } OperationMode;
    
    typedef void* HANDLE;

private:
    HANDLE m_keyhandle;
    std::string m_device_name;
    std::string m_serial_number;
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
    CEpos(const EposParameter);
    CEpos(const EposParameter, const HANDLE);
    ~CEpos();

    bool init();
    void write();
    void read();

    std::string device_name() {return m_device_name;}

    double* GetPosition();
    double* GetPositionCmd();
    double* GetVelocity();
    double* GetVelocityCmd();
    double* GetEffort();
    
};


