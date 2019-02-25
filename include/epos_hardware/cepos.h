#include "epos_hardware/Definitions.h"
#include "epos_hardware/util.h"

#define STATUSWORD(b, v) ((v >> b) & 1)
#define READY_TO_SWITCH_ON    (0)
#define SWITCHED_ON           (1)
#define ENABLE                (2)
#define FAULT                 (3)
#define VOLTAGE_ENABLED       (4)
#define QUICK_STOP            (5)
#define WARNING               (7)
#define TARGET_REACHED        (10)
#define CURRENT_LIMIT_ACTIVE  (11)

typedef struct
{
    unsigned int velocity;
    unsigned int acceleration;
    unsigned int deceleration;
}PositionProfile;

typedef struct
{
    unsigned int acceleration;
    unsigned int deceleration;
}VelocityProfile;

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

    //Profile config
    PositionProfile position_profile;
    VelocityProfile velocity_profile;

}EposParameter;

typedef enum{
    PROFILE_POSITION_MODE = 1,
    PROFILE_VELOCITY_MODE = 3
} OperationMode;

class CEpos {   
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

    int m_max_profile_velocity = 65535;
    
    bool m_halt_velocity;

    std::string motor_type;
    double m_torque_constant;
    double m_nominal_current;
    double m_max_current;

    unsigned short m_statusword;

public:
    CEpos(const EposParameter);
    CEpos(const EposParameter, const HANDLE);
    ~CEpos();

    void write();
    void read();

    HANDLE GetHANDLE() {return m_keyhandle;}

    std::string device_name() {return m_device_name;}
    std::string serial_number() {return m_serial_number;}
    unsigned int GetID() {return m_nodeid;}

    OperationMode GetMode() {return m_OperationMode;}
    unsigned short statusword() {return m_statusword;}
    bool hasInit() {return m_has_init;}

    double GetPosition() {return m_position;}
    double GetPositionCmd() {return m_position_cmd;}
    double GetVelocity() {return m_velocity;}
    double GetVelocityCmd() {return m_velocity_cmd;}
    double GetCurrent() {return m_current;}
    double GetEffort() {return m_effort;}
    double GetNominalCurrent() {return m_nominal_current;}
    double GetMaxCurrent() {return m_max_current;}


    double* GetPositionPtr();
    double* GetPositionCmdPtr();
    double* GetVelocityPtr();
    double* GetVelocityCmdPtr();
    double* GetEffortPtr();
    
};


