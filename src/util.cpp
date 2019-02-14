#include "maxon_hardware/util.h"
#include <boost/foreach.hpp>
#include <sstream>

#define MAX_STRING_SIZE 1000

bool SerialNumberFromHex(const std::string &str, uint64_t* serial_number){
    std::stringstream ss;
    ss << std::hex << str;
    ss >> *serial_number;
    return true;
}

int GetErrorInfo(unsigned int error_code, std::string* error_string){
    char buffer[MAX_STRING_SIZE];
    int result = VCS_GetErrorInfo(error_code, buffer, MAX_STRING_SIZE);
    if (result){
        *error_string = buffer;
    }
    return result;
}

int GetDeviceNameList(std::vector<std::string>* device_names, unsigned int* error_code){
    char buffer[MAX_STRING_SIZE];
    int end_of_selection;
    int result = VCS_GetDeviceNameSelection(true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result){
        return result;
    }

    device_names->push_back(buffer);

    while(!end_of_selection){
        result = VCS_GetDeviceNameSelection(false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
        if (!result){
            return result;
        }
        device_names->push_back(buffer);
    }

    return 1;
}

int GetProtocolStackNameList(const std::string device_name, std::vector<std::string>* protocol_stack_names, unsigned int* error_code){
    char buffer[MAX_STRING_SIZE];
    int end_of_selection;
    int result = VCS_GetProtocolStackNameSelection((char*)device_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if (!result){
        return result;
    }

    protocol_stack_names->push_back(buffer);

    while(!end_of_selection){
        result = VCS_GetProtocolStackNameSelection((char*)device_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
        if (!result){
            return result;
        }
        protocol_stack_names->push_back(buffer);
    }
    return 1;
}

int GetIntefaceNameList(const std::string device_name, const std::string protocol_stack_name, std::vector<std::string>* interface_names, unsigned int* error_code){
    char buffer[MAX_STRING_SIZE];
    int end_of_selection;
    int result = VCS_GetInterfaceNameSelection((char*)device_name.c_str(), (char*) protocol_stack_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if (!result){
        return result;
    }
    interface_names->push_back(buffer);

    while(!end_of_selection){
        result = VCS_GetInterfaceNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
        if (!result){
            return result;
        }
        interface_names->push_back(buffer);
    }
    return 1;
}

int GetPortNameList(const std::string device_name, const std::string protocol_stack_name, const std::string inteface_name, 
                    std::vector<std::string>* port_names, unsigned int* error_code){
    char buffer[MAX_STRING_SIZE];
    int end_of_selection;
    int result = VCS_GetPortNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)inteface_name.c_str(),
                                            true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);

    if (!result){
        return result;
    }

    port_names->push_back(buffer);

    while(!end_of_selection){
        result = VCS_GetPortNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)inteface_name.c_str(),
                                            false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
        if (!result){
            return result;
        }
        port_names->push_back(buffer);
    }

    return 1;
}

int GetBaudrateList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
                    const std::string port_name, std::vector<unsigned int>* baudrates, unsigned int* error_code){
    unsigned int baudrate;
    int end_of_selection;
    int result = VCS_GetBaudrateSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), (char*)port_name.c_str(),
                                            true, &baudrate, &end_of_selection, error_code);
    if (!result){
        return result;
    }

    baudrates->push_back(baudrate);

    while(!end_of_selection){
        result = VCS_GetBaudrateSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), (char*)port_name.c_str(),
                                            false, &baudrate, &end_of_selection, error_code);
        if (!result){
            return result;
        }
        baudrates->push_back(baudrate);
    }
}

int CreateDeviceKeyHandle(std::string actuator, std::string protocol, std::string interface, unsigned long nodeid, std::string serial_number, HANDLE* keyhandle){
    std::vector<std::string> port_names;
    
    unsigned int error_code;

    int result = GetPortNameList(actuator, protocol, interface, &port_names, &error_code);

    BOOST_FOREACH(const std::string &port_name, port_names){
        HANDLE handle = VCS_OpenDevice((char*)actuator.c_str(), (char*)protocol.c_str(), (char*)interface.c_str(), (char*)port_name.c_str(), &error_code);
        if (handle){
            unsigned int bytes_read;
            uint64_t Serial_Number_From_Epos;
            uint64_t Serial_Number_From_Config;

            if (!VCS_GetObject(handle, nodeid, 0x2004, 0x00, &Serial_Number_From_Epos, 8, &bytes_read, &error_code)){
                if (SerialNumberFromHex(serial_number, &Serial_Number_From_Config)){
                    if (Serial_Number_From_Epos == Serial_Number_From_Config){
                        memcpy(keyhandle, handle, sizeof(handle));
                        return 1;
                    }
                }    
            }
        }
    }

    return result;
}

int EnumerateDeviceKeyHandle(){

}
