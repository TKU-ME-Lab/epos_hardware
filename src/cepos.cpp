#include "maxon_hardware/cepos.h"

#define MAX_STRING_SIZE 1000

int GetDeviceNameList(std::vector<std::string>* device_names, unsigned int* error_code){
    char buffer[MAX_STRING_SIZE];
    int end_of_selection; //BOOL
    int result;

    result = VCS_GetDeviceNameSelection(true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
        return result;
    device_names->push_back(buffer);

    while(!end_of_selection) {
        result = VCS_GetDeviceNameSelection(false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
        if(!result)
        return result;
        device_names->push_back(buffer);
    }

  return 1;
}

int GetProtocolStackNameList(const std::string device_name, std::vector<std::string>* protocol_stack_names, unsigned int* error_code){
    char buffer[MAX_STRING_SIZE];
    int end_of_selection; //BOOL
    int result;

    result = VCS_GetProtocolStackNameSelection((char*)device_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
     return result;
    protocol_stack_names->push_back(buffer);

    while(!end_of_selection) {
    result = VCS_GetProtocolStackNameSelection((char*)device_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
        if(!result)
        return result;
        protocol_stack_names->push_back(buffer);
    }

  return 1;
}

int GetInterfaceNameList(const std::string device_name, const std::string protocol_stack_name, std::vector<std::string>* interface_names, unsigned int* error_code){
    char buffer[MAX_STRING_SIZE];
    int end_of_selection; //BOOL
    int result;

    result = VCS_GetInterfaceNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
        return result;
    interface_names->push_back(buffer);

    while(!end_of_selection) {
        result = VCS_GetInterfaceNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
        if(!result)
        return result;
        interface_names->push_back(buffer);
    }

    return 1;
}

int GetPortNameList(const std::string device, const std::string protocol, const std::string interface, std::vector<std::string>* ports, unsigned int* error_code){
    char buffer[MAX_STRING_SIZE];
    int end_of_selection = false;
    int result = VCS_GetPortNameSelection((char*)device.c_str(), (char*)protocol.c_str(), (char*)interface.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if (!result){
        return result;
    }
    ports->push_back(buffer);

    while(!end_of_selection){
        result = VCS_GetPortNameSelection((char*)device.c_str(), (char*)protocol.c_str(), (char*)interface.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
        if (!result){
            return result;
        }
        ports->push_back(buffer);
    }
}

