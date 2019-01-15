#include <boost/foreach.hpp>
#include <iostream>
#include <vector>
#include <string>
#include "maxon_hardware/Definitions.h"

#define MAX_STRING_SIZE 100

int GetDeviceNameList(std::vector<std::string>* device_name, unsigned int* error_code){
    char buffer[MAX_STRING_SIZE];
    int end_of_selection = false;
    int result;

    result = VCS_GetDeviceNameSelection(true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if (!result){
        return result;
    }
    device_name->push_back(buffer);

    while(!end_of_selection){
        result = VCS_GetDeviceNameSelection(false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
        if (!result){
            return result;
        }
        device_name->push_back(buffer);
    }
}

int GetProtocolStackNameList(const std::string device, std::vector<std::string>* protocol_statck_namse, unsigned int* error_code){
    char buffer[MAX_STRING_SIZE];
    bool end_of_selection = false;
    int result;
    

    //result = VCS_GetProtocolStackNameSelection()
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

int main(void){
    std::vector<std::string> device_names;
    std::vector<std::string> protocol_names;
    unsigned int error_code;

    if (GetDeviceNameList(&device_names, &error_code)){
        BOOST_FOREACH(std::string device, device_names){    
            if (GetProtocolStackNameList(device, &protocol_names, &error_code)){

            }
        }
    }
    //GetProtocolStackNameList(device_name[1], &protocol_names, &error_code);

    // for (int index = 0; index < device_name.size(); index++){
    //     std::cout << device_name[index] << std::endl;
    // }
    // BOOST_FOREACH(std::string device, device_name){
    //     std::cout << device << std::endl;
    // }

    return 0;
}