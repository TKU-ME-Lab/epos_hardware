#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include "maxon_hardware/Definitions.h"


int GetDeviceNameList(std::vector<std::string>*, unsigned int* );
int GetProtocolStackNameList(const std::string, std::vector<std::string>*, unsigned int*);
int GetInterfaceNameList(const std::string, const std::string, std::vector<std::string>*, unsigned int*);
int GetPortNameList(const std::string, const std::string, const std::string, std::vector<std::string>*, unsigned int*);
