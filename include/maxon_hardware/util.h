#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include <maxon_hardware/Definitions.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

typedef void* HANDLE;

bool SerialNumberFromHex(const std::string&, uint64_t*);

int GetErrorInfo(unsigned int, std::string*);

int GetDeviceNameList(std::vector<std::string>*, unsigned int*);

int GetProtocolStackNameList(const std::string, std::vector<std::string>*, unsigned int*);

int GetInterfaceNameList(const std::string, const std::string, std::vector<std::string>*, unsigned int*);

int GetPortNameList(const std::string, const std::string, const std::string, std::vector<std::string>*, unsigned int*);

int GetBaudrateList(const std::string, const std::string, const std::string, const std::string, std::vector<unsigned int>, unsigned int*);

int CreateDeviceKeyHandle(HANDLE*);

int EnumerateDeviceKeyHandle();