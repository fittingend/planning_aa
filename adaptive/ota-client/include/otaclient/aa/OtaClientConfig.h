#ifndef OTACLIENT_AA_OTA_CLIENT_CONFIG_H
#define OTACLIENT_AA_OTA_CLIENT_CONFIG_H

#include <cstdint>
#include <string>
#include <vector>

namespace otaclient
{
namespace aa
{

struct OtaApplicationConfig
{
    std::string name;
    std::string status;
};

struct OtaClientConfig
{
    std::string clientName;
    std::vector<OtaApplicationConfig> apps;
    std::uint32_t registrationRetryMs{2000U};

    static OtaClientConfig LoadFromFile(const std::string& path);
    static std::string ResolveConfigPath();
    static bool ParseDomainId(const char* value, std::uint32_t& domainId,
                              std::string& error);

    std::vector<std::string> AppNames() const;
};

} // namespace aa
} // namespace otaclient

#endif
