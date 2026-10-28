#include "otaclient/aa/OtaClientConfig.h"

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <cerrno>
#include <cstdlib>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace otaclient
{
namespace aa
{
namespace
{

constexpr std::uint32_t kMinRetryMs = 100U;
constexpr std::uint32_t kMaxRetryMs = 300000U;
constexpr std::uint32_t kMaxDomainId = 232U;

std::string DefaultConfigPath()
{
#ifdef OTA_CLIENT_DEFAULT_CONFIG_PATH
    return OTA_CLIENT_DEFAULT_CONFIG_PATH;
#else
    return "config/ota_client_config.json";
#endif
}

} // namespace

OtaClientConfig OtaClientConfig::LoadFromFile(const std::string& path)
{
    boost::property_tree::ptree root;
    try
    {
        boost::property_tree::read_json(path, root);
    }
    catch (const boost::property_tree::json_parser::json_parser_error& error)
    {
        throw std::runtime_error("cannot parse OTA client config '" + path +
                                 "': " + error.message());
    }

    OtaClientConfig config;
    try
    {
        config.clientName = root.get<std::string>("client_name");
        config.registrationRetryMs =
            root.get<std::uint32_t>("registration_retry_ms");

        std::set<std::string> appNames;
        for (const auto& entry : root.get_child("apps"))
        {
            OtaApplicationConfig app;
            app.name = entry.second.get<std::string>("name");
            app.status = entry.second.get<std::string>("status");

            if (app.name.empty())
            {
                throw std::runtime_error("app name must not be empty");
            }
            if (app.status.empty())
            {
                throw std::runtime_error("status for app '" + app.name +
                                         "' must not be empty");
            }
            if (!appNames.insert(app.name).second)
            {
                throw std::runtime_error("duplicate app name: " + app.name);
            }
            config.apps.push_back(std::move(app));
        }
    }
    catch (const boost::property_tree::ptree_error& error)
    {
        throw std::runtime_error("invalid OTA client config '" + path +
                                 "': " + error.what());
    }

    if (config.clientName.empty())
    {
        throw std::runtime_error("client_name must not be empty");
    }
    if (config.apps.empty())
    {
        throw std::runtime_error("apps must contain at least one application");
    }
    if (config.registrationRetryMs < kMinRetryMs ||
        config.registrationRetryMs > kMaxRetryMs)
    {
        std::ostringstream message;
        message << "registration_retry_ms must be between " << kMinRetryMs
                << " and " << kMaxRetryMs;
        throw std::runtime_error(message.str());
    }

    return config;
}

std::string OtaClientConfig::ResolveConfigPath()
{
    if (const char* configured = std::getenv("OTA_CLIENT_CONFIG"))
    {
        if (*configured != '\0')
        {
            return configured;
        }
    }

    if (const char* paraRoot = std::getenv("PARA_ROOT"))
    {
        if (*paraRoot != '\0')
        {
            return std::string(paraRoot) +
                   "/opt/OTAClient/etc/ota_client_config.json";
        }
    }

    return DefaultConfigPath();
}

bool OtaClientConfig::ParseDomainId(const char* value, std::uint32_t& domainId,
                                    std::string& error)
{
    domainId = 0U;
    error.clear();
    if (value == nullptr || *value == '\0')
    {
        return true;
    }

    errno = 0;
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' || parsed < 0 ||
        parsed > static_cast<long>(kMaxDomainId))
    {
        error = "MMF_DDS_DOMAIN_ID must be an integer between 0 and " +
                std::to_string(kMaxDomainId) + "; got '" + value + "'";
        return false;
    }

    domainId = static_cast<std::uint32_t>(parsed);
    return true;
}

std::vector<std::string> OtaClientConfig::AppNames() const
{
    std::vector<std::string> result;
    result.reserve(apps.size());
    for (const auto& app : apps)
    {
        result.push_back(app.name);
    }
    return result;
}

} // namespace aa
} // namespace otaclient
