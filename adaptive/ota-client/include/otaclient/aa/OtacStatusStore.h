#ifndef OTACLIENT_AA_OTAC_STATUS_STORE_H
#define OTACLIENT_AA_OTAC_STATUS_STORE_H

#include "otaclient/aa/OtaClientConfig.h"

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace otaclient
{
namespace aa
{

struct AppStatusRecord
{
    std::string status;
    std::chrono::system_clock::time_point updatedAt;
};

enum class StatusUpdateResult
{
    kUpdated,
    kNotRegistered,
    kClientIdMismatch,
    kUnknownApp
};

class OtacStatusStore
{
public:
    explicit OtacStatusStore(const OtaClientConfig& config);

    std::string ClientName() const;
    std::vector<std::string> AppNames() const;

    bool AcceptRegistration(const std::string& responseClientName,
                            bool succeeded, std::int32_t clientId);
    void InvalidateRegistration();
    bool IsRegistered() const;
    std::int32_t ClientId() const;

    StatusUpdateResult UpdateStatus(std::int32_t clientId,
                                    const std::string& appName,
                                    const std::string& status);
    bool GetAppStatus(const std::string& appName, AppStatusRecord& record) const;

private:
    mutable std::mutex m_mutex;
    std::string m_clientName;
    std::vector<std::string> m_appNames;
    std::map<std::string, AppStatusRecord> m_appStatus;
    bool m_registered{false};
    std::int32_t m_clientId{-1};
};

} // namespace aa
} // namespace otaclient

#endif
