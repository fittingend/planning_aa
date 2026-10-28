#include "otaclient/aa/OtacStatusStore.h"

namespace otaclient
{
namespace aa
{

OtacStatusStore::OtacStatusStore(const OtaClientConfig& config)
    : m_clientName(config.clientName)
{
    m_appNames.reserve(config.apps.size());
    for (const auto& app : config.apps)
    {
        m_appNames.push_back(app.name);
        m_appStatus.emplace(
            app.name,
            AppStatusRecord{app.status, std::chrono::system_clock::time_point{}});
    }
}

std::string OtacStatusStore::ClientName() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_clientName;
}

std::vector<std::string> OtacStatusStore::AppNames() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_appNames;
}

bool OtacStatusStore::AcceptRegistration(
    const std::string& responseClientName, bool succeeded,
    std::int32_t clientId)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!succeeded || responseClientName != m_clientName || clientId < 0)
    {
        return false;
    }

    m_clientId = clientId;
    m_registered = true;
    return true;
}

void OtacStatusStore::InvalidateRegistration()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_registered = false;
    m_clientId = -1;
}

bool OtacStatusStore::IsRegistered() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_registered;
}

std::int32_t OtacStatusStore::ClientId() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_clientId;
}

StatusUpdateResult OtacStatusStore::UpdateStatus(
    std::int32_t clientId, const std::string& appName,
    const std::string& status)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_registered)
    {
        return StatusUpdateResult::kNotRegistered;
    }
    if (clientId != m_clientId)
    {
        return StatusUpdateResult::kClientIdMismatch;
    }

    const auto found = m_appStatus.find(appName);
    if (found == m_appStatus.end())
    {
        return StatusUpdateResult::kUnknownApp;
    }

    found->second.status = status;
    found->second.updatedAt = std::chrono::system_clock::now();
    return StatusUpdateResult::kUpdated;
}

bool OtacStatusStore::GetAppStatus(const std::string& appName,
                                   AppStatusRecord& record) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto found = m_appStatus.find(appName);
    if (found == m_appStatus.end())
    {
        return false;
    }

    record = found->second;
    return true;
}

} // namespace aa
} // namespace otaclient
