#include "otaclient/aa/OtacMain.h"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace otaclient
{
namespace aa
{

OtacMain::OtacMain()
    : m_workers(1)
    , m_logger(ara::log::CreateLogger(
          "OTAC", "OtacMain", ara::log::LogLevel::kVerbose))
{
}

OtacMain::~OtacMain()
{
    Terminate();
}

bool OtacMain::Initialize()
{
    const std::string configPath = OtaClientConfig::ResolveConfigPath();
    std::uint32_t domainId = 0U;
    std::string domainError;

    if (!OtaClientConfig::ParseDomainId(
            std::getenv("MMF_DDS_DOMAIN_ID"), domainId, domainError))
    {
        m_logger.LogError() << "[OTAClient] " << domainError.c_str();
        return false;
    }

    try
    {
        m_config = OtaClientConfig::LoadFromFile(configPath);
        m_statusStore = std::make_shared<OtacStatusStore>(m_config);
        m_participant = dds::domain::DomainParticipant(domainId);
        m_managerService = std::make_unique<OtacManagerService>(
            m_participant, m_logger, m_config, m_statusStore);
        m_managerService->Initialize();
    }
    catch (const std::exception& error)
    {
        m_logger.LogError() << "[OTAClient] Initialization failed: "
                            << error.what();
        return false;
    }

    std::cout << "[OTAClient] Configuration loaded: path=" << configPath
              << ", client=" << m_config.clientName
              << ", domain=" << domainId << std::endl;
    m_logger.LogInfo() << "[OTAClient] Configuration loaded: path="
                       << configPath.c_str()
                       << ", client=" << m_config.clientName.c_str()
                       << ", domain=" << domainId;
    m_running.store(true);
    return true;
}

void OtacMain::Start()
{
    m_logger.LogVerbose() << "OtacMain::Start";
}

void OtacMain::Terminate()
{
    m_running.store(false);
    if (m_managerService)
    {
        m_managerService->Stop();
    }
}

void OtacMain::Run(
    const volatile std::sig_atomic_t* terminationRequested)
{
    m_workers.Async([this, terminationRequested] {
        while (m_running.load() &&
               (terminationRequested == nullptr ||
                *terminationRequested == 0))
        {
            try
            {
                if (m_managerService)
                {
                    m_managerService->ProcessOnce();
                }
            }
            catch (const std::exception& error)
            {
                m_logger.LogError() << "[OTAClient] DDS processing error: "
                                    << error.what();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (m_managerService)
        {
            m_managerService->Stop();
        }
        m_running.store(false);
    });

    m_workers.Wait();
}

} // namespace aa
} // namespace otaclient
