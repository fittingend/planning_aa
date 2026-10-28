#ifndef OTACLIENT_AA_OTAC_MAIN_H
#define OTACLIENT_AA_OTAC_MAIN_H

#include "ara/log/logger.h"
#include "dds/dds.hpp"
#include "otaclient/aa/OtaClientConfig.h"
#include "otaclient/aa/OtacManagerService.h"
#include "otaclient/aa/OtacStatusStore.h"
#include "para/swc/port_pool.h"

#include <atomic>
#include <csignal>
#include <memory>

namespace otaclient
{
namespace aa
{

class OtacMain
{
public:
    OtacMain();
    ~OtacMain();

    bool Initialize();
    void Start();
    void Terminate();
    void Run(const volatile std::sig_atomic_t* terminationRequested = nullptr);

private:
    std::atomic<bool> m_running{false};
    ::para::swc::PortPool m_workers;
    ara::log::Logger& m_logger;
    OtaClientConfig m_config;
    dds::domain::DomainParticipant m_participant{dds::core::null};
    std::shared_ptr<OtacStatusStore> m_statusStore;
    std::unique_ptr<OtacManagerService> m_managerService;
};

} // namespace aa
} // namespace otaclient

#endif
