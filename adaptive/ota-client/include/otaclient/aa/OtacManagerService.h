#ifndef OTACLIENT_AA_OTAC_MANAGER_SERVICE_H
#define OTACLIENT_AA_OTAC_MANAGER_SERVICE_H

#include "OtamClientServiceMsg.hpp"
#include "ara/log/logger.h"
#include "dds/dds.hpp"
#include "otaclient/aa/OtaClientConfig.h"
#include "otaclient/aa/OtacStatusStore.h"

#include <atomic>
#include <chrono>
#include <memory>

namespace otaclient
{
namespace aa
{

class OtacManagerService
{
public:
    OtacManagerService(dds::domain::DomainParticipant& participant,
                       ara::log::Logger& logger,
                       const OtaClientConfig& config,
                       std::shared_ptr<OtacStatusStore> statusStore);
    ~OtacManagerService();

    void Initialize();
    void ProcessOnce();
    void Stop();

private:
    void HandleReadySamples();
    void HandleRegistrationResponses();
    void HandleAppUpdateStatus();
    void SendRegistrationIfDue();
    void ResetRegistration(const char* reason);

    dds::domain::DomainParticipant& m_participant;
    ara::log::Logger& m_logger;
    OtaClientConfig m_config;
    std::shared_ptr<OtacStatusStore> m_statusStore;

    dds::pub::Publisher m_publisher;
    dds::sub::Subscriber m_subscriber;

    dds::topic::Topic<otamanager::SetOtaClientRegistration> m_topicRegisterReq;
    dds::topic::Topic<otamanager::SetOtaClientRegistrationResponse> m_topicRegisterResp;
    dds::topic::Topic<otamanager::NotifyOtaManagerReady> m_topicReady;
    dds::topic::Topic<otamanager::NotifyAppUpdateStatus> m_topicStatus;

    dds::pub::DataWriter<otamanager::SetOtaClientRegistration> m_writerRegister;
    dds::sub::DataReader<otamanager::SetOtaClientRegistrationResponse> m_readerRegister;
    dds::sub::DataReader<otamanager::NotifyOtaManagerReady> m_readerReady;
    dds::sub::DataReader<otamanager::NotifyAppUpdateStatus> m_readerStatus;

    otamanager::SetOtaClientRegistration m_regMsg;
    std::chrono::steady_clock::time_point m_nextRegistrationAttempt;
    std::chrono::milliseconds m_retryDelay;
    std::chrono::milliseconds m_maxRetryDelay;
    bool m_registrationAttempted{false};
    std::atomic<bool> m_running{false};
};

} // namespace aa
} // namespace otaclient

#endif
