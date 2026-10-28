#include "otaclient/aa/OtacManagerService.h"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <utility>

namespace otaclient
{
namespace aa
{
namespace
{

constexpr const char* kTopicPrefix =
    "/mmf/otamanager/otaclient_service/";

dds::pub::qos::DataWriterQos RegistrationWriterQos(
    const dds::pub::Publisher& publisher)
{
    auto qos = publisher.default_datawriter_qos();
    qos << dds::core::policy::Reliability::Reliable()
        << dds::core::policy::Durability::TransientLocal()
        << dds::core::policy::History::KeepLast(1U);
    return qos;
}

dds::sub::qos::DataReaderQos TransientReaderQos(
    const dds::sub::Subscriber& subscriber, std::uint32_t depth)
{
    auto qos = subscriber.default_datareader_qos();
    qos << dds::core::policy::Reliability::Reliable()
        << dds::core::policy::Durability::TransientLocal()
        << dds::core::policy::History::KeepLast(depth);
    return qos;
}

dds::sub::qos::DataReaderQos StatusReaderQos(
    const dds::sub::Subscriber& subscriber)
{
    auto qos = subscriber.default_datareader_qos();
    qos << dds::core::policy::Reliability::Reliable()
        << dds::core::policy::Durability::Volatile()
        << dds::core::policy::History::KeepLast(32U);
    return qos;
}

} // namespace

OtacManagerService::OtacManagerService(
    dds::domain::DomainParticipant& participant, ara::log::Logger& logger,
    const OtaClientConfig& config,
    std::shared_ptr<OtacStatusStore> statusStore)
    : m_participant(participant)
    , m_logger(logger)
    , m_config(config)
    , m_statusStore(std::move(statusStore))
    , m_publisher(participant)
    , m_subscriber(participant)
    , m_topicRegisterReq(
          m_participant, std::string(kTopicPrefix) + "SetOtaClientRegistration")
    , m_topicRegisterResp(
          m_participant,
          std::string(kTopicPrefix) + "SetOtaClientRegistrationResponse")
    , m_topicReady(
          m_participant, std::string(kTopicPrefix) + "NotifyOtaManagerReady")
    , m_topicStatus(
          m_participant, std::string(kTopicPrefix) + "NotifyAppUpdateStatus")
    , m_writerRegister(m_publisher, m_topicRegisterReq,
                       RegistrationWriterQos(m_publisher))
    , m_readerRegister(m_subscriber, m_topicRegisterResp,
                       TransientReaderQos(m_subscriber, 16U))
    , m_readerReady(m_subscriber, m_topicReady,
                    TransientReaderQos(m_subscriber, 1U))
    , m_readerStatus(m_subscriber, m_topicStatus,
                     StatusReaderQos(m_subscriber))
    , m_retryDelay(config.registrationRetryMs)
    , m_maxRetryDelay(std::max<std::uint32_t>(
          config.registrationRetryMs, 30000U))
{
    m_regMsg.client_name(m_config.clientName);
    m_regMsg.app_list(m_config.AppNames());
}

OtacManagerService::~OtacManagerService()
{
    Stop();
}

void OtacManagerService::Initialize()
{
    m_statusStore->InvalidateRegistration();
    m_registrationAttempted = false;
    m_retryDelay = std::chrono::milliseconds(m_config.registrationRetryMs);
    m_nextRegistrationAttempt = std::chrono::steady_clock::now();
    m_running.store(true);

    m_logger.LogInfo() << "[OTAClient] DDS service initialized for client="
                       << m_config.clientName.c_str();
}

void OtacManagerService::ProcessOnce()
{
    if (!m_running.load())
    {
        return;
    }

    HandleReadySamples();
    HandleRegistrationResponses();
    HandleAppUpdateStatus();
    SendRegistrationIfDue();
}

void OtacManagerService::HandleReadySamples()
{
    auto samples = m_readerReady.take();
    for (auto& sample : samples)
    {
        if (sample.info().valid())
        {
            ResetRegistration("OTA Manager ready signal");
        }
    }
}

void OtacManagerService::HandleRegistrationResponses()
{
    auto samples = m_readerRegister.take();
    for (auto& sample : samples)
    {
        if (!sample.info().valid())
        {
            continue;
        }

        const auto& message = sample.data();
        if (message.client_name() != m_config.clientName)
        {
            m_logger.LogVerbose()
                << "[OTAClient] Ignored registration response for client="
                << message.client_name().c_str();
            continue;
        }
        if (!m_registrationAttempted)
        {
            m_logger.LogVerbose()
                << "[OTAClient] Ignored stale registration response.";
            continue;
        }

        const bool succeeded = message.ret_code() == ReturnValue::SUCCESS;
        if (m_statusStore->AcceptRegistration(
                message.client_name(), succeeded,
                static_cast<std::int32_t>(message.client_id())))
        {
            m_retryDelay =
                std::chrono::milliseconds(m_config.registrationRetryMs);
            std::cout << "[OTAClient] Registration succeeded: client_id="
                      << static_cast<std::int32_t>(message.client_id())
                      << std::endl;
            m_logger.LogInfo()
                << "[OTAClient] Registration succeeded: client_id="
                << static_cast<std::int32_t>(message.client_id());
        }
        else
        {
            m_logger.LogWarn()
                << "[OTAClient] Registration rejected: ret_code="
                << static_cast<std::int32_t>(message.ret_code());
        }
    }
}

void OtacManagerService::HandleAppUpdateStatus()
{
    auto samples = m_readerStatus.take();
    for (auto& sample : samples)
    {
        if (!sample.info().valid())
        {
            continue;
        }

        const auto& message = sample.data();
        const auto result = m_statusStore->UpdateStatus(
            static_cast<std::int32_t>(message.client_id()),
            message.app_name(), message.status());

        if (result == StatusUpdateResult::kUpdated)
        {
            std::cout << "[OTAClient] App update status stored: app="
                      << message.app_name()
                      << ", status=" << message.status()
                      << ", client_id=" << message.client_id() << std::endl;
            m_logger.LogInfo()
                << "[OTAClient] App update status stored: app="
                << message.app_name().c_str()
                << ", status=" << message.status().c_str()
                << ", client_id="
                << static_cast<std::int32_t>(message.client_id());
        }
        else if (result == StatusUpdateResult::kClientIdMismatch)
        {
            m_logger.LogVerbose()
                << "[OTAClient] Ignored status for another client_id="
                << static_cast<std::int32_t>(message.client_id());
        }
        else if (result == StatusUpdateResult::kUnknownApp)
        {
            m_logger.LogWarn() << "[OTAClient] Ignored status for unknown app="
                               << message.app_name().c_str();
        }
        else
        {
            m_logger.LogVerbose()
                << "[OTAClient] Ignored status before registration.";
        }
    }
}

void OtacManagerService::SendRegistrationIfDue()
{
    if (m_statusStore->IsRegistered() ||
        std::chrono::steady_clock::now() < m_nextRegistrationAttempt)
    {
        return;
    }

    m_writerRegister.write(m_regMsg);
    m_registrationAttempted = true;
    std::cout << "[OTAClient] Registration request published: client="
              << m_config.clientName
              << ", retry_ms=" << m_retryDelay.count() << std::endl;
    m_logger.LogInfo()
        << "[OTAClient] Registration request published: client="
        << m_config.clientName.c_str()
        << ", retry_ms=" << static_cast<std::uint32_t>(m_retryDelay.count());

    m_nextRegistrationAttempt = std::chrono::steady_clock::now() + m_retryDelay;
    m_retryDelay = std::min(m_retryDelay * 2, m_maxRetryDelay);
}

void OtacManagerService::ResetRegistration(const char* reason)
{
    m_statusStore->InvalidateRegistration();
    m_registrationAttempted = false;
    m_retryDelay = std::chrono::milliseconds(m_config.registrationRetryMs);
    m_nextRegistrationAttempt = std::chrono::steady_clock::now();
    std::cout << "[OTAClient] Registration invalidated: " << reason
              << std::endl;
    m_logger.LogInfo() << "[OTAClient] Registration invalidated: " << reason;
}

void OtacManagerService::Stop()
{
    m_running.store(false);
}

} // namespace aa
} // namespace otaclient
