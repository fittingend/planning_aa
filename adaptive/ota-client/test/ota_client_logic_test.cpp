#include "otaclient/aa/OtaClientConfig.h"
#include "otaclient/aa/OtacStatusStore.h"

#include <cstdint>
#include <iostream>
#include <string>

namespace
{

int Expect(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        return 1;
    }
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "usage: OTAClientLogicTest CONFIG_PATH\n";
        return 2;
    }

    int failures = 0;
    const auto config =
        otaclient::aa::OtaClientConfig::LoadFromFile(argv[1]);
    failures += Expect(config.clientName == "autosar_ota_client",
                       "client_name was not loaded");
    failures += Expect(config.AppNames().size() == 1U,
                       "application list was not loaded");
    failures += Expect(config.registrationRetryMs == 2000U,
                       "registration retry was not loaded");

    std::uint32_t domainId = 0U;
    std::string error;
    failures += Expect(
        otaclient::aa::OtaClientConfig::ParseDomainId("42", domainId, error) &&
            domainId == 42U,
        "valid DDS domain was rejected");
    failures += Expect(
        !otaclient::aa::OtaClientConfig::ParseDomainId("42x", domainId, error),
        "malformed DDS domain was accepted");
    failures += Expect(
        !otaclient::aa::OtaClientConfig::ParseDomainId("233", domainId, error),
        "out-of-range DDS domain was accepted");

    otaclient::aa::OtacStatusStore store(config);
    failures += Expect(
        !store.AcceptRegistration("another_client", true, 7),
        "foreign registration response was accepted");
    failures += Expect(
        !store.AcceptRegistration(config.clientName, false, 7),
        "failed registration response was accepted");
    failures += Expect(
        store.AcceptRegistration(config.clientName, true, 0),
        "valid registration response was rejected");
    failures += Expect(store.IsRegistered() && store.ClientId() == 0,
                       "successful client_id was not stored");

    failures += Expect(
        store.UpdateStatus(99, "adsw_monitor", "Running") ==
            otaclient::aa::StatusUpdateResult::kClientIdMismatch,
        "foreign client status was accepted");
    failures += Expect(
        store.UpdateStatus(0, "unknown_app", "Running") ==
            otaclient::aa::StatusUpdateResult::kUnknownApp,
        "unknown application status was accepted");
    failures += Expect(
        store.UpdateStatus(0, "adsw_monitor", "Running") ==
            otaclient::aa::StatusUpdateResult::kUpdated,
        "own application status was rejected");

    otaclient::aa::AppStatusRecord status;
    failures += Expect(
        store.GetAppStatus("adsw_monitor", status) &&
            status.status == "Running" &&
            status.updatedAt.time_since_epoch().count() != 0,
        "application status was not persisted");

    store.InvalidateRegistration();
    failures += Expect(
        store.UpdateStatus(0, "adsw_monitor", "Succeeded") ==
            otaclient::aa::StatusUpdateResult::kNotRegistered,
        "status was accepted after manager restart invalidation");

    if (failures == 0)
    {
        std::cout << "OTA client logic tests passed\n";
    }
    return failures == 0 ? 0 : 1;
}
