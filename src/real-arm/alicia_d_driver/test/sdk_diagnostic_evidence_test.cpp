#include <gtest/gtest.h>
#include "alicia_d_driver/sdk_diagnostic_evidence.hpp"
#include <limits>
#include <algorithm>

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(SdkDiagnosticEvidence, VersionQueryIsExactOfficialReadOnlyFrame)
{
    EXPECT_EQ((std::vector<uint8_t>{0xAA, 0x01, 0x00, 0x01, 0xFE, 0x23, 0xFF}),
              sdk_readonly_version_query());
}

TEST(SdkDiagnosticEvidence, MotionQueriesAreExactSdkWhitelist)
{
    EXPECT_EQ((std::vector<uint8_t>{0xAA, 0x06, 0x02, 0x01, 0xFE, 0xF4, 0xFF}),
              sdk_readonly_motion_query(SdkReadonlyMotionQuery::Velocity));
    // Preserve the official self-check legacy wire length, not a guessed CRC.
    EXPECT_EQ((std::vector<uint8_t>{0xAA, 0xFE, 0x00, 0x00, 0xFE, 0x93, 0xFF}),
              sdk_readonly_motion_query(SdkReadonlyMotionQuery::SelfCheck));
    EXPECT_TRUE(sdk_readonly_motion_query(SdkReadonlyMotionQuery::None).empty());
}

TEST(SdkDiagnosticEvidence, QueryBatchNeverStartsAutomaticallyOrRepeats)
{
    SdkReadonlyMotionBatch batch;
    EXPECT_EQ(SdkReadonlyMotionQuery::None, batch.take(10., true));
    ASSERT_TRUE(batch.request(10.));
    EXPECT_FALSE(batch.request(10.1));
    EXPECT_EQ(SdkReadonlyMotionQuery::None, batch.take(10.2, false));
    EXPECT_EQ(SdkReadonlyMotionQuery::Velocity, batch.take(10.3, true));
    EXPECT_EQ(SdkReadonlyMotionQuery::SelfCheck, batch.take(10.4, true));
    EXPECT_EQ(SdkReadonlyMotionQuery::None, batch.take(10.5, true));
    EXPECT_FALSE(batch.request(14.99));
    EXPECT_TRUE(batch.request(15.));
}

TEST(SdkDiagnosticEvidence, QuietSlotDeferralExpiresInsteadOfSendingLater)
{
    SdkReadonlyMotionBatch batch;
    ASSERT_TRUE(batch.request(20.));
    EXPECT_EQ(SdkReadonlyMotionQuery::None, batch.take(24.9, false));
    EXPECT_EQ(SdkReadonlyMotionQuery::None, batch.take(25., true));
    EXPECT_EQ(SdkReadonlyMotionQuery::None, batch.take(30., true));
    ASSERT_TRUE(batch.request(30.));
    EXPECT_EQ(SdkReadonlyMotionQuery::Velocity, batch.take(30.1, true));
    EXPECT_EQ(SdkReadonlyMotionQuery::None, batch.take(35., true));
}

TEST(SdkDiagnosticEvidence, InvalidOrReversedClockNeverSends)
{
    SdkReadonlyMotionBatch batch;
    EXPECT_FALSE(batch.request(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(batch.request(-1.));
    ASSERT_TRUE(batch.request(10.));
    EXPECT_EQ(SdkReadonlyMotionQuery::None, batch.take(9., true));
    EXPECT_EQ(SdkReadonlyMotionQuery::None, batch.take(10.1, true));
    EXPECT_FALSE(batch.request(9.));
}

TEST(SdkDiagnosticEvidence, TransportCancellationDropsRemainderWithoutResettingRateLimit)
{
    SdkReadonlyMotionBatch batch;
    ASSERT_TRUE(batch.request(10.));
    EXPECT_EQ(SdkReadonlyMotionQuery::Velocity, batch.take(10.1, true));
    batch.cancel();
    EXPECT_EQ(SdkReadonlyMotionQuery::None, batch.take(10.2, true));
    EXPECT_FALSE(batch.request(10.3));
    EXPECT_TRUE(batch.request(15.));
    EXPECT_EQ(SdkReadonlyMotionQuery::Velocity, batch.take(15.1, true));
}

TEST(SdkDiagnosticEvidence, VelocityRetainsUnsignedWordsWithoutClampOrJointMapping)
{
    std::vector<uint8_t> payload(20, 0);
    payload[0] = 0x34; payload[1] = 0x12;
    payload[18] = 0xFF; payload[19] = 0xFF;
    std::vector<uint16_t> words;
    ASSERT_TRUE(sdk_decode_follower_velocity_words(payload, words));
    ASSERT_EQ(10u, words.size());
    EXPECT_EQ(0x1234, words[0]);
    EXPECT_EQ(0xFFFF, words[9]);
    for (size_t length : {0u, 12u, 19u, 21u, 22u}) {
        const auto previous = words;
        EXPECT_FALSE(sdk_decode_follower_velocity_words(std::vector<uint8_t>(length), words));
        EXPECT_EQ(previous, words);
    }
}

TEST(SdkDiagnosticEvidence, SelfCheckKeepsReservedBitsAndRejectsWrongLayout)
{
    SdkSelfCheckEvidence evidence;
    ASSERT_TRUE(sdk_decode_follower_self_check({0xFF, 0x03}, evidence));
    EXPECT_EQ(0u, evidence.abnormal_channel_bits);
    EXPECT_EQ(0u, evidence.reserved_bits);
    ASSERT_TRUE(sdk_decode_follower_self_check({0xFE, 0x83}, evidence));
    EXPECT_EQ(0x83FEu, evidence.raw_mask);
    EXPECT_EQ(1u, evidence.abnormal_channel_bits);
    EXPECT_EQ(0x8000u, evidence.reserved_bits);
    for (size_t length : {0u, 1u, 3u, 20u}) {
        EXPECT_FALSE(sdk_decode_follower_self_check(std::vector<uint8_t>(length), evidence));
        EXPECT_EQ(0x83FEu, evidence.raw_mask);
    }
}

static std::vector<uint8_t> version_payload()
{
    std::vector<uint8_t> data(24, 0);
    const std::string serial = "TEST000000000001";
    std::copy(serial.begin(), serial.end(), data.begin());
    data[16] = 0x30; data[17] = 0x02; // hardware 560
    data[20] = 0x62; data[21] = 0x02; // firmware 610
    return data;
}

TEST(SdkDiagnosticEvidence, VersionDecodesLittleEndianRawWithoutCompatibilityGuess)
{
    SdkDeviceIdentity identity;
    ASSERT_TRUE(sdk_decode_device_identity(version_payload(), identity));
    EXPECT_EQ("TEST000000000001", identity.serial);
    EXPECT_EQ(560u, identity.hardware_raw);
    EXPECT_EQ(610u, identity.firmware_raw);
    auto full = version_payload();
    full[16] = 0x12; full[17] = 0x34; full[18] = 0x56; full[19] = 0x78;
    ASSERT_TRUE(sdk_decode_device_identity(full, identity));
    EXPECT_EQ(0x78563412u, identity.hardware_raw);
}

TEST(SdkDiagnosticEvidence, UnknownLengthAndInvalidSerialCannotProduceIdentity)
{
    for (size_t length : {0u, 16u, 23u, 25u, 64u}) {
        SdkDeviceIdentity identity;
        identity.serial = "unchanged";
        EXPECT_FALSE(sdk_decode_device_identity(std::vector<uint8_t>(length, 'A'), identity));
        EXPECT_EQ("unchanged", identity.serial);
    }
    for (uint8_t byte : {0x01, 0x1B, 0x7F, 0xFF}) {
        auto data = version_payload();
        data[3] = byte;
        SdkDeviceIdentity identity;
        EXPECT_FALSE(sdk_decode_device_identity(data, identity));
    }
    SdkDeviceIdentity identity;
    EXPECT_FALSE(sdk_decode_device_identity(std::vector<uint8_t>(24, 0), identity));
}

TEST(SdkDiagnosticEvidence, NullPaddingIsAllowedButEmbeddedNullIsNot)
{
    auto data = version_payload();
    data[15] = 0;
    SdkDeviceIdentity identity;
    EXPECT_TRUE(sdk_decode_device_identity(data, identity));
    data[2] = 0;
    EXPECT_FALSE(sdk_decode_device_identity(data, identity));
}

TEST(SdkDiagnosticEvidence, ActualRejected66DegreeChannelRemainsUnknownAndHigh)
{
    std::vector<uint8_t> raw(10, 38);
    std::vector<float> filtered(10, 38);
    raw[7] = 0x42;
    filtered[7] = std::numeric_limits<float>::quiet_NaN();
    const auto evidence = sdk_temperature_evidence(raw, filtered, 60);
    EXPECT_EQ(66u, evidence.raw_max_c);
    EXPECT_TRUE(evidence.raw_over_limit);
    EXPECT_FALSE(evidence.all_channels_accepted);
    EXPECT_FALSE(evidence.healthy_sample());
    EXPECT_NE(std::string::npos, evidence.channel_quality.find("unknown_rejected"));
}

TEST(SdkDiagnosticEvidence, RejectedCoolSampleAlsoCannotProveHealthy)
{
    std::vector<uint8_t> raw(10, 38);
    std::vector<float> filtered(10, 38);
    filtered[0] = std::numeric_limits<float>::quiet_NaN();
    const auto evidence = sdk_temperature_evidence(raw, filtered, 60);
    EXPECT_FALSE(evidence.raw_over_limit);
    EXPECT_FALSE(evidence.healthy_sample());
}

TEST(SdkDiagnosticEvidence, MissingChannelOrWrongFollowerLengthIsNotHealthy)
{
    for (size_t length : {0u, 6u, 9u, 11u}) {
        EXPECT_FALSE(sdk_temperature_evidence(std::vector<uint8_t>(length, 38),
                                              std::vector<float>(length, 38), 60).healthy_sample());
    }
    EXPECT_FALSE(sdk_temperature_evidence(std::vector<uint8_t>(10, 38),
                                          std::vector<float>(9, 38), 60).healthy_sample());
}

TEST(SdkDiagnosticEvidence, BoundaryAndInvalidLimitNeverClaimHealthy)
{
    const std::vector<uint8_t> raw(10, 60);
    const std::vector<float> filtered(10, 60);
    EXPECT_FALSE(sdk_temperature_evidence(raw, filtered, 60).healthy_sample());
    EXPECT_FALSE(sdk_temperature_evidence(raw, filtered, 0).healthy_sample());
    EXPECT_FALSE(sdk_temperature_evidence(raw, filtered,
        std::numeric_limits<double>::quiet_NaN()).healthy_sample());
    EXPECT_TRUE(sdk_temperature_evidence(std::vector<uint8_t>(10, 38),
                                         std::vector<float>(10, 38), 60).healthy_sample());
}
