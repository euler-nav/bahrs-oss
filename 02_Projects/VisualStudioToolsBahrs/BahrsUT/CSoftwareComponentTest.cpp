/// @file CSoftwareComponentTest.cpp
/// @brief Implementation of unit tests for software component port template.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "gtest/gtest.h"
#include "CSoftwareComponentPort.h"

namespace
{
  uint64_t guTestTimestampUs{ 0U };
} // namespace

extern "C" uint64_t GetMicroseconds()
{
  return guTestTimestampUs;
}


/// ## Test design
/// 
/// ### Test case 1
/// #### Test inputs and preconditions
/// The template is instantiated with one port type. At least 3 different port types shall be tested.
///
/// #### Expected results
/// The static member bValue_ of the instantiated type TArePortIdsAscending is true.
/// 
/// ### Test case 2
/// #### Test inputs and preconditions
/// The template is instantiated with multiple port types with ascending port IDs. At least 3 different combinations shall be tested.
///
/// #### Expected results
/// The static member bValue_ of the instantiated type TArePortIdsAscending is true.
///
/// ### Test case 3
/// #### Test inputs and preconditions
/// The template is instantiated with multiple port types with non-ascending port IDs. At least 3 different combinations shall be tested.
///
/// #### Expected results
/// The static member bValue_ of the instantiated type TArePortIdsAscending is false.
TEST(TArePortIdsAscending, common)
{
  struct SDummyType1 { int a; };
  struct SDummyType2 { double b; };
  struct SDummyType3 { char c; };

  // Declare all port type aliases once to avoid duplication.
  using CPort0    = CSoftwareComponentPort<int, 0>;
  using CPort1    = CSoftwareComponentPort<SDummyType1, 1>;
  using CPort2    = CSoftwareComponentPort<int, 2>;
  using CPort7    = CSoftwareComponentPort<SDummyType1, 7>;
  using CPort10   = CSoftwareComponentPort<long, 10>;
  using CPort21   = CSoftwareComponentPort<uint8_t, 21>;
  using CPort29   = CSoftwareComponentPort<uint16_t, 29>;
  using CPort34   = CSoftwareComponentPort<uint32_t, 34>;
  using CPort200  = CSoftwareComponentPort<short, 200>;
  using CPort250  = CSoftwareComponentPort<SDummyType3, 250>;
  using CPort255  = CSoftwareComponentPort<SDummyType2, 255>;
  using CPort5_1  = CSoftwareComponentPort<int, 5>;
  using CPort5_2  = CSoftwareComponentPort<short, 5>;
  using CPort17   = CSoftwareComponentPort<SDummyType1, 17>;
  using CPort55   = CSoftwareComponentPort<SDummyType1, 55>;
  using CPort72   = CSoftwareComponentPort<SDummyType2, 72>;
  using CPort100  = CSoftwareComponentPort<long, 100>;
  using CPort131  = CSoftwareComponentPort<SDummyType3, 131>;

  //
  // Test case 1
  //
  {
    EXPECT_TRUE((TArePortIdsAscending<CPort0>::bValue_));
    EXPECT_TRUE((TArePortIdsAscending<CPort7>::bValue_));
    EXPECT_TRUE((TArePortIdsAscending<CPort255>::bValue_));
  }

  //
  // Test case 2
  //
  {
    EXPECT_TRUE((TArePortIdsAscending<CPort0, CPort1>::bValue_));
    EXPECT_TRUE((TArePortIdsAscending<CPort1, CPort2>::bValue_));
    EXPECT_TRUE((TArePortIdsAscending<CPort200, CPort250>::bValue_));
    EXPECT_TRUE((TArePortIdsAscending<CPort0, CPort1, CPort2>::bValue_));
    EXPECT_TRUE((TArePortIdsAscending<CPort10, CPort200, CPort250>::bValue_));
    EXPECT_TRUE((TArePortIdsAscending<CPort21, CPort29, CPort34, CPort250>::bValue_));
    EXPECT_TRUE((TArePortIdsAscending<CPort1, CPort2, CPort10, CPort21, CPort29, CPort34, CPort250>::bValue_));
  }

  //
  // Test case 3
  //
  {
    EXPECT_FALSE((TArePortIdsAscending<CPort2, CPort1>::bValue_));
    EXPECT_FALSE((TArePortIdsAscending<CPort5_1, CPort5_2>::bValue_));
    EXPECT_FALSE((TArePortIdsAscending<CPort17, CPort1, CPort55>::bValue_));
    EXPECT_FALSE((TArePortIdsAscending<CPort1, CPort2, CPort55, CPort17, CPort131, CPort100>::bValue_));
    EXPECT_FALSE((TArePortIdsAscending<CPort55, CPort72, CPort55, CPort72, CPort100>::bValue_));
    EXPECT_FALSE((TArePortIdsAscending<CPort5_1, CPort2, CPort1, CPort55, CPort72, CPort131>::bValue_));
  }
}

struct SPortInvalidationTestData
{
  uint32_t uValue_{ 0U };
};

TEST(CSoftwareComponentPortAutoInvalidation, KeepsFreshDataBeforeTimeout)
{
  CSoftwareComponentPort<SPortInvalidationTestData, 10U, 10U> oPort;
  SPortInvalidationTestData oWrittenData{};
  SPortInvalidationTestData oReadData{};

  oWrittenData.uValue_ = 42U;
  guTestTimestampUs = 1'000'000U;
  EXPECT_TRUE(oPort.Write(oWrittenData));

  guTestTimestampUs = 1'009'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, oWrittenData.uValue_);
}

TEST(CSoftwareComponentPortAutoInvalidation, ResetsStaleDataToDefault)
{
  CSoftwareComponentPort<SPortInvalidationTestData, 11U, 10U> oPort;
  SPortInvalidationTestData oWrittenData{};
  SPortInvalidationTestData oReadData{};

  oWrittenData.uValue_ = 84U;
  guTestTimestampUs = 2'000'000U;
  EXPECT_TRUE(oPort.Write(oWrittenData));

  guTestTimestampUs = 2'011'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, 0U);

  guTestTimestampUs = 2'012'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, 0U);
}

TEST(CSoftwareComponentPortAutoInvalidation, KeepsDataWhenTimeoutDisabled)
{
  CSoftwareComponentPort<SPortInvalidationTestData, 12U, 0U> oPort;
  SPortInvalidationTestData oWrittenData{};
  SPortInvalidationTestData oReadData{};

  oWrittenData.uValue_ = 21U;
  guTestTimestampUs = 3'000'000U;
  EXPECT_TRUE(oPort.Write(oWrittenData));

  guTestTimestampUs = 9'000'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, oWrittenData.uValue_);
}

TEST(CSoftwareComponentPortAutoInvalidation, AppliesToCPortReaderReadPorts)
{
  CSoftwareComponentPort<SPortInvalidationTestData, 13U, 10U> oPort;
  SPortInvalidationTestData oWrittenData{};

  oWrittenData.uValue_ = 11U;
  guTestTimestampUs = 4'000'000U;
  EXPECT_TRUE(oPort.Write(oWrittenData));

  guTestTimestampUs = 4'011'000U;
  const auto koReadResult = CPortReader::ReadPorts(0U, oPort);

  ASSERT_TRUE(koReadResult.has_value());
  EXPECT_EQ(std::get<0>(*koReadResult).uValue_, 0U);
}

TEST(CSoftwareComponentPortAutoInvalidation, KeepsDataAtExactTimeoutBoundary)
{
  CSoftwareComponentPort<SPortInvalidationTestData, 14U, 10U> oPort;
  SPortInvalidationTestData oWrittenData{};
  SPortInvalidationTestData oReadData{};

  oWrittenData.uValue_ = 33U;
  guTestTimestampUs = 5'000'000U;
  EXPECT_TRUE(oPort.Write(oWrittenData));

  guTestTimestampUs = 5'010'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, oWrittenData.uValue_);
}

TEST(CSoftwareComponentPortAutoInvalidation, RestoresDataAfterWriteFollowingInvalidation)
{
  CSoftwareComponentPort<SPortInvalidationTestData, 15U, 10U> oPort;
  SPortInvalidationTestData oFirstWrittenData{};
  SPortInvalidationTestData oSecondWrittenData{};
  SPortInvalidationTestData oReadData{};

  oFirstWrittenData.uValue_ = 19U;
  guTestTimestampUs = 6'000'000U;
  EXPECT_TRUE(oPort.Write(oFirstWrittenData));

  guTestTimestampUs = 6'011'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, 0U);

  oSecondWrittenData.uValue_ = 77U;
  guTestTimestampUs = 6'012'000U;
  EXPECT_TRUE(oPort.Write(oSecondWrittenData));

  guTestTimestampUs = 6'013'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, oSecondWrittenData.uValue_);
}

TEST(CSoftwareComponentPortAutoInvalidation, KeepsDefaultBeforeFirstWriteAfterTimeout)
{
  CSoftwareComponentPort<SPortInvalidationTestData, 16U, 10U> oPort;
  SPortInvalidationTestData oReadData{};

  guTestTimestampUs = 7'020'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, 0U);
}

TEST(CSoftwareComponentPortAutoInvalidation, KeepsDataWhenReadTimeIsEarlierThanLastWrite)
{
  CSoftwareComponentPort<SPortInvalidationTestData, 17U, 10U> oPort;
  SPortInvalidationTestData oWrittenData{};
  SPortInvalidationTestData oReadData{};

  oWrittenData.uValue_ = 55U;
  guTestTimestampUs = 8'000'000U;
  EXPECT_TRUE(oPort.Write(oWrittenData));

  guTestTimestampUs = 7'999'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, oWrittenData.uValue_);
}

struct SPortInvalidationCountingData
{
  SPortInvalidationCountingData()
  {
    ++suDefaultConstructionCount_;
  }

  static inline uint32_t suDefaultConstructionCount_{ 0U };
  uint32_t uValue_{ 0U };
};

TEST(CSoftwareComponentPortAutoInvalidation, InvalidatesOncePerTimeoutWhileStale)
{
  CSoftwareComponentPort<SPortInvalidationCountingData, 18U, 10U> oPort;
  SPortInvalidationCountingData oWrittenData{};
  SPortInvalidationCountingData oReadData{};

  oWrittenData.uValue_ = 66U;
  guTestTimestampUs = 9'000'000U;
  EXPECT_TRUE(oPort.Write(oWrittenData));

  // The port data is reset by assigning a default-constructed object, so each invalidation
  // increments the default construction counter exactly once.
  uint32_t uCountBefore = SPortInvalidationCountingData::suDefaultConstructionCount_;
  guTestTimestampUs = 9'011'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, 0U);
  EXPECT_EQ(SPortInvalidationCountingData::suDefaultConstructionCount_ - uCountBefore, 1U);

  // Invalidation restarts the timeout: no new invalidation within the next timeout period.
  uCountBefore = SPortInvalidationCountingData::suDefaultConstructionCount_;
  guTestTimestampUs = 9'015'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  guTestTimestampUs = 9'021'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(SPortInvalidationCountingData::suDefaultConstructionCount_ - uCountBefore, 0U);

  // The next invalidation occurs once the timeout has elapsed again.
  uCountBefore = SPortInvalidationCountingData::suDefaultConstructionCount_;
  guTestTimestampUs = 9'022'000U;
  EXPECT_TRUE(oPort.Read(oReadData));
  EXPECT_EQ(oReadData.uValue_, 0U);
  EXPECT_EQ(SPortInvalidationCountingData::suDefaultConstructionCount_ - uCountBefore, 1U);
}


namespace
{
  template<typename TData, uint8_t uPortId>
  class CTestPort
  {
  public:
    using DataType = TData;
    static constexpr uint8_t skuPortId_{ uPortId };

    explicit CTestPort(const TData& rData, bool bLockResult = true)
      : oData_{ rData }
      , bLockResult_{ bLockResult }
    {
    }

    bool Lock()
    {
      ++uLockCount_;
      return bLockResult_;
    }

    void Unlock()
    {
      ++uUnlockCount_;
    }

    TData readUnlocked()
    {
      ++uReadCount_;
      return oData_;
    }

    uint32_t GetLockCount() const
    {
      return uLockCount_;
    }

    uint32_t GetUnlockCount() const
    {
      return uUnlockCount_;
    }

    uint32_t GetReadCount() const
    {
      return uReadCount_;
    }

  private:
    TData oData_{};
    bool bLockResult_{ true };
    uint32_t uLockCount_{ 0U };
    uint32_t uUnlockCount_{ 0U };
    uint32_t uReadCount_{ 0U };
  };
} // namespace

class CPortReaderReadPortsTest : public ::testing::Test
{
protected:
  using CPort0 = CTestPort<int32_t, 0U>;
  using CPort1 = CTestPort<int32_t, 1U>;
  using CPort2 = CTestPort<int32_t, 2U>;
  using CPort3 = CTestPort<int32_t, 3U>;
  using CPort4 = CTestPort<int32_t, 4U>;
  using CPort5Float = CTestPort<float, 5U>;
  using CPort6Char = CTestPort<char, 6U>;
};

/// ## Test design
///
/// ### Test inputs and preconditions
/// Single port with a valid lock and initialized data.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns a valid optional containing the read data tuple.
/// The port is locked once, read once, and unlocked once.
TEST_F(CPortReaderReadPortsTest, ReadPortsSinglePort)
{
  CPort0 oPort0{ 7 };

  auto oResult = CPortReader::ReadPorts(0, oPort0);

  ASSERT_TRUE(oResult.has_value());
  EXPECT_EQ(std::get<0>(*oResult), 7);
  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 1U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 1U);
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// Two ports with ascending IDs are instantiated. Each port reports a successful lock.
/// The port data is initialized to distinct values.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns a valid optional containing the read data tuple.
/// Each port is locked once, read once, and unlocked once.
TEST_F(CPortReaderReadPortsTest, ReadPortsTwoPorts)
{
  CPort0 oPort0{ 12 };
  CPort1 oPort1{ 24 };

  auto oResult = CPortReader::ReadPorts(0, oPort0, oPort1);

  ASSERT_TRUE(oResult.has_value());
  EXPECT_EQ(std::get<0>(*oResult), 12);
  EXPECT_EQ(std::get<1>(*oResult), 24);

  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 1U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort1.GetLockCount(), 1U);
  EXPECT_EQ(oPort1.GetReadCount(), 1U);
  EXPECT_EQ(oPort1.GetUnlockCount(), 1U);
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// Single port reports a failed lock.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns an empty optional.
/// The port lock is attempted once, with no reads or unlocks.
TEST_F(CPortReaderReadPortsTest, ReadPortsSinglePortLockFails)
{
  CPort0 oPort0{ 7, false };

  auto oResult = CPortReader::ReadPorts(0, oPort0);

  EXPECT_FALSE(oResult.has_value());
  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 0U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 0U);
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// Two ports with ascending IDs are instantiated. The first port reports a failed lock.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns an empty optional.
/// The first port lock is attempted once, the second is not locked, no unlocks and no reads occur for any port.
TEST_F(CPortReaderReadPortsTest, ReadPortsTwoPortsLockFails)
{
  CPort0 oPort0{ 12, false };
  CPort1 oPort1{ 24 };

  auto oResult = CPortReader::ReadPorts(0, oPort0, oPort1);

  EXPECT_FALSE(oResult.has_value());
  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 0U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 0U);

  EXPECT_EQ(oPort1.GetLockCount(), 0U);
  EXPECT_EQ(oPort1.GetReadCount(), 0U);
  EXPECT_EQ(oPort1.GetUnlockCount(), 0U);
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// Three ports with ascending IDs are instantiated. Each port reports a successful lock.
/// The port data is initialized to distinct values.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns a valid optional containing the read data tuple.
/// Each port is locked once, read once, and unlocked once.
TEST_F(CPortReaderReadPortsTest, ReadPortsAllLocked)
{
  CPort0 oPort0{ 10 };
  CPort1 oPort1{ 20 };
  CPort2 oPort2{ 30 };

  auto oResult = CPortReader::ReadPorts(0, oPort0, oPort1, oPort2);

  ASSERT_TRUE(oResult.has_value());
  EXPECT_EQ(std::get<0>(*oResult), 10);
  EXPECT_EQ(std::get<1>(*oResult), 20);
  EXPECT_EQ(std::get<2>(*oResult), 30);

  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 1U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort1.GetLockCount(), 1U);
  EXPECT_EQ(oPort1.GetReadCount(), 1U);
  EXPECT_EQ(oPort1.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort2.GetLockCount(), 1U);
  EXPECT_EQ(oPort2.GetReadCount(), 1U);
  EXPECT_EQ(oPort2.GetUnlockCount(), 1U);
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// The first port reports a failed lock. Remaining ports are configured to lock successfully.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns an empty optional.
/// The first port lock is attempted once, other ports are not locked, no reads occur, and no unlocks are issued.
TEST_F(CPortReaderReadPortsTest, ReadPortsFirstLockFails)
{
  CPort0 oPort0{ 10, false };
  CPort1 oPort1{ 20 };
  CPort2 oPort2{ 30 };

  auto oResult = CPortReader::ReadPorts(0, oPort0, oPort1, oPort2);

  EXPECT_FALSE(oResult.has_value());
  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 0U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 0U);

  EXPECT_EQ(oPort1.GetLockCount(), 0U);
  EXPECT_EQ(oPort1.GetReadCount(), 0U);
  EXPECT_EQ(oPort1.GetUnlockCount(), 0U);

  EXPECT_EQ(oPort2.GetLockCount(), 0U);
  EXPECT_EQ(oPort2.GetReadCount(), 0U);
  EXPECT_EQ(oPort2.GetUnlockCount(), 0U);
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// The first two ports lock successfully and the last port reports a failed lock.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns an empty optional.
/// The successfully locked ports are unlocked, while no reads occur for any port.
TEST_F(CPortReaderReadPortsTest, ReadPortsLastLockFails)
{
  CPort0 oPort0{ 10 };
  CPort1 oPort1{ 20 };
  CPort2 oPort2{ 30, false };

  auto oResult = CPortReader::ReadPorts(0, oPort0, oPort1, oPort2);

  EXPECT_FALSE(oResult.has_value());
  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 0U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort1.GetLockCount(), 1U);
  EXPECT_EQ(oPort1.GetReadCount(), 0U);
  EXPECT_EQ(oPort1.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort2.GetLockCount(), 1U);
  EXPECT_EQ(oPort2.GetReadCount(), 0U);
  EXPECT_EQ(oPort2.GetUnlockCount(), 0U);
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// Four ports with ascending IDs are instantiated. Each port reports a successful lock.
/// The port data is initialized to distinct values.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns a valid optional containing the read data tuple.
/// Each tuple element matches the corresponding port data.
TEST_F(CPortReaderReadPortsTest, ReadPortsFourPorts)
{
  CPort0 oPort0{ 11 };
  CPort1 oPort1{ 22 };
  CPort2 oPort2{ 33 };
  CPort3 oPort3{ 44 };

  auto oResult = CPortReader::ReadPorts(0, oPort0, oPort1, oPort2, oPort3);

  ASSERT_TRUE(oResult.has_value());
  EXPECT_EQ(std::get<0>(*oResult), 11);
  EXPECT_EQ(std::get<1>(*oResult), 22);
  EXPECT_EQ(std::get<2>(*oResult), 33);
  EXPECT_EQ(std::get<3>(*oResult), 44);

  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 1U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort1.GetLockCount(), 1U);
  EXPECT_EQ(oPort1.GetReadCount(), 1U);
  EXPECT_EQ(oPort1.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort2.GetLockCount(), 1U);
  EXPECT_EQ(oPort2.GetReadCount(), 1U);
  EXPECT_EQ(oPort2.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort3.GetLockCount(), 1U);
  EXPECT_EQ(oPort3.GetReadCount(), 1U);
  EXPECT_EQ(oPort3.GetUnlockCount(), 1U);
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// Five ports with ascending IDs are instantiated. Each port reports a successful lock.
/// The port data is initialized to distinct values.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns a valid optional containing the read data tuple.
/// Each tuple element matches the corresponding port data.
TEST_F(CPortReaderReadPortsTest, ReadPortsFivePorts)
{
  CPort0 oPort0{ 101 };
  CPort1 oPort1{ 202 };
  CPort2 oPort2{ 303 };
  CPort3 oPort3{ 404 };
  CPort4 oPort4{ 505 };

  auto oResult = CPortReader::ReadPorts(0, oPort0, oPort1, oPort2, oPort3, oPort4);

  ASSERT_TRUE(oResult.has_value());
  EXPECT_EQ(std::get<0>(*oResult), 101);
  EXPECT_EQ(std::get<1>(*oResult), 202);
  EXPECT_EQ(std::get<2>(*oResult), 303);
  EXPECT_EQ(std::get<3>(*oResult), 404);
  EXPECT_EQ(std::get<4>(*oResult), 505);

  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 1U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort1.GetLockCount(), 1U);
  EXPECT_EQ(oPort1.GetReadCount(), 1U);
  EXPECT_EQ(oPort1.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort2.GetLockCount(), 1U);
  EXPECT_EQ(oPort2.GetReadCount(), 1U);
  EXPECT_EQ(oPort2.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort3.GetLockCount(), 1U);
  EXPECT_EQ(oPort3.GetReadCount(), 1U);
  EXPECT_EQ(oPort3.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort4.GetLockCount(), 1U);
  EXPECT_EQ(oPort4.GetReadCount(), 1U);
  EXPECT_EQ(oPort4.GetUnlockCount(), 1U);
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// Three ports with ascending IDs are instantiated using mixed data types.
/// Each port reports a successful lock and the port data is initialized to distinct values.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns a valid optional containing the read data tuple.
/// Each tuple element matches the corresponding port data.
TEST_F(CPortReaderReadPortsTest, ReadPortsMixedTypes)
{
  CPort0 oPort0{ 17 };
  CPort5Float oPort5{ 12.5F };
  CPort6Char oPort6{ 'R' };

  auto oResult = CPortReader::ReadPorts(0, oPort0, oPort5, oPort6);

  ASSERT_TRUE(oResult.has_value());
  EXPECT_EQ(std::get<0>(*oResult), 17);
  EXPECT_EQ(std::get<1>(*oResult), 12.5F);
  EXPECT_EQ(std::get<2>(*oResult), 'R');
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// Four ports with ascending IDs are instantiated. The third port reports a failed lock.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns an empty optional.
/// The successfully locked ports are unlocked, and no reads occur for any port.
TEST_F(CPortReaderReadPortsTest, ReadPortsFourPortsLockFails)
{
  CPort0 oPort0{ 11 };
  CPort1 oPort1{ 22 };
  CPort2 oPort2{ 33, false };
  CPort3 oPort3{ 44 };

  auto oResult = CPortReader::ReadPorts(0, oPort0, oPort1, oPort2, oPort3);

  EXPECT_FALSE(oResult.has_value());
  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 0U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort1.GetLockCount(), 1U);
  EXPECT_EQ(oPort1.GetReadCount(), 0U);
  EXPECT_EQ(oPort1.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort2.GetLockCount(), 1U);
  EXPECT_EQ(oPort2.GetReadCount(), 0U);
  EXPECT_EQ(oPort2.GetUnlockCount(), 0U);

  EXPECT_EQ(oPort3.GetLockCount(), 0U);
  EXPECT_EQ(oPort3.GetReadCount(), 0U);
  EXPECT_EQ(oPort3.GetUnlockCount(), 0U);
}

/// ## Test design
///
/// ### Test inputs and preconditions
/// Five ports with ascending IDs are instantiated. The last port reports a failed lock.
///
/// ### Expected results
/// `CPortReader::ReadPorts()` returns an empty optional.
/// The successfully locked ports are unlocked, and no reads occur for any port.
TEST_F(CPortReaderReadPortsTest, ReadPortsFivePortsLockFails)
{
  CPort0 oPort0{ 101 };
  CPort1 oPort1{ 202 };
  CPort2 oPort2{ 303 };
  CPort3 oPort3{ 404 };
  CPort4 oPort4{ 505, false };

  auto oResult = CPortReader::ReadPorts(0, oPort0, oPort1, oPort2, oPort3, oPort4);

  EXPECT_FALSE(oResult.has_value());
  EXPECT_EQ(oPort0.GetLockCount(), 1U);
  EXPECT_EQ(oPort0.GetReadCount(), 0U);
  EXPECT_EQ(oPort0.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort1.GetLockCount(), 1U);
  EXPECT_EQ(oPort1.GetReadCount(), 0U);
  EXPECT_EQ(oPort1.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort2.GetLockCount(), 1U);
  EXPECT_EQ(oPort2.GetReadCount(), 0U);
  EXPECT_EQ(oPort2.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort3.GetLockCount(), 1U);
  EXPECT_EQ(oPort3.GetReadCount(), 0U);
  EXPECT_EQ(oPort3.GetUnlockCount(), 1U);

  EXPECT_EQ(oPort4.GetLockCount(), 1U);
  EXPECT_EQ(oPort4.GetReadCount(), 0U);
  EXPECT_EQ(oPort4.GetUnlockCount(), 0U);
}
