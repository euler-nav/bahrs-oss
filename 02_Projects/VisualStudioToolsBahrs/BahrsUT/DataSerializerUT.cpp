/// @file DataSerializerUT.cpp
/// @brief Implementation of unit tests for the DataSerializer class.
/// @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "gtest/gtest.h"
#include "DataSerializer.h"

class DataSerializerTest : public ::testing::Test
{
protected:
  enum class EEnum1 : uint8_t
  {
    eLabel
  };

  enum class EEnum2 : int32_t
  {
    eLabel1,
    eLabel2
  };

  uint8_t uVar1_{ 0 };
  uint16_t uVar2_{ 0 };
  uint32_t uVar3_{ 0 };
  uint64_t uVar4_{ 0 };
  int8_t iVar1_{ 0 };
  int16_t iVar2_{ 0 };
  int32_t iVar3_{ 0 };
  int64_t iVar4_{ 0 };
  float fVar1_{ 0 };
  double dVar1_{ 0 };
  char cVar1_{ 'A' };
  EEnum1 eVar1_{ EEnum1::eLabel };
  EEnum2 eVar2_{ EEnum2::eLabel2 };
};

TEST_F(DataSerializerTest, SizeOfAllVariables1)
{
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(uVar1_), uint32_t(1));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(uVar2_), uint32_t(2));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(uVar3_), uint32_t(4));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(uVar4_), uint32_t(8));

  EXPECT_EQ(NLibCommon::SizeOfAllVariables(iVar1_), uint32_t(1));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(iVar2_), uint32_t(2));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(iVar3_), uint32_t(4));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(iVar4_), uint32_t(8));

  EXPECT_EQ(NLibCommon::SizeOfAllVariables(fVar1_), uint32_t(4));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(dVar1_), uint32_t(8));

  EXPECT_EQ(NLibCommon::SizeOfAllVariables(cVar1_), uint32_t(1));

  EXPECT_EQ(NLibCommon::SizeOfAllVariables(eVar1_), uint32_t(1));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(eVar2_), uint32_t(4));
}

TEST_F(DataSerializerTest, SizeOfAllVariables2)
{
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(uVar1_, uVar2_), uint32_t(3));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(dVar1_, uVar1_, uVar4_, eVar1_, fVar1_), uint32_t(22));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(eVar2_, cVar1_), uint32_t(5));
  EXPECT_EQ(NLibCommon::SizeOfAllVariables(eVar2_, uVar2_, fVar1_, fVar1_, fVar1_, cVar1_, dVar1_), uint32_t(27));
}

TEST_F(DataSerializerTest, Serialize)
{
  struct STmpStruct1
  {
    int16_t iVal_;
    char cVal_;
    uint8_t uVal_;
    float fVal_;
  };

  STmpStruct1 oObj1{ -250, 'k', 128U, 345.67F };

  // Test single variables
  std::vector<uint8_t> oBytes;
  uint8_t* upRefBytes;

  NLibCommon::Serialize(oBytes, oObj1.iVal_);
  upRefBytes = reinterpret_cast<uint8_t*>(&oObj1.iVal_);
  EXPECT_EQ(oBytes.size(), 2);
  EXPECT_EQ(oBytes[0], upRefBytes[0]);
  EXPECT_EQ(oBytes[1], upRefBytes[1]);

  NLibCommon::Serialize(oBytes, oObj1.cVal_);
  EXPECT_EQ(oBytes.size(), 1);
  EXPECT_EQ(oBytes[0], oObj1.cVal_);


  NLibCommon::Serialize(oBytes, oObj1.uVal_);
  EXPECT_EQ(oBytes.size(), 1);
  EXPECT_EQ(oBytes[0], oObj1.uVal_);

  NLibCommon::Serialize(oBytes, oObj1.fVal_);
  upRefBytes = reinterpret_cast<uint8_t*>(&oObj1.fVal_);
  EXPECT_EQ(oBytes.size(), 4);
  EXPECT_EQ(oBytes[0], upRefBytes[0]);
  EXPECT_EQ(oBytes[1], upRefBytes[1]);
  EXPECT_EQ(oBytes[2], upRefBytes[2]);
  EXPECT_EQ(oBytes[3], upRefBytes[3]);

  // Test multiple variables
  NLibCommon::Serialize(oBytes, oObj1.uVal_, oObj1.iVal_, oObj1.fVal_, oObj1.cVal_);
  EXPECT_EQ(oBytes.size(), 8);
  EXPECT_EQ(oBytes[0], oObj1.uVal_);

  upRefBytes = reinterpret_cast<uint8_t*>(&oObj1.iVal_);
  EXPECT_EQ(oBytes[1], upRefBytes[0]);
  EXPECT_EQ(oBytes[2], upRefBytes[1]);

  upRefBytes = reinterpret_cast<uint8_t*>(&oObj1.fVal_);
  EXPECT_EQ(oBytes[3], upRefBytes[0]);
  EXPECT_EQ(oBytes[4], upRefBytes[1]);
  EXPECT_EQ(oBytes[5], upRefBytes[2]);
  EXPECT_EQ(oBytes[6], upRefBytes[3]);

  EXPECT_EQ(oBytes[7], oObj1.cVal_);

  struct STmpStruct2
  {
    uint32_t uVal1_;
    uint32_t uVal2_;
    char cVal_;
    int16_t iVal_;
    float fVal1_;
    double dVal_;
    float fVal2_;
  };

  STmpStruct2 oObj2{ 112237888U, 765485U, 'N', -21001, 10.5F, 553000.333, -0.3F };

  NLibCommon::Serialize(oBytes, oObj2.uVal1_, oObj2.uVal2_, oObj2.cVal_, oObj2.iVal_, oObj2.fVal1_, oObj2.dVal_, oObj2.fVal2_);
  EXPECT_EQ(oBytes.size(), 27);

  upRefBytes = reinterpret_cast<uint8_t*>(&oObj2.uVal1_);
  EXPECT_EQ(oBytes[0], upRefBytes[0]);
  EXPECT_EQ(oBytes[1], upRefBytes[1]);
  EXPECT_EQ(oBytes[2], upRefBytes[2]);
  EXPECT_EQ(oBytes[3], upRefBytes[3]);

  upRefBytes = reinterpret_cast<uint8_t*>(&oObj2.uVal2_);
  EXPECT_EQ(oBytes[4], upRefBytes[0]);
  EXPECT_EQ(oBytes[5], upRefBytes[1]);
  EXPECT_EQ(oBytes[6], upRefBytes[2]);
  EXPECT_EQ(oBytes[7], upRefBytes[3]);

  EXPECT_EQ(oBytes[8], oObj2.cVal_);

  upRefBytes = reinterpret_cast<uint8_t*>(&oObj2.iVal_);
  EXPECT_EQ(oBytes[9], upRefBytes[0]);
  EXPECT_EQ(oBytes[10], upRefBytes[1]);

  upRefBytes = reinterpret_cast<uint8_t*>(&oObj2.fVal1_);
  EXPECT_EQ(oBytes[11], upRefBytes[0]);
  EXPECT_EQ(oBytes[12], upRefBytes[1]);
  EXPECT_EQ(oBytes[13], upRefBytes[2]);
  EXPECT_EQ(oBytes[14], upRefBytes[3]);

  upRefBytes = reinterpret_cast<uint8_t*>(&oObj2.dVal_);
  EXPECT_EQ(oBytes[15], upRefBytes[0]);
  EXPECT_EQ(oBytes[16], upRefBytes[1]);
  EXPECT_EQ(oBytes[17], upRefBytes[2]);
  EXPECT_EQ(oBytes[18], upRefBytes[3]);
  EXPECT_EQ(oBytes[19], upRefBytes[4]);
  EXPECT_EQ(oBytes[20], upRefBytes[5]);
  EXPECT_EQ(oBytes[21], upRefBytes[6]);
  EXPECT_EQ(oBytes[22], upRefBytes[7]);

  upRefBytes = reinterpret_cast<uint8_t*>(&oObj2.fVal2_);
  EXPECT_EQ(oBytes[23], upRefBytes[0]);
  EXPECT_EQ(oBytes[24], upRefBytes[1]);
  EXPECT_EQ(oBytes[25], upRefBytes[2]);
  EXPECT_EQ(oBytes[26], upRefBytes[3]);
}

TEST_F(DataSerializerTest, Deserialize)
{
  uint8_t uVal8;
  int16_t iVal16;
  uint32_t uVal32;
  int64_t iVal64;
  float fVal;
  double dVal;
  char cVal;
  EEnum2 eLabel;

  // Test single variables

  std::vector<uint8_t> oBytes;  

  {
    uint8_t uRefVal = 233U;
    oBytes.push_back(uRefVal);

    EXPECT_TRUE(NLibCommon::Deserialize(oBytes, uVal8));
    EXPECT_EQ(uVal8, uRefVal);
    oBytes.clear();
  }

  {
    int16_t iRefVal = -12233;
    uint8_t* upData = reinterpret_cast<uint8_t*>(&iRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(iRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    EXPECT_TRUE(NLibCommon::Deserialize(oBytes, iVal16));
    EXPECT_EQ(iVal16, iRefVal);
    oBytes.clear();
  }

  {
    uint32_t uRefVal = 3756456123U;
    uint8_t* upData = reinterpret_cast<uint8_t*>(&uRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(uRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    EXPECT_TRUE(NLibCommon::Deserialize(oBytes, uVal32));
    EXPECT_EQ(uVal32, uRefVal);
    oBytes.clear();
  }

  {
    int64_t iRefVal = -2987345111367987543;
    uint8_t* upData = reinterpret_cast<uint8_t*>(&iRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(iRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    EXPECT_TRUE(NLibCommon::Deserialize(oBytes, iVal64));
    EXPECT_EQ(iVal64, iRefVal);
    oBytes.clear();
  }

  {
    float fRefVal = -567.4533F;
    uint8_t* upData = reinterpret_cast<uint8_t*>(&fRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(fRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    EXPECT_TRUE(NLibCommon::Deserialize(oBytes, fVal));
    EXPECT_EQ(fVal, fRefVal);
    oBytes.clear();
  }

  {
    double dRefVal = 42134567.4533;
    uint8_t* upData = reinterpret_cast<uint8_t*>(&dRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(dRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    EXPECT_TRUE(NLibCommon::Deserialize(oBytes, dVal));
    EXPECT_EQ(dVal, dRefVal);
    oBytes.clear();
  }

  {
    char cRefVal = 'U';
    oBytes.push_back(cRefVal);

    EXPECT_TRUE(NLibCommon::Deserialize(oBytes, cVal));
    EXPECT_EQ(cVal, cRefVal);
    oBytes.clear();
  }

  {
    EEnum2 eRefVal = EEnum2::eLabel2;
    uint8_t* upData = reinterpret_cast<uint8_t*>(&eRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(eRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    EXPECT_TRUE(NLibCommon::Deserialize(oBytes, eLabel));
    EXPECT_EQ(eLabel, eRefVal);
    oBytes.clear();
  }

  // Test multiple variables

  {
    double dRefVal = -7.4533;
    char cRefVal = '2';
    int16_t iRefVal = 2376;

    uint8_t* upData = reinterpret_cast<uint8_t*>(&dRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(dRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    oBytes.push_back(cRefVal);

    upData = reinterpret_cast<uint8_t*>(&iRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(iRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    EXPECT_TRUE(NLibCommon::Deserialize(oBytes, dVal, cVal, iVal16));
    EXPECT_EQ(dVal, dRefVal);
    EXPECT_EQ(cVal, cRefVal);
    EXPECT_EQ(iVal16, iRefVal);

    EXPECT_FALSE(NLibCommon::Deserialize(oBytes, cVal, iVal16));
    EXPECT_FALSE(NLibCommon::Deserialize(oBytes, dVal, iVal16));
    EXPECT_FALSE(NLibCommon::Deserialize(oBytes, dVal, cVal));
    oBytes.clear();
  }

  {
    float fRefVal = 71.98F;
    int16_t iRefVal = -376;
    uint32_t uRefVal = 61780U;
    EEnum1 eRefLabel = EEnum1::eLabel;
    double dRefVal = -2.77;

    uint8_t* upData = reinterpret_cast<uint8_t*>(&fRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(fRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    upData = reinterpret_cast<uint8_t*>(&iRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(iRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    upData = reinterpret_cast<uint8_t*>(&uRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(uRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    upData = reinterpret_cast<uint8_t*>(&eRefLabel);

    for (uint32_t uIndex = 0U; uIndex < sizeof(eRefLabel); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    upData = reinterpret_cast<uint8_t*>(&dRefVal);

    for (uint32_t uIndex = 0U; uIndex < sizeof(dRefVal); ++uIndex)
    {
      oBytes.push_back(upData[uIndex]);
    }

    EEnum1 eLabel1;

    EXPECT_TRUE(NLibCommon::Deserialize(oBytes, fVal, iVal16, uVal32, eLabel1, dVal));
    EXPECT_EQ(fVal, fRefVal);
    EXPECT_EQ(iVal16, iRefVal);
    EXPECT_EQ(uVal32, uRefVal);
    EXPECT_EQ(eLabel1, eRefLabel);
    EXPECT_EQ(dVal, dRefVal);

    EXPECT_FALSE(NLibCommon::Deserialize(oBytes, fVal, iVal16, uVal32, dVal));
    EXPECT_FALSE(NLibCommon::Deserialize(oBytes, uVal32, eLabel1, dVal));
    EXPECT_FALSE(NLibCommon::Deserialize(oBytes, dVal));
    oBytes.clear();
  }
}
