/**
* @file TLabeledArray.h
* @brief Implementation of a labeled array template.
* @author Fedor Baklanov
* @date 19 December 2023
* @copyright Copyright 2023 AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef T_LABELED_ARRAY_H
#define T_LABELED_ARRAY_H

#include <type_traits>
#include <array>

/**
 * @brief A labeled array class template. MUST NOT be instatiated directly.
 * Use the marco DECLARE_LABELED_ARRAY_TEMPLATE to create labels automatically and declare the corresponding array template.
 * Allows access to elements by labels only.
 * @tparam ArrayElementType Array element type 
 * @tparam LabelEnum Enum used to label the array elements
*/
template<typename ArrayElementType, typename LabelEnum>
class TLabeledArrayInternal
{
public:
  static_assert(std::is_enum<LabelEnum>::value == true, "TLabeledArrayInternal: the 2nd template parameter must be an enum.");

  static constexpr uint32_t skuSize_{ static_cast<uint32_t>(LabelEnum::eCount) }; ///< Size of the labeled array

  using ELabels = LabelEnum; ///< Label enum type alias

  /**
   * @brief Default cnstructor. Does not initialize elements.
  */
  TLabeledArrayInternal() = default;

  /**
   * @brief Initialize all the array elements with the given value.
   * @param  Value
  */
  TLabeledArrayInternal(const ArrayElementType& element)
  {
    for (uint32_t uIndex = 0U; uIndex < skuSize_; ++uIndex)
    {
      data_[uIndex] = element;
    }
  }

  /**
   * @brief Initialize from a list.
   * The list length must be the same as the array length. The template will be discarded by means of SFINAE technique
   * if the element count in the list does not match array length, or is equal to 1.
  */
  template<typename... ArgTypes, typename std::enable_if<(sizeof...(ArgTypes) == skuSize_) && (skuSize_ > 1U), bool>::type = true>
  constexpr TLabeledArrayInternal(const ArgTypes&... elements) : data_{ elements... }
  {
  }

  /**
   * @brief Get pointer to the first element.
  */
  ArrayElementType* begin()
  {
    return &data_[0];
  }

  /**
   * @brief Get pointer to the first element.
  */
  const ArrayElementType* begin() const
  {
    return &data_[0];
  }

  /**
   * @brief Get pointer to the element after the last one.
  */
  ArrayElementType* end()
  {
    return begin() + skuSize_;
  }

  /**
   * @brief Get pointer to the element after the last one.
  */
  const ArrayElementType* end() const
  {
    return begin() + skuSize_;
  }

  /** 
   * @brief Get array element by label. Asserts if the label is not in the valid range.
  */
  ArrayElementType& operator[](LabelEnum eLabel)
  {
    uint32_t uIndex = static_cast<uint32_t>(eLabel);
    assert(uIndex < skuSize_);
    return data_[uIndex];
  }

  /**
   * @brief Get array element by label. Asserts if the label is not in the valid range.
  */
  const ArrayElementType& operator[](LabelEnum eLabel) const
  {
    uint32_t uIndex = static_cast<uint32_t>(eLabel);
    assert(uIndex < skuSize_);
    return data_[uIndex];
  }

#ifndef GOOGLETEST_INCLUDE_GTEST_GTEST_H_
private:
#endif /* GOOGLETEST_INCLUDE_GTEST_GTEST_H_ */
  ArrayElementType data_[skuSize_];
};


#define DECLARE_LABELED_ARRAY_TEMPLATE(TemplateName, ... ) \
struct TemplateName \
{ \
  enum class EnumType : uint32_t \
  { \
    __VA_ARGS__, \
    eCount \
  }; \
\
  template<typename ArrayElementType> using TLabeledArray = TLabeledArrayInternal<ArrayElementType, EnumType>; \
\
  using CArrayOfAllLabels = std::array<EnumType, static_cast<size_t>(EnumType::eCount)>; \
\
  static CArrayOfAllLabels GetArrayOfAllLabels() \
  { \
    CArrayOfAllLabels oAllLabels; \
\
    for (uint32_t uIndex = 0U; uIndex < oAllLabels.size(); ++uIndex) \
    { \
      oAllLabels[uIndex] = static_cast<EnumType>(uIndex); \
    } \
\
    return oAllLabels; \
  } \
};


#endif /* T_LABELED_ARRAY_H */
