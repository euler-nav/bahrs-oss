/// @file DataSerializer.h
/// @brief Declaration of functions and templates for data serialization.
/// @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef DATA_SERIALIZER_H
#define DATA_SERIALIZER_H

#include <stdint.h>
#include <vector>
#include <type_traits>

namespace NLibCommon
{
  /// @brief Returns size of the argument.
  /// @tparam Type Type of the argument.
  /// @param arg Argument.
  /// @return Size of the argument.
  template<typename Type> constexpr uint32_t SizeOfAllVariables(Type arg)
  {
    return sizeof(arg);
  }

  /// @brief Returns cumulative size of all argumet types.
  /// @tparam FirstType Type of the first argument.
  /// @tparam ...Types Argument types.
  /// @param firstVar The first argument.
  /// @param ...vars The rest arguments.
  /// @return Cumulative size of all arguments.
  template<typename FirstType, typename... Types> constexpr uint32_t SizeOfAllVariables(FirstType firstVar, Types... vars)
  {
    return SizeOfAllVariables(firstVar) + SizeOfAllVariables(vars...);
  }

  namespace internal
  {
    template<typename Type> void serialize(std::vector<uint8_t>& orBytes, const Type& arg)
    {
      static_assert((std::is_arithmetic<Type>::value || std::is_enum<Type>::value) && (!std::is_same<Type, bool>::value), "Error serialize(): unsupported argument type.");

      const uint8_t* upData = reinterpret_cast<const uint8_t*>(&arg);

      for (uint32_t uByteToCopy = 0U; uByteToCopy < sizeof(arg); ++uByteToCopy)
      {
        orBytes.push_back(upData[uByteToCopy]);
      }
    }

    template<typename FirstType, typename... Types> void serialize(std::vector<uint8_t>& orBytes, const FirstType& firstArg, const Types&... args)
    {
      internal::serialize(orBytes, firstArg);
      internal::serialize(orBytes, args...);
    }

    template<typename Type> void deserialize(const std::vector<uint8_t>& korBytes, uint32_t uStartIndex, Type& arg)
    {
      static_assert((std::is_arithmetic<Type>::value || std::is_enum<Type>::value) && (!std::is_same<Type, bool>::value), "Error deserialize(): unsupported argument type.");

      uint8_t* upDestination = reinterpret_cast<uint8_t*>(&arg);
      uint8_t uBytesToCopy = sizeof(arg);

      while (uBytesToCopy > 0U)
      {
        --uBytesToCopy;
        upDestination[uBytesToCopy] = korBytes[uStartIndex + uBytesToCopy];
      }
    }

    template<typename FirstType, typename... Types> void deserialize(const std::vector<uint8_t>& korBytes, uint32_t uStartIndex, FirstType& firstArg, Types&... args)
    {
      internal::deserialize(korBytes, uStartIndex, firstArg);
      internal::deserialize(korBytes, uStartIndex + sizeof(firstArg), args...);
    }
  }

  /// @brief Serialize a sequence on variables
  /// All arguments must be of arithmetic type or an enum, and not boolean.
  /// @tparam ...Types Argument types.
  /// @param orBytes Output byte vector.
  /// @param ...args Sequence of arguments to be serialized.
  /// @return True on success, false on failure.
  template<typename... Types> bool Serialize(std::vector<uint8_t>& orBytes, const Types&... args)
  {
    bool bStatus{ false };
    
    orBytes.clear();
    orBytes.reserve(SizeOfAllVariables(args...));

    if (SizeOfAllVariables(args...) <= orBytes.capacity())
    {
      internal::serialize(orBytes, args...);
      bStatus = true;
    }

    return bStatus;
  }

  /// @brief Parse a sequence of bytes and assign values to arguments.
  /// All arguments must be of arithmetic type or an enum, and not boolean.
  /// @tparam ...Types Argument types.
  /// @param korBytes Byte vector to parse.
  /// @param ...args Arguments to assign values to.
  /// @return True on success, false on failure.
  template<typename... Types> bool Deserialize(const std::vector<uint8_t>& korBytes, Types&... args)
  {
    bool bStatus{ false };

    if (korBytes.size() == SizeOfAllVariables(args...))
    {
      internal::deserialize(korBytes, 0U, args...);
      return true;
    }

    return bStatus;
  }
}

#endif // DATA_SERIALIZER_H
