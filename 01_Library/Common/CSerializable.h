/**
 * @file CSerializable.h
 * @brief Declaration of serializable object interface.
 * @copyright Copyright 2024 AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#ifndef C_SERIALIZABLE_H
#define C_SERIALIZABLE_H

#include <stdint.h>
#include <vector>

namespace NLibCommon
{
  /**
   * @brief Base class for serializable objects.
   */
  class CSerializable
  {
  public:
    /**
     * @brief Generate byte representation of the object.
     * @return Empty byte vector on failure, otherwise representation of the objects in the form of byte sequence.
     */
    virtual std::vector<uint8_t> ToByteVector()
    {
      return std::vector<uint8_t>();
    }

    /**
     * @brief Create (assign) an object from its byte representation.
     * @param korBytes Byte sequence to create the object from.
     * \return True on success, false on failure.
     */
    virtual bool FromByteVector(const std::vector<uint8_t>& korBytes)
    {
      return false;
    }
  };
}

#endif /* C_SERIALIZABLE_H */
