/// @file CSoftwareComponentBase.h
/// @brief Declaration of base class template for all software components.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_SOFTWARE_COMPONENT_BASE_H
#define C_SOFTWARE_COMPONENT_BASE_H

#include "HelperSoftwareComponentMacros.h"
#include <assert.h>

template<class tDerivedComponentClass, unsigned tInstanceCount> class CSoftwareComponent
{
public:
  // Deletion of copy and move operations ensures that
  // also derived classes can neither be copied nor moved.
  FORBID_CLASS_COPY_AND_MOVE(CSoftwareComponent)

  // The virtual declarations below will prevent code
  // from compiling if derived classes do not implement
  // these mandatory APIs.

  /// @brief Initialize the software component.
  virtual void Init() = 0;

  /// @brief Get initialization status of the software component.
  /// @return True -- the component is initialized, false otherwise.
  virtual bool IsInitialized() = 0;

  static constexpr unsigned skuInstanceCount_ {tInstanceCount};

  /// @brief Get a reference to a software component instance. The function asserts if
  /// the instance index is invalid.
  /// @param uInstanceIndex Zero based instance index.
  /// @return Reference to the instance.
  static tDerivedComponentClass& GetInstance(unsigned uInstanceIndex = 0U)
  {
    // The assertion is a safety measure. We will be able to catch the
    // bug even if the implementation of the getInstanceImpl() does not
    // implement a similar check.
    assert(uInstanceIndex < skuInstanceCount_);
    return tDerivedComponentClass::getInstanceImpl(uInstanceIndex);
  }

protected:
  // Making the default constructor and destructor objects protected
  // guarantees that class instances cannot be declared anywhere
  // except for static methods of the derived class.
  CSoftwareComponent() = default;
  ~CSoftwareComponent() = default;

private:

};


#endif // C_SOFTWARE_COMPONENT_BASE_H
