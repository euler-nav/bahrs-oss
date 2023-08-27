/**
 * @file CSoftwareComponentExample.h
 * @brief An example showing how to declare a software component.
 * @author Fedor Baklanov
 * @date 27 May 2023
 */

#ifndef C_SOFTWARE_COMPONENT_EXAMPLE_H
#define C_SOFTWARE_COMPONENT_EXAMPLE_H

#include "General/CSoftwareComponent.h"

class CSoftwareComponentExample : public CSoftwareComponent<CSoftwareComponentExample, 3U>
{
  // The following lines are mandatory for all SW components
  friend class CSoftwareComponent<CSoftwareComponentExample, 3U>;
  FORBID_CLASS_COPY_AND_MOVE(CSoftwareComponentExample)
  DECLARE_MANDATORY_APIS(CSoftwareComponentExample)

  // Implement your custom functionality below.
public:
  void MyPublicMethod();
}


#endif /* C_SOFTWARE_COMPONENT_EXAMPLE_H */
