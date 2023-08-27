/**
 * @file CSoftwareComponetPort.h
 * @brief Declaration of software component port template.
 * @author Fedor Baklanov
 * @date 04 February 2022
 */

#ifndef C_SOFTWARE_COMPONENT_PORT_H
#define C_SOFTWARE_COMPONENT_PORT_H

#ifndef _MSC_VER
#include "cmsis_os.h"
#else
#include <stdint.h>
#endif /* _MSC_VER */

/**
 * The class implements a software component port. The port is a mutex-protected variable used by
 * software components to pass data from one to another.
 */
template<typename tPortDataType, uint8_t uPortId> class CSoftwareComponentPort
{
public:
  CSoftwareComponentPort() = default;

  ~CSoftwareComponentPort() = default;

  CSoftwareComponentPort(const CSoftwareComponentPort &orOther) = delete;

  CSoftwareComponentPort(CSoftwareComponentPort &&orOther) = delete;

  CSoftwareComponentPort& operator=(const CSoftwareComponentPort &orOther) = delete;

  CSoftwareComponentPort& operator=(CSoftwareComponentPort &&orOther) = delete;

  /**
   * Write data to the port.
   *
   * \param korData A reference to the data to be written to the port.
   * \return True -- success, false -- failure.
   */
  bool Write(const tPortDataType& korData)
  {
    bool bRetVal = false;

#ifndef _MSC_VER
    if (true == bPortReady_)
    {
      osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 5);

      if (osOK == eStatus)
      {
        oPortData_ = korData;
        osMutexRelease(pMutexHandle_);
        bRetVal = true;
      }
    }
#else
    oPortData_ = korData;
    bRetVal = true;
#endif /* _MSC_VER */

    return bRetVal;
  }

  /**
   * Read from the port.
   *
   * \param orData A reference to the output data structure.
   * \return True -- success, false -- failure.
   */
  bool Read(tPortDataType& orData)
  {
    bool bRetVal = false;

#ifndef _MSC_VER
    if (true == bPortReady_)
    {
      osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 5);

      if (osOK == eStatus)
      {
        orData = oPortData_;
        osMutexRelease(pMutexHandle_);
        bRetVal = true;
      }
    }
#else
    orData = oPortData_;
    bRetVal = true;
#endif /* _MSC_VER */

    return bRetVal;
  }

  /**
   * Initialize the port. The function creates the required mutex object.
   *
   * \return True -- success, false -- failure.
   */
  bool Init()
  {
    bool bRetVal = true;

#ifndef _MSC_VER
    pMutexHandle_ = osMutexNew(&sMutexAttributes_);

    if (NULL == pMutexHandle_)
    {
      bRetVal = false;
    }
    else
    {
      bPortReady_ = true;
      oPortData_ = tPortDataType();
    }
#else
    bPortReady_ = true;
    oPortData_ = tPortDataType();
#endif /* _MSC_VER */

    return bRetVal;
  }

  const uint8_t kuId_ { uPortId };

protected:

private:

#ifndef _MSC_VER
  /**
   * The typedef copied from a CMSIS_OS header.
   */
  typedef StaticSemaphore_t osStaticMutexDef_t;

  osMutexId_t pMutexHandle_; ///< Mutex handle.
  osStaticMutexDef_t sMutexControlBlock_; ///< Memory reserved for the mutex control block.
  const osMutexAttr_t sMutexAttributes_ { NULL, 0, &sMutexControlBlock_, sizeof(sMutexControlBlock_) }; ///< A structure with mutex attributes required for creation.
#endif /* _MSC_VER */

  tPortDataType oPortData_; ///< Port data storage.
  bool bPortReady_ {false}; ///< The flag to indicate if the port has been initialized successfully.

};


#endif /* C_SOFTWARE_COMPONENT_PORT_H */
