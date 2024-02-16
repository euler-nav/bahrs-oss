/**
 * @file CDroneCanHandler.h
 * @brief Declaration of the CDroneCanHandler class (DroneCAN wrapper basic software component).
 * @author Denis Ledkov
 * @date 16 February 2024
 */

#ifndef C_DRONECAN_HANDLER_H
#define C_DRONECAN_HANDLER_H

#include "General/CSoftwareComponentBase.h"

class CDroneCanHandler : public CSoftwareComponent<CDroneCanHandler, 1U>
{
	friend class CSoftwareComponent<CDroneCanHandler, 1U>;
	FORBID_CLASS_COPY_AND_MOVE(CDroneCanHandler)
	DECLARE_MANDATORY_APIS(CDroneCanHandler)

public:

	/**
	 * @brief Compose a navigation data message according to DroneCAN standard and send it
	 * to CAN bus.
	 */
	void SendNavigationDataMessage();

protected:

private:
	CDroneCanHandler () = default;
	~CDroneCanHandler () = default;
};

#endif /* C_DRONECAN_HANDLER_H */
