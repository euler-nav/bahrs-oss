/*
 * Standard services: Data Upload Service and Data Download Service
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_SERVICES_STD_DATA_UPLOAD_DOWNLOAD_H_
#define CANAEROSPACE_SERVICES_STD_DATA_UPLOAD_DOWNLOAD_H_

#include "../canaerospace.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CANAS_SRV_DATA_MAX_PAYLOAD_LEN 1020

static const int32_t CANAS_SRV_DDS_RESPONSE_XOFF    = 0;
static const int32_t CANAS_SRV_DDS_RESPONSE_XON     = 1;
static const int32_t CANAS_SRV_DDS_RESPONSE_ABORT   = -1;
static const int32_t CANAS_SRV_DDS_RESPONSE_INVALID = -2;

static const int32_t CANAS_SRV_DUS_RESPONSE_OK      = 0;
static const int32_t CANAS_SRV_DUS_RESPONSE_ABORT   = -1;
static const int32_t CANAS_SRV_DUS_RESPONSE_INVALID = -2;

/**
 * This callback issued when Start Download Request Message is received by this node.
 * Returned value then used to detect whether the application is ready to accept this transfer.
 * If application rejects the transfer, the remote node will get the returned error code via response message.
 * @param [in] pi               Instance pointer
 * @param [in] memid            MEMID of this transfer
 * @param [in] expected_datalen Approximate number of payload bytes the remote node wants to transmit.
 * @return                      Response code: XON/XOFF to accept transfer, anything else to reject the transfer.
 */
typedef int32_t (*CanasSrvDdsSlaveRequestCallback)(CanasInstance* pi, uint32_t memid, uint16_t expected_datalen);

/**
 * This callback is called when incoming DDS transfer completes.
 * If transfer fails callback will never be called.
 * @param [in] pi      Instance pointer
 * @param [in] memid   MEMID of this transfer
 * @param [in] pdata   Temporary pointer to the data buffer. Buffer will be destroyed once callback returns.
 * @param [in] datalen Number of bytes in the buffer.
 */
typedef void (*CanasSrvDdsSlaveDoneCallback)(CanasInstance* pi, uint32_t memid, void* pdata, uint16_t datalen);

/**
 * This callback is called when the local node receives Start Upload Request Message.
 * Returned value then used to detect whether the application is ready to send the requested data.
 * If application rejects the transfer, the remote node will get the returned error code via response message.
 * If application wants to accept the transfer, it must copy the relevant payload to the pointer provided.
 * @param [in]  pi                    Instance pointer
 * @param [in]  memid                 MEMID of this transfer
 * @param [in]  expected_datalen      Approximate number of messages the remote node wants to receive. Can be ignored.
 * @param [in]  pdatabuff             Data to be transmitted must be copied to this pointer
 * @param [out] pprovided_datalen_out Length of data to be transmitted, must be initialized by the application
 * @return                            Response code: OK to accept the transfer, anything else to reject.
 */
typedef int32_t (*CanasSrvDusSlaveRequestCallback)(CanasInstance* pi, uint32_t memid, uint16_t expected_datalen,
    void* pdatabuff, uint16_t* pprovided_datalen_out);

/**
 * Transfer result codes
 */
typedef enum
{
    CANAS_SRV_DATA_SESSION_OK,
    CANAS_SRV_DATA_SESSION_TIMEOUT,
    CANAS_SRV_DATA_SESSION_LOCAL_ERROR,
    CANAS_SRV_DATA_SESSION_REMOTE_ERROR,
    CANAS_SRV_DATA_SESSION_CHECKSUM_ERROR,
    CANAS_SRV_DATA_SESSION_UNEXPECTED_RESPONSE
} CanasSrvDataSessionStatus;

typedef struct
{
    CanasSrvDataSessionStatus status; //!< Result code
    int32_t remote_error_code;        //!< Defined only if status == @ref CANAS_SRV_DATA_SESSION_REMOTE_ERROR
    uint8_t node_id;                  //!< Remote Node ID
    uint32_t memid;                   //!< MEMID of this transfer
    void* parg;                       //!< Custom argument
} CanasSrvDdsMasterDoneCallbackArgs;
typedef void (*CanasSrvDdsMasterDoneCallback)(CanasInstance* pi, CanasSrvDdsMasterDoneCallbackArgs* pargs);

typedef struct
{
    CanasSrvDataSessionStatus status; //!< Result code
    int32_t remote_error_code;        //!< Defined only if status == @ref CANAS_SRV_DATA_SESSION_REMOTE_ERROR
    uint8_t node_id;                  //!< Remote Node ID
    uint32_t memid;                   //!< MEMID of this transfer
    void* pdata;                      //!< Pointer to the data buffer
    int datalen;                      //!< Number of bytes in the data buffer
    void* parg;                       //!< Custom argument
} CanasSrvDusMasterDoneCallbackArgs;
typedef void (*CanasSrvDusMasterDoneCallback)(CanasInstance* pi, CanasSrvDusMasterDoneCallbackArgs* pargs);

/**
 * Initialize DUS and/or DDS services
 * @param [in] pi                  Instance pointer
 * @param [in] max_active_sessions Maximum number of simultaneous transfers
 * @param [in] rx_request_callback Slave callback, optional
 * @param [in] rx_done_callback    Slave callback; NULL to suppress DDS initialization
 * @param [in] tx_request_callback Slave callback; NULL to suppress DUS initialization
 * @return                         @ref CanasErrorCode
 */
int canasSrvDataInit(CanasInstance* pi, uint8_t max_active_sessions,
                     CanasSrvDdsSlaveRequestCallback rx_request_callback,
                     CanasSrvDdsSlaveDoneCallback rx_done_callback,
                     CanasSrvDusSlaveRequestCallback tx_request_callback);

/**
 * Override default settings. Normally should not be used.
 * Each parameter may be NULL to left it unchanged.
 * @param [in] pi                    Instance pointer
 * @param [in] ptx_interval_usec     Interval between subsequent outgoing messages, for each transfer
 * @param [in] psession_timeout_usec Session timeout. XOFF state may trigger this timeout too.
 * @return                           @ref CanasErrorCode
 */
int canasSrvDataOverrideDefaults(CanasInstance* pi, uint32_t* ptx_interval_usec, uint32_t* psession_timeout_usec);

/**
 * Send data to a remote node.
 * @param [in] pi           Instance pointer
 * @param [in] node_id      Remote Node ID
 * @param [in] memid        MEMID
 * @param [in] pdata        Data will be copied from this pointer to internal buffer
 * @param [in] datalen      Data length
 * @param [in] callback     Master callback
 * @param [in] callback_arg Custom argument
 * @return                  @ref CanasErrorCode
 */
int canasSrvDdsDownloadTo(CanasInstance* pi, uint8_t node_id, uint32_t memid, const void* pdata, uint16_t datalen,
                          CanasSrvDdsMasterDoneCallback callback, void* callback_arg);

/**
 * Request data from a remote node
 * @param [in] pi               Instance pointer
 * @param [in] node_id          Remote Node ID
 * @param [in] memid            MEMID
 * @param [in] expected_datalen Number of bytes you expect to receive
 * @param [in] callback         Master callback
 * @param [in] callback_arg     Custom argument
 * @return                      @ref CanasErrorCode
 */
int canasSrvDusUploadFrom(CanasInstance* pi, uint8_t node_id, uint32_t memid, uint16_t expected_datalen,
                          CanasSrvDusMasterDoneCallback callback, void* callback_arg);

#ifdef __cplusplus
}
#endif
#endif
