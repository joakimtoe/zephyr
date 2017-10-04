#include <usb/usb_dc.h>
#include "nrf_drv_usbd.h"

/**
 * @brief attach USB for device connection
 *
 * Function to attach USB for device connection. Upon success, the USB PLL
 * is enabled, and the USB device is now capable of transmitting and receiving
 * on the USB bus and of generating interrupts.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_attach(void)
{
    return 0;
}

/**
 * @brief detach the USB device
 *
 * Function to detach the USB device. Upon success, the USB hardware PLL
 * is powered down and USB communication is disabled.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_detach(void)
{
    return 0;
}

/**
 * @brief reset the USB device
 *
 * This function returns the USB device and firmware back to it's initial state.
 * N.B. the USB PLL is handled by the usb_detach function
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_reset(void)
{
    return 0;
}

/**
 * @brief set USB device address
 *
 * @param[in] addr device address
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_set_address(const u8_t addr)
{
    return 0;
}

/**
 * @brief set USB device controller status callback
 *
 * Function to set USB device controller status callback. The registered
 * callback is used to report changes in the status of the device controller.
 *
 * @param[in] cb callback function
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
    return 0;
}

/**
 * @brief configure endpoint
 *
 * Function to configure an endpoint. usb_dc_ep_cfg_data structure provides
 * the endpoint configuration parameters: endpoint address, endpoint maximum
 * packet size and endpoint type.
 *
 * @param[in] cfg Endpoint config
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const cfg)
{
    return 0;
}

/**
 * @brief set stall condition for the selected endpoint
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_set_stall(const u8_t ep)
{
    return 0;
}

/**
 * @brief clear stall condition for the selected endpoint
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_clear_stall(const u8_t ep)
{
    return 0;
}

/**
 * @brief check if selected endpoint is stalled
 *
 * @param[in]  ep       Endpoint address corresponding to the one
 *                      listed in the device configuration table
 * @param[out] stalled  Endpoint stall status
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_is_stalled(const u8_t ep, u8_t *const stalled)
{
    return 0;
}

/**
 * @brief halt the selected endpoint
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_halt(const u8_t ep)
{
    return 0;
}

/**
 * @brief enable the selected endpoint
 *
 * Function to enable the selected endpoint. Upon success interrupts are
 * enabled for the corresponding endpoint and the endpoint is ready for
 * transmitting/receiving data.
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_enable(const u8_t ep)
{
    return 0;
}

/**
 * @brief disable the selected endpoint
 *
 * Function to disable the selected endpoint. Upon success interrupts are
 * disabled for the corresponding endpoint and the endpoint is no longer able
 * for transmitting/receiving data.
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_disable(const u8_t ep)
{
    return 0;
}

/**
 * @brief flush the selected endpoint
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_flush(const u8_t ep)
{
    return 0;
}

/**
 * @brief write data to the specified endpoint
 *
 * This function is called to write data to the specified endpoint. The supplied
 * usb_ep_callback function will be called when data is transmitted out.
 *
 * @param[in]  ep        Endpoint address corresponding to the one
 *                       listed in the device configuration table
 * @param[in]  data      pointer to data to write
 * @param[in]  data_len  length of data requested to write. This may
 *                       be zero for a zero length status packet.
 * @param[out] ret_bytes bytes scheduled for transmission. This value
 *                       may be NULL if the application expects all
 *                       bytes to be written
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_write(const u8_t ep, const u8_t *const data,
                    const u32_t data_len, u32_t *const ret_bytes)
{
    ret_code_t ret_code;
    nrf_drv_usbd_ep_t nrf_usb_ep = NRF_DRV_USBD_EPOUT(ep);
    nrf_drv_usbd_transfer_t transfer = {
        .p_data = {
            .tx = data,
        },
        .size = data_len,
        .flags = 0,
    };

    ret_code = nrf_drv_usbd_ep_transfer(nrf_usb_ep, &transfer);
    if (ret_code == NRF_SUCCESS)
    {
        return 0;
    }

    return -1;
}

/**
 * @brief read data from the specified endpoint
 *
 * This function is called by the Endpoint handler function, after an OUT
 * interrupt has been received for that EP. The application must only call this
 * function through the supplied usb_ep_callback function. This function clears
 * the ENDPOINT NAK, if all data in the endpoint FIFO has been read,
 * so as to accept more data from host.
 *
 * @param[in]  ep           Endpoint address corresponding to the one
 *                          listed in the device configuration table
 * @param[in]  data         pointer to data buffer to write to
 * @param[in]  max_data_len max length of data to read
 * @param[out] read_bytes   Number of bytes read. If data is NULL and
 *                          max_data_len is 0 the number of bytes
 *                          available for read should be returned.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_read(const u8_t ep, u8_t *const data,
                   const u32_t max_data_len, u32_t *const read_bytes)
{
    return 0;
}

/**
 * @brief set callback function for the specified endpoint
 *
 * Function to set callback function for notification of data received and
 * available to application or transmit done on the selected endpoint,
 * NULL if callback not required by application code.
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 * @param[in] cb callback function
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_set_callback(const u8_t ep, const usb_dc_ep_callback cb)
{
    return 0;
}

/**
 * @brief read data from the specified endpoint
 *
 * This is similar to usb_dc_ep_read, the difference being that, it doesn't
 * clear the endpoint NAKs so that the consumer is not bogged down by further
 * upcalls till he is done with the processing of the data. The caller should
 * reactivate ep by invoking usb_dc_ep_read_continue() do so.
 *
 * @param[in]  ep           Endpoint address corresponding to the one
 *                          listed in the device configuration table
 * @param[in]  data         pointer to data buffer to write to
 * @param[in]  max_data_len max length of data to read
 * @param[out] read_bytes   Number of bytes read. If data is NULL and
 *                          max_data_len is 0 the number of bytes
 *                          available for read should be returned.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_read_wait(u8_t ep, u8_t *data, u32_t max_data_len,
                        u32_t *read_bytes)
{
    return 0;
}

/**
 * @brief Continue reading data from the endpoint
 *
 * Clear the endpoint NAK and enable the endpoint to accept more data
 * from the host. Usually called after usb_dc_ep_read_wait() when the consumer
 * is fine to accept more data. Thus these calls together acts as flow control
 * mechanism.
 *
 * @param[in]  ep           Endpoint address corresponding to the one
 *                          listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_read_continue(u8_t ep)
{
    return 0;
}