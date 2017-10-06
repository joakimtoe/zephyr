#include <usb/usb_dc.h> 
#include "nrf_drv_usbd.h" 
#include "nrf_power.h" 
#include "nrf_clock.h" 
 
/** 
 * @brief Startup delay 
 * 
 * Number of microseconds to start USBD after powering up. 
 * Kind of port insert debouncing. 
 */ 
#define STARTUP_DELAY 100 
 
/* 
 * USB controller private structure. 
 */ 
typedef struct 
{ 
    usb_dc_status_callback status_cb; 
    usb_dc_ep_callback     in_ep_cb[9]; 
    usb_dc_ep_callback     out_ep_cb[9]; 
}usb_nrf5_ctrl_prv_t; 
 
static usb_nrf5_ctrl_prv_t usb_nrf5_ctrl = {.status_cb = 0,  
                                            .in_ep_cb = {0},  
                                            .out_ep_cb = {0}}; 
 
static inline void usb_nrf5_udelay(u32_t us) 
{ 
	k_busy_wait(us); 
} 
 
/** 
 * @brief USB power state 
 * 
 * The single enumerator that holds all data about current state of USB 
 * related POWER. 
 * 
 * Organized this way that higher power state has higher numeric value 
 */ 
typedef enum 
{ 
    NRF_DRV_POWER_USB_STATE_DISCONNECTED, /**< No power on USB lines detected */ 
    NRF_DRV_POWER_USB_STATE_CONNECTED,    /**< The USB power is detected, but USB power regulator is not ready */ 
    NRF_DRV_POWER_USB_STATE_READY         /**< From the power point of view USB is ready for working */ 
}nrf_drv_power_usb_state_t; 
 
__STATIC_INLINE nrf_drv_power_usb_state_t nrf_drv_power_usbstatus_get(void) 
{ 
    uint32_t status = nrf_power_usbregstatus_get(); 
    if (0 == (status & NRF_POWER_USBREGSTATUS_VBUSDETECT_MASK)) 
    { 
        return NRF_DRV_POWER_USB_STATE_DISCONNECTED; 
    } 
    if (0 == (status & NRF_POWER_USBREGSTATUS_OUTPUTRDY_MASK)) 
    { 
        return NRF_DRV_POWER_USB_STATE_CONNECTED; 
    } 
    return NRF_DRV_POWER_USB_STATE_READY; 
} 
 
static void usbd_event_handler(nrf_drv_usbd_evt_t const * const p_event) 
{ 
    switch (p_event->type) 
    { 
        case NRF_DRV_USBD_EVT_SOF: 
        break; 
        // 
        case NRF_DRV_USBD_EVT_RESET: 
            /* Inform upper layers */ 
            if (usb_nrf5_ctrl.status_cb) { 
                usb_nrf5_ctrl.status_cb(USB_DC_RESET, NULL); 
            }       
            break; 
        //         
        case NRF_DRV_USBD_EVT_SUSPEND: 
            /* Inform upper layers */ 
            if (usb_nrf5_ctrl.status_cb) { 
                usb_nrf5_ctrl.status_cb(USB_DC_SUSPEND, NULL); 
            }       
            break; 
        //         
        case NRF_DRV_USBD_EVT_RESUME: 
            /* Inform upper layers */ 
            if (usb_nrf5_ctrl.status_cb) { 
                usb_nrf5_ctrl.status_cb(USB_DC_RESUME, NULL); 
            }              
            break; 
        //         
        case NRF_DRV_USBD_EVT_WUREQ: 
        break; 
        //         
        case NRF_DRV_USBD_EVT_SETUP: 
        {
            u8_t ep_idx = NRF_USBD_EP_NR_GET(NRF_DRV_USBD_EPOUT0);
            if (usb_nrf5_ctrl.out_ep_cb[ep_idx])
            {
                usb_nrf5_ctrl.out_ep_cb[ep_idx]((u8_t)NRF_DRV_USBD_EPOUT0, USB_DC_EP_SETUP);
            }
        } 
        break; 
        //         
        case NRF_DRV_USBD_EVT_EPTRANSFER: 
        { 
            u8_t ep_idx = NRF_USBD_EP_NR_GET(p_event->data.eptransfer.ep); 
            if (NRF_USBD_EPIN_CHECK(p_event->data.eptransfer.ep)) 
            { 
                if (usb_nrf5_ctrl.in_ep_cb[ep_idx]) 
                { 
                    usb_nrf5_ctrl.in_ep_cb[ep_idx]((u8_t)p_event->data.eptransfer.ep, USB_DC_EP_DATA_IN); 
                } 
            } 
            else 
            { 
                if (usb_nrf5_ctrl.out_ep_cb[ep_idx]) 
                { 
                    usb_nrf5_ctrl.out_ep_cb[ep_idx]((u8_t)p_event->data.eptransfer.ep, USB_DC_EP_DATA_OUT); 
                } 
            } 
        } 
        break; 
        //         
        default: 
        break; 
    } 
} 
 
static void init_power_clock(void) 
{ 
    nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART); 
    nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART); 
 
    while (!(nrf_clock_hf_is_running(NRF_CLOCK_HFCLK_HIGH_ACCURACY) && 
             nrf_clock_lf_is_running())) 
    { 
        /* Just waiting */ 
    } 
} 
 
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
    ret_code_t ret_code; 
 
    init_power_clock(); 
 
    if (NRF_DRV_USBD_ERRATA_ENABLE) 
    { 
        (void)nrf_drv_usbd_errata_104(); 
        (void)nrf_drv_usbd_errata_154(); 
    } 
 
    ret_code = nrf_drv_usbd_init(usbd_event_handler); 
    if (ret_code != NRF_SUCCESS) 
    { 
        return -1; 
    } 
 
    usb_nrf5_udelay(STARTUP_DELAY*5); 

    if (!nrf_drv_usbd_is_enabled()) 
    { 
        nrf_drv_usbd_enable(); 
    } 
    /* Wait for regulator power up */ 
    while (NRF_DRV_POWER_USB_STATE_CONNECTED 
            == 
            nrf_drv_power_usbstatus_get()) 
    { 
        /* Just waiting */ 
    } 

    if (NRF_DRV_POWER_USB_STATE_READY == nrf_drv_power_usbstatus_get()) 
    { 
        if (!nrf_drv_usbd_is_started()) 
        { 
            nrf_drv_usbd_start(false);

            /* Inform upper layers */
            if (usb_nrf5_ctrl.status_cb)
            {
                usb_nrf5_ctrl.status_cb(USB_DC_CONNECTED, NULL);
            }
        } 
    } 
    else 
    { 
        nrf_drv_usbd_disable(); 
        return -1; 
    } 
 
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
    ret_code_t ret_code; 
 
    if (nrf_drv_usbd_is_started()) 
    { 
        nrf_drv_usbd_stop(); 
    } 
     
    if (nrf_drv_usbd_is_enabled()) 
    { 
        nrf_drv_usbd_disable(); 
    } 
 
    if (nrf_drv_usbd_is_initialized()) 
    { 
        ret_code = nrf_drv_usbd_uninit(); 
        if (ret_code != NRF_SUCCESS) 
        { 
            return -1; 
        } 
    } 
 
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
    usb_nrf5_ctrl.status_cb = cb; 
 
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
    nrf_drv_usbd_ep_max_packet_size_set(cfg->ep_addr, cfg->ep_mps); 
    // TODO: handle ep_type?? 
 
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
    nrf_drv_usbd_ep_stall((nrf_drv_usbd_ep_t)ep); 
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
    nrf_drv_usbd_ep_stall_clear((nrf_drv_usbd_ep_t)ep); 
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
    bool stall = false; 
 
    if (!stalled) 
    { 
        return -1; 
    } 
 
    stall = nrf_drv_usbd_ep_stall_check((nrf_drv_usbd_ep_t)ep); 
 
    *stalled = (u8_t)stall; 
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
    nrf_drv_usbd_ep_stall((nrf_drv_usbd_ep_t)ep); 
    if (nrf_drv_usbd_ep_enable_check((nrf_drv_usbd_ep_t)ep)) 
    { 
        nrf_drv_usbd_ep_disable((nrf_drv_usbd_ep_t)ep); 
    } 
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
    nrf_drv_usbd_ep_enable((nrf_drv_usbd_ep_t)ep); 
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
    nrf_drv_usbd_ep_disable((nrf_drv_usbd_ep_t)ep); 
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
 
    if (NRF_USBD_EPOUT_CHECK(ep)) 
    { 
        return -1; 
    } 
    const nrf_drv_usbd_transfer_t transfer = { 
        .p_data = { 
            .tx = data, 
        }, 
        .size = data_len, 
        .flags = 0, 
    }; 
 
    ret_code = nrf_drv_usbd_ep_transfer((nrf_drv_usbd_ep_t)ep, &transfer); 
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
    ret_code_t ret_code; 
 
    if (NRF_USBD_EPIN_CHECK(ep)) 
    { 
        return -1; 
    } 
 
    const nrf_drv_usbd_transfer_t transfer = { 
        .p_data = { 
            .rx = data, 
        }, 
        .size = max_data_len, 
        .flags = 0, 
    }; 
 
    ret_code = nrf_drv_usbd_ep_transfer((nrf_drv_usbd_ep_t)ep, &transfer); 
    if (ret_code == NRF_SUCCESS) 
    { 
        return 0; 
    } 
 
    return -1; 
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
    u8_t ep_idx = NRF_USBD_EP_NR_GET(ep); 
     
    if (NRF_USBD_EPIN_CHECK(ep)) 
    { 
        usb_nrf5_ctrl.in_ep_cb[ep_idx] = cb; 
    } 
    else 
    { 
        usb_nrf5_ctrl.out_ep_cb[ep_idx] = cb; 
    } 
 
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
    return -1; 
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
    return -1; 
} 
