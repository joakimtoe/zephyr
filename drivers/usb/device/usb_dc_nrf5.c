#include <soc.h>
#include <string.h>
#include <usb/usb_dc.h>
#include <usb/usb_device.h>
#include "nrf_drv_usbd.h" 
#include "nrf_power.h" 
#include "nrf_clock.h"

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_USB_DRIVER_LEVEL
#include <logging/sys_log.h>

/** 
 * @brief Startup delay 
 * 
 * Number of microseconds to start USBD after powering up. 
 * Kind of port insert debouncing. 
 */ 
#define STARTUP_DELAY 100

/* 
 * Endpoint state. 
 */
typedef struct
{
    u16_t ep_mps;                  /** Endpoint max packet size */
    u8_t ep_type;                  /** Endpoint type */
    usb_dc_ep_callback cb;         /** Endpoint callback function */
    u8_t ep_stalled;               /** Endpoint stall flag */
    u8_t buf[NRF_DRV_USBD_EPSIZE]; /** Read buffer */
    u32_t read_count;              /** Number of bytes in read buffer  */
    u32_t read_offset;             /** Current offset in read buffer */
}usb_nrf5_ep_state_t;

/* 
 * USB device controller nrf5 private structure. 
 */ 
typedef struct 
{ 
    usb_dc_status_callback  status_cb;
    usb_nrf5_ep_state_t     in_ep[NRF_USBD_EPIN_CNT];
    usb_nrf5_ep_state_t     out_ep[NRF_USBD_EPOUT_CNT];
}usb_nrf5_ctrl_prv_t;

static usb_nrf5_ctrl_prv_t usb_nrf5;

int usb_dc_ep_start_read(u8_t ep, u8_t *data, u32_t max_data_len);

static usb_nrf5_ep_state_t *usb_nrf5_ep_state_get(u8_t ep)
{
    usb_nrf5_ep_state_t *p_ep_states;

    if (!NRF_USBD_EP_VALIDATE(ep))
    {
        return NULL;
    }

    if (NRF_USBD_EPOUT_CHECK(ep))
    {
        p_ep_states = usb_nrf5.out_ep;
    }
    else
    {
        p_ep_states = usb_nrf5.in_ep;
    }

    return &p_ep_states[NRF_USBD_EP_NR_GET(ep)];
}

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
            SYS_LOG_DBG("NRF_DRV_USBD_EVT_SOF");
        // 
        case NRF_DRV_USBD_EVT_RESET:
            SYS_LOG_DBG("NRF_DRV_USBD_EVT_RESET");
            /* Inform upper layers */ 
            if (usb_nrf5.status_cb) { 
                usb_nrf5.status_cb(USB_DC_RESET, NULL); 
            }       
            break; 
        //         
        case NRF_DRV_USBD_EVT_SUSPEND:
            SYS_LOG_DBG("NRF_DRV_USBD_EVT_SUSPEND");
            /* Inform upper layers */ 
            if (usb_nrf5.status_cb) { 
                usb_nrf5.status_cb(USB_DC_SUSPEND, NULL); 
            }       
            break; 
        //         
        case NRF_DRV_USBD_EVT_RESUME:
            SYS_LOG_DBG("NRF_DRV_USBD_EVT_RESUME");
            /* Inform upper layers */ 
            if (usb_nrf5.status_cb) { 
                usb_nrf5.status_cb(USB_DC_RESUME, NULL); 
            }              
            break; 
        //         
        case NRF_DRV_USBD_EVT_WUREQ:
            SYS_LOG_DBG("NRF_DRV_USBD_EVT_WUREQ");
            break; 
        //         
        case NRF_DRV_USBD_EVT_SETUP: 
        {
            SYS_LOG_DBG("NRF_DRV_USBD_EVT_SETUP");
            nrf_drv_usbd_setup_t setup;
            nrf_drv_usbd_setup_get(&setup);

            usb_nrf5_ep_state_t * p_ep_state = usb_nrf5_ep_state_get(NRF_DRV_USBD_EPOUT0);

            p_ep_state->read_count = sizeof(nrf_drv_usbd_setup_t);
            p_ep_state->read_offset = 0;
            memcpy(p_ep_state->buf, &setup, p_ep_state->read_count);

            if (p_ep_state->cb)
            {
                p_ep_state->cb((u8_t)NRF_DRV_USBD_EPOUT0, USB_DC_EP_SETUP);

                if (!(setup.wLength == 0) &&
                    !(REQTYPE_GET_DIR(setup.bmRequestType) == REQTYPE_DIR_TO_HOST))
                {
                    usb_dc_ep_start_read(NRF_DRV_USBD_EPOUT0,
                                         p_ep_state->buf,
                                         setup.wLength);
                }
            }
        } 
        break; 
        //         
        case NRF_DRV_USBD_EVT_EPTRANSFER: 
        {
            SYS_LOG_DBG("NRF_DRV_USBD_EVT_EPTRANSFER");
            u8_t ep_idx = NRF_USBD_EP_NR_GET(p_event->data.eptransfer.ep); 
            if (NRF_USBD_EPIN_CHECK(p_event->data.eptransfer.ep)) 
            { 
                if (usb_nrf5.in_ep[ep_idx].cb) 
                { 
                    usb_nrf5.in_ep[ep_idx].cb((u8_t)p_event->data.eptransfer.ep, USB_DC_EP_DATA_IN); 
                } 
            } 
            else 
            { 
                if (usb_nrf5.out_ep[ep_idx].cb) 
                { 
                    usb_nrf5.out_ep[ep_idx].cb((u8_t)p_event->data.eptransfer.ep, USB_DC_EP_DATA_OUT); 
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
    SYS_LOG_DBG("usb_dc_attach");
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
            nrf_drv_usbd_start(true);
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
    SYS_LOG_DBG("usb_dc_detach");
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
    SYS_LOG_DBG("usb_dc_reset");
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
    SYS_LOG_DBG("usb_dc_set_address: ep=0x%02x", addr);
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
    SYS_LOG_DBG("usb_dc_set_status_callback");
    usb_nrf5.status_cb = cb; 
 
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
    SYS_LOG_DBG("usb_dc_ep_configure: ep 0x%02x, ep_mps %u, ep_type %u",
                cfg->ep_addr, cfg->ep_mps, cfg->ep_type);

    nrf_drv_usbd_ep_max_packet_size_set(cfg->ep_addr, cfg->ep_mps);

    usb_nrf5_ep_state_t *p_ep_state = usb_nrf5_ep_state_get(cfg->ep_addr);

    if (!p_ep_state)
    {
        return -EINVAL;
    }

    // TODO: handle ep_type??
    p_ep_state->ep_mps = cfg->ep_mps;
    p_ep_state->ep_type = cfg->ep_type;

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
    SYS_LOG_DBG("usb_dc_ep_set_stall: ep=0x%02x", ep);
    nrf_drv_usbd_ep_stall((nrf_drv_usbd_ep_t)ep);

    usb_nrf5_ep_state_t *p_ep_state = usb_nrf5_ep_state_get(ep);

    if (!p_ep_state)
    {
        return -EINVAL;
    }

    p_ep_state->ep_stalled = 1;

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
    SYS_LOG_DBG("usb_dc_ep_clear_stall: ep=0x%02x", ep);
    nrf_drv_usbd_ep_stall_clear((nrf_drv_usbd_ep_t)ep);

    usb_nrf5_ep_state_t *p_ep_state = usb_nrf5_ep_state_get(ep);

    if (!p_ep_state)
    {
        return -EINVAL;
    }

    p_ep_state->ep_stalled = 0;
    p_ep_state->read_count = 0;

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
    SYS_LOG_DBG("usb_dc_ep_is_stalled: ep=0x%02x", ep);
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
    SYS_LOG_DBG("usb_dc_ep_halt: ep=0x%02x", ep);
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
    SYS_LOG_DBG("usb_dc_ep_enable: ep=0x%02x", ep);

    usb_nrf5_ep_state_t *p_ep_state = usb_nrf5_ep_state_get(ep);
    if (!p_ep_state)
    {
        return -EINVAL;
    }

    nrf_drv_usbd_ep_enable((nrf_drv_usbd_ep_t)ep);

    if (NRF_USBD_EPOUT_CHECK(ep) && ep != NRF_DRV_USBD_EPOUT0)
    {
        int ret = usb_dc_ep_start_read(ep,
                                   p_ep_state->buf,
                                   NRF_DRV_USBD_EPSIZE);
        if (ret)
            return ret;
    }
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
    SYS_LOG_DBG("usb_dc_ep_disable: ep=0x%02x", ep);
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
    SYS_LOG_DBG("usb_dc_ep_flush: ep=0x%02x", ep);
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
    SYS_LOG_DBG("usb_dc_ep_write: ep=0x%02x", ep);
    ret_code_t ret_code; 

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

int usb_dc_ep_start_read(u8_t ep, u8_t *data, u32_t max_data_len)
{
    SYS_LOG_DBG("usb_dc_ep_start_read: ep=0x%02x, len %u", ep, max_data_len);

    /* we flush EP0_IN by doing a 0 length receive on it 
    if (!EP_IS_OUT(ep) && (ep != EP0_IN || max_data_len))
    {
        SYS_LOG_ERR("invalid ep 0x%02x", ep);
        return -EINVAL;
    } */

    if (max_data_len > NRF_DRV_USBD_EPSIZE)
    {
        max_data_len = NRF_DRV_USBD_EPSIZE;
    }

    const nrf_drv_usbd_transfer_t transfer = {
        .p_data = {
            .rx = data,
        },
        .size = max_data_len,
        .flags = 0,
    };

    ret_code_t ret_code = nrf_drv_usbd_ep_transfer((nrf_drv_usbd_ep_t)ep, &transfer);
    if (ret_code != NRF_SUCCESS)
    {
        SYS_LOG_ERR("nrf_drv_usbd_ep_transfer failed(0x%02x), %d",
                    ep, (int)ret_code);
        return -EIO;
    }

    return 0;
}

int usb_dc_ep_get_read_count(u8_t ep, u32_t *read_bytes)
{
    if (!NRF_USBD_EPOUT_CHECK(ep))
    {
        SYS_LOG_ERR("invalid ep 0x%02x", ep);
        return -EINVAL;
    }

    *read_bytes = nrf_usbd_epout_size_get(ep);

    return 0;
}

int usb_dc_ep_read(const u8_t ep, u8_t *const data,
                   const u32_t max_data_len, u32_t *const read_bytes)
{
    usb_nrf5_ep_state_t *p_ep_state = usb_nrf5_ep_state_get(ep);
    u32_t read_count = p_ep_state->read_count;

    SYS_LOG_DBG("ep 0x%02x, %u bytes, %u+%u, %p", ep,
                max_data_len, p_ep_state->read_offset, read_count, data);

    if (max_data_len)
    {
        if (read_count > max_data_len)
        {
            read_count = max_data_len;
        }

        if (read_count)
        {
            memcpy(data,
                   &p_ep_state->buf[p_ep_state->read_offset],
                   read_count);
            p_ep_state->read_count -= read_count;
            p_ep_state->read_offset += read_count;
        }

        if ((ep != NRF_DRV_USBD_EPOUT0) && (!p_ep_state->read_count))
        {
            usb_dc_ep_start_read(ep,
                                 p_ep_state->buf,
                                 NRF_DRV_USBD_EPSIZE);
        }
    }

    if (read_bytes)
    {
        *read_bytes = read_count;
    }

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
    SYS_LOG_DBG("usb_dc_ep_set_callback: ep=0x%02x", ep);
    usb_nrf5_ep_state_t *p_ep_state = usb_nrf5_ep_state_get(ep);

    if (!p_ep_state)
    {
        return -EINVAL;
    }

    p_ep_state->cb = cb;

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
    SYS_LOG_DBG("usb_dc_ep_read_wait: ep=0x%02x", ep);
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
    SYS_LOG_DBG("usb_dc_ep_read_continue: ep=0x%02x", ep);
    return -1; 
} 
