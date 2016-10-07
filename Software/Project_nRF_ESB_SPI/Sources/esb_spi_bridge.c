#include "nrf_esb.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_spis.h"

#define ESB_PACKET_COUNT   4
#define DAP_PACKET_SIZE   64

typedef enum
{ 
    TX_IDLE = 0,
    TX_BUSY, 
    TX_DONE
} TransferStatus_TypeDef;

static volatile TransferStatus_TypeDef esb_tx_status;
static volatile TransferStatus_TypeDef spi_tx_status;

static uint32_t S2E_IndexI;
static uint32_t S2E_IndexO;
static uint32_t E2S_IndexI;
static uint32_t E2S_IndexO;

static nrf_esb_payload_t tx_payload[ESB_PACKET_COUNT];
static nrf_esb_payload_t rx_payload[ESB_PACKET_COUNT];
uint8_t spi_temp_buffer[DAP_PACKET_SIZE];

nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
#define SPIS_INSTANCE 1 /**< SPIS instance index. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */

void nrf_esb_error_handler(uint32_t err_code, uint32_t line)
{
#if DEBUG //lint -e553
    while(true);
#else
    NVIC_SystemReset();
#endif

}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_tx_status = TX_IDLE;
}

/**
 * @brief SPIS user event handler.
 * @param event
 */
void spis_event_handler(nrf_drv_spis_event_t event)
{
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {
        S2E_IndexI = (S2E_IndexI + 1) % ESB_PACKET_COUNT;
        nrf_drv_spis_buffers_set(&spis, spi_temp_buffer, DAP_PACKET_SIZE, tx_payload[S2E_IndexI].data, DAP_PACKET_SIZE);
    }
}

/**
 * @brief ESB user event handler.
 * @param event
 */
void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
    case NRF_ESB_EVENT_TX_SUCCESS:
        esb_tx_status = TX_DONE;
        break;
    case NRF_ESB_EVENT_TX_FAILED:
        (void) nrf_esb_flush_tx();
        (void) nrf_esb_start_tx();
        esb_tx_status = TX_DONE;
        break;
    case NRF_ESB_EVENT_RX_RECEIVED:
        while (nrf_esb_read_rx_payload(&(rx_payload[E2S_IndexI])) == NRF_SUCCESS)
        {
            if(rx_payload[E2S_IndexI].length > 0)
            {
                E2S_IndexI = (E2S_IndexI + 1) % ESB_PACKET_COUNT;
            }
        }
        break;
    }
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config.payload_length           = 64;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.selective_auto_ack       = true;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
}


int main(void)
{
    ret_code_t err_code;
    unsigned int i;

    // Enable the constant latency sub power mode to minimize the time it takes
    // for the SPIS peripheral to become active after the CSN line is asserted
    // (when the CPU is in sleep mode).
    NRF_POWER->TASKS_CONSTLAT = 1;

    clocks_start();

    for (i = 0; i < ESB_PACKET_COUNT; i++)
    {
        tx_payload[i].pipe = 0;
        tx_payload[i].length = DAP_PACKET_SIZE;
    }
    
    S2E_IndexI = 0;
    S2E_IndexO = 0;
    E2S_IndexI = 0;
    E2S_IndexO = 0;
    
    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
    spi_config.ss_pin = 6;
//    spi_config.mode = NRF_DRV_SPI_MODE_0;
    nrf_drv_spi_init(&spi, &spi_config, spi_event_handler);
    nrf_gpio_pin_set(6);
    nrf_gpio_cfg_output(6);
    

    nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG(SPIS_INSTANCE);
    spis_config.csn_pin = 3;
    nrf_drv_spis_init(&spis, &spis_config, spis_event_handler);

    nrf_drv_spis_buffers_set(&spis, spi_temp_buffer, DAP_PACKET_SIZE, tx_payload[S2E_IndexI].data, DAP_PACKET_SIZE);

    esb_tx_status = TX_IDLE;
    spi_tx_status = TX_IDLE;

    while (true)
    {
        if ((S2E_IndexI != S2E_IndexO) && (esb_tx_status == TX_IDLE))
        {
            // Stop RX
            nrf_esb_stop_rx();
        
            nrf_esb_config.mode = NRF_ESB_MODE_PTX;
            nrf_esb_init(&nrf_esb_config);
        
            esb_tx_status = TX_BUSY;

            tx_payload[S2E_IndexO].pipe = 0;
            tx_payload[S2E_IndexO].length = DAP_PACKET_SIZE;
            tx_payload[S2E_IndexO].noack = false;
        
            nrf_esb_write_payload(&(tx_payload[S2E_IndexO]));
            
            S2E_IndexO = (S2E_IndexO + 1) % ESB_PACKET_COUNT;
        }

        if (esb_tx_status == TX_DONE)
        {
            esb_tx_status = TX_IDLE;
            nrf_esb_config.mode = NRF_ESB_MODE_PRX;
            nrf_esb_init(&nrf_esb_config);
            nrf_esb_start_rx();
        }
        
        if ((E2S_IndexI != E2S_IndexO) && (spi_tx_status == TX_IDLE))
        {
            spi_tx_status = TX_BUSY;

            nrf_drv_spi_transfer(&spi, rx_payload[E2S_IndexO].data, DAP_PACKET_SIZE, spi_temp_buffer, DAP_PACKET_SIZE);
            E2S_IndexO = (E2S_IndexO + 1) % ESB_PACKET_COUNT;
        }
    }
}
/*lint -restore */
