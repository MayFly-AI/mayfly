/*! ----------------------------------------------------------------------------
 * @file    deca_device_api.h
 * @brief   DW3000 API Functions
 *
 * @attention
 *
 * Copyright 2013 - 2021(c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef _DECA_DW_DEVICE_API_H_
#define _DECA_DW_DEVICE_API_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "deca_types.h"

    /* Returns the value to set in CP_CFG0_ID for STS length. The x is the enum value from dwt_sts_lengths_e */
#define GET_STS_REG_SET_VALUE(x) ((uint16_t)1 << ((x) + 2))

#define DWT_SFDTOC_DEF 129 // default SFD timeout value
#define MAX_RETRIES_FOR_PLL (50) /*The PLL calibration should take less than 400us, typically it is < 100us (however on some corners with ch9 it can take ~900us)*/

#define SQRT_FACTOR       181 /*Factor of sqrt(2) for calculation*/
#define STS_LEN_SUPPORTED 7 /*The supported STS length options*/
#define SQRT_SHIFT_VAL    7
#define SHIFT_VALUE       11
#define MOD_VALUE         2048
#define HALF_MOD          (MOD_VALUE >> 1)

#define MAX_RETRIES_FOR_PGF (3)
#define DELAY_20uUSec       (20) /*Delay of 20uSec(measured 24uSec)*/

    // DW3000 IDLE/INIT mode definitions
    typedef enum
    {
        DWT_DW_INIT = 0x0,
        DWT_DW_IDLE = 0x1,
        DWT_DW_IDLE_RC = 0x2,
    } dwt_idle_init_modes_e;

    // Defined constants for "mode" bit field parameter passed to dwt_setleds() function.
    typedef enum
    {
        DWT_LEDS_DISABLE = 0x00,
        DWT_LEDS_ENABLE = 0x01,
        DWT_LEDS_INIT_BLINK = 0x02,
        // Default blink time. Blink time is expressed in multiples of 14 ms. The value defined here is ~225 ms.
        DWT_LEDS_BLINK_TIME_DEF = 0x10,
    } dwt_setleds_mode_e;


    typedef enum
    {
        DWT_READ_OTP_PID = 0x10, // read part ID from OTP
        DWT_READ_OTP_LID = 0x20, // read lot ID from OTP
        DWT_READ_OTP_BAT = 0x40, // read ref voltage from OTP
        DWT_READ_OTP_TMP = 0x80, // read ref temperature from OTP
    } dwt_read_otp_modes_e;


    /* Enum used for selecting location to load DGC data from */
    typedef enum
    {
        DWT_DGC_LOAD_FROM_SW = 0,
        DWT_DGC_LOAD_FROM_OTP
    } dwt_dgc_load_location;


    // DW3000 SLEEP and WAKEUP configuration parameters
    typedef enum
    {
        DWT_PGFCAL = 0x0800,
        DWT_GOTORX = 0x0200,
        DWT_GOTOIDLE = 0x0100,
        DWT_SEL_OPS3 = 0x00C0,
        DWT_SEL_OPS2 = 0x0080, // Short OPS table
        DWT_SEL_OPS1 = 0x0040, // SCP
        DWT_SEL_OPS0 = 0x0000, // Long OPS table
        DWT_ALT_OPS = 0x0020,
        DWT_LOADLDO = 0x0010,
        DWT_LOADDGC = 0x0008,
        DWT_LOADBIAS = 0x0004,
        DWT_RUNSAR = 0x0002,
        DWT_CONFIG = 0x0001, // download the AON array into the HIF (configuration download)
    } dwt_on_wake_param_e;


    typedef enum
    {
        DBL_BUFF_OFF = 0x0,
        DBL_BUFF_ACCESS_BUFFER_0 = 0x1,
        DBL_BUFF_ACCESS_BUFFER_1 = 0x3,
    } dwt_dbl_buff_conf_e;


    /* */
    typedef enum
    {
        DWT_SUCCESS = 0,
        DWT_ERROR = -1,
        DWT_ERR_PLL_LOCK = -2,
        DWT_ERR_RX_CAL_PGF = -3,
        DWT_ERR_RX_CAL_RESI = -4,
        DWT_ERR_RX_CAL_RESQ = -5,
        DWT_ERR_RX_ADC_CAL = -6,
    } dwt_error_e;

    typedef enum
    {
        DW3000_SPI_RD_BIT = 0x0000U,
        DW3000_SPI_RD_FAST_CMD = 0x0001U,
        DW3000_SPI_WR_FAST_CMD = 0x0002U,
        DW3000_SPI_WR_BIT = 0x8000U,
        DW3000_SPI_AND_OR_8 = 0x8001U,
        DW3000_SPI_AND_OR_16 = 0x8002U,
        DW3000_SPI_AND_OR_32 = 0x8003U,
    } spi_modes_e;


    
    // TX/RX call-back data
    typedef struct
    {
        uint32_t status;     // initial value of register as ISR is entered
        uint16_t status_hi;  // initial value of register as ISR is entered, if relevant for that event type
        uint16_t datalength; // length of frame
        uint8_t  rx_flags;   // RX frame flags, see above
        uint8_t  dss_stat;   // Dual SPI status reg 11:38, 2 LSbits relevant : bit0 (DWT_CB_DSS_SPI1_AVAIL) and bit1 (DWT_CB_DSS_SPI2_AVAIL)
        struct dwchip_s *dw;
    } dwt_cb_data_t;

    // Call-back type for all interrupt events
    typedef void (*dwt_cb_t)(const dwt_cb_data_t *);



    // Defined constants when SPI CRC mode is used:
    typedef enum
    {
        DWT_SPI_CRC_MODE_NO = 0, /* No CRC */
        DWT_SPI_CRC_MODE_WR, /* This is used to enable SPI CRC check (the SPI CRC check will be enabled on DW3000 and CRC-8 added for SPI write transactions) */
        DWT_SPI_CRC_MODE_WRRD /* This is used to optionally enable additional CRC check on the SPI read operations, while the CRC check on the SPI write
                                 operations is also enabled */
    } dwt_spi_crc_mode_e;

    // Define DW3000 PDOA modes
    typedef enum
    {
        DWT_PDOA_M0 = 0x0, // DW PDOA mode is off
        DWT_PDOA_M1 = 0x1, // DW PDOA mode 1
        DWT_PDOA_M3 = 0x3, // DW PDOA mode 3
    } dwt_pdoa_mode_e;
    /*This Enum holds the index for factor calculation.*/
    typedef enum
    {
        DWT_STS_LEN_32 = 0,
        DWT_STS_LEN_64 = 1,
        DWT_STS_LEN_128 = 2,
        DWT_STS_LEN_256 = 3,
        DWT_STS_LEN_512 = 4,
        DWT_STS_LEN_1024 = 5,
        DWT_STS_LEN_2048 = 6
    } dwt_sts_lengths_e;

    // Define DW3000 STS modes
    typedef enum
    {
        DWT_STS_MODE_OFF = 0x0, // STS is off
        DWT_STS_MODE_1 = 0x1,   // STS mode 1
        DWT_STS_MODE_2 = 0x2,   // STS mode 2
        DWT_STS_MODE_ND = 0x3,  // STS with no data
        DWT_STS_MODE_SDC = 0x8, // Enable Super Deterministic Codes
        DWT_STS_CONFIG_MASK = 0xB,
        DWT_STS_CONFIG_MASK_NO_SDC = 0x3,
    } dwt_sts_mode_e;
    //! enums for selecting PHR rate
    typedef enum
    {
        DWT_PHRRATE_STD = 0x0, // standard PHR rate
        DWT_PHRRATE_DTA = 0x1, // PHR at data rate (6M81)
    } dwt_phr_rate_e;
    //! enums for selecting PHR modes
    typedef enum
    {
        DWT_PHRMODE_STD = 0x0, // standard PHR mode
        DWT_PHRMODE_EXT = 0x1, // DW proprietary extended frames PHR mode
    } dwt_phr_mode_e;

    //! enums for selecting the bit rate for data TX (and RX)
    //! These are defined for write (with just a shift) the TX_FCTRL register
    typedef enum
    {
        DWT_BR_850K = 0,   //!< UWB bit rate 850 kbits/s
        DWT_BR_6M8 = 1,    //!< UWB bit rate 6.8 Mbits/s
        DWT_BR_NODATA = 2, //!< No data (SP3 packet format)
    } dwt_uwb_bit_rate_e;
    //! enums for specifying SFD Types and size
    typedef enum
    {
        DWT_SFD_IEEE_4A = 0, //!< IEEE 8-bit ternary
        DWT_SFD_DW_8 = 1,    //!< DW 8-bit
        DWT_SFD_DW_16 = 2,   //!< DW 16-bit
        DWT_SFD_IEEE_4Z = 3, //!< IEEE 8-bit binary (4z)
        DWT_SFD_LEN8 = 8,    //!< IEEE, and DW 8-bit are length 8
        DWT_SFD_LEN16 = 16,  //!< DW 16-bit is length 16
    } dwt_sfd_type_e;

    //! enums for specifying Preamble Acquisition Chunk (PAC) Size in symbols
    typedef enum
    {
        DWT_PAC8 = 0,  //!< PAC  8 (recommended for RX of preamble length  128 and below
        DWT_PAC16 = 1, //!< PAC 16 (recommended for RX of preamble length  256
        DWT_PAC32 = 2, //!< PAC 32 (recommended for RX of preamble length  512
        DWT_PAC4 = 3,  //!< PAC  4 (recommended for RX of preamble length  < 127
    } dwt_pac_size_e;


    //! enums for specifying TX Preamble length in symbols
    //! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register
    //! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
    typedef enum
    {
        DWT_PLEN_4096 = 0x03, //! Standard preamble length 4096 symbols
        DWT_PLEN_2048 = 0x0A, //! Non-standard preamble length 2048 symbols
        DWT_PLEN_1536 = 0x06, //! Non-standard preamble length 1536 symbols
        DWT_PLEN_1024 = 0x02, //! Standard preamble length 1024 symbols
        DWT_PLEN_512 = 0x0d,  //! Non-standard preamble length 512 symbols
        DWT_PLEN_256 = 0x09,  //! Non-standard preamble length 256 symbols
        DWT_PLEN_128 = 0x05,  //! Non-standard preamble length 128 symbols
        DWT_PLEN_64 = 0x01,   //! Standard preamble length 64 symbols
        DWT_PLEN_32 = 0x04,   //! Non-standard length 32
        DWT_PLEN_72 = 0x07,   //! Non-standard length 72
    } dwt_tx_plen_e;

    /*! ------------------------------------------------------------------------------------------------------------------
     * Structure typedef: dwt_config_t
     *
     * Structure for setting device configuration via dwt_configure() function
     *
     */
    typedef struct
    {
        uint8_t chan;                 //!< Channel number (5 or 9)
        dwt_tx_plen_e txPreambLength; //!< DWT_PLEN_64..DWT_PLEN_4096
        dwt_pac_size_e rxPAC;         //!< Acquisition Chunk Size (Relates to RX preamble length)
        uint8_t txCode;               //!< TX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
        uint8_t rxCode;               //!< RX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
        dwt_sfd_type_e sfdType;       //!< SFD type (0 for short IEEE 8-bit standard, 1 for DW 8-bit, 2 for DW 16-bit, 3 for 4z BPRF)
        dwt_uwb_bit_rate_e dataRate;  //!< Data rate {DWT_BR_850K or DWT_BR_6M8}
        dwt_phr_mode_e phrMode;       //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
        dwt_phr_rate_e phrRate;       //!< PHR rate {0x0 - standard DWT_PHRRATE_STD, 0x1 - at datarate DWT_PHRRATE_DTA}
        uint16_t sfdTO;               //!< SFD timeout value (in symbols)
        dwt_sts_mode_e stsMode;       //!< STS mode (no STS, STS before PHR or STS after data)
        dwt_sts_lengths_e stsLength;  //!< STS length (the allowed values are listed in dwt_sts_lengths_e
        dwt_pdoa_mode_e pdoaMode;     //!< PDOA mode
#ifndef WIN32
    } __attribute__((packed)) dwt_config_t;
#else
} dwt_config_t;
#endif // WIN32


    typedef struct
    {
        uint8_t PGdly;
        // TX POWER
        // 31:24     TX_CP_PWR
        // 23:16     TX_SHR_PWR
        // 15:8      TX_PHR_PWR
        // 7:0       TX_DATA_PWR
        uint32_t power;
        uint16_t PGcount;
#ifndef WIN32
    } __attribute__((packed)) dwt_txconfig_t;
#else
} dwt_txconfig_t;
#endif // WIN32


    typedef enum
    {
        // Common for all devices
        GPIO_PIN0_EXTTXE = 0x2 << 0,     /* Deprecated, only works for DW3000. The pin operates as the EXTTXE output (output TX state) */
        GPIO_PIN1_EXTRXE = 0x2 << (1*3), /* Deprecated, only works for DW3000. The pin operates as the EXTRXE output (output RX state) */
        GPIO_PIN2_RXLED  = 0x1 << (2*3), /* The pin operates as the RXLED output */
        GPIO_PIN3_TXLED  = 0x1 << (3*3), /* The pin operates as the TXLED output */
        GPIO_PIN4_EXTDA  = 0x1 << (4*3), /* Deprecated, only works for DW3000. The pin operates to support external DA/PA */
        GPIO_PIN4_EXTTXE = 0x2 << (4*3), /* Deprecated, only works for DW37xx. The pin operates as the EXTTXE output (output TX state) */
        GPIO_PIN5_EXTTX  = 0x1 << (5*3), /* Deprecated, only works for DW3000. The pin operates to support external PA / TX enable */
        GPIO_PIN5_EXTRXE = 0x2 << (5*3), /* Deprecated, only works for DW37xx. The pin operates as the EXTRXE output (output RX state) */
        GPIO_PIN6_EXTRX  = 0x1 << (6*3), /* Deprecated, only works for DW3000. The pin operates to support external LNA */
        // DW3000
        DW3000_GPIO_PIN0_GPIO        = 0x0,
        DW3000_GPIO_PIN0_RXOKLED     = 0x1,
        DW3000_GPIO_PIN0_PDOA_SW_TX  = 0x2,
        DW3000_GPIO_PIN1_GPIO        = 0x0 << (1*3),
        DW3000_GPIO_PIN1_SFDLED      = 0x1 << (1*3),
        DW3000_GPIO_PIN1_PDOA_SW_RX  = 0x2 << (1*3),
        DW3000_GPIO_PIN2_GPIO        = 0x0 << (2*3),
        DW3000_GPIO_PIN2_RXLED       = 0x1 << (2*3),
        DW3000_GPIO_PIN2_PDOA_SW_RF1 = 0x2 << (2*3),
        DW3000_GPIO_PIN3_GPIO        = 0x0 << (3*3),
        DW3000_GPIO_PIN3_TXLED       = 0x1 << (3*3),
        DW3000_GPIO_PIN3_PDOA_SW_RF2 = 0x2 << (3*3),
        DW3000_GPIO_PIN4_GPIO        = 0x0 << (4*3),
        DW3000_GPIO_PIN4_EXTPA       = 0x1 << (4*3),
        DW3000_GPIO_PIN4_IRQ         = 0x2 << (4*3),
        DW3000_GPIO_PIN5_GPIO        = 0x0 << (5*3),
        DW3000_GPIO_PIN5_EXTTXE      = 0x1 << (5*3),
        DW3000_GPIO_PIN6_GPIO        = 0x0 << (6*3),
        DW3000_GPIO_PIN6_EXTRXE      = 0x1 << (6*3),
        DW3000_GPIO_PIN7_SYNC        = 0x0 << (7*3),
        DW3000_GPIO_PIN7_GPIO        = 0x1 << (7*3),
        DW3000_GPIO_PIN8_IRQ         = 0x0 << (8*3),
        DW3000_GPIO_PIN8_GPIO        = 0x1 << (8*3),
        // DW3700 and DW3720
        DW37XX_GPIO_PIN0_SPI2_CLK    = 0x0,
        DW37XX_GPIO_PIN0_RXOKLED     = 0x1,
        DW37XX_GPIO_PIN0_GPIO        = 0x2,
        DW37XX_GPIO_PIN1_SPI2_MISO   = 0x0 << (1*3),
        DW37XX_GPIO_PIN1_SFDLED      = 0x1 << (1*3),
        DW37XX_GPIO_PIN1_GPIO        = 0x2 << (1*3),
        DW37XX_GPIO_PIN2_IRQ2        = 0x0 << (2*3),
        DW37XX_GPIO_PIN2_RXLED       = 0x1 << (2*3),
        DW37XX_GPIO_PIN2_GPIO        = 0x2 << (2*3),
        DW37XX_GPIO_PIN3_SPI2_MOSI   = 0x0 << (3*3),
        DW37XX_GPIO_PIN3_TXLED       = 0x1 << (3*3),
        DW37XX_GPIO_PIN3_GPIO        = 0x2 << (3*3),
        DW37XX_GPIO_PIN4_GPIO        = 0x0 << (4*3),
        DW37XX_GPIO_PIN4_COEX_IN     = 0x1 << (4*3),
        DW37XX_GPIO_PIN4_PDOA_SW_TX  = 0x2 << (4*3),
        DW37XX_GPIO_PIN5_GPIO        = 0x0 << (5*3),
        DW37XX_GPIO_PIN5_COEX_OUT    = 0x1 << (5*3),
        DW37XX_GPIO_PIN5_PDOA_SW_RX  = 0x2 << (5*3),
        DW37XX_GPIO_PIN6_GPIO        = 0x0 << (6*3),
        DW37XX_GPIO_PIN6_EXT_SW_RX   = 0x1 << (6*3),
        DW37XX_GPIO_PIN6_PDOA_SW_RF1 = 0x2 << (6*3),
        DW37XX_GPIO_PIN7_SYNC        = 0x0 << (7*3),
        DW37XX_GPIO_PIN7_GPIO        = 0x1 << (7*3),
        DW37XX_GPIO_PIN7_PDOA_SW_RF2 = 0x2 << (7*3),
        DW37XX_GPIO_PIN8_IRQ         = 0x0 << (8*3),
        DW37XX_GPIO_PIN8_GPIO        = 0x1 << (8*3)
    } dwt_gpio_pin_e;

    /* Enum used for selecting channel for DGC on-wake kick. */
    typedef enum
    {
        DWT_DGC_SEL_CH5 = 0,
        DWT_DGC_SEL_CH9
    } dwt_dgc_chan_sel;















    // Defined constants for "lna_pa" bit field parameter passed to dwt_setlnapamode() function
    typedef enum
    {
        DWT_LNA_PA_DISABLE = 0x00,
        DWT_LNA_ENABLE = 0x01,
        DWT_PA_ENABLE = 0x02,
        DWT_TXRX_EN = 0x04,
    } dwt_setlnapmodes_e;
    /******************************************************************************
     * @brief Bit definition of the SYS_ENABLE register
     * exported for dwt_setinterrupt() API
     **/
    typedef enum
    {
        DWT_INT_TIMER1_BIT_MASK = (int)(0x80000000), // TIMER1 expiry
        DWT_INT_TIMER0_BIT_MASK = 0x40000000UL,      // TIMER0 expiry
        DWT_INT_ARFE_BIT_MASK = 0x20000000UL,        // Frame filtering error
        DWT_INT_CPERR_BIT_MASK = 0x10000000UL,       // STS quality warning/error
        DWT_INT_HPDWARN_BIT_MASK = 0x8000000UL,      // Half period warning flag when delayed TX/RX is used
        DWT_INT_RXSTO_BIT_MASK = 0x4000000UL,        // SFD timeout
        DWT_INT_PLL_HILO_BIT_MASK = 0x2000000UL,     // PLL calibration flag
        DWT_INT_RCINIT_BIT_MASK = 0x1000000UL,       // Device has entered IDLE_RC
        DWT_INT_SPIRDY_BIT_MASK = 0x800000UL,        // SPI ready flag
        DWT_INT_RXPTO_BIT_MASK = 0x200000UL,         // Preamble timeout
        DWT_INT_RXOVRR_BIT_MASK = 0x100000UL,        // RX overrun event when double RX buffer is used
        DWT_INT_VWARN_BIT_MASK = 0x80000UL,          // Brownout event detected
        DWT_INT_CIAERR_BIT_MASK = 0x40000UL,         // CIA error
        DWT_INT_RXFTO_BIT_MASK = 0x20000UL,          // RX frame wait timeout
        DWT_INT_RXFSL_BIT_MASK = 0x10000UL,          // Reed-Solomon error (RX sync loss)
        DWT_INT_RXFCE_BIT_MASK = 0x8000U,            // RX frame CRC error
        DWT_INT_RXFCG_BIT_MASK = 0x4000U,            // RX frame CRC good
        DWT_INT_RXFR_BIT_MASK = 0x2000U,             // RX ended - frame ready
        DWT_INT_RXPHE_BIT_MASK = 0x1000U,            // PHY header error
        DWT_INT_RXPHD_BIT_MASK = 0x800U,             // PHY header detected
        DWT_INT_CIADONE_BIT_MASK = 0x400U,           // CIA done
        DWT_INT_RXSFDD_BIT_MASK = 0x200U,            // SFD detected
        DWT_INT_RXPRD_BIT_MASK = 0x100U,             // Preamble detected
        DWT_INT_TXFRS_BIT_MASK = 0x80U,              // Frame sent
        DWT_INT_TXPHS_BIT_MASK = 0x40U,              // Frame PHR sent
        DWT_INT_TXPRS_BIT_MASK = 0x20U,              // Frame preamble sent
        DWT_INT_TXFRB_BIT_MASK = 0x10U,              // Frame transmission begins
        DWT_INT_AAT_BIT_MASK = 0x8U,                 // Automatic ACK transmission pending
        DWT_INT_SPICRCE_BIT_MASK = 0x4U,             // SPI CRC error
        DWT_INT_CP_LOCK_BIT_MASK = 0x2U,             // PLL locked
        DWT_INT_IRQS_BIT_MASK = 0x1U,                // Interrupt set
    } dwt_int_conf_e;
    // Defined constants for "mode" bitmask parameter passed into dwt_starttx() function.
    typedef enum
    {
        DWT_START_TX_IMMEDIATE = 0x00, //! Send the frame immediately
        DWT_START_TX_DELAYED = 0x01,   //! Send the frame at specified time (time must be less that half period away)
        DWT_RESPONSE_EXPECTED = 0x02,  //! Will enable the receiver after TX has completed
        DWT_START_TX_DLY_REF = 0x04,   //! Send the frame at specified time (time in DREF_TIME register + any time in DX_TIME register)
        DWT_START_TX_DLY_RS = 0x08,    //! Send the frame at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
        DWT_START_TX_DLY_TS = 0x10,    //! Send the frame at specified time (time in TX_TIME_LO register + any time in DX_TIME register)
        DWT_START_TX_CCA = 0x20,       //! Send the frame if no preamble detected within PTO time
    } dwt_starttx_mode_e;
/* User defined RX timeouts (frame wait timeout and preamble detect timeout) mask. */
#define SYS_STATUS_ALL_RX_TO (DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | DWT_INT_CPERR_BIT_MASK)

/* All RX errors mask. */
#define SYS_STATUS_ALL_RX_ERR                                                                                                                                  \
    (DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK | DWT_INT_ARFE_BIT_MASK | DWT_INT_CIAERR_BIT_MASK       \
        | DWT_INT_CPERR_BIT_MASK)

#define DWT_TIME_UNITS (1.0 / 499.2e6 / 128.0) //!< = 15.65e-12 s


typedef enum {
	DWT_DISABLE_INT=0,		//Disable these INT
	DWT_ENABLE_INT,			//Enable these INT
	DWT_ENABLE_INT_ONLY		//Enable only these INT
} dwt_INT_options_e;





//Transmit PONG


//! fast commands
#define CMD_DB_TOGGLE     0x13   //!< Toggle double buffer pointer
#define CMD_CLR_IRQS      0x12   //!< Clear all events/clear interrupt
#define CMD_CCA_TX_W4R    0x11   //!< Check if channel clear prior to TX, enable RX when TX done
#define CMD_DTX_REF_W4R   0x10   //!< Start delayed TX (as DTX_REF below), enable RX when TX done
#define CMD_DTX_RS_W4R    0xF    //!< Start delayed TX (as DTX_RS below), enable RX when TX done
#define CMD_DTX_TS_W4R    0xE    //!< Start delayed TX (as DTX_TS below), enable RX when TX done
#define CMD_DTX_W4R       0xD    //!< Start delayed TX (as DTX below), enable RX when TX done
#define CMD_TX_W4R        0xC    //!< Start TX (as below), enable RX when TX done
#define CMD_CCA_TX        0xB    //!< Check if channel clear prior to TX
#define CMD_DRX_REF       0xA    //!< Enable RX @ time = DREF_TIME + DX_TIME
#define CMD_DTX_REF       0x9    //!< Start delayed TX (RMARKER will be @ time = DREF_TIME + DX_TIME)
#define CMD_DRX_RS        0x8    //!< Enable RX @ time = RX_TIME + DX_TIME
#define CMD_DTX_RS        0x7    //!< Start delayed TX (RMARKER will be @ time = RX_TIME + DX_TIME)
#define CMD_DRX_TS        0x6    //!< Enable RX @ time = TX_TIME + DX_TIME
#define CMD_DTX_TS        0x5    //!< Start delayed TX (RMARKER will be @ time = TX_TIME + DX_TIME)
#define CMD_DRX           0x4    //!< Enable RX @ time specified in DX_TIME register
#define CMD_DTX           0x3    //!< Start delayed TX (RMARKER will be @ time set in DX_TIME register)
#define CMD_RX            0x2    //!< Enable RX
#define CMD_TX            0x1    //!< Start TX
#define CMD_TXRXOFF       0x0    //!< Turn off TX or RX, clear any TX/RX events and put DW3000 into IDLE

    // Defined constants for "mode" bitmask parameter passed into dwt_rxenable() function.
    typedef enum
    {
        DWT_START_RX_IMMEDIATE = 0x00, //! Enable the receiver immediately
        DWT_START_RX_DELAYED = 0x01,   //! Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
        //! If delay RX fails due to "late" error then if this flag is set, the RX will not be re-enabled immediately, and device will be in IDLE when function
        //! exits.
        DWT_IDLE_ON_DLY_ERR = 0x02,
        DWT_START_RX_DLY_REF = 0x04, //! Enable the receiver at specified time (time in DREF_TIME register + any time in DX_TIME register)
        DWT_START_RX_DLY_RS = 0x08,  //! Enable the receiver at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
        DWT_START_RX_DLY_TS = 0x10,  //! Enable the receiver at specified time (time in TX_TIME_LO register + any time in DX_TIME register)
    } dwt_startrx_mode_e;


#ifdef __cplusplus
}
#endif

#endif // _DECA_DW_DEVICE_API_H_
