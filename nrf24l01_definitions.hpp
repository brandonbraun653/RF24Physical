#ifndef NRF24L01_DEFINITIONS_HPP
#define NRF24L01_DEFINITIONS_HPP

namespace NRF24L
{
    /*----------------------------------------------
    General Definitions
    ----------------------------------------------*/
    constexpr size_t PAYLOAD_LEN = 32;
    constexpr uint8_t SPI_BUFFER_LEN = 1 + PAYLOAD_LEN;    /* Accounts for max payload of 32 bytes + 1 byte for the command */

    /*----------------------------------------------
    Register Addresses
    ----------------------------------------------*/
    constexpr uint8_t REG_CONFIG                = 0x00;  /* Configuration Register */
    constexpr uint8_t REG_EN_AA                 = 0x01;  /* Enable Auto Acknowledgment */
    constexpr uint8_t REG_EN_RXADDR             = 0x02;  /* Enable RX Addresses */
    constexpr uint8_t REG_SETUP_AW              = 0x03;  /* Setup of Address Width */
    constexpr uint8_t REG_SETUP_RETR            = 0x04;  /* Setup of Automatic Retransmission */
    constexpr uint8_t REG_RF_CH                 = 0x05;  /* RF Channel Frequency Settings */
    constexpr uint8_t REG_RF_SETUP              = 0x06;  /* RF Channel Settings Register */
    constexpr uint8_t REG_STATUS                = 0x07;  /* Status Register */
    constexpr uint8_t REG_OBSERVE_TX            = 0x08;  /* Transmit Observe */
    constexpr uint8_t REG_CD                    = 0x09;  /* Carrier Detect */
    constexpr uint8_t REG_RX_ADDR_P0            = 0x0A;  /* Receive Address Data Pipe 0 */
    constexpr uint8_t REG_RX_ADDR_P1            = 0x0B;  /* Receive Address Data Pipe 1 */
    constexpr uint8_t REG_RX_ADDR_P2            = 0x0C;  /* Receive Address Data Pipe 2 */
    constexpr uint8_t REG_RX_ADDR_P3            = 0x0D;  /* Receive Address Data Pipe 3 */
    constexpr uint8_t REG_RX_ADDR_P4            = 0x0E;  /* Receive Address Data Pipe 4 */
    constexpr uint8_t REG_RX_ADDR_P5            = 0x0F;  /* Receive Address Data Pipe 5 */
    constexpr uint8_t REG_TX_ADDR               = 0x10;  /* Transmit Address */
    constexpr uint8_t REG_RX_PW_P0              = 0x11;  /* Number of bytes in RX Payload Data Pipe 0 */
    constexpr uint8_t REG_RX_PW_P1              = 0x12;  /* Number of bytes in RX Payload Data Pipe 1 */
    constexpr uint8_t REG_RX_PW_P2              = 0x13;  /* Number of bytes in RX Payload Data Pipe 2 */
    constexpr uint8_t REG_RX_PW_P3              = 0x14;  /* Number of bytes in RX Payload Data Pipe 3 */
    constexpr uint8_t REG_RX_PW_P4              = 0x15;  /* Number of bytes in RX Payload Data Pipe 4 */
    constexpr uint8_t REG_RX_PW_P5              = 0x16;  /* Number of bytes in RX Payload Data Pipe 5 */
    constexpr uint8_t REG_FIFO_STATUS           = 0x17;  /* FIFO Status Register */
    constexpr uint8_t REG_DYNPD                 = 0x1C;  /* Enable Dynamic Payload Length for Data Pipes */
    constexpr uint8_t REG_FEATURE               = 0x1D;  /* Feature Register */

    /*----------------------------------------------
    Command Instructions
    ----------------------------------------------*/
    constexpr uint8_t CMD_REGISTER_MASK         = 0x1F;  /* Masks off the largest available register address */
    constexpr uint8_t CMD_R_REGISTER            = 0x00;  /* Read command and status registers  */
    constexpr uint8_t CMD_W_REGISTER            = 0x20;  /* Write command and status registers  */
    constexpr uint8_t CMD_R_RX_PAYLOAD          = 0x61;  /* Read RX Payload (1-32 bytes) */
    constexpr uint8_t CMD_W_TX_PAYLOAD          = 0xA0;  /* Write TX Payload (1-32 bytes) */
    constexpr uint8_t CMD_FLUSH_TX              = 0xE1;  /* Flush TX FIFO, used in TX Mode */
    constexpr uint8_t CMD_FLUSH_RX              = 0xE2;  /* Flush RX FIFO, used in RX Mode */
    constexpr uint8_t CMD_REUSE_TX_PL           = 0xE3;  /* Reuse last transmitted payload (PTX device only) */
    constexpr uint8_t CMD_ACTIVATE              = 0x50;  /* This command, followed by 0x73, activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK */
    constexpr uint8_t CMD_R_RX_PL_WID           = 0x60;  /* Read RX payload width for the top payload in the RX FIFO */
    constexpr uint8_t CMD_W_ACK_PAYLOAD         = 0xA8;  /* Write Payload together with ACK packet */
    constexpr uint8_t CMD_W_TX_PAYLOAD_NO_ACK   = 0xB0;  /* Disables AUTOACK on this specific packet */
    constexpr uint8_t CMD_NOP                   = 0xFF;  /* No operation */

    /*----------------------------------------------
    Register Bit Fields and Masks
    ----------------------------------------------*/

    /* Register: Config */
    constexpr uint8_t CONFIG_MSK                = 0x7F;

    constexpr uint8_t CONFIG_MASK_RX_DR_Pos     = 6u;
    constexpr uint8_t CONFIG_MASK_RX_DR_Msk     = 1u << CONFIG_MASK_RX_DR_Pos;
    constexpr uint8_t CONFIG_MASK_RX_DR         = CONFIG_MASK_RX_DR_Msk;

    constexpr uint8_t CONFIG_MASK_TX_DS_Pos     = 5u;
    constexpr uint8_t CONFIG_MASK_TX_DS_Msk     = 1u << CONFIG_MASK_TX_DS_Pos;
    constexpr uint8_t CONFIG_MASK_TX_DS         = CONFIG_MASK_TX_DS_Msk;

    constexpr uint8_t CONFIG_MASK_MAX_RT_Pos    = 4u;
    constexpr uint8_t CONFIG_MASK_MAX_RT_Msk    = 1u << CONFIG_MASK_MAX_RT_Pos;
    constexpr uint8_t CONFIG_MASK_MAX_RT        = CONFIG_MASK_MAX_RT_Msk;

    constexpr uint8_t CONFIG_EN_CRC_Pos         = 3u;
    constexpr uint8_t CONFIG_EN_CRC_Msk         = 1u << CONFIG_EN_CRC_Pos;
    constexpr uint8_t CONFIG_EN_CRC             = CONFIG_EN_CRC_Msk;

    constexpr uint8_t CONFIG_CRCO_Pos           = 2u;
    constexpr uint8_t CONFIG_CRCO_Msk           = 1u << CONFIG_CRCO_Pos;
    constexpr uint8_t CONFIG_CRCO               = CONFIG_CRCO_Msk;

    constexpr uint8_t CONFIG_PWR_UP_Pos         = 1u;
    constexpr uint8_t CONFIG_PWR_UP_Msk         = 1u << CONFIG_PWR_UP_Pos;
    constexpr uint8_t CONFIG_PWR_UP             = CONFIG_PWR_UP_Msk;

    constexpr uint8_t CONFIG_PRIM_RX_Pos        = 0u;
    constexpr uint8_t CONFIG_PRIM_RX_Msk        = 1u << CONFIG_PRIM_RX_Pos;
    constexpr uint8_t CONFIG_PRIM_RX            = CONFIG_PRIM_RX_Msk;

    /* Register: EN_AA */
    constexpr uint8_t EN_AA_MSK                 = 0x3F;

    constexpr uint8_t EN_AA_P5_Pos              = 5u;
    constexpr uint8_t EN_AA_P5_Msk              = 1u << EN_AA_P5_Pos;
    constexpr uint8_t EN_AA_P5                  = EN_AA_P5_Msk;

    constexpr uint8_t EN_AA_P4_Pos              = 4u;
    constexpr uint8_t EN_AA_P4_Msk              = 1u << EN_AA_P4_Pos;
    constexpr uint8_t EN_AA_P4                  = EN_AA_P4_Msk;

    constexpr uint8_t EN_AA_P3_Pos              = 3u;
    constexpr uint8_t EN_AA_P3_Msk              = 1u << EN_AA_P3_Pos;
    constexpr uint8_t EN_AA_P3                  = EN_AA_P3_Msk;

    constexpr uint8_t EN_AA_P2_Pos              = 2u;
    constexpr uint8_t EN_AA_P2_Msk              = 1u << EN_AA_P2_Pos;
    constexpr uint8_t EN_AA_P2                  = EN_AA_P2_Msk;

    constexpr uint8_t EN_AA_P1_Pos              = 1u;
    constexpr uint8_t EN_AA_P1_Msk              = 1u << EN_AA_P1_Pos;
    constexpr uint8_t EN_AA_P1                  = EN_AA_P1_Msk;

    constexpr uint8_t EN_AA_P0_Pos              = 0u;
    constexpr uint8_t EN_AA_P0_Msk              = 1u << EN_AA_P0_Pos;
    constexpr uint8_t EN_AA_P0                  = EN_AA_P0_Msk;

    /* Register: EN_RXADDR */
    constexpr uint8_t EN_RXADDR_MSK             = 0x3F;

    constexpr uint8_t EN_RXADDR_P5_Pos          = 5u;
    constexpr uint8_t EN_RXADDR_P5_Msk          = 1u << EN_RXADDR_P5_Pos;
    constexpr uint8_t EN_RXADDR_P5              = EN_RXADDR_P5_Msk;

    constexpr uint8_t EN_RXADDR_P4_Pos          = 4u;
    constexpr uint8_t EN_RXADDR_P4_Msk          = 1u << EN_RXADDR_P4_Pos;
    constexpr uint8_t EN_RXADDR_P4              = EN_RXADDR_P4_Msk;

    constexpr uint8_t EN_RXADDR_P3_Pos          = 3u;
    constexpr uint8_t EN_RXADDR_P3_Msk          = 1u << EN_RXADDR_P3_Pos;
    constexpr uint8_t EN_RXADDR_P3              = EN_RXADDR_P3_Msk;

    constexpr uint8_t EN_RXADDR_P2_Pos          = 2u;
    constexpr uint8_t EN_RXADDR_P2_Msk          = 1u << EN_RXADDR_P2_Pos;
    constexpr uint8_t EN_RXADDR_P2              = EN_RXADDR_P2_Msk;

    constexpr uint8_t EN_RXADDR_P1_Pos          = 1u;
    constexpr uint8_t EN_RXADDR_P1_Msk          = 1u << EN_RXADDR_P1_Pos;
    constexpr uint8_t EN_RXADDR_P1              = EN_RXADDR_P1_Msk;

    constexpr uint8_t EN_RXADDR_P0_Pos          = 0u;
    constexpr uint8_t EN_RXADDR_P0_Msk          = 1u << EN_RXADDR_P0_Pos;
    constexpr uint8_t EN_RXADDR_P0              = EN_RXADDR_P0_Msk;

    /* Register: SETUP_AW */


    /* Register: SETUP_RETR */
    constexpr uint8_t SETUP_RETR_Msk = 0xFF;

    constexpr uint8_t SETUP_RETR_ARD_Pos        = 4u;
    constexpr uint8_t SETUP_RETR_ARD_Msk        = 0x0F << SETUP_RETR_ARD_Pos;
    constexpr uint8_t SETUP_RETR_ARD            = SETUP_RETR_ARD_Msk;

    constexpr uint8_t SETUP_RETR_ARC_Pos        = 0u;
    constexpr uint8_t SETUP_RETR_ARC_Msk        = 0x0F << SETUP_RETR_ARC_Pos;
    constexpr uint8_t SETUP_RETR_ARC            = SETUP_RETR_ARC_Msk;

    /* Register: RF_CH */
    constexpr uint8_t RF_CH_Msk                 = 0x7F;

    /* Register: RF_SETUP */
    constexpr uint8_t RF_SETUP_Msk              = 0x1F;

    constexpr uint8_t RF_SETUP_RF_DR_LOW_Pos    = 5u;
    constexpr uint8_t RF_SETUP_RF_DR_LOW_Msk    = 1u << RF_SETUP_RF_DR_LOW_Pos;
    constexpr uint8_t RF_SETUP_RF_DR_LOW        = RF_SETUP_RF_DR_LOW_Msk;

    constexpr uint8_t RF_SETUP_PLL_LOCK_Pos     = 4u;
    constexpr uint8_t RF_SETUP_PLL_LOCK_Msk     = 1u << RF_SETUP_PLL_LOCK_Pos;
    constexpr uint8_t RF_SETUP_PLL_LOCK         = RF_SETUP_PLL_LOCK_Msk;

    constexpr uint8_t RF_SETUP_RF_DR_HIGH_Pos   = 3u;
    constexpr uint8_t RF_SETUP_RF_DR_HIGH_Msk   = 1u << RF_SETUP_RF_DR_HIGH_Pos;
    constexpr uint8_t RF_SETUP_RF_DR_HIGH       = RF_SETUP_RF_DR_HIGH_Msk;

    constexpr uint8_t RF_SETUP_RF_DR_Pos        = 3u;
    constexpr uint8_t RF_SETUP_RF_DR_Msk        = 1u << RF_SETUP_RF_DR_Pos;
    constexpr uint8_t RF_SETUP_RF_DR            = RF_SETUP_RF_DR_Msk;

    constexpr uint8_t RF_SETUP_RF_PWR_Pos       = 1u;
    constexpr uint8_t RF_SETUP_RF_PWR_Msk       = 3u << RF_SETUP_RF_PWR_Pos;
    constexpr uint8_t RF_SETUP_RF_PWR           = RF_SETUP_RF_PWR_Msk;

    constexpr uint8_t RF_SETUP_LNA_HCURR_Pos    = 0u;
    constexpr uint8_t RF_SETUP_LNA_HCURR_Msk    = 1u << RF_SETUP_LNA_HCURR_Pos;
    constexpr uint8_t RF_SETUP_LNA_HCURR        = RF_SETUP_LNA_HCURR_Msk;

    /* Register: STATUS */


    /* Register: OBSERVE_TX */


    /* Register: CD */


    /* Register: RX_ADDR_P0 */


    /* Register: RX_ADDR_P1 */


    /* Register: RX_ADDR_P2 */


    /* Register: RX_ADDR_P3 */


    /* Register: RX_ADDR_P4 */


    /* Register: RX_ADDR_P5 */


    /* Register: TX_ADDR */


    /* Register: RX_PW_P0 */


    /* Register: RX_PW_P1 */


    /* Register: RX_PW_P2 */


    /* Register: RX_PW_P3 */


    /* Register: RX_PW_P4 */


    /* Register: RX_PW_P5 */


    /* Register: FIFO_STATUS */


    /* Register: ACK_PLD */


    /* Register: TX_PLD */


    /* Register: RX_PLD */


    /* Register: DYNPD */


    /* Register: FEATURE */
    constexpr uint8_t FEATURE_MSK               = 0x07;

    constexpr uint8_t FEATURE_EN_DPL_Pos        = 2u;
    constexpr uint8_t FEATURE_EN_DPL_Msk        = 1u << FEATURE_EN_DPL_Pos;
    constexpr uint8_t FEATURE_EN_DPL            = FEATURE_EN_DPL_Msk;

    constexpr uint8_t FEATURE_EN_ACK_PAY_Pos    = 1u;
    constexpr uint8_t FEATURE_EN_ACK_PAY_Msk    = 1u << FEATURE_EN_ACK_PAY_Pos;
    constexpr uint8_t FEATURE_EN_ACK_PAY        = FEATURE_EN_ACK_PAY_Msk;

    constexpr uint8_t FEATURE_EN_DYN_ACK_Pos    = 0u;
    constexpr uint8_t FEATURE_EN_DYN_ACK_Msk    = 1u << FEATURE_EN_DYN_ACK_Pos;
    constexpr uint8_t FEATURE_EN_DYN_ACK        = FEATURE_EN_DYN_ACK_Msk;

};


#endif /* NRF24L01_DEFINITIONS_HPP */
