#ifndef NRF24L01_DEFINITIONS_HPP
#define NRF24L01_DEFINITIONS_HPP

namespace NRF24L
{
    /*----------------------------------------------
    General Definitions
    ----------------------------------------------*/
    constexpr uint8_t PAYLOAD_LEN = 32;
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
    constexpr uint8_t EN_RXADDR_MSK = 0x3F;

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

};


#endif /* NRF24L01_DEFINITIONS_HPP */