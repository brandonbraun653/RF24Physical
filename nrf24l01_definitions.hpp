#pragma once
#ifndef NRF24L01_DEFINITIONS_HPP
#define NRF24L01_DEFINITIONS_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdio>

namespace NRF24L
{
    /*----------------------------------------------
    General Definitions
    ----------------------------------------------*/
    constexpr size_t MAX_ADDRESS_WIDTH = 5;
    constexpr size_t MAX_PAYLOAD_WIDTH = 32;

    enum class Mode : uint8_t
    {
        POWER_DOWN,
        STANDBY_I,
        STANDBY_II,
        RX,
        TX,

        MAX_MODES,
        UNKNOWN_MODE
    };

    enum Pipe : uint8_t
    {
        PIPE_0 = 0,
        PIPE_1,
        PIPE_2,
        PIPE_3,
        PIPE_4,
        PIPE_5,

        MAX_NUM_PIPES,
        UNKNOWN_PIPE
    };

    /*----------------------------------------------
    Command Instructions
    ----------------------------------------------*/
    namespace Command
    {
        constexpr uint8_t REGISTER_MASK = 0x1F;       /* Masks off the largest available register address */
        constexpr uint8_t R_REGISTER = 0x00;          /* Read command and status registers  */
        constexpr uint8_t W_REGISTER = 0x20;          /* Write command and status registers  */
        constexpr uint8_t R_RX_PAYLOAD = 0x61;        /* Read RX Payload (1-32 bytes) */
        constexpr uint8_t W_TX_PAYLOAD = 0xA0;        /* Write TX Payload (1-32 bytes) */
        constexpr uint8_t FLUSH_TX = 0xE1;            /* Flush TX FIFO, used in TX Mode */
        constexpr uint8_t FLUSH_RX = 0xE2;            /* Flush RX FIFO, used in RX Mode */
        constexpr uint8_t REUSE_TX_PL = 0xE3;         /* Reuse last transmitted payload (PTX device only) */
        constexpr uint8_t ACTIVATE = 0x50;            /* This command, followed by 0x73, activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK */
        constexpr uint8_t R_RX_PL_WID = 0x60;         /* Read RX payload width for the top payload in the RX FIFO */
        constexpr uint8_t W_ACK_PAYLOAD = 0xA8;       /* Write Payload together with ACK packet */
        constexpr uint8_t W_TX_PAYLOAD_NO_ACK = 0xB0; /* Disables AUTOACK on this specific packet */
        constexpr uint8_t NOP = 0xFF;                 /* No operation */
    }

    /*----------------------------------------------
    Register Addresses
    ----------------------------------------------*/
    namespace Register
    {
        constexpr uint8_t CONFIG = 0x00;      /* Configuration Register */
        constexpr uint8_t EN_AA = 0x01;       /* Enable Auto Acknowledgment */
        constexpr uint8_t EN_RXADDR = 0x02;   /* Enable RX Addresses */
        constexpr uint8_t SETUP_AW = 0x03;    /* Setup of Address Width */
        constexpr uint8_t SETUP_RETR = 0x04;  /* Setup of Automatic Retransmission */
        constexpr uint8_t RF_CH = 0x05;       /* RF Channel Frequency Settings */
        constexpr uint8_t RF_SETUP = 0x06;    /* RF Channel Settings Register */
        constexpr uint8_t STATUS = 0x07;      /* Status Register */
        constexpr uint8_t OBSERVE_TX = 0x08;  /* Transmit Observe */
        constexpr uint8_t CD = 0x09;          /* Carrier Detect */
        constexpr uint8_t RX_ADDR_P0 = 0x0A;  /* Receive Address Data Pipe 0 */
        constexpr uint8_t RX_ADDR_P1 = 0x0B;  /* Receive Address Data Pipe 1 */
        constexpr uint8_t RX_ADDR_P2 = 0x0C;  /* Receive Address Data Pipe 2 */
        constexpr uint8_t RX_ADDR_P3 = 0x0D;  /* Receive Address Data Pipe 3 */
        constexpr uint8_t RX_ADDR_P4 = 0x0E;  /* Receive Address Data Pipe 4 */
        constexpr uint8_t RX_ADDR_P5 = 0x0F;  /* Receive Address Data Pipe 5 */
        constexpr uint8_t TX_ADDR = 0x10;     /* Transmit Address */
        constexpr uint8_t RX_PW_P0 = 0x11;    /* Number of bytes in RX Payload Data Pipe 0 */
        constexpr uint8_t RX_PW_P1 = 0x12;    /* Number of bytes in RX Payload Data Pipe 1 */
        constexpr uint8_t RX_PW_P2 = 0x13;    /* Number of bytes in RX Payload Data Pipe 2 */
        constexpr uint8_t RX_PW_P3 = 0x14;    /* Number of bytes in RX Payload Data Pipe 3 */
        constexpr uint8_t RX_PW_P4 = 0x15;    /* Number of bytes in RX Payload Data Pipe 4 */
        constexpr uint8_t RX_PW_P5 = 0x16;    /* Number of bytes in RX Payload Data Pipe 5 */
        constexpr uint8_t FIFO_STATUS = 0x17; /* FIFO Status Register */
        constexpr uint8_t DYNPD = 0x1C;       /* Enable Dynamic Payload Length for Data Pipes */
        constexpr uint8_t FEATURE = 0x1D;     /* Feature Register */
	}

    /*----------------------------------------------
    Register Bit Fields and Masks
    ----------------------------------------------*/
    namespace CONFIG
    {
        constexpr uint8_t Mask = 0x7F;

        constexpr uint8_t MASK_RX_DR_Pos = 6u;
        constexpr uint8_t MASK_RX_DR_Msk = 1u << MASK_RX_DR_Pos;
        constexpr uint8_t MASK_RX_DR = MASK_RX_DR_Msk;

        constexpr uint8_t MASK_TX_DS_Pos = 5u;
        constexpr uint8_t MASK_TX_DS_Msk = 1u << MASK_TX_DS_Pos;
        constexpr uint8_t MASK_TX_DS = MASK_TX_DS_Msk;

        constexpr uint8_t MASK_MAX_RT_Pos = 4u;
        constexpr uint8_t MASK_MAX_RT_Msk = 1u << MASK_MAX_RT_Pos;
        constexpr uint8_t MASK_MAX_RT = MASK_MAX_RT_Msk;

        constexpr uint8_t EN_CRC_Pos = 3u;
        constexpr uint8_t EN_CRC_Msk = 1u << EN_CRC_Pos;
        constexpr uint8_t EN_CRC = EN_CRC_Msk;

        constexpr uint8_t CRCO_Pos = 2u;
        constexpr uint8_t CRCO_Msk = 1u << CRCO_Pos;
        constexpr uint8_t CRCO = CRCO_Msk;

        constexpr uint8_t PWR_UP_Pos = 1u;
        constexpr uint8_t PWR_UP_Msk = 1u << PWR_UP_Pos;
        constexpr uint8_t PWR_UP = PWR_UP_Msk;

        constexpr uint8_t PRIM_RX_Pos = 0u;
        constexpr uint8_t PRIM_RX_Msk = 1u << PRIM_RX_Pos;
        constexpr uint8_t PRIM_RX = PRIM_RX_Msk;
    }

    namespace EN_AA
	{
        constexpr uint8_t Mask = 0x3F;

        constexpr uint8_t P5_Pos = 5u;
        constexpr uint8_t P5_Msk = 1u << P5_Pos;
        constexpr uint8_t P5 = P5_Msk;

        constexpr uint8_t P4_Pos = 4u;
        constexpr uint8_t P4_Msk = 1u << P4_Pos;
        constexpr uint8_t P4 = P4_Msk;

        constexpr uint8_t P3_Pos = 3u;
        constexpr uint8_t P3_Msk = 1u << P3_Pos;
        constexpr uint8_t P3 = P3_Msk;

        constexpr uint8_t P2_Pos = 2u;
        constexpr uint8_t P2_Msk = 1u << P2_Pos;
        constexpr uint8_t P2 = P2_Msk;

        constexpr uint8_t P1_Pos = 1u;
        constexpr uint8_t P1_Msk = 1u << P1_Pos;
        constexpr uint8_t P1 = P1_Msk;

        constexpr uint8_t P0_Pos = 0u;
        constexpr uint8_t P0_Msk = 1u << P0_Pos;
        constexpr uint8_t P0 = P0_Msk;
    }

    namespace EN_RXADDR
	{
        constexpr uint8_t Mask = 0x3F;

        constexpr uint8_t P5_Pos = 5u;
        constexpr uint8_t P5_Msk = 1u << P5_Pos;
        constexpr uint8_t P5 = P5_Msk;

        constexpr uint8_t P4_Pos = 4u;
        constexpr uint8_t P4_Msk = 1u << P4_Pos;
        constexpr uint8_t P4 = P4_Msk;

        constexpr uint8_t P3_Pos = 3u;
        constexpr uint8_t P3_Msk = 1u << P3_Pos;
        constexpr uint8_t P3 = P3_Msk;

        constexpr uint8_t P2_Pos = 2u;
        constexpr uint8_t P2_Msk = 1u << P2_Pos;
        constexpr uint8_t P2 = P2_Msk;

        constexpr uint8_t P1_Pos = 1u;
        constexpr uint8_t P1_Msk = 1u << P1_Pos;
        constexpr uint8_t P1 = P1_Msk;

        constexpr uint8_t P0_Pos = 0u;
        constexpr uint8_t P0_Msk = 1u << P0_Pos;
        constexpr uint8_t P0 = P0_Msk;
    }

    namespace SETUP_AW
    {
        constexpr uint8_t Msk = 0x03;

        constexpr uint8_t AW_Pos = 0u;
        constexpr uint8_t AW_Wid = 0x03;
        constexpr uint8_t AW_Msk = AW_Wid << AW_Pos;
        constexpr uint8_t AW = AW_Msk;
    }

    namespace SETUP_RETR
	{
        constexpr uint8_t Mask = 0xFF;

        constexpr uint8_t ARD_Pos = 4u;
        constexpr uint8_t ARD_Msk = 0x0F << ARD_Pos;
        constexpr uint8_t ARD = ARD_Msk;

        constexpr uint8_t ARC_Pos = 0u;
        constexpr uint8_t ARC_Msk = 0x0F << ARC_Pos;
        constexpr uint8_t ARC = ARC_Msk;
    }

    namespace RF_CH
	{
        constexpr uint8_t Mask = 0x7F;
    }

    namespace RF_SETUP
	{
        constexpr uint8_t Mask = 0x1F;

        constexpr uint8_t RF_DR_LOW_Pos = 5u;
        constexpr uint8_t RF_DR_LOW_Msk = 1u << RF_DR_LOW_Pos;
        constexpr uint8_t RF_DR_LOW = RF_DR_LOW_Msk;

        constexpr uint8_t PLL_LOCK_Pos = 4u;
        constexpr uint8_t PLL_LOCK_Msk = 1u << PLL_LOCK_Pos;
        constexpr uint8_t PLL_LOCK = PLL_LOCK_Msk;

        constexpr uint8_t RF_DR_HIGH_Pos = 3u;
        constexpr uint8_t RF_DR_HIGH_Msk = 1u << RF_DR_HIGH_Pos;
        constexpr uint8_t RF_DR_HIGH = RF_DR_HIGH_Msk;

        constexpr uint8_t RF_DR_Pos = 3u;
        constexpr uint8_t RF_DR_Msk = 1u << RF_DR_Pos;
        constexpr uint8_t RF_DR = RF_DR_Msk;

        constexpr uint8_t RF_PWR_Pos = 1u;
        constexpr uint8_t RF_PWR_Wid = 0x03;
        constexpr uint8_t RF_PWR_Msk = RF_PWR_Wid << RF_PWR_Pos;
        constexpr uint8_t RF_PWR = RF_PWR_Msk;

        constexpr uint8_t LNA_HCURR_Pos = 0u;
        constexpr uint8_t LNA_HCURR_Msk = 1u << LNA_HCURR_Pos;
        constexpr uint8_t LNA_HCURR = LNA_HCURR_Msk;
    }

    namespace STATUS
    {
        constexpr uint8_t Mask          = 0x7F;

        constexpr uint8_t RX_DR_Pos     = 6u;
        constexpr uint8_t RX_DR_Msk     = 1u << RX_DR_Pos;
        constexpr uint8_t RX_DR         = RX_DR_Msk;

        constexpr uint8_t TX_DS_Pos     = 5u;
        constexpr uint8_t TX_DS_Msk     = 1u << TX_DS_Pos;
        constexpr uint8_t TX_DS         = TX_DS_Msk;

        constexpr uint8_t MAX_RT_Pos    = 4u;
        constexpr uint8_t MAX_RT_Msk    = 1u << MAX_RT_Pos;
        constexpr uint8_t MAX_RT        = MAX_RT_Msk;

        constexpr uint8_t RX_P_NO_Pos   = 1u;
        constexpr uint8_t RX_P_NO_Wid   = 0x07;
        constexpr uint8_t RX_P_NO_Msk   = RX_P_NO_Wid << RX_P_NO_Pos;
        constexpr uint8_t RX_P_NO       = RX_P_NO_Msk;

        constexpr uint8_t TX_FULL_Pos   = 0u;
        constexpr uint8_t TX_FULL_Msk   = 1u << TX_FULL_Pos;
        constexpr uint8_t TX_FULL       = TX_FULL_Msk;

        #if defined(DEBUG)
        typedef struct
        {
            bool Bit_RX_DR = false;
            bool Bit_TX_DS = false;
            bool Bit_MAX_RT = false;
            bool Bit_TX_FULL = false;
            uint8_t Field_RX_P_NO = 0u;

            void convert(const uint8_t reg)
            {
                Bit_RX_DR = reg & RX_DR;
                Bit_TX_DS = reg & TX_DS;
                Bit_MAX_RT = reg & MAX_RT;
                Bit_TX_FULL = reg & TX_FULL;
                Field_RX_P_NO = reg & RX_P_NO;
            }
        } BitField;
        #endif
	}

    namespace OBSERVE_TX
    {
        constexpr uint8_t Mask = 0xFF;

        constexpr uint8_t PLOS_CNT_Pos = 4u;
        constexpr uint8_t PLOS_CNT_Wid = 0x0F;
        constexpr uint8_t PLOS_CNT_Msk = PLOS_CNT_Wid << PLOS_CNT_Pos;
        constexpr uint8_t PLOS_CNT = PLOS_CNT_Msk;

        constexpr uint8_t ARC_CNT_Pos = 0u;
        constexpr uint8_t ARC_CNT_Wid = 0x0F;
        constexpr uint8_t ARC_CNT_Msk = ARC_CNT_Wid << ARC_CNT_Pos;
        constexpr uint8_t ARC_CNT = ARC_CNT_Msk;
    }

    namespace CD
    {
        constexpr uint8_t Mask      = 0x01;
        constexpr uint8_t Reset     = 0x00;

        constexpr uint8_t CD_Pos    = 0u;
        constexpr uint8_t CD_Msk    = 1u << CD_Pos;
        constexpr uint8_t CD        = CD_Msk;
    }

    namespace RX_ADDR_P0
    {
        constexpr uint64_t Mask     = 0xFFFFFFFFFF;
        constexpr uint64_t Reset    = 0xE7E7E7E7E7;
    }

    namespace RX_ADDR_P1
    {
        constexpr uint64_t Mask     = 0xFFFFFFFFFF;
        constexpr uint64_t Reset    = 0xC2C2C2C2C2;
    }

    namespace RX_ADDR_P2
    {
        constexpr uint8_t Mask  = 0xFF;
        constexpr uint8_t Reset = 0xC3;
    }

    namespace RX_ADDR_P3
    {
        constexpr uint8_t Mask  = 0xFF;
        constexpr uint8_t Reset = 0xC4;
    }

    namespace RX_ADDR_P4
    {
        constexpr uint8_t Mask  = 0xFF;
        constexpr uint8_t Reset = 0xC5;
    }

    namespace RX_ADDR_P5
    {
        constexpr uint8_t Mask  = 0xFF;
        constexpr uint8_t Reset = 0xC6;
    }

    namespace TX_ADDR
    {
        constexpr uint64_t Mask     = 0xFFFFFFFFFF;
        constexpr uint64_t Reset    = 0xE7E7E7E7E7;
    }

    namespace RX_PW
    {
        constexpr uint8_t Mask  = 0x3F;
        constexpr uint8_t Reset = 0x00;
    }

    namespace FIFO_STATUS
    {
        constexpr uint8_t Mask = 0x7F;

        constexpr uint8_t TX_REUSE_Pos = 6u;
        constexpr uint8_t TX_REUSE_Msk = 1u << TX_REUSE_Pos;
        constexpr uint8_t TX_REUSE = TX_REUSE_Msk;

        constexpr uint8_t TX_FULL_Pos = 5u;
        constexpr uint8_t TX_FULL_Msk = 1u << TX_FULL_Pos;
        constexpr uint8_t TX_FULL = TX_FULL_Msk;

        constexpr uint8_t TX_EMPTY_Pos = 4u;
        constexpr uint8_t TX_EMPTY_Msk = 1u << TX_EMPTY_Pos;
        constexpr uint8_t TX_EMPTY = TX_EMPTY_Msk;

        constexpr uint8_t RX_FULL_Pos = 1u;
        constexpr uint8_t RX_FULL_Msk = 1u << RX_FULL_Pos;
        constexpr uint8_t RX_FULL = RX_FULL_Msk;

        constexpr uint8_t RX_EMPTY_Pos = 0u;
        constexpr uint8_t RX_EMPTY_Msk = 1u << RX_EMPTY_Pos;
        constexpr uint8_t RX_EMPTY = RX_EMPTY_Msk;
    }

    namespace DYNPD
	{
        constexpr uint8_t Mask = 0x3F;

        constexpr uint8_t DPL_P5_Pos = 5u;
        constexpr uint8_t DPL_P5_Msk = 1u << DPL_P5_Pos;
        constexpr uint8_t DPL_P5 = DPL_P5_Msk;

        constexpr uint8_t DPL_P4_Pos = 4u;
        constexpr uint8_t DPL_P4_Msk = 1u << DPL_P4_Pos;
        constexpr uint8_t DPL_P4 = DPL_P4_Msk;

        constexpr uint8_t DPL_P3_Pos = 3u;
        constexpr uint8_t DPL_P3_Msk = 1u << DPL_P3_Pos;
        constexpr uint8_t DPL_P3 = DPL_P3_Msk;

        constexpr uint8_t DPL_P2_Pos = 2u;
        constexpr uint8_t DPL_P2_Msk = 1u << DPL_P2_Pos;
        constexpr uint8_t DPL_P2 = DPL_P2_Msk;

        constexpr uint8_t DPL_P1_Pos = 1u;
        constexpr uint8_t DPL_P1_Msk = 1u << DPL_P1_Pos;
        constexpr uint8_t DPL_P1 = DPL_P1_Msk;

        constexpr uint8_t DPL_P0_Pos = 0u;
        constexpr uint8_t DPL_P0_Msk = 1u << DPL_P0_Pos;
        constexpr uint8_t DPL_P0 = DPL_P0_Msk;
    }

    namespace FEATURE
	{
        constexpr uint8_t MSK = 0x07;

        constexpr uint8_t EN_DPL_Pos = 2u;
        constexpr uint8_t EN_DPL_Msk = 1u << EN_DPL_Pos;
        constexpr uint8_t EN_DPL = EN_DPL_Msk;

        constexpr uint8_t EN_ACK_PAY_Pos = 1u;
        constexpr uint8_t EN_ACK_PAY_Msk = 1u << EN_ACK_PAY_Pos;
        constexpr uint8_t EN_ACK_PAY = EN_ACK_PAY_Msk;

        constexpr uint8_t EN_DYN_ACK_Pos = 0u;
        constexpr uint8_t EN_DYN_ACK_Msk = 1u << EN_DYN_ACK_Pos;
        constexpr uint8_t EN_DYN_ACK = EN_DYN_ACK_Msk;
    }

}


#endif /* NRF24L01_DEFINITIONS_HPP */
