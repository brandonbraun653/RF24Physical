#ifndef NRF24L01_DEFINITIONS_HPP
#define NRF24L01_DEFINITIONS_HPP

/*----------------------------------------------
General Definitions
----------------------------------------------*/
#define NRF24L_PAYLOAD_LEN              (32)
#define NRF24L_SPI_BUFFER_LEN           (1 + NRF24L_PAYLOAD_LEN)    /* Accounts for max payload of 32 bytes + 1 byte for the command */

/*----------------------------------------------
Register Addresses
----------------------------------------------*/
#define NRF24L_REG_CONFIG               (0x00)  /* Configuration Register */
#define NRF24L_REG_EN_AA                (0x01)  /* Enable Auto Acknowledgment */
#define NRF24L_REG_EN_RXADDR            (0x02)  /* Enable RX Addresses */
#define NRF24L_REG_SETUP_AW             (0x03)  /* Setup of Address Width */
#define NRF24L_REG_SETUP_RETR           (0x04)  /* Setup of Automatic Retransmission */
#define NRF24L_REG_RF_CH                (0x05)  /* RF Channel Frequency Settings */
#define NRF24L_REG_RF_SETUP             (0x06)  /* RF Channel Settings Register */
#define NRF24L_REG_STATUS               (0x07)  /* Status Register */
#define NRF24L_REG_OBSERVE_TX           (0x08)  /* Transmit Observe */
#define NRF24L_REG_CD                   (0x09)  /* Carrier Detect */
#define NRF24L_REG_RX_ADDR_P0           (0x0A)  /* Receive Address Data Pipe 0 */
#define NRF24L_REG_RX_ADDR_P1           (0x0B)  /* Receive Address Data Pipe 1 */
#define NRF24L_REG_RX_ADDR_P2           (0x0C)  /* Receive Address Data Pipe 2 */
#define NRF24L_REG_RX_ADDR_P3           (0x0D)  /* Receive Address Data Pipe 3 */
#define NRF24L_REG_RX_ADDR_P4           (0x0E)  /* Receive Address Data Pipe 4 */
#define NRF24L_REG_RX_ADDR_P5           (0x0F)  /* Receive Address Data Pipe 5 */
#define NRF24L_REG_TX_ADDR              (0x10)  /* Transmit Address */
#define NRF24L_REG_RX_PW_P0             (0x11)  /* Number of bytes in RX Payload Data Pipe 0 */
#define NRF24L_REG_RX_PW_P1             (0x12)  /* Number of bytes in RX Payload Data Pipe 1 */
#define NRF24L_REG_RX_PW_P2             (0x13)  /* Number of bytes in RX Payload Data Pipe 2 */
#define NRF24L_REG_RX_PW_P3             (0x14)  /* Number of bytes in RX Payload Data Pipe 3 */
#define NRF24L_REG_RX_PW_P4             (0x15)  /* Number of bytes in RX Payload Data Pipe 4 */
#define NRF24L_REG_RX_PW_P5             (0x16)  /* Number of bytes in RX Payload Data Pipe 5 */
#define NRF24L_REG_FIFO_STATUS          (0x17)  /* FIFO Status Register */
#define NRF24L_REG_DYNPD                (0x1C)  /* Enable Dynamic Payload Length for Data Pipes */
#define NRF24L_REG_FEATURE              (0x1D)  /* Feature Register */

/*----------------------------------------------
Command Instructions
----------------------------------------------*/
#define NRF24L_CMD_REGISTER_MASK        (0x1F)  /* Masks off the largest available register address */
#define NRF24L_CMD_R_REGISTER           (0x00)  /* Read command and status registers  */
#define NRF24L_CMD_W_REGISTER           (0x20)  /* Write command and status registers  */
#define NRF24L_CMD_R_RX_PAYLOAD         (0x61)  /* Read RX Payload (1-32 bytes) */
#define NRF24L_CMD_W_TX_PAYLOAD         (0xA0)  /* Write TX Payload (1-32 bytes) */
#define NRF24L_CMD_FLUSH_TX             (0xE1)  /* Flush TX FIFO, used in TX Mode */
#define NRF24L_CMD_FLUSH_RX             (0xE2)  /* Flush RX FIFO, used in RX Mode */
#define NRF24L_CMD_REUSE_TX_PL          (0xE3)  /* Reuse last transmitted payload (PTX device only) */
#define NRF24L_CMD_ACTIVATE             (0x50)  /* This command, followed by 0x73, activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK */
#define NRF24L_CMD_R_RX_PL_WID          (0x60)  /* Read RX payload width for the top payload in the RX FIFO */
#define NRF24L_CMD_W_ACK_PAYLOAD        (0xA8)  /* Write Payload together with ACK packet */
#define NRF24L_CMD_W_TX_PAYLOAD_NO_ACK  (0xB0)  /* Disables AUTOACK on this specific packet */
#define NRF24L_CMD_NOP             (0xFF)  /* No operation */


#endif /* NRF24L01_DEFINITIONS_HPP */