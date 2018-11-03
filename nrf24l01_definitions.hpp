#ifndef NRF24L01_DEFINITIONS_HPP
#define NRF24L01_DEFINITIONS_HPP

/*----------------------------------------------
Register Addresses
----------------------------------------------*/
#define NRF24L_REG_CONFIG       (0x00)          /* Configuration Register */
#define NRF24L_REG_EN_AA        (0x01)          /* Enable Auto Acknowledgment */
#define NRF24L_REG_EN_RXADDR    (0x02)          /* Enable RX Addresses */
#define NRF24L_REG_SETUP_AW     (0x03)          /* Setup of Address Width */
#define NRF24L_REG_SETUP_RETR   (0x04)          /* Setup of Automatic Retransmission */
#define NRF24L_REG_RF_CH        (0x05)          /* RF Channel Frequency Settings */
#define NRF24L_REG_RF_SETUP     (0x06)          /* RF Channel Settings Register */
#define NRF24L_REG_STATUS       (0x07)          /* Status Register */
#define NRF24L_REG_OBSERVE_TX   (0x08)          /* Transmit Observe */
#define NRF24L_REG_CD           (0x09)          /* Carrier Detect */
#define NRF24L_REG_RX_ADDR_P0   (0x0A)          /* Receive Address Data Pipe 0 */
#define NRF24L_REG_RX_ADDR_P1   (0x0B)          /* Receive Address Data Pipe 1 */
#define NRF24L_REG_RX_ADDR_P2   (0x0C)          /* Receive Address Data Pipe 2 */
#define NRF24L_REG_RX_ADDR_P3   (0x0D)          /* Receive Address Data Pipe 3 */
#define NRF24L_REG_RX_ADDR_P4   (0x0E)          /* Receive Address Data Pipe 4 */
#define NRF24L_REG_RX_ADDR_P5   (0x0F)          /* Receive Address Data Pipe 5 */
#define NRF24L_REG_TX_ADDR      (0x10)          /* Transmit Address */
#define NRF24L_REG_RX_PW_P0     (0x11)          /* Number of bytes in RX Payload Data Pipe 0 */
#define NRF24L_REG_RX_PW_P1     (0x12)          /* Number of bytes in RX Payload Data Pipe 1 */
#define NRF24L_REG_RX_PW_P2     (0x13)          /* Number of bytes in RX Payload Data Pipe 2 */
#define NRF24L_REG_RX_PW_P3     (0x14)          /* Number of bytes in RX Payload Data Pipe 3 */
#define NRF24L_REG_RX_PW_P4     (0x15)          /* Number of bytes in RX Payload Data Pipe 4 */
#define NRF24L_REG_RX_PW_P5     (0x16)          /* Number of bytes in RX Payload Data Pipe 5 */
#define NRF24L_REG_FIFO_STATUS  (0x17)          /* FIFO Status Register */
#define NRF24L_REG_DYNPD        (0x1C)          /* Enable Dynamic Payload Length for Data Pipes */
#define NRF24L_REG_FEATURE      (0x1D)          /* Feature Register */


#endif /* NRF24L01_DEFINITIONS_HPP */