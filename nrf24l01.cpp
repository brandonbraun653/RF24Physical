#include "nrf24l01.hpp"

#include <cstring>
#include <algorithm>

#if !defined(_WIN32) && !defined(_WIN64) && !defined(MOD_TEST)
#define HW_TEST
#endif

namespace NRF24L
{
    using namespace Chimera;

    static uint8_t spi_rxbuff[SPI_BUFFER_LEN];
    static_assert(sizeof(spi_rxbuff) == SPI_BUFFER_LEN, "SPI receive buffer is the wrong length");

    static uint8_t spi_txbuff[SPI_BUFFER_LEN];
    static_assert(sizeof(spi_txbuff) == SPI_BUFFER_LEN, "SPI transmit buffer is the wrong length");

    static const uint8_t child_pipe[] =
    {
        REG_RX_ADDR_P0,
        REG_RX_ADDR_P1,
        REG_RX_ADDR_P2,
        REG_RX_ADDR_P3,
        REG_RX_ADDR_P4,
        REG_RX_ADDR_P5
    };

    static const uint8_t child_payload_size[] =
    {
        REG_RX_PW_P0,
        REG_RX_PW_P1,
        REG_RX_PW_P2,
        REG_RX_PW_P3,
        REG_RX_PW_P4,
        REG_RX_PW_P5
    };

    static const uint8_t child_pipe_enable[] =
    {
        EN_RXADDR_P5,
        EN_RXADDR_P4,
        EN_RXADDR_P3,
        EN_RXADDR_P2,
        EN_RXADDR_P5,
        EN_RXADDR_P0
    };

    NRF24L01::NRF24L01(Chimera::SPI::SPIClass_sPtr spiInstance, Chimera::GPIO::GPIOClass_sPtr chipEnable)
    {
        this->spi = spiInstance;
        this->chipEnable = chipEnable;


    }

    bool NRF24L01::begin()
    {
        uint8_t setup = 0u;

        /*-------------------------------------------------
        Setup the MCU hardware to the correct state
        -------------------------------------------------*/
        spi->setChipSelectControlMode(SPI::ChipSelectMode::MANUAL);
        chipEnable->mode(GPIO::Drive::OUTPUT_PUSH_PULL);
        chipEnable->write(GPIO::State::LOW);

        spi->setChipSelect(GPIO::State::LOW);
        spi->setChipSelect(GPIO::State::HIGH);
        delayMilliseconds(10);

        /*-------------------------------------------------
        Reset config register and enable 16-bit CRC
        -------------------------------------------------*/
        write_register(REG_CONFIG, 0x0C);
        if (read_register(REG_CONFIG) != 0x0C)
        {
            return false;
        }

        /*-------------------------------------------------
        Set 1500uS timeout. Don't lower or the 250KBS mode will break.
        -------------------------------------------------*/
        setRetries(5, 15);

        /*-------------------------------------------------
        Check whether or not we have a P variant of the chip
        -------------------------------------------------*/
        pVariant = setDataRate(RF24_250KBPS);
        setup = read_register(REG_RF_SETUP);

        /*-------------------------------------------------
        Set datarate to the slowest, most reliable speed supported by all hardware
        -------------------------------------------------*/
        setDataRate(RF24_1MBPS);

        /*-------------------------------------------------
        Disable all the fancy features
        -------------------------------------------------*/
        write_register(REG_FEATURE, 0u);

        if (read_register(REG_FEATURE))
        {
            return false;
        }

        write_register(REG_DYNPD, 0u);

        if (read_register(REG_DYNPD))
        {
            return false;
        }

        dynamic_payloads_enabled = false;

        /*-------------------------------------------------
        Set the default channel to a value that likely won't congest the spectrum
        -------------------------------------------------*/
        setChannel(76);

        /*-------------------------------------------------
        Clear the buffers
        -------------------------------------------------*/
        flush_tx();
        flush_rx();

        /*-------------------------------------------------
        Power up the module and enable PTX. Stay in standby mode by not writing CE high
        -------------------------------------------------*/
        powerUp();

        auto cfg = read_register(REG_CONFIG);
        cfg &= ~CONFIG_PRIM_RX;
        write_register(REG_CONFIG, cfg);

        if (read_register(REG_CONFIG) != cfg)
        {
            return false;
        }

        return (setup != 0 && setup != 0xFF);
    }

    void NRF24L01::startListening()
    {

    }

    void NRF24L01::stopListening()
    {

    }

    bool NRF24L01::available()
    {
        return false;
    }

    bool NRF24L01::available(uint8_t *pipe_num)
    {
        return false;
    }

    bool NRF24L01::rxFifoFull()
    {
        return false;
    }

    void NRF24L01::powerDown()
    {

    }

    void NRF24L01::powerUp()
    {
        uint8_t cfg = read_register(REG_CONFIG);

        if (!(cfg & CONFIG_PWR_UP))
        {
            write_register(REG_CONFIG, CONFIG_PWR_UP);
            delayMilliseconds(5);
        }
    }

    bool NRF24L01::write(const void *buffer, uint8_t len)
    {
        return write(buffer, len, false);
    }

    bool NRF24L01::write(const void *buf, uint8_t len, const bool multicast)
    {
        return false;
    }

    bool NRF24L01::writeFast(const void *buf, uint8_t len)
    {
        return false;
    }

    bool NRF24L01::writeFast(const void *buf, uint8_t len, const bool multicast)
    {
        return false;
    }

    bool NRF24L01::writeBlocking(const void *buf, uint8_t len, uint32_t timeout)
    {
        return false;
    }

    bool NRF24L01::txStandBy()
    {
        return false;
    }

    bool NRF24L01::txStandBy(uint32_t timeout, bool startTx)
    {
        return false;
    }

    void NRF24L01::writeAckPayload(uint8_t pipe, const void *buf, uint8_t len)
    {

    }

    bool NRF24L01::isAckPayloadAvailable()
    {
        return false;
    }

    void NRF24L01::whatHappened(bool &tx_ok, bool &tx_fail, bool &rx_ready)
    {

    }

    void NRF24L01::startFastWrite(const void *buf, uint8_t len, const bool multicast, bool startTx)
    {

    }

    void NRF24L01::startWrite(const void *buf, uint8_t len, const bool multicast)
    {

    }

    void NRF24L01::reUseTX()
    {

    }

    uint8_t NRF24L01::flush_tx()
    {
        return write_cmd(CMD_FLUSH_TX);
    }

    uint8_t NRF24L01::flush_rx()
    {
        return write_cmd(CMD_FLUSH_RX);
    }

    bool NRF24L01::testCarrier()
    {
        return false;
    }

    bool NRF24L01::testRPD()
    {
        return false;
    }

    bool NRF24L01::isValid()
    {
        return false;
    }

    void NRF24L01::closeReadingPipe(uint8_t pipe)
    {

    }

    void NRF24L01::setAddressWidth(uint8_t address_width)
    {

    }

    void NRF24L01::setRetries(const uint8_t delay, const uint8_t count)
    {
        uint8_t ard = (delay & 0x0F) << SETUP_RETR_ARD_Pos;
        uint8_t arc = (count & 0x0F) << SETUP_RETR_ARC_Pos;
        uint8_t setup_retr = ard | arc;

        write_register(REG_SETUP_RETR, setup_retr);
    }

    void NRF24L01::setChannel(uint8_t channel)
    {
        uint8_t ch = channel & RF_CH_Msk;
        write_register(REG_RF_CH, ch);
    }

    uint8_t NRF24L01::getChannel()
    {
        return 0;
    }

    void NRF24L01::setPayloadSize(uint8_t size)
    {

    }

    uint8_t NRF24L01::getPayloadSize()
    {
        return 0;
    }

    uint8_t NRF24L01::getDynamicPayloadSize()
    {
        return 0;
    }

    void NRF24L01::enableAckPayload()
    {

    }

    void NRF24L01::enableDynamicPayloads()
    {

    }

    void NRF24L01::disableDynamicPayloads()
    {

    }

    void NRF24L01::enableDynamicAck()
    {

    }

    bool NRF24L01::isPVariant()
    {
        return false;
    }

    void NRF24L01::setAutoAck(bool enable)
    {

    }

    void NRF24L01::setAutoAck(uint8_t pipe, bool enable)
    {

    }

    void NRF24L01::setPALevel(uint8_t level)
    {

    }

    uint8_t NRF24L01::getPALevel()
    {
        return 0;
    }

    bool NRF24L01::setDataRate(const nrf24_datarate speed)
    {
        uint8_t setup = read_register(REG_RF_SETUP);

        switch (speed)
        {
        case RF24_250KBPS:
            if (pVariant)
            {
                setup |= RF_SETUP_RF_DR_LOW;
                setup &= ~RF_SETUP_RF_DR_HIGH;
            }
            else
            {
                return false;
            }
            break;

        case RF24_1MBPS:
            setup &= ~(RF_SETUP_RF_DR_HIGH | RF_SETUP_RF_DR_LOW);
            break;

        case RF24_2MBPS:
            setup &= ~RF_SETUP_RF_DR_LOW;
            setup |= RF_SETUP_RF_DR_HIGH;
            break;

        default:
            break;
        }

        write_register(REG_RF_SETUP, setup);

        return (read_register(REG_RF_SETUP) == setup);
    }

    nrf24_datarate NRF24L01::getDataRate()
    {
        return RF24_2MBPS;
    }

    void NRF24L01::setCRCLength(nrf24_crclength length)
    {

    }

    nrf24_crclength NRF24L01::getCRCLength()
    {
        return RF24_CRC_16;
    }

    void NRF24L01::disableCRC()
    {

    }

    void NRF24L01::maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready)
    {

    }

    uint8_t NRF24L01::read_register(const uint8_t &reg, uint8_t *const buf, size_t & len)
    {
        if(len > PAYLOAD_LEN)
        {
            len = PAYLOAD_LEN;
        }

        spi_txbuff[0] = (CMD_R_REGISTER | (CMD_REGISTER_MASK & reg));
        memset(&spi_txbuff[1], CMD_NOP, len);

        begin_transaction();
        spi_write_read(spi_txbuff, spi_rxbuff, len);
        end_transaction();

        /* Return only the status code of the chip. The register values will be in the rx buff */
        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::read_register(const uint8_t &reg)
    {
        size_t txLength = 2;
        spi_txbuff[0] = (CMD_R_REGISTER | (CMD_REGISTER_MASK & reg));
        spi_txbuff[1] = CMD_NOP;

        begin_transaction();
        spi_write_read(spi_txbuff, spi_rxbuff, txLength);
        end_transaction();

        /* Current register value is in the second byte of the receive buffer */
        return spi_rxbuff[1];
    }

    uint8_t NRF24L01::write_register(const uint8_t &reg, const uint8_t *const buf, size_t &len)
    {
        if(len > PAYLOAD_LEN)
        {
            len = PAYLOAD_LEN;
        }

        spi_txbuff[0] = (CMD_W_REGISTER | (CMD_REGISTER_MASK & reg));
        memcpy(&spi_txbuff[1], buf, len);

        len += 1;
        begin_transaction();
        spi_write_read(spi_txbuff, spi_rxbuff, len);
        end_transaction();

        /* Status code is in the first byte of the receive buffer */
        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::write_register(const uint8_t &reg, const uint8_t &value)
    {
        size_t txLength = 2;
        spi_txbuff[0] = (CMD_W_REGISTER | (CMD_REGISTER_MASK & reg));
        spi_txbuff[1] = value;

        begin_transaction();
        spi_write_read(spi_txbuff, spi_rxbuff, txLength);
        end_transaction();

        /* Status code is in the first byte of the receive buffer */
        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::write_payload(const void *const buf, size_t & len, const uint8_t &writeType)
    {
        const uint8_t * tx_buf = reinterpret_cast<const uint8_t *>(buf);

        /*-------------------------------------------------
        Calculate the number of bytes that do nothing
        -------------------------------------------------*/
        len = std::min(len, payload_size);
        uint8_t blank_len = static_cast<uint8_t>(dynamic_payloads_enabled ? 0 : (payload_size - len));
        size_t size = len + blank_len + 1;

        /*-------------------------------------------------
        Format the write command and fill the rest with NOPs
        -------------------------------------------------*/
        spi_txbuff[0] = writeType;                  /* Write command type*/
        memcpy(&spi_txbuff[1], tx_buf, len);        /* Payload information */
        memset(&spi_txbuff[len], 0, blank_len);     /* Null out the remaining buffer space*/

        begin_transaction();
        spi_write_read(spi_txbuff, spi_rxbuff, size);
        end_transaction();

        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::read_payload(void *const buf, size_t &len)
    {
        if(len > payload_size)
        {
            len = payload_size;
        }

        /*-------------------------------------------------
        Calculate the number of bytes that do nothing
        -------------------------------------------------*/
        uint8_t * rx_buf = reinterpret_cast<uint8_t *>(buf);
        uint8_t blank_len = static_cast<uint8_t>(dynamic_payloads_enabled ? 0 : (payload_size - len));
        size_t size = len + blank_len + 1;

        /*-------------------------------------------------
        Format the read command and fill the rest with NOPs
        -------------------------------------------------*/
        spi_txbuff[0] = CMD_R_RX_PAYLOAD;
        memset(&spi_txbuff[1], CMD_NOP, (size - 1));

        begin_transaction();
        spi_write_read(spi_txbuff, spi_rxbuff, size);
        end_transaction();

        /*-------------------------------------------------
        The status byte is first, RX payload is all remaining
        -------------------------------------------------*/
        memcpy(rx_buf, spi_rxbuff, (size - 1));
        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::get_status()
    {
        return 0;
    }

    size_t NRF24L01::spi_write(const uint8_t *const tx_buffer, size_t &len)
    {
        spi->writeBytes(tx_buffer, len);
        return len;
    }

    size_t NRF24L01::spi_read(uint8_t *const rx_buffer, size_t &len)
    {
        spi->readBytes(rx_buffer, len);
        return len;
    }

    size_t NRF24L01::spi_write_read(const uint8_t *const tx_buffer, uint8_t *const rx_buffer, size_t &len)
    {
        spi->readWriteBytes(tx_buffer, rx_buffer, len);
        return len;
    }

    void NRF24L01::begin_transaction()
    {
        spi->setChipSelect(GPIO::State::LOW);
    }

    void NRF24L01::end_transaction()
    {
        spi->setChipSelect(GPIO::State::HIGH);
    }

    void NRF24L01::openWritePipe(const uint8_t *const address)
    {
        write_register(REG_RX_ADDR_P0, address, addr_width);
        write_register(REG_TX_ADDR, address, addr_width);
        write_register(REG_RX_PW_P0, static_cast<uint8_t>(payload_size));
    }

    void NRF24L01::openReadPipe(const uint8_t &number, const uint8_t *const address)
    {
        size_t width = addr_width;

        if(number == 0)
        {
            memcpy(pipe0_reading_address, address, width);
        }

        if(number <= 6)
        {
            if(number < 2)
            {
                write_register(child_pipe[number], address, width);
            }
            else
            {
                width = 1;
                write_register(child_pipe[number], address, width);
            }

            write_register(child_payload_size[number], static_cast<uint8_t>(payload_size));

            uint8_t reg_en_rxaddr = read_register(REG_EN_RXADDR);

            write_register(REG_EN_RXADDR, (reg_en_rxaddr | child_pipe_enable[number]));
        }
    }

    bool NRF24L01::isConnected()
    {
        uint8_t setup = read_register(REG_SETUP_AW);

        return ((setup >= 1) && (setup <= 3));
    }

    bool NRF24L01::read(void *const buffer, uint8_t &len)
    {
        return false;
    }

    uint8_t NRF24L01::write_cmd(const uint8_t cmd)
    {
        size_t txLength = 1;
        spi_txbuff[0] = cmd;

        begin_transaction();
        spi_write_read(spi_txbuff, spi_rxbuff, txLength);
        end_transaction();

        /* Give back the status register value */
        return spi_rxbuff[0];
    }
};
