#include "nrf24l01.hpp"

#include <cstring>
#include <algorithm>

#if !defined(_WIN32) && !defined(_WIN64) && !defined(MOD_TEST)
#define HW_TEST
#endif

namespace NRF24L
{
    using namespace Chimera;
    using namespace Chimera::GPIO;

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
        EN_RXADDR_P0,
        EN_RXADDR_P1,
        EN_RXADDR_P2,
        EN_RXADDR_P3,
        EN_RXADDR_P4,
        EN_RXADDR_P5
    };

    NRF24L01::NRF24L01(Chimera::SPI::SPIClass_sPtr spiInstance, Chimera::GPIO::GPIOClass_sPtr chipEnable)
    {
        this->spi = spiInstance;
        this->chipEnable = chipEnable;

        memset(spi_txbuff, 0, sizeof(spi_txbuff));
        memset(spi_rxbuff, 0, sizeof(spi_rxbuff));

        /*-------------------------------------------------
        Default address width is 5 bytes
        -------------------------------------------------*/
        addr_width = 5u;
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
        pVariant = setDataRate(DataRate::DR_250KBPS);
        setup = read_register(REG_RF_SETUP);

        /*-------------------------------------------------
        Set datarate to the slowest, most reliable speed supported by all hardware
        -------------------------------------------------*/
        setDataRate(DataRate::DR_1MBPS);

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
        chipEnable->write(State::LOW);

        delayMilliseconds(1);

        if (read_register(REG_FEATURE) & FEATURE_EN_ACK_PAY)
        {
            delayMilliseconds(1);
            flush_tx();
        }

        uint8_t cfg = read_register(REG_CONFIG) & ~CONFIG_PRIM_RX;
        write_register(REG_CONFIG, cfg);

        uint8_t en_rx = read_register(REG_EN_RXADDR) | child_pipe_enable[0];
        write_register(REG_EN_RXADDR, en_rx);
    }

    bool NRF24L01::available()
    {
        return available(nullptr);
    }

    bool NRF24L01::available(uint8_t *const pipe_num)
    {
        if(!(read_register(REG_FIFO_STATUS) & FIFO_STATUS_RX_EMPTY))
        {
            /*-------------------------------------------------
            Figure out which pipe has data available
            -------------------------------------------------*/
            if(pipe_num)
            {
                *pipe_num = (get_status() >> STATUS_RX_P_NO_Pos) & STATUS_RX_P_NO_Wid;
            }

            return true;
        }

        return false;
    }

    bool NRF24L01::rxFifoFull()
    {
        return read_register(REG_FIFO_STATUS) & FIFO_STATUS_RX_FULL;
    }

    void NRF24L01::powerDown()
    {
        /*-------------------------------------------------
        Make sure the enable pin is low to force standby mode
        -------------------------------------------------*/
        chipEnable->write(State::LOW);

        /*-------------------------------------------------
        Twiddle that power bit real good
        -------------------------------------------------*/
        uint8_t reg = read_register(REG_CONFIG) & ~CONFIG_PWR_UP;
        write_register(REG_CONFIG, reg);
    }

    void NRF24L01::powerUp()
    {
        /*-------------------------------------------------
        If not powered up already, do it. The worst startup delay is
        about 5mS, so just wait that amount.
        -------------------------------------------------*/
        if(!(read_register(REG_CONFIG) & CONFIG_PWR_UP))
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
        startFastWrite(buf, len, multicast);

        while (!(get_status() & (STATUS_TX_DS | STATUS_MAX_RT)))
        {
            delayMilliseconds(100);
        }

        chipEnable->write(State::LOW);

        uint8_t status_val = STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT;
        uint8_t status = write_register(REG_STATUS, status_val);

        if (status & STATUS_MAX_RT)
        {
            flush_tx();
            return false;
        }

        return true;
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

    void NRF24L01::startFastWrite(const void *const buf, size_t len, const bool multicast, bool startTx)
    {
        uint8_t payloadType = CMD_W_TX_PAYLOAD;

        if (multicast)
        {
            payloadType = CMD_W_TX_PAYLOAD_NO_ACK;
        }

        write_payload(buf, len, payloadType);

        if (startTx)
        {
            chipEnable->write(State::HIGH);
        }
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

    void NRF24L01::setAddressWidth(const uint8_t address_width)
    {
        switch(address_width)
        {
        case 3:
            write_register(REG_SETUP_AW, static_cast<uint8_t>(0x01));
            addr_width = address_width;
            break;

        case 4:
            write_register(REG_SETUP_AW, static_cast<uint8_t>(0x02));
            addr_width = address_width;
            break;

        case 5:
            write_register(REG_SETUP_AW, static_cast<uint8_t>(0x03));
            addr_width = address_width;
            break;

        default:
            write_register(REG_SETUP_AW, static_cast<uint8_t>(0x00));
            addr_width = 2u;
            break;
        }
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

    void NRF24L01::disableAckPayload()
    {

    }

    void NRF24L01::enableDynamicPayloads()
    {
        /*-------------------------------------------------
        Send the activate command to enable selection of features
        -------------------------------------------------*/
        write_register(CMD_ACTIVATE, static_cast<uint8_t>(0x73));

        /*-------------------------------------------------
        Enable the dynamic payload feature bit
        -------------------------------------------------*/
        uint8_t feature = read_register(REG_FEATURE) | FEATURE_EN_DPL;
        write_register(REG_FEATURE, feature);

        /*-------------------------------------------------
        Enable dynamic payload on all pipes. This requires that
        auto-acknowledge be enabled.
        -------------------------------------------------*/
        write_register(REG_EN_AA, EN_AA_Msk);
        write_register(REG_DYNPD, DYNPD_Msk);

        dynamic_payloads_enabled = true;
    }

    void NRF24L01::disableDynamicPayloads()
    {
        /*-------------------------------------------------
        Disable for all pipes
        -------------------------------------------------*/
        write_register(REG_DYNPD, ~DYNPD_Msk);
        write_register(REG_EN_AA, ~EN_AA_Msk);
        write_register(REG_FEATURE, ~FEATURE_EN_DPL);
    }

    void NRF24L01::enableDynamicAck()
    {
        uint8_t feature = read_register(REG_FEATURE) | FEATURE_EN_DYN_ACK;
        write_register(REG_FEATURE, feature);
    }

    void NRF24L01::disableDynamicAck()
    {
        uint8_t feature = read_register(REG_FEATURE) & ~FEATURE_EN_DYN_ACK;
        write_register(REG_FEATURE, feature);
    }

    bool NRF24L01::isPVariant()
    {
        return pVariant;
    }

    void NRF24L01::setAutoAck(bool enable)
    {

    }

    void NRF24L01::setAutoAck(uint8_t pipe, bool enable)
    {

    }

    void NRF24L01::setPALevel(const PowerAmplitude level)
    {
        uint8_t pwr = static_cast<uint8_t>(level);
        uint8_t setup = read_register(REG_RF_SETUP);

        if(pwr > 3)
        {
            /*-------------------------------------------------
            +1 to support the SI24R1 chip extra bit
            -------------------------------------------------*/
            setup |= (static_cast<uint8_t>(PowerAmplitude::MAX) << 1) + 1;
        }
        else
        {
            setup |= (pwr << 1) + 1;
        }

        write_register(REG_RF_SETUP, setup);
    }

    PowerAmplitude NRF24L01::getPALevel()
    {
        return static_cast<PowerAmplitude>((read_register(REG_RF_SETUP) & RF_SETUP_RF_PWR) >> 1);
    }

    bool NRF24L01::setDataRate(const DataRate speed)
    {
        uint8_t setup = read_register(REG_RF_SETUP);

        switch (speed)
        {
        case DataRate::DR_250KBPS:
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

        case DataRate::DR_1MBPS:
            setup &= ~(RF_SETUP_RF_DR_HIGH | RF_SETUP_RF_DR_LOW);
            break;

        case DataRate::DR_2MBPS:
            setup &= ~RF_SETUP_RF_DR_LOW;
            setup |= RF_SETUP_RF_DR_HIGH;
            break;

        default:
            break;
        }

        write_register(REG_RF_SETUP, setup);

        return (read_register(REG_RF_SETUP) == setup);
    }

    DataRate NRF24L01::getDataRate()
    {
        return static_cast<DataRate>(read_register(REG_RF_SETUP) & (RF_SETUP_RF_DR_HIGH | RF_SETUP_RF_DR_LOW));
    }

    void NRF24L01::setCRCLength(const CRCLength length)
    {
        uint8_t config = read_register(REG_CONFIG) & ~(CONFIG_CRCO | CONFIG_EN_CRC);

        switch(length)
        {
        case CRCLength::CRC_8:
            config |= CONFIG_EN_CRC;
            config &= ~CONFIG_CRCO;
            break;

        case CRCLength::CRC_16:
            config |= CONFIG_EN_CRC | CONFIG_CRCO;
            break;

        default:
            break;
        }

        write_register(REG_CONFIG, config);
    }

    CRCLength NRF24L01::getCRCLength()
    {
        CRCLength result = CRCLength::CRC_DISABLED;

        uint8_t config = read_register(REG_CONFIG) & (CONFIG_CRCO | CONFIG_EN_CRC);
        uint8_t AA = read_register(REG_EN_AA);

        if((config & CONFIG_EN_CRC) || AA)
        {
            if(config & CONFIG_CRCO)
            {
                result = CRCLength::CRC_16;
            }
            else
            {
                result = CRCLength::CRC_8;
            }
        }

        return result;
    }

    void NRF24L01::disableCRC()
    {
        uint8_t disable = read_register(REG_CONFIG) & ~CONFIG_EN_CRC;
        write_register(REG_CONFIG, disable);
    }

    void NRF24L01::maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready)
    {
        uint8_t config = read_register(REG_CONFIG);

        config &= ~(CONFIG_MASK_MAX_RT | CONFIG_MASK_TX_DS | CONFIG_MASK_RX_DR);
        config |= (tx_fail << CONFIG_MASK_MAX_RT_Pos) | (tx_ok << CONFIG_MASK_TX_DS_Pos) | (rx_ready << CONFIG_MASK_RX_DR_Pos);

        write_register(REG_CONFIG, config);
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
        return write_cmd(CMD_NOP);
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
