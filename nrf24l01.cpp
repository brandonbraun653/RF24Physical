/* Module Include */
#include "nrf24l01.hpp"

/* Local C/C++ Includes */
#include <cstring>
#include <algorithm>

#if !defined(_WIN32) && !defined(_WIN64) && !defined(MOD_TEST)
#define HW_TEST
#endif

namespace NRF24L
{
    using namespace Chimera;
    using namespace Chimera::GPIO;

    static constexpr uint8_t numPipes = 6;

    static const uint8_t pipeRXAddressReg[numPipes] =
    {
        Register::RX_ADDR_P0,
        Register::RX_ADDR_P1,
        Register::RX_ADDR_P2,
        Register::RX_ADDR_P3,
        Register::RX_ADDR_P4,
        Register::RX_ADDR_P5
    };
    static_assert(sizeof(pipeRXAddressReg) == (numPipes * sizeof(numPipes)), "Too many/few items in the array!");

    static const uint8_t pipeEnableRXAddressReg[numPipes] =
    {
        EN_RXADDR::P0,
        EN_RXADDR::P1,
        EN_RXADDR::P2,
        EN_RXADDR::P3,
        EN_RXADDR::P4,
        EN_RXADDR::P5
    };
    static_assert(sizeof(pipeEnableRXAddressReg) == (numPipes * sizeof(numPipes)), "Too many/few items in the array!");

    static const uint8_t pipeRXPayloadWidthReg[numPipes] =
    {
        Register::RX_PW_P0,
        Register::RX_PW_P1,
        Register::RX_PW_P2,
        Register::RX_PW_P3,
        Register::RX_PW_P4,
        Register::RX_PW_P5
    };
    static_assert(sizeof(pipeRXPayloadWidthReg) == (numPipes * sizeof(numPipes)), "Too many/few items in the array!");

    NRF24L01::NRF24L01(Chimera::SPI::SPIClass_sPtr spiInstance, Chimera::GPIO::GPIOClass_sPtr chipEnable)
    {
        this->spi = spiInstance;
        this->chipEnable = chipEnable;

        memset(spi_txbuff.begin(), 0, spi_txbuff.size());
        memset(spi_rxbuff.begin(), 0, spi_rxbuff.size());

        /*-------------------------------------------------
        Initialize class variables
        -------------------------------------------------*/
        addr_width = MAX_ADDR_WID;
        payload_size = PAYLOAD_LEN;
        dynamic_payloads_enabled = false;
        pVariant = false;
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

        /*-------------------------------------------------
        Delay some time to allow the chip to initialize
        -------------------------------------------------*/
        delayMilliseconds(100);

        /*-------------------------------------------------
        Reset config register and enable 16-bit CRC
        -------------------------------------------------*/
        writeRegister(Register::CONFIG, 0x0C);
        if (readRegister(Register::CONFIG) != 0x0C)
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
        setup = readRegister(Register::RF_SETUP);

        /*-------------------------------------------------
        Set datarate to the slowest, most reliable speed supported by all hardware
        -------------------------------------------------*/
        setDataRate(DataRate::DR_1MBPS);

        /*-------------------------------------------------
        Disable all the fancy features
        -------------------------------------------------*/
        writeRegister(Register::FEATURE, 0u);

        if (readRegister(Register::FEATURE))
        {
            return false;
        }

        writeRegister(Register::DYNPD, 0u);

        if (readRegister(Register::DYNPD))
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

        auto cfg = readRegister(Register::CONFIG);
        cfg &= ~CONFIG::PRIM_RX;
        writeRegister(Register::CONFIG, cfg);

        if (readRegister(Register::CONFIG) != cfg)
        {
            return false;
        }

        return (setup != 0 && setup != 0xFF);
    }

    void NRF24L01::startListening()
    {
        uint8_t cfgVal = readRegister(Register::CONFIG) | CONFIG::PRIM_RX;
        writeRegister(Register::CONFIG, cfgVal);

        uint8_t statusVal = STATUS::RX_DR | STATUS::TX_DS | STATUS::MAX_RT;
        writeRegister(Register::STATUS, statusVal);

        chipEnable->write(State::HIGH);

        // Restore the pipe0 adddress, if exists
        if(pipe0_reading_address[0] > 0)
        {
            writeRegister(Register::RX_ADDR_P0, pipe0_reading_address, addr_width);
        }
        else
        {
            closeReadingPipe(0);
        }

        // Flush buffers
        if(readRegister(Register::FEATURE) & FEATURE::EN_ACK_PAY)
        {
            flush_tx();
        }
    }

    void NRF24L01::stopListening()
    {
        /*-------------------------------------------------
        Set the chip into standby mode I
        -------------------------------------------------*/
        chipEnable->write(State::LOW);
        delayMilliseconds(1);

        /*-------------------------------------------------
        EN_ACK_PAY sets a dynamic payload length. The buffer needs to be flushed so
        we don't automatically transmit data the next time we write chipEnable high.
        -------------------------------------------------*/
        if (readRegister(Register::FEATURE) & FEATURE::EN_ACK_PAY)
        {
            delayMilliseconds(1);
            flush_tx();
        }

        /*-------------------------------------------------
        Disable RX mode
        -------------------------------------------------*/
        uint8_t cfg = readRegister(Register::CONFIG) & ~CONFIG::PRIM_RX;
        writeRegister(Register::CONFIG, cfg);

        /*-------------------------------------------------
        Ensure RX Pipe 0 is ready for listening
        -------------------------------------------------*/
        uint8_t en_rx = readRegister(Register::EN_RXADDR) | pipeEnableRXAddressReg[0];
        writeRegister(Register::EN_RXADDR, en_rx);
    }

    bool NRF24L01::available()
    {
        uint8_t dummyVar;
        return available(dummyVar);
    }

    bool NRF24L01::available(uint8_t &pipe_num)
    {
        if(!(readRegister(Register::FIFO_STATUS) & FIFO_STATUS::RX_EMPTY))
        {
            /*-------------------------------------------------
            Figure out which pipe has data available
            -------------------------------------------------*/
            if(pipe_num)
            {
                pipe_num = (get_status() >> STATUS::RX_P_NO_Pos) & STATUS::RX_P_NO_Wid;
            }

            return true;
        }

        return false;
    }

    void NRF24L01::read(uint8_t *const buffer, size_t len)
    {
        readPayload(buffer, len);

        uint8_t statusVal = STATUS::RX_DR | STATUS::MAX_RT | STATUS::TX_DS;
        writeRegister(Register::STATUS, statusVal);
    }

    bool NRF24L01::write(const uint8_t *const buffer, size_t len)
    {
        return write(buffer, len, false);
    }

    bool NRF24L01::write(const uint8_t *const buffer, size_t len, const bool multicast)
    {
        bool result = false;

        /*-------------------------------------------------
        Immediately dump data into the TX FIFO. Assuming we are in
        the correct mode, this should start the transfer.
        -------------------------------------------------*/
        startFastWrite(buffer, len, multicast);

        /*-------------------------------------------------
        Wait for the Data Sent or Max Retry interrupt to occur
        -------------------------------------------------*/
        while (!registerAnySet(Register::STATUS, (STATUS::TX_DS | STATUS::MAX_RT)))
        {
            delayMilliseconds(10);
        }

        /*-------------------------------------------------
        Go back to standby mode
        -------------------------------------------------*/
        chipEnable->write(State::LOW);

        /*-------------------------------------------------
        If we hit the Max Retries, we have a problem and the whole TX FIFO is screwed
        -------------------------------------------------*/
        if (registerBitmaskSet(Register::STATUS, STATUS::MAX_RT))
        {
            flush_tx();
        }
        else
        {
            result = true;
        }

        /*-------------------------------------------------
        Clear all the interrupt flags
        -------------------------------------------------*/
        setRegisterBits(Register::STATUS, (STATUS::RX_DR | STATUS::TX_DS | STATUS::MAX_RT));

        return result;
    }

    void NRF24L01::openWritePipe(const uint8_t *const address)
    {
        /*-------------------------------------------------
        Set the receive address for pipe 0, this one has a maximum of 5 byte width
        -------------------------------------------------*/
        writeRegister(Register::RX_ADDR_P0, address, addr_width);

        /*-------------------------------------------------
        Make sure we transmit back to the same address we expect receive from
        -------------------------------------------------*/
        writeRegister(Register::TX_ADDR, address, addr_width);

        /*-------------------------------------------------
        Set a static payload length for all receptions on pipe 0. There must also be
        an equal number of bytes clocked into the TX_FIFO when data is transmitted out.
        -------------------------------------------------*/
        writeRegister(Register::RX_PW_P0, static_cast<uint8_t>(payload_size));
    }

    void NRF24L01::openReadPipe(const uint8_t number, const uint8_t *const address)
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
                writeRegister(pipeRXAddressReg[number], address, width);
            }
            else
            {
                width = 1;
                writeRegister(pipeRXAddressReg[number], address, width);
            }

            writeRegister(pipeRXPayloadWidthReg[number], static_cast<uint8_t>(payload_size));

            uint8_t reg_en_rxaddr = readRegister(Register::EN_RXADDR);

            writeRegister(Register::EN_RXADDR, (reg_en_rxaddr | pipeEnableRXAddressReg[number]));
        }
    }

    bool NRF24L01::isConnected()
    {
        uint8_t setup = readRegister(Register::SETUP_AW);

        return ((setup >= 1) && (setup <= 3));
    }

    bool NRF24L01::rxFifoFull()
    {
        return readRegister(Register::FIFO_STATUS) & FIFO_STATUS::RX_FULL;
    }

    void NRF24L01::powerUp()
    {
        /*-------------------------------------------------
        If not powered up already, do it. The worst startup delay is
        about 5mS, so just wait that amount.
        -------------------------------------------------*/
        if(!registerBitmaskSet(Register::CONFIG, CONFIG::PWR_UP))
        {
            writeRegister(Register::CONFIG, CONFIG::PWR_UP);
            delayMilliseconds(5);
        }
    }

    void NRF24L01::powerDown()
    {
        /*-------------------------------------------------
        Force standby mode and power down the chip
        -------------------------------------------------*/
        chipEnable->write(State::LOW);
        clearRegisterBits(Register::CONFIG, CONFIG::PWR_UP);
    }

    bool NRF24L01::writeFast(const uint8_t *const buffer, size_t len)
    {
        return writeFast(buffer, len, false);
    }

    bool NRF24L01::writeFast(const uint8_t *const buffer, size_t len, const bool multicast)
    {
        /*-------------------------------------------------
        Exit cleanly if we errored out previously
        -------------------------------------------------*/
        uint8_t status = get_status();

        do
	    {
            if(status & STATUS::MAX_RT)
            {
                clearRegisterBits(Register::STATUS, STATUS::MAX_RT);
                return false;
            }
            else
            {
                status = get_status();
            }
	    } while (status & STATUS::TX_FULL);

        /*-------------------------------------------------
        Write data and return without checking the status of the transfer.
        -------------------------------------------------*/
        startFastWrite(buffer, len, multicast);
        return true;
    }

    bool NRF24L01::writeBlocking(const uint8_t *const buffer, size_t len, uint32_t timeout)
    {
        return false;
    }

    void NRF24L01::startFastWrite(const uint8_t *const buffer, size_t len, const bool multicast, bool startTx)
    {
        /*-------------------------------------------------
        Decide if we want to transmit with ACK or with NOACK
        -------------------------------------------------*/
        uint8_t payloadType = Command::W_TX_PAYLOAD;

        if (multicast)
        {
            payloadType = Command::W_TX_PAYLOAD_NO_ACK;
        }

        /*-------------------------------------------------
        Write the payload to the TX FIFO and optionally start the transfer
        -------------------------------------------------*/
        writePayload(buffer, len, payloadType);

        if (startTx)
        {
            chipEnable->write(State::HIGH);
        }
    }

    void NRF24L01::startWrite(const uint8_t *const buffer, size_t len, const bool multicast)
    {
        /*-------------------------------------------------
        Decide if we want to transmit with ACK or with NOACK
        -------------------------------------------------*/
        uint8_t payloadType = Command::W_TX_PAYLOAD;

        if (multicast)
        {
            payloadType = Command::W_TX_PAYLOAD_NO_ACK;
        }

        /*-------------------------------------------------
        Write the payload to the TX FIFO and start the transfer
        -------------------------------------------------*/
        writePayload(buffer, len, payloadType);

        chipEnable->write(State::HIGH);
        chipEnable->write(State::LOW);
    }

    bool NRF24L01::txStandBy()
    {
        /*-------------------------------------------------
        Wait for the TX FIFO to signal it's empty
        -------------------------------------------------*/
        while (!registerBitmaskSet(Register::FIFO_STATUS, FIFO_STATUS::TX_EMPTY))
        {
            /*-------------------------------------------------
            If we hit the Max Retries, we have a problem and the whole TX FIFO is screwed.
            Go back to standby mode and clear out the FIFO.
            -------------------------------------------------*/
            if (registerBitmaskSet(Register::STATUS, STATUS::MAX_RT))
            {
                setRegisterBits(Register::STATUS, STATUS::MAX_RT);
                chipEnable->write(State::LOW);
                flush_tx();
                return false;
            }
        }

        chipEnable->write(State::LOW);
        return true;
    }

    bool NRF24L01::txStandBy(const uint32_t timeout, const bool startTx)
    {
        return false;
    }

    void NRF24L01::writeAckPayload(const uint8_t pipe, const uint8_t *const buffer, size_t len)
    {
        //TODO: Magic numbers abound in this function. Get rid of them.
        size_t size = std::min(len, static_cast<size_t>(32)) + 1u;

        spi_txbuff[0] = Command::W_ACK_PAYLOAD | (pipe & 0x07);
        memcpy(&spi_txbuff[1], buffer, size);

        begin_transaction();
        spi->writeBytes(spi_txbuff.begin(), size);
        end_transaction();
    }

    bool NRF24L01::isAckPayloadAvailable()
    {
        return !(readRegister(Register::FIFO_STATUS) & FIFO_STATUS::RX_EMPTY);
    }

    void NRF24L01::whatHappened(bool &tx_ok, bool &tx_fail, bool &rx_ready)
    {
        uint8_t statusActual = readRegister(Register::STATUS);
        uint8_t statusCleared = statusActual | STATUS::RX_DR | STATUS::TX_DS | STATUS::MAX_RT;
        writeRegister(Register::STATUS, statusCleared);

        tx_ok = statusActual & STATUS::TX_DS;
        tx_fail = statusActual & STATUS::MAX_RT;
        rx_ready = statusActual & STATUS::RX_DR;
    }

    void NRF24L01::reUseTX()
    {
        writeRegister(Register::STATUS, STATUS::MAX_RT);
        write_cmd(Command::REUSE_TX_PL);
        chipEnable->write(State::LOW);
        chipEnable->write(State::HIGH);
    }

    bool NRF24L01::testCarrier()
    {
        return (readRegister(Register::CD) & 0x01);
    }

    void NRF24L01::closeReadingPipe(const uint8_t pipe)
    {
        if (pipe < numPipes)
        {
            uint8_t rxaddrVal = readRegister(Register::EN_RXADDR) & ~pipeEnableRXAddressReg[pipe];
            writeRegister(Register::EN_RXADDR, rxaddrVal);
        }
    }

    void NRF24L01::setAddressWidth(const uint8_t address_width)
    {
        switch(address_width)
        {
        case 3:
            writeRegister(Register::SETUP_AW, static_cast<uint8_t>(0x01));
            addr_width = address_width;
            break;

        case 4:
            writeRegister(Register::SETUP_AW, static_cast<uint8_t>(0x02));
            addr_width = address_width;
            break;

        case 5:
            writeRegister(Register::SETUP_AW, static_cast<uint8_t>(0x03));
            addr_width = address_width;
            break;

        default:
            writeRegister(Register::SETUP_AW, static_cast<uint8_t>(0x00));
            addr_width = 2u;
            break;
        }
    }

    void NRF24L01::setRetries(const uint8_t delay, const uint8_t count)
    {
        uint8_t ard = (delay & 0x0F) << SETUP_RETR::ARD_Pos;
        uint8_t arc = (count & 0x0F) << SETUP_RETR::ARC_Pos;
        uint8_t setup_retr = ard | arc;

        writeRegister(Register::SETUP_RETR, setup_retr);
    }

    void NRF24L01::setChannel(const uint8_t channel)
    {
        uint8_t ch = channel & RF_CH::Mask;
        writeRegister(Register::RF_CH, ch);
    }

    void NRF24L01::setPayloadSize(const uint8_t size)
    {
        payload_size = std::min(size, static_cast<uint8_t>(32));
    }

    uint8_t NRF24L01::getChannel()
    {
        return readRegister(Register::RF_CH);
    }

    uint8_t NRF24L01::getPayloadSize()
    {
        return payload_size;
    }

    uint8_t NRF24L01::getDynamicPayloadSize()
    {
        uint8_t result = 0u;

        spi_txbuff[0] = Command::R_RX_PL_WID;
        spi_rxbuff[1] = Command::NOP;

        begin_transaction();
        spi->readWriteBytes(spi_txbuff.begin(), spi_rxbuff.begin(), 2, false);
        end_transaction();

        result = spi_rxbuff[1];

        if (result > 32)
        {
            flush_rx();
            delayMilliseconds(2);
            return 0;
        }

        return result;
    }

    uint8_t NRF24L01::flush_tx()
    {
        return write_cmd(Command::FLUSH_TX);
    }

    uint8_t NRF24L01::flush_rx()
    {
        return write_cmd(Command::FLUSH_RX);
    }

    void NRF24L01::enableAckPayload()
    {
        uint8_t featureVal = readRegister(Register::FEATURE) | FEATURE::EN_ACK_PAY | FEATURE::EN_DPL;
        writeRegister(Register::FEATURE, featureVal);

        uint8_t dynpdVal = readRegister(Register::DYNPD) | DYNPD::DPL_P0 | DYNPD::DPL_P1;
        writeRegister(Register::DYNPD, dynpdVal);

        dynamic_payloads_enabled = true;
    }

    void NRF24L01::enableDynamicPayloads()
    {
        /*-------------------------------------------------
        Send the activate command to enable selection of features
        -------------------------------------------------*/
        writeRegister(Command::ACTIVATE, static_cast<uint8_t>(0x73));

        /*-------------------------------------------------
        Enable the dynamic payload feature bit
        -------------------------------------------------*/
        uint8_t feature = readRegister(Register::FEATURE) | FEATURE::EN_DPL;
        writeRegister(Register::FEATURE, feature);

        /*-------------------------------------------------
        Enable dynamic payload on all pipes. This requires that
        auto-acknowledge be enabled.
        -------------------------------------------------*/
        writeRegister(Register::EN_AA, EN_AA::Mask);
        writeRegister(Register::DYNPD, DYNPD::Mask);

        dynamic_payloads_enabled = true;
    }

    void NRF24L01::disableDynamicPayloads()
    {
        /*-------------------------------------------------
        Disable for all pipes
        -------------------------------------------------*/
        writeRegister(Register::DYNPD, ~DYNPD::Mask);
        writeRegister(Register::EN_AA, ~EN_AA::Mask);
        writeRegister(Register::FEATURE, ~FEATURE::EN_DPL);
    }

    void NRF24L01::enableDynamicAck()
    {
        uint8_t feature = readRegister(Register::FEATURE) | FEATURE::EN_DYN_ACK;
        writeRegister(Register::FEATURE, feature);
    }

    void NRF24L01::disableDynamicAck()
    {
        uint8_t feature = readRegister(Register::FEATURE) & ~FEATURE::EN_DYN_ACK;
        writeRegister(Register::FEATURE, feature);
    }

    bool NRF24L01::isPVariant()
    {
        return pVariant;
    }

    void NRF24L01::setAutoAck(const bool enable)
    {
        if (enable)
        {
            writeRegister(Register::EN_AA, EN_AA::Mask);
        }
        else
        {
            writeRegister(Register::EN_AA, 0u);
        }
    }

    void NRF24L01::setAutoAck(const uint8_t pipe, const bool enable)
    {
        if (pipe <= 6)
        {
            uint8_t en_aa = readRegister(Register::EN_AA);

            if (enable)
            {
                en_aa |= 1u << pipe;
            }
            else
            {
                en_aa &= ~(1u << pipe);
            }

            writeRegister(Register::EN_AA, en_aa);
        }
    }

    void NRF24L01::setPALevel(const PowerAmplitude level)
    {
        /*-------------------------------------------------
        Merge bits from level into setup according to a mask
        https://graphics.stanford.edu/~seander/bithacks.html#MaskedMerge
        -------------------------------------------------*/
        uint8_t setup = readRegister(Register::RF_SETUP);
        setup ^= (setup ^ static_cast<uint8_t>(level)) & RF_SETUP::RF_PWR_Msk;

        writeRegister(Register::RF_SETUP, setup);
    }

    PowerAmplitude NRF24L01::getPALevel()
    {
        return static_cast<PowerAmplitude>((readRegister(Register::RF_SETUP) & RF_SETUP::RF_PWR) >> 1);
    }

    bool NRF24L01::setDataRate(const DataRate speed)
    {
        uint8_t setup = readRegister(Register::RF_SETUP);

        switch (speed)
        {
        case DataRate::DR_250KBPS:
            if (pVariant)
            {
                setup |= RF_SETUP::RF_DR_LOW;
                setup &= ~RF_SETUP::RF_DR_HIGH;
            }
            else
            {
                return false;
            }
            break;

        case DataRate::DR_1MBPS:
            setup &= ~(RF_SETUP::RF_DR_HIGH | RF_SETUP::RF_DR_LOW);
            break;

        case DataRate::DR_2MBPS:
            setup &= ~RF_SETUP::RF_DR_LOW;
            setup |= RF_SETUP::RF_DR_HIGH;
            break;

        default:
            break;
        }

        writeRegister(Register::RF_SETUP, setup);

        return (readRegister(Register::RF_SETUP) == setup);
    }

    DataRate NRF24L01::getDataRate()
    {
        return static_cast<DataRate>(readRegister(Register::RF_SETUP) & (RF_SETUP::RF_DR_HIGH | RF_SETUP::RF_DR_LOW));
    }

    void NRF24L01::setCRCLength(const CRCLength length)
    {
        uint8_t config = readRegister(Register::CONFIG) & ~(CONFIG::CRCO | CONFIG::EN_CRC);

        switch(length)
        {
        case CRCLength::CRC_8:
            config |= CONFIG::EN_CRC;
            config &= ~CONFIG::CRCO;
            break;

        case CRCLength::CRC_16:
            config |= CONFIG::EN_CRC | CONFIG::CRCO;
            break;

        default:
            break;
        }

        writeRegister(Register::CONFIG, config);
    }

    CRCLength NRF24L01::getCRCLength()
    {
        CRCLength result = CRCLength::CRC_DISABLED;

        uint8_t config = readRegister(Register::CONFIG) & (CONFIG::CRCO | CONFIG::EN_CRC);
        uint8_t AA = readRegister(Register::EN_AA);

        if((config & CONFIG::EN_CRC) || AA)
        {
            if(config & CONFIG::CRCO)
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
        uint8_t disable = readRegister(Register::CONFIG) & ~CONFIG::EN_CRC;
        writeRegister(Register::CONFIG, disable);
    }

    void NRF24L01::maskIRQ(const bool tx_ok, const bool tx_fail, const bool rx_ready)
    {
        uint8_t config = readRegister(Register::CONFIG);

        config &= ~(CONFIG::MASK_MAX_RT | CONFIG::MASK_TX_DS | CONFIG::MASK_RX_DR);
        config |= (tx_fail << CONFIG::MASK_MAX_RT_Pos) | (tx_ok << CONFIG::MASK_TX_DS_Pos) | (rx_ready << CONFIG::MASK_RX_DR_Pos);

        writeRegister(Register::CONFIG, config);
    }

    uint8_t NRF24L01::readRegister(const uint8_t reg, uint8_t *const buf, size_t len)
    {
        if(len > PAYLOAD_LEN)
        {
            len = PAYLOAD_LEN;
        }

        spi_txbuff[0] = (Command::R_REGISTER | (Command::REGISTER_MASK & reg));
        memset(&spi_txbuff[1], Command::NOP, len);

        begin_transaction();
        spi_write_read(spi_txbuff.begin(), spi_rxbuff.begin(), len);
        end_transaction();

        #if defined(DEBUG)
        statusReg.convert(spi_rxbuff[0]);
        #endif

        /* Return only the status code of the chip. The register values will be in the rx buff */
        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::readRegister(const uint8_t reg)
    {
        size_t txLength = 2;
        spi_txbuff[0] = (Command::R_REGISTER | (Command::REGISTER_MASK & reg));
        spi_txbuff[1] = Command::NOP;

        begin_transaction();
        spi_write_read(spi_txbuff.begin(), spi_rxbuff.begin(), txLength);
        end_transaction();

        #if defined(DEBUG)
        statusReg.convert(spi_rxbuff[0]);
        #endif 

        /* Current register value is in the second byte of the receive buffer */
        return spi_rxbuff[1];
    }

    uint8_t NRF24L01::writeRegister(const uint8_t reg, const uint8_t *const buf, size_t len)
    {
        if(len > PAYLOAD_LEN)
        {
            len = PAYLOAD_LEN;
        }

        spi_txbuff[0] = (Command::W_REGISTER | (Command::REGISTER_MASK & reg));
        memcpy(&spi_txbuff[1], buf, len);

        len += 1;
        begin_transaction();
        spi_write_read(spi_txbuff.begin(), spi_rxbuff.begin(), len);
        end_transaction();

        #if defined(DEBUG)
        statusReg.convert(spi_rxbuff[0]);
        #endif

        /* Status code is in the first byte of the receive buffer */
        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::writeRegister(const uint8_t reg, const uint8_t value)
    {
        size_t txLength = 2;
        spi_txbuff[0] = (Command::W_REGISTER | (Command::REGISTER_MASK & reg));
        spi_txbuff[1] = value;

        begin_transaction();
        spi_write_read(spi_txbuff.begin(), spi_rxbuff.begin(), txLength);
        end_transaction();

        #if defined(DEBUG)
        statusReg.convert(spi_rxbuff[0]);
        #endif

        /* Status code is in the first byte of the receive buffer */
        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::writePayload(const uint8_t *const buf, size_t len, const uint8_t writeType)
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
        spi_write_read(spi_txbuff.begin(), spi_rxbuff.begin(), size);
        end_transaction();

        #if defined(DEBUG)
        statusReg.convert(spi_rxbuff[0]);
        #endif

        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::readPayload(uint8_t *const buf, size_t len)
    {
        if(len > payload_size)
        {
            len = payload_size;
        }

        /*-------------------------------------------------
        Calculate the number of bytes that do nothing
        -------------------------------------------------*/
        uint8_t *const rx_buf = reinterpret_cast<uint8_t *const>(buf);
        uint8_t blank_len = static_cast<uint8_t>(dynamic_payloads_enabled ? 0 : (payload_size - len));
        size_t size = len + blank_len + 1;

        /*-------------------------------------------------
        Format the read command and fill the rest with NOPs
        -------------------------------------------------*/
        spi_txbuff[0] = Command::R_RX_PAYLOAD;
        memset(&spi_txbuff[1], Command::NOP, (size - 1));

        begin_transaction();
        spi_write_read(spi_txbuff.begin(), spi_rxbuff.begin(), size);
        end_transaction();

        #if defined(DEBUG)
        statusReg.convert(spi_rxbuff[0]);
        #endif

        /*-------------------------------------------------
        The status byte is first, RX payload is all remaining
        -------------------------------------------------*/
        memcpy(rx_buf, spi_rxbuff.begin(), (size - 1));
        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::get_status()
    {
        return write_cmd(Command::NOP);
    }

    size_t NRF24L01::spi_write(const uint8_t *const tx_buffer, size_t len)
    {
        spi->writeBytes(tx_buffer, len);
        return len;
    }

    size_t NRF24L01::spi_read(uint8_t *const rx_buffer, size_t len)
    {
        spi->readBytes(rx_buffer, len);
        return len;
    }

    size_t NRF24L01::spi_write_read(const uint8_t *const tx_buffer, uint8_t *const rx_buffer, size_t len)
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

    uint8_t NRF24L01::write_cmd(const uint8_t cmd)
    {
        size_t txLength = 1;
        spi_txbuff[0] = cmd;

        begin_transaction();
        spi_write_read(spi_txbuff.begin(), spi_rxbuff.begin(), txLength);
        end_transaction();

        /* Give back the status register value */
        return spi_rxbuff[0];
    }

    bool NRF24L01::registerBitmaskSet(const uint8_t reg, const uint8_t bitmask)
    {
        return (readRegister(reg) & bitmask) == bitmask;
    }

    bool NRF24L01::registerAnySet(const uint8_t reg, const uint8_t bitmask)
    {
        return readRegister(reg) & bitmask;
    }

    void NRF24L01::clearRegisterBits(const uint8_t reg, const uint8_t bitmask)
    {
        uint8_t regVal = readRegister(reg);
        regVal &= ~bitmask;
        writeRegister(reg, regVal);
    }

    void NRF24L01::setRegisterBits(const uint8_t reg, const uint8_t bitmask)
    {
        uint8_t regVal = readRegister(reg);
        regVal |= bitmask;
        writeRegister(reg, regVal);
    }

}
