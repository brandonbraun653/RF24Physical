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

    static const std::array<uint8_t, MAX_NUM_PIPES> pipeRXAddressReg =
    {
        Register::RX_ADDR_P0,
        Register::RX_ADDR_P1,
        Register::RX_ADDR_P2,
        Register::RX_ADDR_P3,
        Register::RX_ADDR_P4,
        Register::RX_ADDR_P5
    };
    static_assert(pipeRXAddressReg.size() == MAX_NUM_PIPES, "Too many/few items in the array!");

    static const std::array<uint8_t, MAX_NUM_PIPES> pipeEnableRXAddressReg =
    {
        EN_RXADDR::P0,
        EN_RXADDR::P1,
        EN_RXADDR::P2,
        EN_RXADDR::P3,
        EN_RXADDR::P4,
        EN_RXADDR::P5
    };
    static_assert(pipeEnableRXAddressReg.size() == MAX_NUM_PIPES, "Too many/few items in the array!");

    static const std::array<uint8_t, MAX_NUM_PIPES> pipeRXPayloadWidthReg =
    {
        Register::RX_PW_P0,
        Register::RX_PW_P1,
        Register::RX_PW_P2,
        Register::RX_PW_P3,
        Register::RX_PW_P4,
        Register::RX_PW_P5
    };
    static_assert(pipeRXPayloadWidthReg.size() == MAX_NUM_PIPES, "Too many/few items in the array!");

    NRF24L01::NRF24L01(Chimera::SPI::SPIClass_sPtr spiInstance, Chimera::GPIO::GPIOClass_sPtr chipEnable)
    {
        this->spi = spiInstance;
        this->chipEnable = chipEnable;

        memset(spi_txbuff.begin(), 0, spi_txbuff.size());
        memset(spi_rxbuff.begin(), 0, spi_rxbuff.size());

        /*-------------------------------------------------
        Initialize class variables
        -------------------------------------------------*/
        addr_width = MAX_ADDRESS_WIDTH;
        payload_size = MAX_PAYLOAD_WIDTH;
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
        deactivateFeatures();
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
        flushTX();
        flushRX();

        /*-------------------------------------------------
        Power up the module and enable PTX. Stay in standby mode by not writing CE high
        -------------------------------------------------*/
        chipEnable->write(State::LOW);
        powerUp();

        return (setup != 0 && setup != 0xFF);
    }

    void NRF24L01::startListening()
    {
        /*-------------------------------------------------
        If we are auto-acknowledging RX packets with a payload, make sure the TX
        FIFO is clean so we don't accidently transmit data.
        -------------------------------------------------*/
        if(registerIsBitmaskSet(Register::FEATURE, FEATURE::EN_ACK_PAY))
        {
            flushTX();
        }

        /*-------------------------------------------------
        Clear interrupt flags and transition to RX mode
        -------------------------------------------------*/
        setRegisterBits(Register::STATUS, STATUS::RX_DR | STATUS::TX_DS | STATUS::MAX_RT);
        setRegisterBits(Register::CONFIG, CONFIG::PRIM_RX);
        chipEnable->write(State::HIGH);
        currentMode = Mode::RX;
    }

    void NRF24L01::stopListening()
    {
        /*-------------------------------------------------
        Set the chip into standby mode I
        -------------------------------------------------*/
        chipEnable->write(State::LOW);
        currentMode = Mode::STANDBY_I;
        delayMilliseconds(1);

        /*-------------------------------------------------
        If we are auto-acknowledging RX packets with a payload, make sure the TX FIFO is clean so
        we don't automatically transmit data the next time we write chipEnable high.
        -------------------------------------------------*/
        if (registerIsBitmaskSet(Register::FEATURE, FEATURE::EN_ACK_PAY))
        {
            flushTX();
        }

        /*-------------------------------------------------
        Disable RX/Enable TX
        -------------------------------------------------*/
        clearRegisterBits(Register::CONFIG, CONFIG::PRIM_RX);

        /*-------------------------------------------------
        Ensure RX Pipe 0 can listen (TX only transmits/receives on pipe 0)
        -------------------------------------------------*/
        setRegisterBits(Register::EN_RXADDR, pipeEnableRXAddressReg[0]);
    }

    bool NRF24L01::available()
    {
        return !registerIsBitmaskSet(Register::FIFO_STATUS, FIFO_STATUS::RX_EMPTY);
    }

    bool NRF24L01::available(uint8_t &pipe_num)
    {
        /*-------------------------------------------------
        Figure out which pipe has data available
        -------------------------------------------------*/
        if(available())
        {
            pipe_num = (getStatus() >> STATUS::RX_P_NO_Pos) & STATUS::RX_P_NO_Wid;
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

    void NRF24L01::read(char *const buffer, size_t len)
    {
        read(reinterpret_cast<uint8_t *const>(buffer), len);
    }

    bool NRF24L01::write(const uint8_t *const buffer, size_t len, const bool requestACK, const bool startTX, const bool autoStandby)
    {
        constexpr uint32_t busyDelay_mS = 1;
        bool writeResult = true;

        /*-------------------------------------------------
        Wait for the FIFO to have room for another packet
        -------------------------------------------------*/
        while(txFIFOFull())
        {
            delayMilliseconds(busyDelay_mS);

            /*-------------------------------------------------
            If max retries hit, we failed and should exit here
            -------------------------------------------------*/
            if (registerIsBitmaskSet(Register::STATUS, STATUS::MAX_RT))
            {
                flushTX();
                writeResult = false;
                break;
            }
        }

        if (writeResult)
        {
            /*-------------------------------------------------
            Dump data into the TX FIFO and start the transfer
            -------------------------------------------------*/
            startFastWrite(buffer, len, requestACK, startTX);

            /*-------------------------------------------------
            Handle ACK/NACK should we be starting the transfer
            -------------------------------------------------*/
            if (startTX)
            {
                /*-------------------------------------------------
                Wait for the Data Sent or Max Retry interrupt to occur
                -------------------------------------------------*/
                while (!registerIsAnySet(Register::STATUS, (STATUS::TX_DS | STATUS::MAX_RT)))
                {
                    delayMilliseconds(busyDelay_mS);
                }

                /*-------------------------------------------------
                Wait for the FIFO to clear or Max Retries hit
                -------------------------------------------------*/
                if (autoStandby)
                {
                    while (!txFIFOEmpty() && !registerIsBitmaskSet(Register::STATUS, STATUS::MAX_RT))
                    {
                        delayMilliseconds(busyDelay_mS);
                    }

                    chipEnable->write(State::LOW);
                    currentMode = Mode::STANDBY_I;
                }

                /*-------------------------------------------------
                If we hit the Max Retries, we have a problem and the whole TX FIFO is screwed
                -------------------------------------------------*/
                if (registerIsBitmaskSet(Register::STATUS, STATUS::MAX_RT))
                {
                    flushTX();
                    writeResult = false;
                }

                /*-------------------------------------------------
                Clear all the interrupt flags
                -------------------------------------------------*/
                setRegisterBits(Register::STATUS, STATUS::RX_DR | STATUS::TX_DS | STATUS::MAX_RT);
            }

        }

        return writeResult;
    }

    bool NRF24L01::write(const char *const buffer, size_t len, const bool requestACK, const bool startTX, const bool autoStandby)
    {
        return write(reinterpret_cast<const uint8_t *const>(buffer), len, requestACK, startTX, autoStandby);
    }

    void NRF24L01::openWritePipe(const std::array<uint8_t, MAX_ADDRESS_WIDTH> address)
    {
        /*-------------------------------------------------
        Set the receive address for pipe 0, this one has a maximum of 5 byte width
        -------------------------------------------------*/
        writeRegister(Register::RX_ADDR_P0, address.begin(), addr_width);

        /*-------------------------------------------------
        Make sure we transmit back to the same address we expect receive from
        -------------------------------------------------*/
        writeRegister(Register::TX_ADDR, address.begin(), addr_width);

        /*-------------------------------------------------
        Set a static payload length for all receptions on pipe 0. There must also be
        an equal number of bytes clocked into the TX_FIFO when data is transmitted out.
        -------------------------------------------------*/
        writeRegister(Register::RX_PW_P0, payload_size);
    }

    void NRF24L01::openReadPipe(const Pipe pipe, const std::array<uint8_t, MAX_ADDRESS_WIDTH> address, const bool autoAck)
    {
        if(pipe < MAX_NUM_PIPES)
        {
            /*-------------------------------------------------
            Pipes 0 & 1 can use the full 5 byte width. The rest only use the LSB.
            -------------------------------------------------*/
            if(pipe < 2)
            {
                /*-------------------------------------------------
                Write only as many bytes as were set in SETUP_AW
                -------------------------------------------------*/
                writeRegister(pipeRXAddressReg[pipe], address.begin(), addr_width);

                /*-------------------------------------------------
                Save pipe 0 address for future use
                -------------------------------------------------*/
                if(pipe == 0)
                {
                    memcpy(pipe0_reading_address.begin(), address.begin(), addr_width);
                }
            }
            else
            {
                writeRegister(pipeRXAddressReg[pipe], address.begin(), 1);
            }

            /*-------------------------------------------------
            Turn on/off the auto acknowledge feature
            -------------------------------------------------*/
            setAutoAck(pipe, autoAck);

            /*-------------------------------------------------
            Let the pipe know how wide the payload will be, then turn it on
            -------------------------------------------------*/
            writeRegister(pipeRXPayloadWidthReg[pipe], payload_size);
            setRegisterBits(Register::EN_RXADDR, pipeEnableRXAddressReg[pipe]);
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

    bool NRF24L01::txFIFOFull()
    {
        return readRegister(Register::FIFO_STATUS) & FIFO_STATUS::TX_FULL;
    }

    bool NRF24L01::txFIFOEmpty()
    {
        return readRegister(Register::FIFO_STATUS) & FIFO_STATUS::TX_EMPTY;
    }

    void NRF24L01::powerUp()
    {
        /*-------------------------------------------------
        If not powered up already, do it. The worst startup delay is
        about 5mS, so just wait that amount.
        -------------------------------------------------*/
        if(!registerIsBitmaskSet(Register::CONFIG, CONFIG::PWR_UP))
        {
            writeRegister(Register::CONFIG, CONFIG::PWR_UP);
            delayMilliseconds(5);

            currentMode = Mode::STANDBY_I;
        }
    }

    void NRF24L01::powerDown()
    {
        /*-------------------------------------------------
        Force standby mode and power down the chip
        -------------------------------------------------*/
        chipEnable->write(State::LOW);
        clearRegisterBits(Register::CONFIG, CONFIG::PWR_UP);
        currentMode = Mode::POWER_DOWN;
    }

    void NRF24L01::startFastWrite(const uint8_t *const buffer, size_t len, const bool requestACK, const bool startTX)
    {
        uint8_t payloadType = 0u;

        if (requestACK)
        {
            /*-------------------------------------------------
            Force waiting for the RX device to send an ACK packet
            -------------------------------------------------*/
            disableDynamicAck();
            payloadType = Command::W_TX_PAYLOAD;
        }
        else
        {
            /*-------------------------------------------------
            Transmit one packet without waiting for an ACK packet from the RX device
            -------------------------------------------------*/
            enableDynamicAck();
            payloadType = Command::W_TX_PAYLOAD_NO_ACK;
        }

        /*-------------------------------------------------
        Write the payload to the TX FIFO and optionally start the transfer
        -------------------------------------------------*/
        writePayload(buffer, len, payloadType);

        if (startTX)
        {
            chipEnable->write(State::HIGH);
            currentMode = Mode::STANDBY_II;
        }
    }

    bool NRF24L01::txStandBy()
    {
        /*-------------------------------------------------
        Wait for the TX FIFO to signal it's empty
        -------------------------------------------------*/
        while (!registerIsBitmaskSet(Register::FIFO_STATUS, FIFO_STATUS::TX_EMPTY))
        {
            /*-------------------------------------------------
            If we hit the Max Retries, we have a problem and the whole TX FIFO is screwed.
            Go back to standby mode and clear out the FIFO.
            -------------------------------------------------*/
            if (registerIsBitmaskSet(Register::STATUS, STATUS::MAX_RT))
            {
                setRegisterBits(Register::STATUS, STATUS::MAX_RT);
                chipEnable->write(State::LOW);
                flushTX();
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
        writeCMD(Command::REUSE_TX_PL);
        chipEnable->write(State::LOW);
        chipEnable->write(State::HIGH);
    }

    void NRF24L01::closeReadPipe(const uint8_t pipe)
    {
        if (pipe < MAX_NUM_PIPES)
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

    void NRF24L01::setPayloadSize(const uint8_t size)
    {
        payload_size = std::min(size, static_cast<uint8_t>(32));
    }

    void NRF24L01::setChannel(const uint8_t channel)
    {
        writeRegister(Register::RF_CH, channel & RF_CH::Mask);
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
            flushRX();
            delayMilliseconds(2);
            return 0;
        }

        return result;
    }

    uint8_t NRF24L01::flushTX()
    {
        return writeCMD(Command::FLUSH_TX);
    }

    uint8_t NRF24L01::flushRX()
    {
        return writeCMD(Command::FLUSH_RX);
    }

    void NRF24L01::activateFeatures()
    {
        if (!features_activated)
        {
            spi_txbuff[0] = Command::ACTIVATE;
            spi_txbuff[1] = 0x73;

            spi_write(spi_txbuff.begin(), 2);
            features_activated = true;
        }
    }

    void NRF24L01::deactivateFeatures()
    {
        if (features_activated)
        {
            /*-------------------------------------------------
            Sending the activation command sequence again also disables the features
            -------------------------------------------------*/
            activateFeatures();
            features_activated = false;
        }
    }

    void NRF24L01::enableAckPayload()
    {
        activateFeatures();
        setRegisterBits(Register::FEATURE, FEATURE::EN_ACK_PAY | FEATURE::EN_DPL);
        setRegisterBits(Register::DYNPD, DYNPD::DPL_P0 | DYNPD::DPL_P1);

        dynamic_payloads_enabled = true;
    }

    void NRF24L01::disableAckPayload()
    {
        if (features_activated)
        {
            clearRegisterBits(Register::FEATURE, FEATURE::EN_ACK_PAY | FEATURE::EN_DPL);
            clearRegisterBits(Register::DYNPD, DYNPD::DPL_P0 | DYNPD::DPL_P1);

            dynamic_payloads_enabled = false;
        }
    }

    void NRF24L01::enableDynamicPayloads()
    {
        /*-------------------------------------------------
        Send the activate command to enable selection of features
        -------------------------------------------------*/
        activateFeatures();

        /*-------------------------------------------------
        Enable the dynamic payload feature bit
        -------------------------------------------------*/
        setRegisterBits(Register::FEATURE, FEATURE::EN_DPL);

        /*-------------------------------------------------
        Enable dynamic payload on all pipes. This requires that
        auto-acknowledge be enabled.
        -------------------------------------------------*/
        setRegisterBits(Register::EN_AA, EN_AA::Mask);
        setRegisterBits(Register::DYNPD, DYNPD::Mask);

        dynamic_payloads_enabled = true;
    }

    void NRF24L01::disableDynamicPayloads()
    {
        /*-------------------------------------------------
        Disable for all pipes
        -------------------------------------------------*/
        if (features_activated)
        {
            clearRegisterBits(Register::DYNPD, DYNPD::Mask);
            clearRegisterBits(Register::EN_AA, EN_AA::Mask);
            clearRegisterBits(Register::FEATURE, FEATURE::EN_DPL);
        }
    }

    void NRF24L01::enableDynamicAck()
    {
        activateFeatures();
        setRegisterBits(Register::FEATURE, FEATURE::EN_DYN_ACK);
    }

    void NRF24L01::disableDynamicAck()
    {
        if (features_activated)
        {
            clearRegisterBits(Register::FEATURE, FEATURE::EN_DYN_ACK);
        }
    }

    bool NRF24L01::isPVariant()
    {
        return pVariant;
    }

    void NRF24L01::setAutoAckAll(const bool enable)
    {
        if (enable)
        {
            setRegisterBits(Register::EN_AA, EN_AA::Mask);
        }
        else
        {
            clearRegisterBits(Register::EN_AA, EN_AA::Mask);
        }
    }

    void NRF24L01::setAutoAck(const Pipe pipe, const bool enable)
    {
        if (pipe < MAX_NUM_PIPES)
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
        if(len > MAX_PAYLOAD_WIDTH)
        {
            len = MAX_PAYLOAD_WIDTH;
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
        if(len > MAX_PAYLOAD_WIDTH)
        {
            len = MAX_PAYLOAD_WIDTH;
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
        memcpy(&spi_txbuff[1], buf, len);           /* Payload information */
        memset(&spi_txbuff[len], 0, blank_len);     /* Null out the remaining buffer space*/

        begin_transaction();
        spi_write_read(spi_txbuff.begin(), spi_rxbuff.begin(), size);
        end_transaction();

        #if defined(DEBUG)
        statusReg.convert(spi_rxbuff[0]);
        #endif

        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::readPayload(uint8_t *const buffer, size_t len)
    {
        /*-------------------------------------------------
        Cap the data length
        -------------------------------------------------*/
        len = std::min(len, MAX_PAYLOAD_WIDTH);

        /*-------------------------------------------------
        Calculate the number of bytes that do nothing. This is important for
        fixed payload widths as the full width must be read out each time.
        -------------------------------------------------*/
        uint8_t blank_len = static_cast<uint8_t>(dynamic_payloads_enabled ? 0 : (payload_size - len));
        size_t size = len + blank_len;

        /*-------------------------------------------------
        Format the read command and fill the rest with NOPs
        -------------------------------------------------*/
        spi_txbuff[0] = Command::R_RX_PAYLOAD;
        memset(&spi_txbuff[1], Command::NOP, size);

        /*-------------------------------------------------
        Read out the payload. The +1 is for the read command.
        -------------------------------------------------*/
        begin_transaction();
        spi_write_read(spi_txbuff.begin(), spi_rxbuff.begin(), size + 1);
        end_transaction();

        #if defined(DEBUG)
        statusReg.convert(spi_rxbuff[0]);
        #endif

        /*-------------------------------------------------
        The status byte is first, RX payload is all remaining bytes
        -------------------------------------------------*/
        memcpy(buffer, &spi_rxbuff[1], size);
        return spi_rxbuff[0];
    }

    uint8_t NRF24L01::getStatus()
    {
        return writeCMD(Command::NOP);
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

    uint8_t NRF24L01::writeCMD(const uint8_t cmd)
    {
        size_t txLength = 1;
        spi_txbuff[0] = cmd;

        begin_transaction();
        spi_write_read(spi_txbuff.begin(), spi_rxbuff.begin(), txLength);
        end_transaction();

        return spi_rxbuff[0];
    }

    bool NRF24L01::registerIsBitmaskSet(const uint8_t reg, const uint8_t bitmask)
    {
        return (readRegister(reg) & bitmask) == bitmask;
    }

    bool NRF24L01::registerIsAnySet(const uint8_t reg, const uint8_t bitmask)
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
