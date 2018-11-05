#include "nrf24l01.hpp"

static uint8_t spi_rxbuff[NRF24L_SPI_BUFFER_LEN];
static_assert(sizeof(spi_rxbuff) == NRF24L_SPI_BUFFER_LEN, "SPI receive buffer is the wrong length");

static uint8_t spi_txbuff[NRF24L_SPI_BUFFER_LEN];
static_assert(sizeof(spi_txbuff) == NRF24L_SPI_BUFFER_LEN, "SPI transmit buffer is the wrong length");

NRF24L01::NRF24L01()
{

}

void NRF24L01::begin()
{

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
    return 0;
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

void NRF24L01::setRetries(uint8_t delay, uint8_t count)
{

}

void NRF24L01::setChannel(uint8_t channel)
{

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

bool NRF24L01::setDataRate(nrf24_datarate speed)
{
    return false;
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

uint8_t NRF24L01::read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
    return 0;
}

uint8_t NRF24L01::read_register(uint8_t reg)
{
    return 0;
}

uint8_t NRF24L01::write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
    return 0;
}

uint8_t NRF24L01::write_register(uint8_t reg, uint8_t value)
{
    uint8_t status;

    spi_txbuff[0] = (NRF24L_CMD_W_REGISTER | (NRF24L_CMD_REGISTER_MASK & reg));
    spi_txbuff[1] = value;

    spi_write_read(spi_txbuff, spi_rxbuff, 2);

    status = spi_rxbuff[0];

    return status;
}

uint8_t NRF24L01::write_payload(const void* buf, uint8_t len, const uint8_t writeType)
{
    return 0;
}

uint8_t NRF24L01::read_payload(void* buf, uint8_t len)
{
    return 0;
}

uint8_t NRF24L01::get_status()
{
    return 0;
}

void NRF24L01::openWritePipe(const uint8_t *address)
{

}

void NRF24L01::openReadPipe(uint8_t number, const uint8_t *address)
{

}

bool NRF24L01::read(void *buffer, uint8_t len)
{
    return false;
}
