#include "nrf24l01.hpp"


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

}

bool NRF24L01::available(uint8_t *pipe_num)
{

}

bool NRF24L01::rxFifoFull()
{

}

void NRF24L01::powerDown()
{

}

void NRF24L01::powerUp()
{

}

bool NRF24L01::write(const void *buffer, uint8_t len)
{

}

bool NRF24L01::write(const void *buf, uint8_t len, const bool multicast)
{

}

bool NRF24L01::writeFast(const void *buf, uint8_t len)
{

}

bool NRF24L01::writeFast(const void *buf, uint8_t len, const bool multicast)
{

}

bool NRF24L01::writeBlocking(const void *buf, uint8_t len, uint32_t timeout)
{

}

bool NRF24L01::txStandBy()
{

}

bool NRF24L01::txStandBy(uint32_t timeout, bool startTx)
{

}

void NRF24L01::writeAckPayload(uint8_t pipe, const void *buf, uint8_t len)
{

}

bool NRF24L01::isAckPayloadAvailable()
{

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

}

bool NRF24L01::testCarrier()
{

}

bool NRF24L01::testRPD()
{

}

bool NRF24L01::isValid()
{

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

}

void NRF24L01::setPayloadSize(uint8_t size)
{

}

uint8_t NRF24L01::getPayloadSize()
{

}

uint8_t NRF24L01::getDynamicPayloadSize()
{

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

}

bool NRF24L01::setDataRate(nrf24_datarate speed)
{

}

nrf24_datarate NRF24L01::getDataRate()
{

}

void NRF24L01::setCRCLength(nrf24_crclength length)
{

}

nrf24_crclength NRF24L01::getCRCLength()
{

}

void NRF24L01::disableCRC()
{

}

void NRF24L01::maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready)
{

}

void NRF24L01::openWritePipe(const uint8_t *address)
{

}

void NRF24L01::openReadPipe(uint8_t number, const uint8_t *address)
{

}

bool NRF24L01::read(void *buffer, uint8_t len)
{

}
