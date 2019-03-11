/********************************************************************************
 * File Name:
 *    rf24phy_python.cpp
 *
 * Description:
 *    Provides python bindings to the RF24Phy interface.
 *
 *  Notes:
 *    1. If receiving compilation errors about unwind_type not being found, follow
 *       instructions from colakoyeh here: https://github.com/boostorg/python/issues/228
 *          a. open unwind_type.hpp
 *          b. move line 33~53 to line 94
 *          c. compile it!
 *
 *    2.
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <RF24Phy/RF24Phy.hpp>

/* Boost Includes */
#include <boost/python.hpp>

/*------------------------------------------------
Expose overloaded class functions
------------------------------------------------*/
bool ( RF24Phy::Phy::*available_void )()               = &RF24Phy::Phy::available;
bool ( RF24Phy::Phy::*available_uint8_t )( uint8_t & ) = &RF24Phy::Phy::available;

bool ( RF24Phy::Phy::*txStandBy_void )()     = &RF24Phy::Phy::txStandBy;
bool ( RF24Phy::Phy::*txStandBy_u32_bool )() = &RF24Phy::Phy::txStandBy;

uint8_t ( RF24Phy::Phy::*readRegister_short )( const uint8_t )                        = &RF24Phy::Phy::readRegister;
uint8_t ( RF24Phy::Phy::*readRegister_full )( const uint8_t, uint8_t *const, size_t ) = &RF24Phy::Phy::readRegister;

uint8_t ( RF24Phy::Phy::*writeRegister_short )( const uint8_t, const uint8_t )               = &RF24Phy::Phy::writeRegister;
uint8_t ( RF24Phy::Phy::*writeRegister_full )( const uint8_t, const uint8_t *const, size_t ) = &RF24Phy::Phy::writeRegister;


// clang-format off
BOOST_PYTHON_MODULE( PyRF24Phy )
{
  using namespace boost::python;
  class_<RF24Phy::Phy>( "Phy", init<Chimera::SPI::SPIClass_sPtr, Chimera::GPIO::GPIOClass_sPtr>() )
      .def( "erase", &RF24Phy::Phy::erase )
      .def( "begin", &RF24Phy::Phy::begin )
      .def( "isInitialized", &RF24Phy::Phy::isInitialized )
      .def( "powerUp", &RF24Phy::Phy::powerUp )
      .def( "powerDown", &RF24Phy::Phy::powerDown )
      .def( "startListening", &RF24Phy::Phy::startListening )
      .def( "pauseListening", &RF24Phy::Phy::pauseListening )
      .def( "resumeListening", &RF24Phy::Phy::resumeListening )
      .def( "stopListening", &RF24Phy::Phy::stopListening )
      .def( "openWritePipe", &RF24Phy::Phy::openWritePipe )
      .def( "openReadPipe", &RF24Phy::Phy::openReadPipe )
      .def( "closeReadPipe", &RF24Phy::Phy::closeReadPipe )
      .def( "available", available_void )
      .def( "available", available_uint8_t )
      .def( "read", &RF24Phy::Phy::read )
      .def( "writeFast", &RF24Phy::Phy::writeFast )
      .def( "isConnected", &RF24Phy::Phy::isConnected )
      .def( "rxFifoFull", &RF24Phy::Phy::rxFifoFull )
      .def( "rxFifoEmpty", &RF24Phy::Phy::rxFifoEmpty )
      .def( "txFifoFull", &RF24Phy::Phy::txFifoFull )
      .def( "txFifoEmpty", &RF24Phy::Phy::txFifoEmpty )
      .def( "txStandBy", txStandBy_void )
      .def( "txStandBy", txStandBy_u32_bool )
      .def( "writeAckPayload", &RF24Phy::Phy::writeAckPayload )
      .def( "isAckPayloadAvailable", &RF24Phy::Phy::isAckPayloadAvailable )
      .def( "whatHappened", &RF24Phy::Phy::whatHappened )
      .def( "reUseTX", &RF24Phy::Phy::reUseTX )
      .def( "setAddressWidth", &RF24Phy::Phy::setAddressWidth )
      .def( "getAddressWidth", &RF24Phy::Phy::getAddressWidth )
      .def( "getAddressBytes", &RF24Phy::Phy::getAddressBytes )
      .def( "setRetries", &RF24Phy::Phy::setRetries )
      .def( "setChannel", &RF24Phy::Phy::setChannel )
      .def( "getChannel", &RF24Phy::Phy::getChannel )
      .def( "setStaticPayloadSize", &RF24Phy::Phy::setStaticPayloadSize )
      .def( "getStaticPayloadSize", &RF24Phy::Phy::getStaticPayloadSize )
      .def( "getDynamicPayloadSize", &RF24Phy::Phy::getDynamicPayloadSize )
      .def( "flushTX", &RF24Phy::Phy::flushTX )
      .def( "flushRX", &RF24Phy::Phy::flushRX )
      .def( "activateFeatures", &RF24Phy::Phy::activateFeatures )
      .def( "deactivateFeatures", &RF24Phy::Phy::deactivateFeatures )
      .def( "enableAckPayload", &RF24Phy::Phy::enableAckPayload )
      .def( "disableAckPayload", &RF24Phy::Phy::disableAckPayload )
      .def( "enableDynamicPayloads", &RF24Phy::Phy::enableDynamicPayloads )
      .def( "disableDynamicPayloads", &RF24Phy::Phy::disableDynamicPayloads )
      .def( "enableDynamicAck", &RF24Phy::Phy::enableDynamicAck )
      .def( "disableDynamicAck", &RF24Phy::Phy::disableDynamicAck )
      .def( "isPVariant", &RF24Phy::Phy::isPVariant )
      .def( "setAutoAckAll", &RF24Phy::Phy::setAutoAckAll )
      .def( "setAutoAck", &RF24Phy::Phy::setAutoAck )
      .def( "setPALevel", &RF24Phy::Phy::setPALevel )
      .def( "getPALevel", &RF24Phy::Phy::getPALevel )
      .def( "setDataRate", &RF24Phy::Phy::setDataRate )
      .def( "getDataRate", &RF24Phy::Phy::getDataRate )
      .def( "setCRCLength", &RF24Phy::Phy::setCRCLength )
      .def( "getCRCLength", &RF24Phy::Phy::getCRCLength )
      .def( "disableCRC", &RF24Phy::Phy::disableCRC )
      .def( "maskIRQ", &RF24Phy::Phy::maskIRQ )
      .def( "readRegister", readRegister_short )
      .def( "readRegister", readRegister_full )
      .def( "writeRegister", writeRegister_short )
      .def( "writeRegister", writeRegister_full )
      .def( "readPayload", &RF24Phy::Phy::readPayload )
      .def( "getStatus", &RF24Phy::Phy::getStatus )
      .def( "getFailureCode", &RF24Phy::Phy::getFailureCode )
      .def( "delayMilliseconds", &RF24Phy::Phy::delayMilliseconds )
      .def( "millis", &RF24Phy::Phy::millis )
      ;
}
// clang-format on