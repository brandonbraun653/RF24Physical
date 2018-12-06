#pragma once
#ifndef NRF24L01_HPP
#define NRF24L01_HPP

/* C/C++ Includes */
#include <stdint.h>
#include <cstdio>
#include <array>

/* Chimera Includes */
#include <Chimera/gpio.hpp>
#include <Chimera/spi.hpp>

/* Module Includes */
#include "nrf24l01_definitions.hpp"

namespace NRF24L
{
    class NRF24L01;

    typedef std::shared_ptr<NRF24L01> NRF24L01_sPtr;
    typedef std::unique_ptr<NRF24L01> NRF24L01_uPtr;

    enum class PowerAmplitude : uint8_t
    {
        MIN     = 0u,   /**< -18 dBm */
        LOW     = 2u,   /**< -12 dBm */
        HIGH    = 4u,   /**<  -6 dBm */
        MAX     = 6u    /**<   0 dBm */
    };

    enum class DataRate : uint8_t
    {
        DR_1MBPS,
        DR_2MBPS,
        DR_250KBPS
    };

    enum class CRCLength : uint8_t
    {
        CRC_DISABLED,
        CRC_8,
        CRC_16
    };

    /**
    *   Base class for interacting with an NRF24L01 wireless module.
    *   Most of this was taken from https://github.com/nRF24/RF24 and modified to
    *   accommodate my particular flavor of hardware abstraction.
    */
    class NRF24L01
    {
    public:
        NRF24L01(Chimera::SPI::SPIClass_sPtr spiInstance, Chimera::GPIO::GPIOClass_sPtr chipEnable);
        NRF24L01() = default;
        ~NRF24L01() = default;

        /**
        *   Begin operation of the chip
        *
        *   Call this before calling any other methods.
        *   @code radio.begin() @endcode
        */
        bool begin();

        /**
        *   Start listening on the pipes opened for reading.
        *
        *   1. Be sure to call openReadingPipe() first.
        *   2. Do not call write() while in this mode, without first calling stopListening().
        *   3. Call available() to check for incoming traffic, and read() to get it.
        *
        *   @code
        *   Open reading pipe 1 using address CCCECCCECC
        *
        *   byte address[] = { 0xCC,0xCE,0xCC,0xCE,0xCC };
        *   radio.openReadingPipe(1,address);
        *   radio.startListening();
        *   @endcode
        */
        void startListening();

        /**
        *   Stop listening for incoming messages, and switch to transmit mode.
        *
        *   Do this before calling write().
        *   @code
        *   radio.stopListening();
        *   radio.write(&data,sizeof(data));
        *   @endcode
        */
        void stopListening();

        /**
        *   Check whether there are bytes available to be read
        *   @code
        *   if(radio.available()){
        *     radio.read(&data,sizeof(data));
        *   }
        *   @endcode
        *   @return True if there is a payload available, false if none is
        */
        bool available();

        /**
        *   Test whether there are bytes available to be read in the
        *   FIFO buffers.
        *
        *   @param[out] pipe_num Which pipe has the payload available
        *
        *   @code
        *   uint8_t pipeNum;
        *   if(radio.available(&pipeNum)){
        *     radio.read(&data,sizeof(data));
        *     Serial.print("Got data on pipe");
        *     Serial.println(pipeNum);
        *   }
        *   @endcode
        *   @return True if there is a payload available, false if none is
        */
        bool available(uint8_t *const pipeNum);

        /**
        *   Read the available payload
        *
        *   The size of data read is the fixed payload size, see getPayloadSize()
        *
        *   @note I specifically chose 'void*' as a data type to make it easier
        *   for beginners to use.  No casting needed.
        *
        *   @note No longer boolean. Use available to determine if packets are
        *   available. Interrupt flags are now cleared during reads instead of
        *   when calling available().
        *
        *   @param buf Pointer to a buffer where the data should be written
        *   @param len Maximum number of bytes to read into the buffer
        *
        *   @code
        *   if(radio.available()){
        *     radio.read(&data,sizeof(data));
        *   }
        *   @endcode
        *   @return No return value. Use available().
        */
        void read(void *const buffer, size_t len);

        /**
        *   Be sure to call openWritingPipe() first to set the destination
        *   of where to write to.
        *
        *   This blocks until the message is successfully acknowledged by
        *   the receiver or the timeout/retransmit maxima are reached.  In
        *   the current configuration, the max delay here is 60-70ms.
        *
        *   The maximum size of data written is the fixed payload size, see
        *   getPayloadSize().  However, you can write less, and the remainder
        *   will just be filled with zeros.
        *
        *   TX/RX/RT interrupt flags will be cleared every time write is called
        *
        *   @param buf Pointer to the data to be sent
        *   @param len Number of bytes to be sent
        *
        *   @code
        *   radio.stopListening();
        *   radio.write(&data,sizeof(data));
        *   @endcode
        *   @return True if the payload was delivered successfully false if not
        */
        bool write(const void *const buffer, size_t len);

        /**
        *   Write for single NOACK writes. Optionally disables acknowledgments/auto retries for a single write.
        *
        *   @note enableDynamicAck() must be called to enable this feature
        *
        *   Can be used with enableAckPayload() to request a response
        *   @see enableDynamicAck()
        *   @see setAutoAck()
        *   @see write()
        *
        *   @param buf Pointer to the data to be sent
        *   @param len Number of bytes to be sent
        *   @param multicast Request ACK (0), NOACK (1)
        */
        bool write(const void *const buffer, size_t len, const bool multicast);

        /**
        *   Open a pipe for writing via byte array. Old addressing format retained
        *   for compatibility. Only one writing pipe can be open at once, but you can change the address
        *   you'll write to. Call stopListening() first.
        *
        *   Addresses are assigned via a byte array, default is 5 byte address length
        *
        *   @param address The address of the pipe to open. Coordinate these pipe
        *   addresses amongst nodes on the network.
        */
        void openWritePipe(const uint8_t *const address);

        /**
        *   Open a pipe for reading
        *
        *   Up to 6 pipes can be open for reading at once.  Open all the required
        *   reading pipes, and then call startListening().
        *
        *   @see openWritingPipe
        *   @see setAddressWidth
        *
        *   @note Pipes 0 and 1 will store a full 5-byte address. Pipes 2-5 will technically
        *   only store a single byte, borrowing up to 4 additional bytes from pipe #1 per the
        *   assigned address width.
        *   @warning Pipes 1-5 should share the same address, except the first byte.
        *   Only the first byte in the array should be unique, e.g.
        *   @code
        *     uint8_t addresses[][6] = {"1Node","2Node"};
        *     openReadingPipe(1,addresses[0]);
        *     openReadingPipe(2,addresses[1]);
        *   @endcode
        *
        *   @warning Pipe 0 is also used by the writing pipe.  So if you open
        *   pipe 0 for reading, and then startListening(), it will overwrite the
        *   writing pipe.  Ergo, do an openWritingPipe() again before write().
        *
        *   @param number Which pipe# to open, 0-5.
        *   @param address The 24, 32 or 40 bit address of the pipe to open.
        */
        void openReadPipe(const uint8_t number, const uint8_t *const address);

        /**
        * TODO: ADD DOC
        *
        */
        bool isConnected();

        /**
        *   Check if the radio needs to be read. Can be used to prevent data loss
        *   @return True if all three 32-byte radio buffers are full
        */
        bool rxFifoFull();

        /**
        *   Leave low-power mode - required for normal radio operation after calling powerDown()
        *
        *   To return to low power mode, call powerDown().
        *   @note This will take up to 5ms for maximum compatibility
        */
        void powerUp();

        /**
        *   Enter low-power mode
        *
        *   To return to normal power mode, call powerUp().
        *
        *   @note After calling startListening(), a basic radio will consume about 13.5mA
        *   at max PA level.
        *   During active transmission, the radio will consume about 11.5mA, but this will
        *   be reduced to 26uA (.026mA) between sending.
        *   In full powerDown mode, the radio will consume approximately 900nA (.0009mA)
        *
        *   @code
        *   radio.powerDown();
        *   avr_enter_sleep_mode(); // Custom function to sleep the device
        *   radio.powerUp();
        *   @endcode
        */
        void powerDown();

        /**
        *   This will not block until the 3 FIFO buffers are filled with data.
        *   Once the FIFOs are full, writeFast will simply wait for success or
        *   timeout, and return 1 or 0 respectively. From a user perspective, just
        *   keep trying to send the same data. The library will keep auto retrying
        *   the current payload using the built in functionality.
        *   @warning It is important to never keep the nRF24L01 in TX mode and FIFO full for more than 4ms at a time. If the auto
        *   retransmit is enabled, the nRF24L01 is never in TX mode long enough to disobey this rule. Allow the FIFO
        *   to clear by issuing txStandBy() or ensure appropriate time between transmissions.
        *
        *   @code
        *   Example (Partial blocking):
        *
        *   			radio.writeFast(&buf,32);  // Writes 1 payload to the buffers
        *   			txStandBy();     		   // Returns 0 if failed. 1 if success. Blocks only until MAX_RT timeout or success. Data flushed on fail.
        *
        *   			radio.writeFast(&buf,32);  // Writes 1 payload to the buffers
        *   			txStandBy(1000);		   // Using extended timeouts, returns 1 if success. Retries failed payloads for 1 seconds before returning 0.
        *   @endcode
        *
        *   @see txStandBy()
        *   @see write()
        *   @see writeBlocking()
        *
        *   @param buf Pointer to the data to be sent
        *   @param len Number of bytes to be sent
        *   @return True if the payload was delivered successfully false if not
        */
        bool writeFast(const void *const buffer, size_t len);

        /**
        *       WriteFast for single NOACK writes. Disables acknowledgments/auto retries for a single write.
        *
        *       @note enableDynamicAck() must be called to enable this feature
        *       @see enableDynamicAck()
        *       @see setAutoAck()
        *
        *       @param buf Pointer to the data to be sent
        *       @param len Number of bytes to be sent
        *       @param multicast Request ACK (0) or NOACK (1)
        */
        bool writeFast(const void *buf, size_t len, const bool multicast);

        /**
        *   This function extends the auto-retry mechanism to any specified duration.
        *   It will not block until the 3 FIFO buffers are filled with data.
        *   If so the library will auto retry until a new payload is written
        *   or the user specified timeout period is reached.
        *   @warning It is important to never keep the nRF24L01 in TX mode and FIFO full for more than 4ms at a time. If the auto
        *   retransmit is enabled, the nRF24L01 is never in TX mode long enough to disobey this rule. Allow the FIFO
        *   to clear by issuing txStandBy() or ensure appropriate time between transmissions.
        *
        *   @code
        *   Example (Full blocking):
        *
        *   			radio.writeBlocking(&buf,32,1000); //Wait up to 1 second to write 1 payload to the buffers
        *   			txStandBy(1000);     			   //Wait up to 1 second for the payload to send. Return 1 if ok, 0 if failed.
        *   					  				   		   //Blocks only until user timeout or success. Data flushed on fail.
        *   @endcode
        *   @note If used from within an interrupt, the interrupt should be disabled until completion, and sei(); called to enable millis().
        *   @see txStandBy()
        *   @see write()
        *   @see writeFast()
        *
        *   @param buf Pointer to the data to be sent
        *   @param len Number of bytes to be sent
        *   @param timeout User defined timeout in milliseconds.
        *   @return True if the payload was loaded into the buffer successfully false if not
        */
        bool writeBlocking(const void *const buffer, size_t len, uint32_t timeout);

        /**
        *   Non-blocking write to the open writing pipe used for buffered writes
        *
        *   @note Optimization: This function now leaves the CE pin high, so the radio
        *   will remain in TX or STANDBY-II Mode until a txStandBy() command is issued. Can be used as an alternative to startWrite()
        *   if writing multiple payloads at once.
        *
        *   @warning It is important to never keep the nRF24L01 in TX mode with FIFO full for more than 4ms at a time. If the auto
        *   retransmit/autoAck is enabled, the nRF24L01 is never in TX mode long enough to disobey this rule. Allow the FIFO
        *   to clear by issuing txStandBy() or ensure appropriate time between transmissions.
        *
        *   @see write()
        *   @see writeFast()
        *   @see startWrite()
        *   @see writeBlocking()
        *
        *   For single NOACK writes see:
        *   @see enableDynamicAck()
        *   @see setAutoAck()
        *
        *   @param[in] buffer       Pointer to the data to be sent
        *   @param[in] len          Number of bytes to be sent
        *   @param[in] multicast    Request ACK (false) or NOACK (true)
        *   @param[in] startTX      Starts the transfer immediately if true
        *   @return True if the payload was delivered successfully false if not
        */
        void startFastWrite(const void *const buffer, size_t len, const bool multicast, const bool startTx = true) ;

        /**
        *   Non-blocking write to the open writing pipe
        *
        *   Just like write(), but it returns immediately. To find out what happened
        *   to the send, catch the IRQ and then call whatHappened().
        *
        *   @see write()
        *   @see writeFast()
        *   @see startFastWrite()
        *   @see whatHappened()
        *
        *   For single noAck writes see:
        *   @see enableDynamicAck()
        *   @see setAutoAck()
        *
        *   @param buf Pointer to the data to be sent
        *   @param len Number of bytes to be sent
        *   @param multicast Request ACK (0) or NOACK (1)
        */
        void startWrite(const void *const buffer, size_t len, const bool multicast);

        /**
        *   This function should be called as soon as transmission is finished to
        *   drop the radio back to STANDBY-I mode. If not issued, the radio will
        *   remain in STANDBY-II mode which, per the data sheet, is not a recommended
        *   operating mode.
        *
        *   @note When transmitting data in rapid succession, it is still recommended by
        *   the manufacturer to drop the radio out of TX or STANDBY-II mode if there is
        *   time enough between sends for the FIFOs to empty. This is not required if auto-ACK
        *   is enabled.
        *
        *   Relies on built-in auto retry functionality.
        *
        *   @code
        *   Example (Partial blocking):
        *
        *   			radio.writeFast(&buf,32);
        *   			radio.writeFast(&buf,32);
        *   			radio.writeFast(&buf,32);  //Fills the FIFO buffers up
        *   			bool ok = txStandBy();     //Returns 0 if failed. 1 if success.
        *   					  				   //Blocks only until MAX_RT timeout or success. Data flushed on fail.
        *   @endcode
        *   @see txStandBy(unsigned long timeout)
        *   @return True if transmission is successful
        */
        bool txStandBy();

        /**
        *   This function allows extended blocking and auto-retries per a user defined timeout
        *   @code
        *   	Fully Blocking Example:
        *
        *   			radio.writeFast(&buf,32);
        *   			radio.writeFast(&buf,32);
        *   			radio.writeFast(&buf,32);   //Fills the FIFO buffers up
        *   			bool ok = txStandBy(1000);  //Returns 0 if failed after 1 second of retries. 1 if success.
        *   					  				    //Blocks only until user defined timeout or success. Data flushed on fail.
        *   @endcode
        *   @note If used from within an interrupt, the interrupt should be disabled until completion, and sei(); called to enable millis().
        *   @param timeout Number of milliseconds to retry failed payloads
        *   @return True if transmission is successful
        */
        bool txStandBy(const uint32_t timeout, const bool startTx = false);

        /**
        *   Write an ACK payload for the specified pipe
        *
        *   The next time a message is received on @p pipe, the data in @p buf will
        *   be sent back in the acknowledgment.
        *   @see enableAckPayload()
        *   @see enableDynamicPayloads()
        *   @warning Only three of these can be pending at any time as there are only 3 FIFO buffers.<br> Dynamic payloads must be enabled.
        *   @note ACK payloads are handled automatically by the radio chip when a payload is received. Users should generally
        *   write an ACK payload as soon as startListening() is called, so one is available when a regular payload is received.
        *   @note ACK payloads are dynamic payloads. This only works on pipes 0&1 by default. Call
        *   enableDynamicPayloads() to enable on all pipes.
        *
        *   @param pipe Which pipe# (typically 1-5) will get this response.
        *   @param buf Pointer to data that is sent
        *   @param len Length of the data to send, up to 32 bytes max.  Not affected
        *   by the static payload set by setPayloadSize().
        */
        void writeAckPayload(const uint8_t pipe, const void *const buffer, size_t len);

        /**
        *   Determine if an ACK payload was received in the most recent call to
        *   write(). The regular available() can also be used.
        *
        *   Call read() to retrieve the ACK payload.
        *
        *   @return True if an ACK payload is available.
        */
        bool isAckPayloadAvailable();

        /**
        *   Call this when you get an interrupt to find out why
        *
        *   Tells you what caused the interrupt, and clears the state of
        *   interrupts.
        *
        *   @param[out] tx_ok The send was successful (TX_DS)
        *   @param[out] tx_fail The send failed, too many retries (MAX_RT)
        *   @param[out] rx_ready There is a message waiting to be read (RX_DS)
        */
        void whatHappened(bool &tx_ok, bool &tx_fail, bool &rx_ready);

        /**
        *   This function is mainly used internally to take advantage of the auto payload
        *   re-use functionality of the chip, but can be beneficial to users as well.
        *
        *   The function will instruct the radio to re-use the data in the FIFO buffers,
        *   and instructs the radio to re-send once the timeout limit has been reached.
        *   Used by writeFast and writeBlocking to initiate retries when a TX failure
        *   occurs. Retries are automatically initiated except with the standard write().
        *   This way, data is not flushed from the buffer until switching between modes.
        *
        *   @note This is to be used AFTER auto-retry fails if wanting to resend
        *   using the built-in payload reuse features.
        *   After issuing reUseTX(), it will keep reending the same payload forever or until
        *   a payload is written to the FIFO, or a flush_tx command is given.
        */
        void reUseTX();

        /**
        *   Test whether there was a carrier on the line for the
        *   previous listening period.
        *
        *   Useful to check for interference on the current channel.
        *
        *   @return true if was carrier, false if not
        */
        bool testCarrier();

        /**
        *   Close a pipe after it has been previously opened.
        *   Can be safely called without having previously opened a pipe.
        *   @param pipe Which pipe # to close, 0-5.
        */
        void closeReadingPipe(const uint8_t pipe);


        /* Optional reconfiguration */

        /**
        *   Set the address width from 3 to 5 bytes (24, 32 or 40 bit)
        *
        *   @param a_width The address width to use: 3,4 or 5
        */
        void setAddressWidth(const uint8_t address_width);

        /**
        *   Set the number and delay of retries upon failed submit
        *
        *   @param delay How long to wait between each retry, in multiples of 250us,
        *   max is 15.  0 means 250us, 15 means 4000us.
        *   @param count How many retries before giving up, max 15
        */
        void setRetries(const uint8_t delay, const uint8_t count);

        /**
        * Set RF communication channel
        *
        * @param channel Which RF channel to communicate on, 0-125
        */
        void setChannel(const uint8_t channel);

        /**
        *   Set Static Payload Size
        *
        *   This implementation uses a pre-established fixed payload size for all
        *   transmissions.  If this method is never called, the driver will always
        *   transmit the maximum payload size (32 bytes), no matter how much
        *   was sent to write().
        *
        *   @todo Implement variable-sized payloads feature
        *
        *   @param size The number of bytes in the payload
        */
        void setPayloadSize(const uint8_t size);

        /**
        *   Get RF communication channel
        *
        *   @return The currently configured RF Channel
        */
        uint8_t getChannel();

        /**
        *   Get Static Payload Size
        *
        *   @see setPayloadSize()
        *
        *   @return The number of bytes in the payload
        */
        uint8_t getPayloadSize();

        /**
        *   Get Dynamic Payload Size
        *
        *   For dynamic payloads, this pulls the size of the payload off
        *   the chip
        *
        *   @note Corrupt packets are now detected and flushed per the
        *   manufacturer.
        *   @code
        *   if(radio.available()){
        *     if(radio.getDynamicPayloadSize() < 1){
        *       // Corrupt payload has been flushed
        *       return;
        *     }
        *     radio.read(&data,sizeof(data));
        *   }
        *   @endcode
        *
        *   @return Payload length of last-received dynamic payload
        */
        uint8_t getDynamicPayloadSize();

        /**
        *   Empty the transmit buffer. This is generally not required in standard operation.
        *   May be required in specific cases after stopListening() , if operating at 250KBPS data rate.
        *
        *   @return Current value of status register
        */
        uint8_t flush_tx();

        /**
        * TODO: ADD DOC
        *
        */
        uint8_t flush_rx();

        /**
        *   Enable custom payloads on the acknowledge packets
        *
        *   ACK payloads are a handy way to return data back to senders without
        *   manually changing the radio modes on both units.
        *
        *   @note ACK payloads are dynamic payloads. This only works on pipes 0&1 by default. Call
        *   enableDynamicPayloads() to enable on all pipes.
        */
        void enableAckPayload();

        /**
        *   Enable dynamically-sized payloads
        *
        *   This way you don't always have to send large packets just to send them
        *   once in a while.  This enables dynamic payloads on ALL pipes.
        */
        void enableDynamicPayloads();

        /**
        *   Disable dynamically-sized payloads
        *
        *   This disables dynamic payloads on ALL pipes. Since ACK Payloads
        *   requires Dynamic Payloads, ACK Payloads are also disabled.
        *   If dynamic payloads are later re-enabled and ACK payloads are desired
        *   then enableAckPayload() must be called again as well.
        */
        void disableDynamicPayloads();

        /**
        *   Enable dynamic ACKs (single write multicast or unicast) for chosen messages
        *
        *   @note To enable full multicast or per-pipe multicast, use setAutoAck()
        *
        *   @warning This MUST be called prior to attempting single write NOACK calls
        *   @code
        *   radio.enableDynamicAck();
        *   radio.write(&data,32,1);  // Sends a payload with no acknowledgement requested
        *   radio.write(&data,32,0);  // Sends a payload using auto-retry/autoACK
        *   @endcode
        */
        void enableDynamicAck();

        /**
        * TODO: ADD DOC
        *
        */
        void disableDynamicAck();

        /**
        *   Determine whether the hardware is an nRF24L01+ or not.
        *
        *   @return true if the hardware is nRF24L01+ (or compatible) and false
        *   if its not.
        */
        bool isPVariant();

        /**
        *   Enable or disable auto-acknowledge packets
        *
        *   This is enabled by default, so it's only needed if you want to turn
        *   it off for some reason.
        *
        *   @param enable Whether to enable (true) or disable (false) auto-ACKs
        */
        void setAutoAck(const bool enable);

        /**
        *   Enable or disable auto-acknowledge packets on a per pipeline basis.
        *
        *   AA is enabled by default, so it's only needed if you want to turn
        *   it off/on for some reason on a per pipeline basis.
        *
        *   @param pipe Which pipeline to modify
        *   @param enable Whether to enable (true) or disable (false) auto-ACKs
        */
        void setAutoAck(const uint8_t pipe, const bool enable);

        /**
        *   Set Power Amplifier (PA) level to one of four levels:
        *   RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
        *
        *   The power levels correspond to the following output levels respectively:
        *   NRF24L01: -18dBm, -12dBm,-6dBM, and 0dBm
        *
        *   SI24R1: -6dBm, 0dBm, 3dBM, and 7dBm.
        *
        *   @param level Desired PA level.
        */
        void setPALevel(const PowerAmplitude level);

        /**
        *   Fetches the current PA level.
        *
        *   NRF24L01: -18dBm, -12dBm, -6dBm and 0dBm
        *   SI24R1:   -6dBm, 0dBm, 3dBm, 7dBm
        *
        *   @return Returns values 0 to 3 representing the PA Level.
        */
        PowerAmplitude getPALevel();

        /**
        *   Set the transmission data rate
        *
        *   @warning setting RF24_250KBPS will fail for non-plus units
        *
        *   @param speed RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
        *   @return true if the change was successful
        */
        bool setDataRate(const DataRate speed);

        /**
        *   Fetches the transmission data rate
        *
        *   @return Returns the hardware's currently configured data rate. The value
        *   is one of 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS, as defined in the
        *   nrf24_datarate enum.
        */
        DataRate getDataRate();

        /**
        *   Set the CRC length
        *   <br>CRC checking cannot be disabled if auto-ACK is enabled
        *   @param length RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
        */
        void setCRCLength(const CRCLength length);

        /**
        *   Get the CRC length
        *   <br>CRC checking cannot be disabled if auto-ACK is enabled
        *   @return RF24_CRC_DISABLED if disabled or RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
        */
        CRCLength getCRCLength();

        /**
        *   Disable CRC validation
        *
        *   @warning CRC cannot be disabled if auto-ACK/ESB is enabled.
        */
        void disableCRC();

        /**
        *   The radio will generate interrupt signals when a transmission is complete,
        *   a transmission fails, or a payload is received. This allows users to mask
        *   those interrupts to prevent them from generating a signal on the interrupt
        *   pin. Interrupts are enabled on the radio chip by default.
        *
        *   @code
        *   	Mask all interrupts except the receive interrupt:
        *
        *   		radio.maskIRQ(1,1,0);
        *   @endcode
        *
        *   @param tx_ok  Mask transmission complete interrupts
        *   @param tx_fail  Mask transmit failure interrupts
        *   @param rx_ready Mask payload received interrupts
        */
        void maskIRQ(const bool tx_ok, const bool tx_fail, const bool rx_ready);

        /**
        *   Read a chunk of data in from a register
        *
        *   @param reg Which register. Use constants from nRF24L01.h
        *   @param buf Where to put the data
        *   @param len How many bytes of data to transfer
        *   @return Current value of status register
        */
        uint8_t read_register(const uint8_t reg, uint8_t *const buffer, size_t len);

        /**
        *   Read single byte from a register
        *
        *   @param reg Which register. Use constants from nRF24L01.h
        *   @return Current value of register @p reg
        */
        uint8_t read_register(const uint8_t reg);

        /**
        *   Write a chunk of data to a register
        *
        *   @param reg Which register. Use constants from nRF24L01.h
        *   @param buf Where to get the data
        *   @param len How many bytes of data to transfer
        *   @return Current value of status register
        */
        uint8_t write_register(const uint8_t reg, const uint8_t *const buffer, size_t len);

        /**
        *   Write a single byte to a register
        *
        *   @param reg Which register. Use constants from nRF24L01.h
        *   @param value The new value to write
        *   @return Current value of status register
        */
        uint8_t write_register(const uint8_t reg, const uint8_t value);

        /**
        *   Write the transmit payload
        *
        *   The size of data written is the fixed payload size, see getPayloadSize()
        *
        *   @param buf Where to get the data
        *   @param len Number of bytes to be sent
        *   @return Current value of status register
        */
        uint8_t write_payload(const void *const buffer, size_t len, const uint8_t writeType);

        /**
        *   Read the receive payload
        *
        *   The size of data read is the fixed payload size, see getPayloadSize()
        *
        *   @param buf Where to put the data
        *   @param len Maximum number of bytes to read
        *   @return Current value of status register
        */
        uint8_t read_payload(void *const buffer, size_t len);

        /**
        *   Retrieve the current status of the chip
        *
        *   @return Current value of status register
        */
        uint8_t get_status();

     protected:

        /** User defined function that will perform an SPI write/read. This must
        *   be overwritten otherwise the program will not compile.
        *
        *   @param[in]  tx_buffer   Data buffer from which to transmit
        *   @param[in]  len         The number of bytes to write
        *
        *   @return The total number of bytes that were written
        */
        virtual size_t spi_write(const uint8_t *const tx_buffer, size_t len);

        /** User defined function that will perform an SPI read. This must
        *   be overwritten otherwise the program will not compile.
        *
        *   @param[in]  rx_buffer   Data buffer to read information into
        *   @param[in]  len         The number of bytes to read
        *
        *   @return The total number of bytes read.
        */
        virtual size_t spi_read(uint8_t *const rx_buffer, size_t len);

        /** User defined function that will perform an SPI write/read. This must
        *   be overwritten otherwise the program will not compile.
        *
        *   @param[in]  tx_buffer   Data buffer from which to transmit
        *   @param[in]  rx_buffer   Data buffer to read information into
        *   @param[in]  len         The number of bytes to write/read
        *
        *   @return The total number of bytes that were written/read
        */
        virtual size_t spi_write_read(const uint8_t *const tx_buffer, uint8_t *const rx_buffer, size_t len);

        /** User defined function that will start the SPI transaction correctly. This
        *   typically means asserting the chip select line either in software or hardware.
        *
        *   @return void
        */
        virtual void begin_transaction();

        /** User defined function that will end the SPI transaction correctly. This
        *   typically means deasserting the chip select line either in software or hardware.
        *
        *   @return void
        */
        virtual void end_transaction();

    private:
        uint8_t write_cmd(const uint8_t cmd);

        bool registerBitmaskSet(const uint8_t reg, const uint8_t bitmask);
        bool registerAnySet(const uint8_t reg, const uint8_t bitmask);
        void clearRegisterBits(const uint8_t reg, const uint8_t bitmask);
        void setRegisterBits(const uint8_t reg, const uint8_t bitmask);

        bool pVariant = false;
        bool dynamic_payloads_enabled = false;
        size_t addr_width = 0;
        size_t payload_size = 0;
        uint8_t pipe0_reading_address[5];

        Chimera::SPI::SPIClass_sPtr spi;
        Chimera::GPIO::GPIOClass_sPtr chipEnable;

        std::array<uint8_t, SPI_BUFFER_LEN> spi_txbuff;
        std::array<uint8_t, SPI_BUFFER_LEN> spi_rxbuff;
    };

}

#endif /* NRF24L01_HPP */
