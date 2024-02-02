#ifndef __I2CEEPROM_H__
#define __I2CEEPROM_H__

//#define DEBUG_I2CEEPROM_READ
//#define DEBUG_I2CEEPROM_WRITE

#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include <commons.h>

#ifndef MIN
 #define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#endif


class i2cEeprom
{
    // fields
    private:
        TwoWire& _wire;
        static const uint8_t _maxWireBuf = 30;
        const uint8_t _addr;
        const uint16_t _pageSize;
        const uint16_t _pages;  // we don't actually need the number of pages

    // methods
    public:
        /// @brief Initialise a new instance of the \ref i2cEeprom class with the 
        /// specified I2C address and bus instance.
        /// @param pageSize The page size in bytes.
        /// @param addr The I2C address.
        /// @param wire The I2C bus instance.
        i2cEeprom(uint16_t pageSize, uint16_t pages, uint8_t addr=0x50, TwoWire& wire=Wire) : 
            _pageSize(pageSize), _pages(pages), _addr(addr), _wire(wire)
        {
            strprintf(Serial, "Base: %02X Page size: %4d Pages: %3d\n", _addr, _pageSize, _pages);            
        }

        /// @brief Writes the supplied data value/structure to the specified address.
        /// @tparam The type of data value.
        /// @param addr The address to write to.
        /// @param val The value.
        /// @return True if successful; otherwise false.
        template <typename T>
        bool write(uint32_t addr, T& val)
        {
            uint8_t* ptr = (uint8_t*) &val;
            uint16_t size = sizeof(T);
            #ifdef DEBUG_I2CEEPROM_WRITE
            strprintf(Serial, "write: writing %d bytes from E2PROM\n", size);
            #endif
            while (size > 0)
            {
                // we can only write upto the page boundary in a single write
                // which means we need to know where the start addr lies in 
                // relation to the page
                uint16_t pageSpace = _pageSize - (addr % _pageSize);
                uint8_t len = MIN(size, pageSpace);
                len = MIN(len, _maxWireBuf);
                #ifdef DEBUG_I2CEEPROM_WRITE
                strprintf(Serial, "write: address:%ld pageSpace: %d len: %d\n", addr, pageSpace, len);
                #endif

                // To write to the device we need to send three bytes:
                // the first is the I2C addr with the bits 18:16 of the address
                // the next two are bits 15:8 and then 7:0.  We can then write the
                // rest of the number of bytes we want
                // TODO: mask the unused bits in the address
                uint8_t control = _addr | (uint8_t)(addr >> 16);
                _wire.beginTransmission(control);
                _wire.write((uint8_t) (addr >> 8));
                _wire.write((uint8_t) addr);

                for (uint8_t u = 0; u < len; u++)
                    _wire.write(*ptr++);
                //_wire.write(ptr, len);
                _wire.endTransmission();

                // bump pointers and counters
                size -= len;
                addr += len;
                ptr += len;

                while (isBusy(control))
                    ;
            }
            return true;
        }

        /// @brief Reads the data from the specified address.
        /// @tparam The type of data value.
        /// @param addr The address to read from.
        /// @param val The value.
        /// @return True if successful; otherwise false.
        template <typename T>
        bool read(uint32_t addr, T& val)
        {
            uint8_t* ptr = (uint8_t*) &val;
            uint16_t size = sizeof(T);
            #ifdef DEBUG_I2CEEPROM_READ
            strprintf(Serial, "read: reading %d bytes from E2PROM\n", size);
            #endif
            while (size > 0)
            {
                // we can only write upto the page boundary in a single write
                // which means we need to know where the start addr lies in 
                // relation to the page
                uint16_t pageSpace = _pageSize - (addr % _pageSize);
                uint8_t len = MIN(size, pageSpace);
                len = MIN(len, _maxWireBuf);
                #ifdef DEBUG_I2CEEPROM_READ
                strprintf(Serial, "read: address:%ld pageSpace: %d len: %d\n", addr, pageSpace, len);
                #endif

                // To read from the device we need to send three bytes:
                // the first is the I2C addr with the bits 18:16 of the address
                // the next two are bits 15:8 and then 7:0.  We can then turn the
                // line around and read the number of bytes we want
                // TODO: mask the unused bits in the address
                uint8_t control = _addr | (uint8_t)(addr >> 16);
                _wire.beginTransmission(control);
                _wire.write((uint8_t) (addr >> 8));
                _wire.write((uint8_t) addr);
                _wire.endTransmission(false);

                _wire.requestFrom(control, len);
                while (0 == _wire.available());
                *ptr++ = _wire.read();

                // bump pointers and counters
                size -= len;
                addr += len;
            }
            return true;
        }
        
    protected:
    private:
        bool isConnected(uint8_t b)
        {
            _wire.beginTransmission(b);
            return (_wire.endTransmission() == 0);
        }

        inline
        bool isBusy(uint8_t b)
        {
            return !isConnected(b);
        }
};

#endif //__I2CEEPROM_H__