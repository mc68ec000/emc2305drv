/***************************************************************************//**
* @file
*  
*     This file contains class definition for II2c.
*
* @note     
*     None.
*
* @warning  
*     None.
*******************************************************************************/


#ifndef  II2C_HPP
#define  II2C_HPP

#include <stdint.h>


/***************************************************************************//**
* @class II2c 
*     
*     This is the I2c interface class. All I2c objects must adhere to this 
*     interface. Component drivers use this interface to communicate across
*     I2c bus.
*
* @note
*     None.
*
* @warning
*     None.
*******************************************************************************/
class II2c
{
public:

   // Return I2C clock rate
   virtual uint32_t  GetClockRate( ) const = 0;

   // read/write with no subaddress (or pre-ambled in data buffer)
   virtual bool      Write( const void *data, uint32_t numBytes ) = 0;
   virtual bool      Read( void *data, uint32_t numBytes ) = 0;

   // Write/read specific to different size of subaddress
   virtual bool      Write( uint8_t startOffset, const void *data, uint32_t numBytes ) = 0;
   virtual bool      Read( uint8_t startOffset, void *data, uint32_t numBytes ) = 0;
   virtual bool      Write( uint16_t startOffset, const void *data, uint32_t numBytes ) = 0;
   virtual bool      Read( uint16_t startOffset, void *data, uint32_t numBytes ) = 0;
   virtual bool      Write24( uint32_t startOffset, const void *data, uint32_t numBytes ) = 0;
   virtual bool      Read24( uint32_t startOffset, void *data, uint32_t numBytes ) = 0;
   virtual bool      Write( uint32_t startOffset, const void *data, uint32_t numBytes ) = 0;
   virtual bool      Read( uint32_t startOffset, void *data, uint32_t numBytes ) = 0;
   
protected:      

   virtual ~II2c( ) { }
};

#endif   // II2C_HPP
