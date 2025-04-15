/*********************************************************************************
Generic Driver Code for the Microchip EMC2305 RPM-based PWM fan controller.
I2C communication is performed using a custom I2C library.

Pg 12 of datasheet: The SMBus/I2C address is set at 0101_111(r/w)b (aka 47 or 0x2F)

Copyright (C) 2023 Zhao, Alex

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*********************************************************************************/

#ifndef _EMC2301_h
#define _EMC2301_h

#include <stdint.h>
#include "i2c.hpp"
#include "Mutex.hpp"

#define NUM_OF_FAN_CHANNELS   (5)

class II2c;

class EMC2305
{
public:
   
   EMC2305( II2c * pI2c );
   ~EMC2305();
   
   bool IsPresent( );
   bool GetRevision( uint8_t &rev );
   bool GetSmbAddr( uint8_t &addr );
   
   bool GetFanSpeed( uint8_t chan, uint16_t &speed );
   
   bool SetPwmFrequencyBase( double frequencyKHz );
   bool SetPwmFrequencyDivider( uint8_t divisor );    
   
   bool SetClockMode( bool extClk );
   bool GetClockMode( bool &extClk );
   
   bool SetPwmOutMode( uint8_t chan, bool pushPull );
   bool GetPwmOutMode( uint8_t chan, bool &pushPUll );

   bool SetFanPwm( uint8_t chan, uint8_t duty );
   bool GetFanPwm( uint8_t chan, uint8_t &duty );
   
   bool SetSpeedControl( uint8_t chan, bool enable );
   bool GetSpeedControl( uint8_t chan, bool &enable );
   
   bool SetRpmTarget( uint8_t chan, uint16_t targetRpm );
   bool GetRpmTarget( uint8_t chan, uint16_t &targetRpm );
   
   bool SetFanMinDrive( uint8_t chan, uint8_t minDrivePercent );
   bool GetFanMinDrive( uint8_t chan, uint8_t &minDrivePercent );

   bool SetFanPoles( uint8_t chan, uint8_t poleCount );
   bool GetFanPoles( uint8_t chan, uint8_t &poleCount );
   
   bool SetTachMinRpm( uint8_t chan, uint16_t minRpm );
   bool GetTachMinRpm( uint8_t chan, uint16_t &minRpm );
   
   bool SetMinValidRpm( uint8_t chan, uint16_t minRpm );
   bool GetMinValidRpm( uint8_t chan, uint16_t &minRpm );
   
   bool ToggleFan( uint8_t chan, bool enable);

   bool SetDriveUpdatePeriod( uint8_t chan, uint16_t periodMs );
   bool ToggleRampControl( uint8_t chan, bool enable );
   bool ToggleGlitchFilter( uint8_t chan, bool enable );
   bool SetDerivativeMode( uint8_t chan, uint8_t modeType );
   bool SetControlErrRange( uint8_t chan, uint8_t errorRangeRpm );
   bool ToggleSpinUpMax( uint8_t chan, bool enable );
   bool SetSpinUpDrive( uint8_t chan, uint8_t drivePercent );
   bool SetSpinUpTime( uint8_t chan, uint16_t timeMs );
   bool SetControlMaxStep( uint8_t chan, uint8_t stepSize );
   
   bool ReadRegister( uint8_t regAddr, uint8_t &regVal );
   bool WriteRegister( uint8_t regAddr, uint8_t regVal );
   bool DumpRegisters( uint8_t regAddr, uint8_t * dataPtr );

protected:
   
   inline void Lock( void )      { m_Mutex.Lock( );      }
   inline void Unlock( void )    { m_Mutex.Unlock( );    }
   
private:
   
   CMutex      m_Mutex;                         //!< Mutual exclusion protection mutex.        
   II2c*       m_pI2c;

   bool WriteRegisterBits( uint8_t registerAddress, uint8_t clearingMask, uint8_t byteToWrite );
   
   bool WriteTachoTarget( uint8_t chan, uint16_t tachoTarget );
   bool ReadTachoTarget( uint8_t chan, uint16_t &tachoTarget );
  
   bool FetchFanSpeed( uint8_t chan );
};


#endif

