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

#include "EMC2305.hpp"
#include "Mutex.hpp"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

namespace
{   
   // External clock is preferenced for better accuracy
   static const uint16_t TACHO_CLOCK_FREQUENCY_IN_HZ     = 32768;
   
   static const uint16_t NUM_OF_SECONDS_PER_MINUTE       = 60;
   static const uint16_t EMC2305_INVALID_RPM_OR_TACH     = 65535;
   static const uint16_t EMC2305_MIN_RPM_VALUE           = 500;
   static const uint16_t EMC2305_MAX_RPM_VALUE           = 25000;
   static const uint8_t  EMC2305_MIN_POLE_COUNT          = 1;
   static const uint8_t  EMC2305_MAX_POLE_COUNT          = 4;
   
   // 2-byte to be written to tacho target register to turn off fan
   static const uint16_t EMC2305_TACHO_TARGET_OFF        = 0x1FFF << 3; // 1111 1111 1111 1000
   
   // Register file offset for each channel
   static const uint8_t EMC2303_REG_OFFSET_PER_CHAN      = 0x10;
   
   /******************************
   *     List of registers      *
   ******************************/
   static const uint8_t EMC2305_REG_ADDR_LOWER_LIMIT     = 0x20;
   
   static const uint8_t EMC2305_REG_CONFIGURATION        = 0x20;
   static const uint8_t EMC2305_REG_PWMOUTPUTCONFIG      = 0x2B;
   static const uint8_t EMC2305_REG_PWMBASEFREQ          = 0x2D;
   
   static const uint8_t EMC2305_REG_FANSETTING           = 0x30;
   static const uint8_t EMC2305_REG_PWMDIVIDE            = 0x31;
   static const uint8_t EMC2305_REG_FANCONFIG1           = 0x32;
   static const uint8_t EMC2305_REG_FANCONFIG2           = 0x33;
   static const uint8_t EMC2305_REG_FANSPINUP            = 0x36;
   static const uint8_t EMC2305_REG_FANMAXSTEP           = 0x37;
   static const uint8_t EMC2305_REG_FANMINDRIVE          = 0x38;
   static const uint8_t EMC2305_REG_FANVALTACHCOUNT      = 0x39;
   static const uint8_t EMC2305_REG_TACHTARGETLSB        = 0x3C;
   static const uint8_t EMC2305_REG_TACHTARGETMSB        = 0x3D;
   static const uint8_t EMC2305_REG_TACHREADMSB          = 0x3E;
   static const uint8_t EMC2305_REG_TACHREADLSB          = 0x3F;
   
   static const uint8_t EMC2305_REG_PRODUCT_FEATURES     = 0xFC;
   static const uint8_t EMC2305_REG_PRODUCT_ID           = 0xFD;
   static const uint8_t EMC2305_REG_MANUFACTURER_ID      = 0xFE;
   static const uint8_t EMC2305_REG_REVISION             = 0xFF;   
   
   /******************************
   *  Register Bits Definitions  *
   ******************************/   
   static const uint8_t EMC2305_REG_SMBUS_ADDR_BITS      = 0x03;
   static const uint8_t EMC2305_REG_SMBUS_ADDR_MASK      = 0x38;
   static const uint8_t EMC2305_REG_PRODUCT_VALUE        = 0x34;
   static const uint8_t EMC2305_REG_MANUFACTURER_VALUE   = 0x5D;
   
   static const uint8_t EMC2305_REG_CONFIG_USE_EXT_CLK   = 0x01;
   
   /* Registers that can have values written directly into them (i.e. the entire register is meant for a single number):
   EMC2305_REG_FANSETTING
   EMC2305_REG_PWMDIVIDE
   EMC2305_REG_FANMAXSTEP (Max 0b00111111 or 0x3F or 63)
   EMC2305_REG_FANMINDRIVE
   EMC2305_REG_FANVALTACHCOUNT (The final value from this register is 32 x (value in register))
   EMC2305_REG_TACHTARGETLSB and EMC2305_REG_TACHTARGETMSB (MUST write both LSB and MSB, with LSB written before MSB)
   */
   
   // EMC2305_REG_PWMBASEFREQ
   static const uint8_t EMC2305_REG_PWMBASEFREQ_26KHZ           = 0x00;
   static const uint8_t EMC2305_REG_PWMBASEFREQ_19KHZ           = 0x01;
   static const uint8_t EMC2305_REG_PWMBASEFREQ_4KHZ            = 0x02;
   static const uint8_t EMC2305_REG_PWMBASEFREQ_2KHZ            = 0x03;
   
   // EMC2305_REG_FANCONFIG1
   static const uint8_t EMC2305_REG_FANCONFIG1_RPMCONTROL       = 0x80;
   static const uint8_t EMC2305_REG_FANCONFIG1_MINRPM_CLEAR     = ~0x60;
   static const uint8_t EMC2305_REG_FANCONFIG1_MINRPM_500       = 0x00;
   static const uint8_t EMC2305_REG_FANCONFIG1_MINRPM_1000      = 0x20;
   static const uint8_t EMC2305_REG_FANCONFIG1_MINRPM_2000      = 0x40;
   static const uint8_t EMC2305_REG_FANCONFIG1_MINRPM_4000      = 0x60;
   static const uint8_t EMC2305_REG_FANCONFIG1_FANPOLES_MASK   = 0x18;
   static const uint8_t EMC2305_REG_FANCONFIG1_FANPOLES_CLEAR   = ~0x18;
   static const uint8_t EMC2305_REG_FANCONFIG1_FANPOLES_1       = 0x00;
   static const uint8_t EMC2305_REG_FANCONFIG1_FANPOLES_2       = 0x08;
   static const uint8_t EMC2305_REG_FANCONFIG1_FANPOLES_3       = 0x10;
   static const uint8_t EMC2305_REG_FANCONFIG1_FANPOLES_4       = 0x18;
   static const uint8_t EMC2305_REG_FANCONFIG1_UPDATE_CLEAR     = ~0x07;
   static const uint8_t EMC2305_REG_FANCONFIG1_UPDATE_100       = 0x00;
   static const uint8_t EMC2305_REG_FANCONFIG1_UPDATE_200       = 0x01;
   static const uint8_t EMC2305_REG_FANCONFIG1_UPDATE_300       = 0x02;
   static const uint8_t EMC2305_REG_FANCONFIG1_UPDATE_400       = 0x03;
   static const uint8_t EMC2305_REG_FANCONFIG1_UPDATE_500       = 0x04;
   static const uint8_t EMC2305_REG_FANCONFIG1_UPDATE_800       = 0x05;
   static const uint8_t EMC2305_REG_FANCONFIG1_UPDATE_1200      = 0x06;
   static const uint8_t EMC2305_REG_FANCONFIG1_UPDATE_1600      = 0x07;
   
   // EMC2305_REG_FANCONFIG2
   static const uint8_t EMC2305_REG_FANCONFIG2_RAMPCONTROL      = 0x40;
   static const uint8_t EMC2305_REG_FANCONFIG2_GLITCHFILTER     = 0x20;
   static const uint8_t EMC2305_REG_FANCONFIG2_DEROPT_CLEAR     = ~0x18;
   static const uint8_t EMC2305_REG_FANCONFIG2_DEROPT_NONE      = 0x00;
   static const uint8_t EMC2305_REG_FANCONFIG2_DEROPT_BASIC     = 0x08;
   static const uint8_t EMC2305_REG_FANCONFIG2_DEROPT_STEP      = 0x10;
   static const uint8_t EMC2305_REG_FANCONFIG2_DEROPT_BOTH      = 0x18;
   static const uint8_t EMC2305_REG_FANCONFIG2_ERRRANGE_CLEAR   = ~0x06;
   static const uint8_t EMC2305_REG_FANCONFIG2_ERRRANGE_0       = 0x00;
   static const uint8_t EMC2305_REG_FANCONFIG2_ERRRANGE_50      = 0x02;
   static const uint8_t EMC2305_REG_FANCONFIG2_ERRRANGE_100     = 0x04;
   static const uint8_t EMC2305_REG_FANCONFIG2_ERRRANGE_200     = 0x06;
   
   // EMC2305_REG_FANSPINUP
   static const uint8_t EMC2305_REG_FANSPINUP_NOKICK            = 0x20;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINLVL_CLEAR     = ~0x1C;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINLVL_30        = 0x00;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINLVL_35        = 0x04;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINLVL_40        = 0x08;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINLVL_45        = 0x0C;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINLVL_50        = 0x10;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINLVL_55        = 0x14;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINLVL_60        = 0x18;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINLVL_65        = 0x1C;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINUPTIME_CLEAR  = ~0x03;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINUPTIME_250    = 0x00;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINUPTIME_500    = 0x01;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINUPTIME_1000   = 0x02;
   static const uint8_t EMC2305_REG_FANSPINUP_SPINUPTIME_2000   = 0x03;
   
   // EMC2305_REG_FANMAXSTEP
   static const uint8_t EMC2305_REG_FANMAXSTEP_MAX = 0b00111111;
   
   // Fan poles to tacho multiplier look-up table
   static const uint8_t EMC2305_FAN_POLES_TACH_MULTIPLIER_LUT[4]  = { 3, 5, 7, 9 };
   
   uint8_t   m_tachMinRpmMultiplier[NUM_OF_FAN_CHANNELS] = { 1, 1, 1, 1, 1 };
   uint8_t   m_tachFanPolesMultiplier[NUM_OF_FAN_CHANNELS] = { 5, 5, 5, 5, 5 };
   uint8_t   m_fanPoleCount[NUM_OF_FAN_CHANNELS] = { 2, 2, 2, 2, 2 };
   
   uint16_t  m_targetTachCount[NUM_OF_FAN_CHANNELS];
   uint16_t  m_fanSpeed[NUM_OF_FAN_CHANNELS];

   bool ConvertTachAndRpm ( uint8_t chan, uint16_t tachOrRpm, uint16_t &rpmOrTach )
   {
      bool retVal = false;
      uint16_t output = 0;
      
      // Refer to Application Note 17.4 from Microchip/SMSC: Convert TACH counts to RPM
      // -----------------------------------------------------------------------------------------------
      // RPM = 1/(poles) * (n-1)/(COUNT * (1/m)) * f(TACH) * 60
      // -----------------------------------------------------------------------------------------------
      // 1) TACHO_CLOCK_FREQUENCY_IN_HZ: It's recommended to use external clock at 32768 Hz
      // 2) m_tachFanPolesMultiplier[n]: The number of edges that measured, typically 5 for 2 pole fan
      // 3) m_tachMinRpmMultiplier[n]: The multiplier defined in the RANGE bits of FAN CONFIG 1 register   
      // 4) m_fanPoleCount[n]: The number of the pole count of the fan, typically 2 for a regular fan
      // -----------------------------------------------------------------------------------------------
      output = (m_tachFanPolesMultiplier[chan] - 1) * m_tachMinRpmMultiplier[chan] * TACHO_CLOCK_FREQUENCY_IN_HZ * NUM_OF_SECONDS_PER_MINUTE / tachOrRpm / m_fanPoleCount[chan];

      // Range check; need to optimize
      if (EMC2305_INVALID_RPM_OR_TACH > output)
      {
         rpmOrTach = output;
         
         retVal = true;
      }
      else
      {
         rpmOrTach = EMC2305_INVALID_RPM_OR_TACH;
      }
      
      return retVal;
   }
   
   // Helper function for address range check
   static inline bool IsValidAddress( uint8_t addr )
   {
      return (EMC2305_REG_ADDR_LOWER_LIMIT <= addr);
   }
   
   // Helper function for channel index range check
   static inline bool IsValidChannel( uint8_t chan )
   {
      return (NUM_OF_FAN_CHANNELS > chan);
   }
}


/***************************************************************************//**
 * EMC2305 Constructor.
 *
 * @param[in]  pI2c        I2C object handle @ref II2c
 *
 * @note
 *    None
 *
 * @warning
 *    None
 ******************************************************************************
 */
EMC2305::EMC2305 (II2c *pI2c) : m_pI2c (pI2c), m_Mutex("emcMutex")
{
   // Enable external clock and PWM output as push-pull
   SetClockMode( true );
      
   // Base config for a fan with 2 poles and 500 min RPM.
   // Fan starts in off state.
   for (uint8_t i = 0; i < NUM_OF_FAN_CHANNELS; i++)
   {
      // Num of pole: 2
      SetFanPoles( i, 2 );
      
      // Minimum RPM: 500
      SetTachMinRpm( i, 500 );
      
      // PWM output: Push-pull
      SetPwmOutMode( i, true );  
      
      m_fanSpeed[i]        = 0;
      m_targetTachCount[i] = EMC2305_TACHO_TARGET_OFF;
   }
}


/***************************************************************************//**
 * EMC2305 default destructor.
 *
 * @note
 *    None.
 *
 * @warning
 *    None.
 ******************************************************************************
 */
EMC2305::~EMC2305()
{
}


/***************************************************************************//**
* Get the Presence State of the EMC2305 device
* 
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::IsPresent( void )
{
   bool retVal = false;
   
   uint8_t regAddr[2]; 
   uint8_t regVal[2];
   
   regAddr[0] = EMC2305_REG_PRODUCT_ID;
   regAddr[1] = EMC2305_REG_MANUFACTURER_ID;
   
   for (uint8_t i = 0; i < 2; i++)
   {
      ReadRegister( regAddr[i], regVal[i] );
   }
   
   if (EMC2305_REG_PRODUCT_VALUE == regVal[0] &&
       EMC2305_REG_MANUFACTURER_VALUE == regVal[1] )
   {
      retVal = true;
   }
   
   return retVal;
}


/***************************************************************************//**
* Get the Revision ID
* 
* @param[out]  rev   The revision ID of EMC2305
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/   
bool EMC2305::GetRevision( uint8_t &rev )
{
   bool retVal = false;
   uint8_t regAddr = 0;
   uint8_t regVal = 0;
   
   regAddr = EMC2305_REG_REVISION;
   
   if (ReadRegister( regAddr, regVal ))
   {
      rev = regVal;
      
      retVal = true;
   }
   
   return retVal;
}


/***************************************************************************//**
* Get the 7-bit Slave ID of I2C/SMBus
* 
* @param[out]  addr  7-bit Slave ID of I2C bus (Not the read/write address)
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/  
bool EMC2305::GetSmbAddr( uint8_t &addr )
{
   bool retVal = false;
   uint8_t regAddr = 0;
   uint8_t regVal = 0;
   
   regAddr = EMC2305_REG_PRODUCT_FEATURES;
   
   if (ReadRegister( regAddr, regVal ))
   {
      addr = (regVal & EMC2305_REG_SMBUS_ADDR_MASK) >> EMC2305_REG_SMBUS_ADDR_BITS;
      
      retVal = true;
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the clock mode
* 
* @param[in]   extClk   True to use external clock or false for internal clock
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::SetClockMode( bool extClk )
{
   bool retVal = false;
   uint8_t regAddr = 0;
   uint8_t regVal = 0;
   
   regAddr = EMC2305_REG_CONFIGURATION;
   
   if (ReadRegister( regAddr, regVal ))
   {
      if (extClk)
      {
         regVal |= EMC2305_REG_CONFIG_USE_EXT_CLK;
      }
      else
      {
         regVal &= ~EMC2305_REG_CONFIG_USE_EXT_CLK;
      }
      
      if ( WriteRegister( regAddr, regVal ) )
      {
         retVal = true;
      }
   }
   
   return retVal;   
}


/***************************************************************************//**
* Get the clock mode
* 
* @param[out]  extClk   True for external clock or false for internal clock
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::GetClockMode( bool &extClk )
{
   bool retVal = false;
   uint8_t regAddr = 0;
   uint8_t regVal = 0;
   
   regAddr = EMC2305_REG_CONFIGURATION;
   
   if (ReadRegister( regAddr, regVal ))
   {
      if (regVal & EMC2305_REG_CONFIG_USE_EXT_CLK)
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the PWM output mode (push-pull or open-drain)
* 
* @param[in]   chan        Fan channel indexing from 0 to 4
* @param[in]   pushPull    True to enable push-pull for PWM outputs
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::SetPwmOutMode( uint8_t chan, bool pushPull )
{
   bool retVal = false;
   uint8_t regAddr = 0;
   uint8_t regVal = 0;
   
   if (IsValidChannel(chan))
   {
      regAddr = EMC2305_REG_PWMOUTPUTCONFIG;
      
      if ( ReadRegister( regAddr, regVal ) )
      {
         if (pushPull)
         {
            regVal |= (0x1 << chan);
         }
         else
         {
            regVal &= ~(0x1 << chan);
         }
         
         if ( WriteRegister( regAddr, regVal ) )
         {
            retVal = true;
         }
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Get the PWM output mode (push-pull or open-drain)
* 
* @param[in]   pushPull    True for push-pull or False for open-drain
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::GetPwmOutMode( uint8_t chan, bool &pushPull )
{
   bool retVal = false;
   uint8_t regAddr = 0;
   uint8_t regVal = 0;
   
   regAddr = EMC2305_REG_PWMOUTPUTCONFIG;
   
   if ( ReadRegister( regAddr, regVal ) )
   {
      if ( regVal & (0x1 << chan) )
      {
         pushPull = true;
      }
      else
      {
         pushPull = false;
      }
      
      retVal = true;
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the PWM duty-cycle of Fan output
* 
* @param[in]   chan     Channel number indexing from 0
* @param[in]   duty     PWM duty-cycle from 0 to 255
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::SetFanPwm( uint8_t chan, uint8_t duty )
{
   bool retVal = false;
   uint8_t regAddr = 0;
   uint8_t regVal = 0;
   
   if ( IsValidChannel(chan) )
   {
      regAddr = EMC2305_REG_FANSETTING + EMC2303_REG_OFFSET_PER_CHAN * chan;
      regVal = duty;
      
      if ( WriteRegister( regAddr, regVal ) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Get the PWM duty-cycle of Fan output
* 
* @param[in]   chan     Channel number indexing from 0
* @param[out]  duty     PWM duty-cycle from 0 to 255
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::GetFanPwm( uint8_t chan, uint8_t &duty )
{
   bool retVal = false;
   uint8_t regAddr = 0;
   uint8_t regVal = 0;
   
   if ( IsValidChannel(chan) )
   {
      regAddr = EMC2305_REG_FANSETTING + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      if ( ReadRegister( regAddr, regVal ) )
      {
         duty = regVal;
         
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the PWM frequence of the Fan output
* 
* @param[in]   freq     PWM frequency in KHz
*
* @return      TRUE on success or FALSE on failure.
*
* @note        Set the base frequency of the PWM output, which could be divided 
* further by setPWMFrequencyDivider(). Any frequency below the next higher base 
* frequency will automatically be converted to the previous frequency.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::SetPwmFrequencyBase(double frequencyKHz)
{
   bool retVal = false;
   uint8_t writeAddr = 0;
   uint8_t writeByte = 0;
   
   writeAddr = EMC2305_REG_PWMBASEFREQ;
   
   if (frequencyKHz < 4.882)
   {
      writeByte = EMC2305_REG_PWMBASEFREQ_2KHZ;
   }
   else if (frequencyKHz < 19.531)
   {
      writeByte = EMC2305_REG_PWMBASEFREQ_4KHZ;
   }
   else if (frequencyKHz < 26)
   {
      writeByte = EMC2305_REG_PWMBASEFREQ_19KHZ;
   }
   else
   {
      writeByte = EMC2305_REG_PWMBASEFREQ_26KHZ;
   }
   
   if ( WriteRegister( writeAddr, writeByte ) )
   {
      retVal = true;
   }

   return retVal;
}


/***************************************************************************//**
* Set the PWM Frequency Divider of the Fan output
* 
* @param[in]   divide   PWM frequency divider
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::SetPwmFrequencyDivider(uint8_t divisor)
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;
   
   RegAddr = EMC2305_REG_PWMDIVIDE;
   RegVal = divisor;
   
   if ( WriteRegister( RegAddr, RegVal ) )
   {
      retVal = true;
   }
  
   return retVal;
}


/***************************************************************************//**
* Set the RPM-base Fan Speed Control algorithm mode
* 
* @param[in]   chan     Channel number indexing from 0
* @param[in]   enable   TRUE to enable control algorithm or FALSE to disable it
*
* @return      TRUE on success or FALSE on failure.
*
* @note        The Target Tach must set in prior.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::SetSpeedControl( uint8_t chan, bool enable )
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;
   
   if (IsValidChannel(chan))
   {
      RegAddr = EMC2305_REG_FANCONFIG1 + EMC2303_REG_OFFSET_PER_CHAN * chan;
      RegVal = enable ? EMC2305_REG_FANCONFIG1_RPMCONTROL : 0;
   
      if( WriteRegisterBits(RegAddr, (~EMC2305_REG_FANCONFIG1_RPMCONTROL) & 0xFF, RegVal) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Get the RPM-base Fan Speed Control algorithm mode
* 
* @param[in]   chan     Channel number indexing from 0
* @param[in]   enable   TRUE to enable control algorithm or FALSE to disable it
*
* @return      TRUE on success or FALSE on failure.
*
* @note        The Target Tach must set in prior.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::GetSpeedControl( uint8_t chan, bool &enable )
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;
   
   if (IsValidChannel(chan))
   {
      RegAddr = EMC2305_REG_FANCONFIG1 + EMC2303_REG_OFFSET_PER_CHAN * chan;
   
      if( ReadRegister( RegAddr, RegVal ) )
      {
         if (RegVal & EMC2305_REG_FANCONFIG1_RPMCONTROL)
         {
            enable = true;
         }
         else
         {
            enable = false;
         }
      }
      
      retVal = true;
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the Minimum RPM value for Tach measurement
* 
* @param[in]   chan     Channel number indexing from 0
* @param[in]   minRpm   Minimum RPM from 500 to 4000
*
* @return      TRUE on success or FALSE on failure.
*
* @note        The tachometer registers have an upper limit for the minimum RPM
* calculation, it is necessary to apply correct multipliers to the fans.The Min
* RPM will be set to the closest lower RPM value.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::SetTachMinRpm( uint8_t chan, uint16_t minRpm )
{
   bool retVal = false;
   uint8_t writeAddr = 0;
   uint8_t writeByte = 0;
   
   if (IsValidChannel(chan))
   {
      writeAddr = EMC2305_REG_FANCONFIG1 + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      if (minRpm < 1000)
      {
         writeByte = EMC2305_REG_FANCONFIG1_MINRPM_500;
         m_tachMinRpmMultiplier[chan] = 1;
      }
      else if (minRpm < 2000)
      {
         writeByte = EMC2305_REG_FANCONFIG1_MINRPM_1000;
         m_tachMinRpmMultiplier[chan] = 2;
      }
      else if (minRpm < 4000)
      {
         writeByte = EMC2305_REG_FANCONFIG1_MINRPM_2000;
         m_tachMinRpmMultiplier[chan] = 4;
      }
      else
      {
         writeByte = EMC2305_REG_FANCONFIG1_MINRPM_4000;
         m_tachMinRpmMultiplier[chan] = 8;
      }
      
      if( WriteRegisterBits(writeAddr, EMC2305_REG_FANCONFIG1_MINRPM_CLEAR, writeByte) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the Pole Count of the fan channel
* 
* @param[in]   chan        Channel number indexing from 0
* @param[in]   poleCount   Pole count from 1 to 4
*
* @return      TRUE on success or FALSE on failure.
*
* @note        Most of the fans have 2 poles; different fans generate different
* amount of pulses per each rotation. Usually it's 5 for a fan with 2 poles.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::SetFanPoles( uint8_t chan, uint8_t poleCount )
{
   bool retVal = false;
   uint8_t writeAddr = 0;
   uint8_t writeByte = 0;

   if (IsValidChannel(chan))
   {
      if (EMC2305_MIN_POLE_COUNT <= poleCount && EMC2305_MAX_POLE_COUNT >= poleCount)
      {
         writeAddr = EMC2305_REG_FANCONFIG1 + EMC2303_REG_OFFSET_PER_CHAN * chan;
            
         // To avoid doubles, we first multiply the fan pole multiplier by 2 to make it an integer.
         // It will be divided by 2 in the equations relating RPM to tachometer readings.
         switch (poleCount)
         {
         default:
         case 1:
            writeByte = EMC2305_REG_FANCONFIG1_FANPOLES_1;
            break;
            
         case 2:
            writeByte = EMC2305_REG_FANCONFIG1_FANPOLES_2;
            break;
            
         case 3:
            writeByte = EMC2305_REG_FANCONFIG1_FANPOLES_3;
            break;
            
         case 4:
            writeByte = EMC2305_REG_FANCONFIG1_FANPOLES_4;
            break;
         }
         
         m_tachFanPolesMultiplier[chan] = EMC2305_FAN_POLES_TACH_MULTIPLIER_LUT[poleCount - 1];
         
         m_fanPoleCount[chan] = poleCount;

         if ( WriteRegisterBits(writeAddr, EMC2305_REG_FANCONFIG1_FANPOLES_CLEAR, writeByte) )
         {
            retVal = true;
         }
      }
   }
  
   return retVal;
}


/***************************************************************************//**
* Set the Pole Count of the fan channel
* 
* @param[in]   chan        Channel number indexing from 0
* @param[out]  poleCount   Pole count from 1 to 4
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::GetFanPoles( uint8_t chan, uint8_t & poleCount )
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;
   uint8_t edgeNum = 0;
   
   if (IsValidChannel(chan))
   {
      RegAddr = EMC2305_REG_FANCONFIG1 + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      if ( ReadRegister( RegAddr, RegVal ) )
      {
         edgeNum = RegVal & EMC2305_REG_FANCONFIG1_FANPOLES_MASK;
         
         switch (edgeNum)
         {
         default:
            poleCount = 2;
            break;
            
         case EMC2305_REG_FANCONFIG1_FANPOLES_1:
            poleCount = 1;
            break;
            
         case EMC2305_REG_FANCONFIG1_FANPOLES_2:
            poleCount = 2;
            break;
            
         case EMC2305_REG_FANCONFIG1_FANPOLES_3:
            poleCount = 3;
            break;
            
         case EMC2305_REG_FANCONFIG1_FANPOLES_4:
            poleCount = 4;
            break;
         }

         retVal = true;
      }
   }
  
   return retVal;
}


/***************************************************************************//**
* Set the period between subsequent PWM drive update of the fan channel selected
* 
* @param[in]   chan        Channel number indexing from 0
* @param[in]   periodMs    Period in Milli-seconds
*
* @return      TRUE on success or FALSE on failure.
*
* @note        This function is valid when ToggleRampControl() has been enabled. 
*              The update period will be set to the closest upper value.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::SetDriveUpdatePeriod( uint8_t chan, uint16_t periodMs )
{
   bool retVal = false;
   uint8_t writeAddr = 0;
   uint8_t writeByte = 0;
   
   if (IsValidChannel(chan))
   {
      writeAddr = EMC2305_REG_FANCONFIG1 + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      if (periodMs < 200)
      {
         writeByte = EMC2305_REG_FANCONFIG1_UPDATE_100;
      }
      else if (periodMs < 300)
      {
         writeByte = EMC2305_REG_FANCONFIG1_UPDATE_200;
      }
      else if (periodMs < 400)
      {
         writeByte = EMC2305_REG_FANCONFIG1_UPDATE_300;
      }
      else if (periodMs < 500)
      {
         writeByte = EMC2305_REG_FANCONFIG1_UPDATE_400;
      }
      else if (periodMs < 800)
      {
         writeByte = EMC2305_REG_FANCONFIG1_UPDATE_500;
      }
      else if (periodMs < 1200)
      {
         writeByte = EMC2305_REG_FANCONFIG1_UPDATE_800;
      }
      else if (periodMs < 1600)
      {
         writeByte = EMC2305_REG_FANCONFIG1_UPDATE_1200;
      }
      else
      {
         writeByte = EMC2305_REG_FANCONFIG1_UPDATE_1600;
      }
      
      if ( WriteRegisterBits( writeAddr, EMC2305_REG_FANCONFIG1_UPDATE_CLEAR, writeByte ) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the Speed Ramp Control of the fan channel selected
* 
* @param[in]   chan        Channel number indexing from 0
* @param[in]   enable      TRUE to enable ramp control or FALSE to disable it
*
* @return      TRUE on success or FALSE on failure.
*
* @note        Toggle ramp control for DIRECT CONTROL MODE, hereby the fan speed 
* will be increased gradually. Disabling this will allow fan speed to be changed 
* instantly. Enabling the RPM-based Fan Speed Control will automatically use the
* ramp control, regardless of the status of this function.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::ToggleRampControl( uint8_t chan, bool enable )
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;   
   
   if (IsValidChannel(chan))
   {
      RegAddr = EMC2305_REG_FANCONFIG2 + EMC2303_REG_OFFSET_PER_CHAN * chan;
      RegVal = enable ? EMC2305_REG_FANCONFIG2_RAMPCONTROL : 0;
     
      if ( WriteRegisterBits(RegAddr, ~EMC2305_REG_FANCONFIG2_RAMPCONTROL, RegVal) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the TACH Input Glitch Filter of the fan channel selected
* 
* @param[in]   chan        Channel number indexing from 0
* @param[in]   enable      TRUE to enable glitch filter or FALSE to disable it
*
* @return      TRUE on success or FALSE on failure.
*
* @note        The glitch filter removes the high frequency noise of the TACH
*              input pin for the better accuracy of RPM measurement.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::ToggleGlitchFilter( uint8_t chan, bool enable )
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;
   
   if (IsValidChannel(chan))
   {
      RegAddr = EMC2305_REG_FANCONFIG2 + EMC2303_REG_OFFSET_PER_CHAN * chan;
      RegVal = enable ? EMC2305_REG_FANCONFIG2_GLITCHFILTER : 0;
      
      if ( WriteRegisterBits(RegAddr, ~EMC2305_REG_FANCONFIG2_GLITCHFILTER, RegVal) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the speed control algorithm Derivative Mode of the fan channel selected
* 
* @param[in]   chan        Channel number indexing from 0
* @param[in]   modeType    0: None; 1: Basic; 2: Step; 3: Both
*
* @return      TRUE on success or FALSE on failure.
*
* @note        Refer to Table 5.15 at page 31 of EMC2305 datasheet.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::SetDerivativeMode( uint8_t chan, uint8_t modeType )
{
   bool retVal = false;
   uint8_t writeAddr = 0;
   uint8_t writeByte = 0;

   if (IsValidChannel(chan))
   {
      writeAddr = EMC2305_REG_FANCONFIG2 + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      switch (modeType)
      {
      default:
      case 0:
         writeByte = EMC2305_REG_FANCONFIG2_DEROPT_NONE;
         break;
         
      case 1:
         writeByte = EMC2305_REG_FANCONFIG2_DEROPT_BASIC;
         break;
         
      case 2:
         writeByte = EMC2305_REG_FANCONFIG2_DEROPT_STEP;
         break;
         
      case 3:
         writeByte = EMC2305_REG_FANCONFIG2_DEROPT_BOTH;
         break;
      }
      
      if ( WriteRegisterBits(writeAddr, EMC2305_REG_FANCONFIG2_DEROPT_CLEAR, writeByte) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the RPM-based speed control error range of the fan channel selected
* 
* @param[in]   chan        Channel number indexing from 0
* @param[in]   errRange    RPM count error range from 0 to 200
*
* @return      TRUE on success or FALSE on failure.
*
* @note        The RPM reading will not be a constant value due to the accuracy
* of the tachometer even if the PWM drive output is a fixed value. EMC2305 must
* know when to stop changing PWM drive as long as the RPM reading is within the
* tolerance. The error range must be a positive value, and the error range will 
* be forced to the closest upper range.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::SetControlErrRange( uint8_t chan, uint8_t errorRangeRPM )
{
   bool retVal = false;
   uint8_t writeAddr = 0;
   uint8_t writeByte = 0;
   
   if (IsValidChannel(chan))
   {
      writeAddr = EMC2305_REG_FANCONFIG2 + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      if (errorRangeRPM == 0) // Account for doubles sometimes not being exactly 0
      {
         writeByte = EMC2305_REG_FANCONFIG2_ERRRANGE_0;
      }
      else if (errorRangeRPM <= 50)
      {
         writeByte = EMC2305_REG_FANCONFIG2_ERRRANGE_50;
      }
      else if (errorRangeRPM <= 100)
      {
         writeByte = EMC2305_REG_FANCONFIG2_ERRRANGE_100;
      }
      else
      {
         writeByte = EMC2305_REG_FANCONFIG2_ERRRANGE_200;
      }
      
      if ( WriteRegisterBits(writeAddr, EMC2305_REG_FANCONFIG2_ERRRANGE_CLEAR, writeByte) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Toggle the max spin up value
* 
* @param[in]   chan        Channel number indexing from 0
* @param[in]   enable      TRUE to enable or FALSE to disable
*
* @return      TRUE on success or FALSE on failure.
*
* @note        The max spin up for the 1/4 time during the spin-up routine.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::ToggleSpinUpMax( uint8_t chan, bool enable )
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;
   
   if (IsValidChannel(chan))
   {
      RegAddr = EMC2305_REG_FANSPINUP + EMC2303_REG_OFFSET_PER_CHAN * chan;
      RegVal = enable ? EMC2305_REG_FANSPINUP_NOKICK : 0;
     
      if( WriteRegisterBits(RegAddr, ~EMC2305_REG_FANSPINUP_NOKICK, RegVal) )
      {
         retVal = true;
      }
   }

   return retVal;
}


/***************************************************************************//**
* Set the PWM for the max spin up value
* 
* @param[in]   chan           Channel number indexing from 0
* @param[in]   drivePercent   PWM percentage from 0% to 100%
*
* @return      TRUE on success or FALSE on failure.
*
* @note        Set the PWM drive percentage that will be used during the spin-up 
* routine; The function is written to use the closest upper percentage value.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::SetSpinUpDrive( uint8_t chan, uint8_t drivePercent )
{
   bool retVal = false;
   uint8_t writeAddr = 0;
   uint8_t writeByte = 0;
   
   if (IsValidChannel(chan))
   {
      writeAddr = EMC2305_REG_FANSPINUP + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      if (drivePercent < 35)
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINLVL_30;
      }
      else if (drivePercent < 40)
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINLVL_35;
      }
      else if (drivePercent < 45)
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINLVL_40;
      }
      else if (drivePercent < 50)
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINLVL_45;
      }
      else if (drivePercent < 55)
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINLVL_50;
      }
      else if (drivePercent < 60)
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINLVL_55;
      }
      else if (drivePercent < 65)
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINLVL_60;
      }
      else
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINLVL_65;
      }

      if ( WriteRegisterBits(writeAddr, EMC2305_REG_FANSPINUP_SPINLVL_CLEAR, writeByte) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the spin up time
* 
* @param[in]   chan        Channel number indexing from 0
* @param[in]   timeMs      Spin-up time in milli-seconds
*
* @return      TRUE on success or FALSE on failure.
*
* @note        Determine the time duration of the spin up routine. This function
* is written to use the closet uppper time value.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::SetSpinUpTime( uint8_t chan, uint16_t timeMs )
{
   bool retVal = false;
   uint8_t writeAddr = 0;
   uint8_t writeByte = 0;
   
   if (IsValidChannel(chan))
   {
      writeAddr = EMC2305_REG_FANSPINUP + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      if (timeMs < 500)
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINUPTIME_250;
      }
      else if (timeMs < 1000)
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINUPTIME_500;
      }
      else if (timeMs < 2000)
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINUPTIME_1000;
      }
      else
      {
         writeByte = EMC2305_REG_FANSPINUP_SPINUPTIME_2000;
      }
      
      if ( WriteRegisterBits(writeAddr, EMC2305_REG_FANSPINUP_SPINUPTIME_CLEAR, writeByte) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the max control step
* 
* @param[in]   chan        Channel number indexing from 0
* @param[in]   stepSize    Step size from 0 to 63 (0x3F)
*
* @return      TRUE on success or FALSE on failure.
*
* @note        Set the maximum change in fan drive that could be performed over
* a single update period. Maximum value is 0b00111111 (63 or 0x3F).
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::SetControlMaxStep( uint8_t chan, uint8_t stepSize )
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;
   
   if (IsValidChannel(chan))
   {
      RegAddr = EMC2305_REG_FANMAXSTEP + EMC2303_REG_OFFSET_PER_CHAN * chan;
      RegVal = (EMC2305_REG_FANMAXSTEP_MAX > stepSize)? stepSize : EMC2305_REG_FANMAXSTEP_MAX;
             
      if ( WriteRegister(RegAddr, RegVal) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the minimum PWM for the fan to avoid unexpected spinning stop issue.
* 
* @param[in]   chan           Channel number indexing from 0
* @param[in]   drivePercent   PWM percentage from 0 % to 100 %
*
* @return      TRUE on success or FALSE on failure.
*
* @note        Set the minimum maintainable PWM duty cycle for the RPM-based Fan 
* Speed Control algorithm. The speed control algorithm will always drive the fan
* at a level higher than this value unless the tachometer target is specifically 
* set to 0xFFF8. 
*              This setting is extremely useful for the fans which stop spinning
* when the PWM output duty-cycle is too low; Because the RPM-based speed control 
* algorithm would drive the PWM output at a higher level, when the the fan stops
* spinning and the tachometer readings is zero; at this moment, then the fan get
* restarted violently. As a result, the PID algorithm will drive the fan too low
* because the tachometer reading is so high. 
               This process will repeat indefinitely. The fan will shift between
* on and off state constantly. Having a minimum drive prevents it from happening.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::SetFanMinDrive( uint8_t chan, uint8_t minDrivePercent )
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;
   
   if (IsValidChannel(chan))
   {
      // Convert the percent to byte format
      RegAddr = EMC2305_REG_FANMINDRIVE + EMC2303_REG_OFFSET_PER_CHAN * chan;
      RegVal = (uint8_t) (minDrivePercent * 255 / 100);
      
      if ( WriteRegister( RegAddr, RegVal ) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Get the minimum PWM for the fan to avoid unexpected spinning stop issue.
* 
* @param[in]   chan           Channel number indexing from 0
* @param[out]  drivePercent   PWM percentage from 0 % to 100 %
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::GetFanMinDrive( uint8_t chan, uint8_t &minDrivePercent )
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;
   
   if (IsValidChannel(chan))
   {
      // Convert the percent to byte format
      RegAddr = EMC2305_REG_FANMINDRIVE + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      if ( ReadRegister( RegAddr, RegVal ) )
      {
         minDrivePercent = (uint8_t)(RegVal * 100 / 255);    
 
         retVal = true;
      } 
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the Minimum Valid RPM for the spin-up routine.
* 
* @param[in]   chan           Channel number indexing from 0
* @param[in]   minRpm         RPM value from 500 to 4000
*
* @return      TRUE on success or FALSE on failure.
*
* @note        Set this minimum RPM value that will be checked at the end of the
* spin up routine to decide if the fan is actually rotating or if it is stalled.
* Internally, the function converts the min RPM to tachometer count that will be 
* written to the appropriate register.
* 
* @warning     This minimum valid RPM is not the absolute minimum RPM because it 
* only serves as a check for the spin up routine. The absolute minimum RPM speed 
* should use setFanMinDrive() which accepts percentage rather than the RPM. Also
* the min valid RPM value is related with setTachMinRPM(). It will automatically 
* increase the RPM to the lower limit if the given RPM is lower than the one set
* in setTachMinRPM().
*******************************************************************************/
bool EMC2305::SetMinValidRpm( uint8_t chan, uint16_t minRpm )
{
   bool retVal = false;
   uint8_t RegAddr = 0;
   uint8_t RegVal = 0;
   uint16_t maxTachCount_  = 0;
   uint16_t tachMinRPM = 0;
   
   if (IsValidChannel(chan))
   {
      RegAddr = EMC2305_REG_FANVALTACHCOUNT + EMC2303_REG_OFFSET_PER_CHAN * chan;
       
      // Ensure the given min RPM is not below the limits of the tachometer.
      switch (m_tachMinRpmMultiplier[chan])
      {
      case 1:
         tachMinRPM = 500;
         break;
      case 2:
         tachMinRPM = 1000;
         break;
      case 3:
         tachMinRPM = 2000;
         break;
      default:
      case 4:
         tachMinRPM = 4000;
         break;
      }
      
      if (minRpm < tachMinRPM)
      {
         minRpm = tachMinRPM;
      }

      if ( ConvertTachAndRpm( chan, minRpm, maxTachCount_ ) )
      {   
         RegVal = (maxTachCount_ >> 5) & 0xFF;
         
         if ( WriteRegister( RegAddr, RegVal ) )
         {
            retVal = true;
         }
      }
   }

   return retVal;
}


/***************************************************************************//**
* Set the Targetd RPM for the fan channel selected
* 
* @param[in]   chan           Channel number indexing from 0
* @param[in]   targetRPM      RPM value from minimum to maximum
*
* @return      TRUE on success or FALSE on failure.
*
* @note        This function calculates the target tachometer value based on the
* the given target RPM, then writes it to the target tachometer registers.
* 
* @warning     Need sanity check for targetRPM to ensure it never cause overflow
* in the calculation for the max value of uint16_t (65535).
*******************************************************************************/
bool EMC2305::SetRpmTarget( uint8_t chan, uint16_t targetRPM )
{
   bool retVal = false;

   if (IsValidChannel(chan))
   {
      if ( EMC2305_MIN_RPM_VALUE <= targetRPM && EMC2305_MAX_RPM_VALUE >= targetRPM )
      {
         if ( ConvertTachAndRpm( chan, targetRPM, m_targetTachCount[chan] ) )
         {
            if( WriteTachoTarget( chan, m_targetTachCount[chan]) )
            {
               retVal = true;
            }
         }
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Get the Targetd RPM for the fan channel selected
* 
* @param[in]   chan           Channel number indexing from 0
* @param[out]  targetRPM      RPM value from minimum to maximum
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::GetRpmTarget( uint8_t chan, uint16_t &targetRPM )
{
   bool retVal = false;
   uint16_t tachoCount;

   if (IsValidChannel(chan))
   {
      if ( ReadTachoTarget( chan, tachoCount ) )
      { 
         if ( ConvertTachAndRpm( chan, tachoCount, targetRPM ) )
         {
            if ( EMC2305_MIN_RPM_VALUE <= targetRPM && EMC2305_MAX_RPM_VALUE >= targetRPM )
            {
               retVal = true;
            }
         }
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Turn the fan on (to the most recent known target RPM) or turn it off
* 
* @param[in]   chan        Channel number indexing from 0
* @param[in]   enable      TRUE to enable or FALSE to disable
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::ToggleFan( uint8_t chan, bool enable )
{
   bool retVal = false;
   
   if (IsValidChannel(chan))
   {
      uint16_t tachoTarget = enable ? m_targetTachCount[chan] : EMC2305_TACHO_TARGET_OFF;
      
      if ( WriteTachoTarget( chan, tachoTarget ) )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Obtain the tachometer reading and convert it to RPM
* 
* @param[in]   chan        Channel number indexing from 0
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::FetchFanSpeed( uint8_t chan )
{
   bool retVal = false;
   uint8_t RegAddr[2] = { 0, 0 };
   uint8_t RegVal[2] = { 0, 0 };
   uint16_t tachoCount = 0;
   
   if (IsValidChannel(chan))
   {
      RegAddr[0] = EMC2305_REG_TACHREADMSB + EMC2303_REG_OFFSET_PER_CHAN * chan;
      RegAddr[1] = EMC2305_REG_TACHREADLSB + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      if ( ReadRegister( RegAddr[0], RegVal[0] ) )
      {
         if ( ReadRegister ( RegAddr[1], RegVal[1] ) )
         {
            tachoCount = ((RegVal[0] << 8) | RegVal[1]) >> 3;

            if ( ConvertTachAndRpm( chan, tachoCount, m_fanSpeed[chan] ) )
            {
               retVal = true;
            }
         }
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Get the fan speed of the channel selected
* 
* @param[in]   chan        Channel number indexing from 0
* @param[out]  speed       Fan speed RPM (Revolutions per minute)
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::GetFanSpeed( uint8_t chan, uint16_t &speed )
{
   bool retVal = false;
   
   if (IsValidChannel(chan))
   {
      if( FetchFanSpeed( chan ) )
      {
         speed = m_fanSpeed[chan];
         
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Bit Mode Register Write API
* 
* @param[in]   registerAddress      Register address
* @param[in]   clearingMask         Register Mask to Clear Bits
* @param[in]   byteToWrite          Byte to Write Register
*
* @return      TRUE on success or FALSE on failure.
*
* @note        To write specific bits in the specific register, while the other 
* bits of the register are unaffected. Read the register content first, masking
* the appropriate bits, and then writing this modified byte into the register.
* 
* @warning     None.
*******************************************************************************/
bool EMC2305::WriteRegisterBits( uint8_t registerAddress, uint8_t clearingMask, uint8_t byteToWrite )
{
   bool retVal = false;
   uint8_t registerContents = 0;
   
   if ( IsValidAddress( registerAddress ) )
   {
      if ( ReadRegister( registerAddress, registerContents ) )
      {
         registerContents &= clearingMask; // Reset the bits at the location of interest
         registerContents |= byteToWrite;  // Write bits to the location of interest
         
         if ( WriteRegister( registerAddress, registerContents ) )
         {
            retVal = true;
         }
      }
   }

   return retVal;
}


/***************************************************************************//**
* Write the Tach Target to the Fan channel
* 
* @param[in]   chan           Channel number indexing from 0
* @param[in]   tachoTarget    Tach target (13-bit) calculated from the RPM
*
* @return      TRUE on success or FALSE on failure.
*
* @note        The low byte (LSB) data must be written first, then the high byte
* (MSB) data, because the PID algorithm state-machine will be triggered when the 
* high byte (MSB) register is written (refer to page 36 of datasheet). 
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::WriteTachoTarget( uint8_t chan, uint16_t tachoTarget )
{
   bool retVal = false;
   uint8_t RegAddr[2] = { 0, 0 };
   uint8_t tachCountLSB = 0;
   uint8_t tachCountMSB = 0;

   if ( IsValidChannel( chan ) )
   {
      RegAddr[0] = EMC2305_REG_TACHTARGETLSB + EMC2303_REG_OFFSET_PER_CHAN * chan;
      RegAddr[1] = EMC2305_REG_TACHTARGETMSB + EMC2303_REG_OFFSET_PER_CHAN * chan;
        
      tachCountLSB = (tachoTarget << 3) & 0xF8;
      tachCountMSB = (tachoTarget >> 5) & 0xFF;
      
      if ( WriteRegister( RegAddr[0], tachCountLSB ) )
      {
         if ( WriteRegister( RegAddr[1], tachCountMSB ) )
         {
            retVal = true;
         }
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Read the Tach Target to the Fan channel
* 
* @param[in]   chan           Channel number indexing from 0
* @param[in]   tachoTarget    Tach target (13-bit) calculated from the RPM
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None. 
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::ReadTachoTarget( uint8_t chan, uint16_t &tachoTarget )
{
   bool retVal = false;
   uint8_t RegAddr[2] = { 0, 0 };
   uint8_t tachCountLSB = 0;
   uint8_t tachCountMSB = 0;

   if ( IsValidChannel( chan ) )
   {
      RegAddr[0] = EMC2305_REG_TACHTARGETLSB + EMC2303_REG_OFFSET_PER_CHAN * chan;
      RegAddr[1] = EMC2305_REG_TACHTARGETMSB + EMC2303_REG_OFFSET_PER_CHAN * chan;
      
      if ( ReadRegister( RegAddr[0], tachCountLSB ) && ReadRegister( RegAddr[1], tachCountMSB ) )
      {       
         tachoTarget = ((tachCountMSB << 8) | tachCountLSB) >> 3;
         
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Register Dump API of EMC2305
* 
* @param[in]   regAddr  Start Register Address
* @param[out]  dataPtr  Data pointer for output
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::DumpRegisters( uint8_t regAddr, uint8_t * dataPtr )
{
   bool retVal = false;
   
   if (NULL != dataPtr && IsValidAddress( regAddr ) )
   {
      Lock();
      
      retVal = m_pI2c->Read( regAddr, dataPtr, 16 );
      
      Unlock();
   }
   
   return retVal;
}


/***************************************************************************//**
* Register Read API of EMC2305
* 
* @param[in]   regAddr  Register Address
* @param[out]  regVal   Register Value
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::ReadRegister( uint8_t regAddr, uint8_t &regVal )
{
   bool retVal = false;
   
   if ( IsValidAddress( regAddr ) )
   {
      Lock();
      
      retVal = m_pI2c->Read( regAddr, &regVal, 1 );
      
      Unlock();
   }

   return retVal;   
}


/***************************************************************************//**
* Register Write API of EMC2305
* 
* @param[in]   regAddr  Register Address
* @param[in]   regVal   Register Value
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool EMC2305::WriteRegister( uint8_t regAddr, uint8_t regVal )
{
   bool retVal = false;
   
   if ( IsValidAddress( regAddr ) )
   {
      Lock();
         
      retVal = m_pI2c->Write( regAddr, &regVal, 1 );
      
      Unlock();
   }

   return retVal;   
}