
/*	Author: Vaughn Nugent
	Date: 12/10/19
	VN_RF95_Driver.h

  * This driver configures the  RF95 Radio to work in LoRa packet mode

  * Requires the BMC_2835 methods of operation for GPIO (spi, delays, irq)
	
  *	The receive mode of this driver configures continuous receive mode and configures the DIO_0 pin to
	go high when a packet is done being stored in the fifo buffer

  * Transmit does not set the DIO_0 high when tx is complete. The send function will wait for the packet sent
    interrupt flag to be set (async true) or will wait the recommended period of time for a packet to be sent (async false)
*/

#pragma once

#ifndef VN_RF95_h
#define VN_RF95_h

//#include <RHSPIDriver.h>
#include <string.h>
#include <bcm2835.h>	//BMC Header file for all gpio functions
#include <atomic>	
using namespace std;

#define VN_RF95_VERSION 6

// Max number of octets the LORA Rx/Tx FIFO can hold
#define RF95_FIFO_SIZE 255
#define PAYLOAD_SIZE   251

// The crystal oscillator frequency of the module
#define RH_RF95_FXOSC 32000000.0

// The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
#define RH_RF95_FSTEP  (RH_RF95_FXOSC / 524288)

// Register names (LoRa Mode, from table 85)
#define RF95_REG_00_FIFO                             0x00
#define RF95_REG_01_OP_MODE							 0x01
// Reserved registers 2-5#define RF95_REG_06_FRF_MSB 
#define RF95_REG_06_FRF_MSB							 0x06 // RF Carrier Frequency, Most Significant Bits
#define RF95_REG_07_FRF_MID                          0x07 // RF Carrier Frequency, Intermediate Bits
#define RF95_REG_08_FRF_LSB                          0x08 // RF Carrier Frequency, Least Significant Bits
#define RF95_REG_09_PA_CONFIG						 0x09 // Power amplifier selection and Output Power control
#define RF95_REG_0A_PA_RAMP							 0x0a // Control of PA ramp time, low phase noise PLL
#define RF95_REG_0B_OCP                              0x0b // Over Current Protection control
#define RF95_REG_0C_LNA                              0x0c // LNA settings
#define RF95_REG_0D_FIFO_ADDR_PTR                    0x0d // Fifo pointer 
#define RF95_REG_0E_FIFO_TX_BASE_ADDR                0x0e // Pointer transmit buffer in fifo 
#define RF95_REG_0F_FIFO_RX_BASE_ADDR                0x0f // Pointer to receive buffer in fifo
#define RF95_REG_10_RX_DATA_PTR                      0x10 // LoRa rx data pointer
#define RF95_REG_11_IRQ_FLAGS_MASK					 0x11 // Optional flag mask
#define RF95_REG_12_IRQ_FLAGS						 0x12 // LoRa state flags register (Wrong name according to rf95 docs
#define RF95_REG_13_BYTES_RECEIVED                   0x13 // Number of received bytes
#define RF95_REG_14_RX_TIMEOUT_MSB					 0x14 // Receiver timeout msb
#define RF95_REG_15_RX_TIMEOUT_LSB                   0x15 // Receiver timeout lsb
#define RF95_REG_PKT_SNR_VALUE						 0x19 // Packet SNR 
#define RF95_REG_PKT_RSSI_VALUE					     0x1a // Packet RSSI
#define RF95_REG_1D_MODEM_CONFIG_1					 0x1d // Modem config, (signal bandwidth)
#define RF95_REG_1E_MODEM_CONFIG_2					 0x1e // Modem config, (spreading factor)
#define RF95_REG_20_PREAMBLE_MSB                     0x20 // Preamble length
#define RF95_REG_21_PREAMBLE_LSB                     0x21 // Preamble length
#define RF95_REG_PAYLOAD_LENGTH						 0x22 // TX Payload length reg
#define RF95_REG_1E_LAST_HEADER                      0x1e // Info from last header
#define RF95_REG_1F_NUM_VAL_HEADERS                  0x1f // Number of valid headers received
#define RF95_REG_24_PKT_RSSI_VALUE                   0x24 // Last Packet rssi Value
#define RF95_REG_25_FIFO_RX_BYTE_ADDR                0x25
#define RF95_REG_31_DETECTION_OPTIMIZE				 0x31 // Documentations says its reserved? 
#define RF95_REG_37_DETECTION_THRESHOLD				 0x37 // Documentations says its reserved? 
// Reserved registers 27-3F
#define RF95_REG_40_DIO_MAPPING1					 0x40 // Pin mapping for DIO Pins 
#define RF95_REG_41_DIO_MAPPING2					 0x41
#define RF95_REG_42_VERSION							 0x42 // Stores the device version 
#define RF95_REG_4B_TCXO							 0x4b
#define RF95_REG_4D_PA_DAC							 0x4d // Power amplifier DAC settings
#define RF95_REG_5B_FORMER_TEMP						 0x5b
#define RF95_REG_61_AGC_REF							 0x61
#define RF95_REG_62_AGC_THRESH1						 0x62
#define RF95_REG_63_AGC_THRESH2						 0x63
#define RF95_REG_64_AGC_THRESH3						 0x64

//RF 95 Operation mode masks 
#define RF95_LONG_RANGE_MODE						 0x80
#define RF95_MODE_SLEEP                              0x00
#define RF95_MODE_STDBY								 0x01
#define RF95_MODE_TX                                 0x03
#define RF95_MODE_FSRX                               0x04
#define RF95_MODE_RXCONTINUOUS                       0x05

//Delays
#define DELAY_AFTER_SLEEP_US						 0x0a // 5us is recommended after setting sleep mode before you can use the device
#define DELAY_AFTER_STBY_US							 0x02 // us Time after setting standby mode in us to delay 
#define DELAY_AFTER_TX								 0xc8 // ms Time delay after tx mode before code can continue 

// RF95_REG_09_PA_CONFIG                             0x09
#define RF95_PA_SELECT								 0x80
#define RF95_MAX_POWER								 0x70
#define RF95_OUTPUT_POWER							 0x0f

// RF95_REG_0A_PA_RAMP                               0x0a
#define RF95_LOW_PN_TX_PLL_OFF                    0x10
#define RF95_PA_RAMP                              0x0f
#define RF95_PA_RAMP_3_4MS                        0x00
#define RF95_PA_RAMP_2MS                          0x01
#define RF95_PA_RAMP_1MS                          0x02
#define RF95_PA_RAMP_500US                        0x03
#define RF95_PA_RAMP_250US                        0x04
#define RF95_PA_RAMP_125US                        0x05
#define RF95_PA_RAMP_100US                        0x06
#define RF95_PA_RAMP_62US                         0x07
#define RF95_PA_RAMP_50US                         0x08
#define RF95_PA_RAMP_40US                         0x09
#define RF95_PA_RAMP_31US                         0x0a
#define RF95_PA_RAMP_25US                         0x0b
#define RF95_PA_RAMP_20US                         0x0c
#define RF95_PA_RAMP_15US                         0x0d
#define RF95_PA_RAMP_12US                         0x0e
#define RF95_PA_RAMP_10US                         0x0f

// RF95 Over Current Protection masks                    
#define RF95_OCP_ON                               0x20
#define RF95_OCP_TRIM                             0x1f

// RF95_REG_0C_LNA                                0x0c
#define RF95_LNA_GAIN                             0xe0
#define RF95_LNA_GAIN_G1                          0x20
#define RF95_LNA_GAIN_G2                          0x40
#define RF95_LNA_GAIN_G3                          0x60                
#define RF95_LNA_GAIN_G4                          0x80
#define RF95_LNA_GAIN_G5                          0xa0
#define RF95_LNA_GAIN_G6                          0xc0
#define RF95_LNA_BOOST_LF                         0x18
#define RF95_LNA_BOOST_LF_DEFAULT                 0x00
#define RF95_LNA_BOOST_HF                         0x03
#define RF95_LNA_BOOST_HF_DEFAULT                 0x00
#define RF95_LNA_BOOST_HF_150PC                   0x11

// IRQ flag values read section 6.4 for bit masks
#define RF95_RX_TIMEOUT								 0x80
#define RF95_RX_DONE                                 0x40
#define RF95_PAYLOAD_CRC_ERROR						 0x20
#define RF95_VALID_HEADER							 0x10
#define RF95_TX_DONE								 0x08
#define RF95_CAD_DONE								 0x04
#define RF95_FHSS_CHANGE_CHANNEL					 0x02
#define RF95_CAD_DETECTED							 0x01

#define RF95_IRQ_RESET								 0xFF

// RF95_REG_1D_BYTES_RECEIVED                      
#define RF95_BW                                   0xf0

#define RF95_BW_7_8KHZ								 0x00
#define RF95_BW_10_4KHZ								 0x10
#define RF95_BW_15_6KHZ								 0x20
#define RF95_BW_20_8KHZ								 0x30
#define RF95_BW_31_25KHZ							 0x40
#define RF95_BW_41_7KHZ								 0x50
#define RF95_BW_62_5KHZ								 0x60
#define RF95_BW_125KHZ								 0x70
#define RF95_BW_250KHZ								 0x80
#define RF95_BW_500KHZ								 0x90
#define RF95_CODING_RATE							 0x0e
#define RF95_CODING_RATE_4_5						 0x02
#define RF95_CODING_RATE_4_6						 0x04
#define RF95_CODING_RATE_4_7						 0x06
#define RF95_CODING_RATE_4_8						 0x08
#define RF95_IMPLICIT_HEADER_MODE_ON				 0x01

// RF95_REG_1E_LAST_HEADER                       
#define RF95_SPREADING_FACTOR						 0xf0
#define RF95_SPREADING_FACTOR_64CPS					 0x60
#define RF95_SPREADING_FACTOR_128CPS				 0x70
#define RF95_SPREADING_FACTOR_256CPS				 0x80
#define RF95_SPREADING_FACTOR_512CPS				 0x90
#define RF95_SPREADING_FACTOR_1024CPS				 0xa0
#define RF95_SPREADING_FACTOR_2048CPS				 0xb0
#define RF95_SPREADING_FACTOR_4096CPS				 0xc0
#define RF95_PAYLOAD_CRC_ON							 0x04
#define RF95_SYM_TIMEOUT_MSB						 0x03

// RF95_REG_4B_TCXO                           
#define RF95_TCXO_TCXO_INPUT_ON						 0x10

// RF95 PA DAC Masks                                  
#define RF95_PA_DAC_DISABLE                          0x04
#define RF95_PA_DAC_ENABLE                           0x87

// GPIO Configuration 
#define OUTPUT BCM2835_GPIO_FSEL_OUTP			//OUTPUT Mode for GPIO pins
#define INPUT BCM2835_GPIO_FSEL_INPT			//INPUT Mode for GPIO pins

enum Modes
{
	RX, TX, STBY, SLEEP
};


class VN_RF95 
{
public:
	
	///Returns an instance of the driver for program use.
	VN_RF95(uint8_t slaveSelectPin);
	
	/// Destructor. Must call deconstructor on program exit. This set the radio into sleep mode to allow for reconection on startup, otherwise you must manually power cycle the unit.
	/// Alternativley this can be achieved by pulsing the reset lines on the chip
	~VN_RF95();
	
	/// Initializes the bcm functions and radio module 
	bool init();

	/// Gets the radio deevice firmware version after init() is called
	uint8_t getDeviceVersion();
	
	/// Returns true when a packet has been detected by the radio module and loads it to the destination buffer. Otherwise returns false
	bool Receive(uint8_t* dest, uint8_t* len);	

	///Sends an entire packet assumes you are accounting for headers on your own 
	bool Send(const uint8_t* data, uint8_t len, bool async);
		
	/// Returns ture when a packet has successfully been sent 
	bool waitPacketSent(int avg_delay = 10);
	
	void SetTxPower(int8_t power, bool useRFO = false);

	void SetPreambleLength(uint16_t bytes);

	bool SetFrequency(float centre);

	void SetSpreadingFactor(int8_t sf);

	void SetSignalBandwidth(long sbw);

	///Sets the radio module to standby mode
	void setModeIdle();

	///Sets the radio module to continuous receive mode
	void setModeRx();		

	/// Set the radio module to sleep mode 
	void sleep();

protected:
	
	///Sets the radio module to transmit, starts transmitter. Only called when a packet needs to be sent.
	void setModeTx();

	void SetMode(Modes mode);
		
/*
	Spi Driver for raspi. these functions are protected to keep spi control 
	under the hood. 
*/
	
#define SPI_WRITE_MASK 0x80	// This is the bit in the SPI address that marks it as a write

	bool SPIinit();

	uint8_t spiRead(uint8_t reg);

	uint8_t spiWrite(uint8_t reg, uint8_t val);
	
	uint8_t spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len);

	uint8_t spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len);

	void setSlaveSelectPin(uint8_t slaveSelectPin);

private:	

	atomic_uint8_t _lastRssi;

	atomic<Modes> _mode;

	atomic_uint8_t  _slaveSelectPin;

	uint8_t _deviceVersion;
};


#endif

