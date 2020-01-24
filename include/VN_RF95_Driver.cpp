/*Author: Vaughn Nugent
* Date:
* VN_RF95_Driver.cpp
* Modified functionality only for the rf95 on the raspberrypi
* 
*/

#include <VN_RF95_Driver.h>

VN_RF95::VN_RF95(uint8_t slaveSelectPin)
	
{
	_slaveSelectPin = slaveSelectPin;	 // SPI Driver init with the CE0 pin 
}

VN_RF95::~VN_RF95()
{
	SetMode(SLEEP);
	bcm2835_delay(50);
	bcm2835_spi_end();	
}

bool VN_RF95::init()
{	
	if (!SPIinit())						// Init the spi driver, if it fails return false!
	{
		return false;
	}
	_deviceVersion = spiRead(RF95_REG_42_VERSION); // Get the device type and check it, this also tests whether we are really connected to a device
	//The rf95 devices should only return a decimal value of 8
	if (_deviceVersion == 0x00 || _deviceVersion == 0xff)
	{
		return false;
	}

	SetMode(SLEEP); // Set sleep mode
	bcm2835_delay(15); 
	// Check we are in sleep mode, with LORA set
	if (spiRead(RF95_REG_01_OP_MODE) != (RF95_MODE_SLEEP | RF95_LONG_RANGE_MODE))
	{
		return false; // No device present?
	}

	// These statements configure the fifo buffer to use the entire buffer as transmit and receive
	spiWrite(RF95_REG_0E_FIFO_TX_BASE_ADDR, 0); // Set the tx base pointer to 0 
	spiWrite(RF95_REG_0F_FIFO_RX_BASE_ADDR, 0); // Set rx base pointer to 0 
	spiWrite(RF95_REG_0D_FIFO_ADDR_PTR, 0x08);	// Set the current fifo pointer to 0 

	//Default modem config
	SetPreambleLength(8); // Default is 8
	// An innocuous ISM frequency, same as RF22's
	SetFrequency(434.0);
	// Lowish power
	SetTxPower(13);
	
	return true;
}

uint8_t VN_RF95::getDeviceVersion()
{
	return _deviceVersion;
}

bool VN_RF95::Receive(uint8_t* dest, uint8_t* len)
{
	bool data_avail = false;
	uint8_t bufLen = 0;
	uint8_t buf[RF95_FIFO_SIZE] = { 0 };								// Empty Packet 
	uint8_t irq_flag = spiRead(RF95_REG_12_IRQ_FLAGS);					// Grab IRQ Flag

	if (_mode == RX && (irq_flag & RF95_RX_DONE) )						// if were in rx mode and we have an rx done signal 
	{	
		uint8_t start_address = spiRead(RF95_REG_10_RX_DATA_PTR);
		SetMode(STBY);													// Set sleep to allow for reading fifo		
		bufLen = spiRead(RF95_REG_PAYLOAD_LENGTH); 						// Have received a packet and its length
		
		if(bufLen>1)
		{
			spiWrite(RF95_REG_0D_FIFO_ADDR_PTR, start_address);			// Reset the fifo read ptr to the beginning of the packet
			spiBurstRead(RF95_REG_00_FIFO, buf, bufLen);				// Read entire message from fifo to the buffer matching the received length
		
			_lastRssi = spiRead(RF95_REG_24_PKT_RSSI_VALUE) - 137;
		
			memcpy(dest, buf, bufLen);
			data_avail = true;
		}																	
		spiWrite(RF95_REG_12_IRQ_FLAGS, RF95_IRQ_RESET);				// Clear all IRQ flags
	}
	SetMode(RX);														// Sleep -> RX clears the fifo		
	*len = bufLen;														// Size of the length of the data in the buffer													 
	return data_avail;
}

bool VN_RF95::Send(const uint8_t* data, uint8_t len, bool async)
{

	if (len > RF95_FIFO_SIZE)						// Check to make sure the entire message including headers is
		len = RF95_FIFO_SIZE;						// smaller than the entire fifo size defined in the header
	if (len < 4)
		return false;								// Not enough data to make a header and send, return false

	waitPacketSent();								// Make sure we dont interrupt an outgoing message
	SetMode(STBY);									// Set to idle mode so we can send data over spi	
	uint8_t packet[RF95_FIFO_SIZE] = { 0 };			// Create a packet full of zeros to the payload size defined in the header

	memcpy(packet, data, len);						// Copy the data from the data array to the packet 
	spiWrite(RF95_REG_0D_FIFO_ADDR_PTR, 0);			// Set Fifo addr pointer to position 0 of the buffer
	spiWrite(RF95_REG_PAYLOAD_LENGTH, 0);			// Reset payload length
	spiBurstWrite(RF95_REG_00_FIFO, packet, len);	// Send only the data defined in the len parameter 
	spiWrite(RF95_REG_PAYLOAD_LENGTH, len);			// Submit the size of the data we want to send to tx size register
	SetMode(TX);									// Start the transmitter	
	
	//Dyncamic reg check for tx done flag. Used to conserve cpu cyles and power
	if (async)	
		bcm2835_delay(DELAY_AFTER_TX);		
	else
	{
		if (len < 20)
		{
			waitPacketSent();
		}			
		else if (len<40)
		{
			waitPacketSent(30);
		}
		else if (len<100)
		{
			waitPacketSent(60);
		}
		else if (len<150)
		{
			waitPacketSent(80);
		}
		else if (len >150)
		{
			waitPacketSent(100);
		}
		else
		{
			waitPacketSent();
		}
	}
		
	return true;
}

bool VN_RF95::waitPacketSent(int avg_delay)
{
	// If we are not currently in transmit mode, there is no packet to wait for
	if (_mode == TX)
	{
		while ((spiRead(RF95_REG_12_IRQ_FLAGS) & RF95_TX_DONE)==0) //Wait for the irq flag that transmit is done
		{
			bcm2835_delay(avg_delay);
		}
		spiWrite(RF95_REG_12_IRQ_FLAGS, RF95_IRQ_RESET); // Clear all IRQ flags
	}
	return true;
}

bool VN_RF95::SetFrequency(float centre)
{
	Modes current_mode = _mode; //Save current mode
	if (_mode != STBY)
		setModeIdle();
	// Frf = FRF / FSTEP
	uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
	spiWrite(RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
	spiWrite(RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
	spiWrite(RF95_REG_08_FRF_LSB, frf & 0xff);

	SetMode(current_mode); // Go back to last mode
	return true;
}

void VN_RF95::SetTxPower(int8_t power, bool useRFO)
{
	// Sigh, different behaviors depending on whether the module use PA_BOOST or the RFO pin
	// for the transmitter output

	Modes current_mode = _mode; //Save current mode
	SetMode(STBY);				// Set idle mode while we make config changes

	if (useRFO)
	{
		if (power > 14)
			power = 14;
		if (power < -1)
			power = -1;
		spiWrite(RF95_REG_09_PA_CONFIG, RF95_MAX_POWER | (power + 1));
	}
	else
	{
		if (power > 23)
			power = 23;
		if (power < 5)
			power = 5;

		// For RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
		// RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
		// for 21, 22 and 23dBm
		if (power > 20)
		{
			spiWrite(RF95_REG_4D_PA_DAC, RF95_PA_DAC_ENABLE);
			power -= 3;
		}
		else
		{
			spiWrite(RF95_REG_4D_PA_DAC, RF95_PA_DAC_DISABLE);
		}

		// RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
		// pin is connected, so must use PA_BOOST
		// Pout = 2 + OutputPower.
		// The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
		// but OutputPower claims it would be 17dBm.
		// My measurements show 20dBm is correct
		spiWrite(RF95_REG_09_PA_CONFIG, RF95_PA_SELECT | (power - 5));
	}
	SetMode(current_mode); // Go back to last mode
}

void VN_RF95::SetPreambleLength(uint16_t bytes)
{
	Modes current_mode = _mode; //Save current mode
	SetMode(STBY);				// Set idle mode while we make config changes

	spiWrite(RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
	spiWrite(RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);

	SetMode(current_mode); // Go back to last mode
}

void VN_RF95::SetSpreadingFactor(int8_t sf)
{
	Modes current_mode = _mode; //Save current mode
	SetMode(STBY);				// Set idle mode while we make config changes

	if (sf < 6)
	{
		sf = 6;
	}
	else if (sf > 12) 
	{
		sf = 12;
	}

	if (sf == 6)
	{
		spiWrite(RF95_REG_31_DETECTION_OPTIMIZE, 0xc5);
		spiWrite(RF95_REG_37_DETECTION_THRESHOLD, 0x0c);
	}
	else 
	{
		spiWrite(RF95_REG_31_DETECTION_OPTIMIZE, 0xc3);
		spiWrite(RF95_REG_37_DETECTION_THRESHOLD, 0x0a);
	}

	spiWrite(RF95_REG_1E_MODEM_CONFIG_2, (spiRead(RF95_REG_1E_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));

	SetMode(current_mode); // Go back to last mode
}


void VN_RF95::SetSignalBandwidth(long sbw)
{
	Modes current_mode = _mode; //Save current mode
	SetMode(STBY);				// Set idle mode while we make config changes

	int bw;

	if (sbw <= 7.8E3)
	{
		bw = 0;
	}
	else if (sbw <= 10.4E3) 
	{
		bw = 1;
	}
	else if (sbw <= 15.6E3) 
	{
		bw = 2;
	}
	else if (sbw <= 20.8E3) 
	{
		bw = 3;
	}
	else if (sbw <= 31.25E3) 
	{
		bw = 4;
	}
	else if (sbw <= 41.7E3) 
	{
		bw = 5;
	}
	else if (sbw <= 62.5E3)
	{
		bw = 6;
	}
	else if (sbw <= 125E3) 
	{
		bw = 7;
	}
	else if (sbw <= 250E3) 
	{
		bw = 8;
	}
	else /*if (sbw <= 250E3)*/ 
	{
		bw = 9;
	}

	spiWrite(RF95_REG_1D_MODEM_CONFIG_1, (spiRead(RF95_REG_1D_MODEM_CONFIG_1) & 0x0f) | (bw << 4)); // Set bw

	SetMode(current_mode); // Go back to last mode
}

void VN_RF95::SetMode(Modes mode)
{
	switch (mode)
	{
		case STBY :
			setModeIdle();
			break;
		case SLEEP :
			sleep();
			break;
		case RX:
			setModeRx();
			break;
		case TX:
			setModeTx();
			break;
		default:
			break;
	}
}

void VN_RF95::setModeIdle()
{
	if (_mode != STBY)
	{
		spiWrite(RF95_REG_01_OP_MODE, RF95_LONG_RANGE_MODE | RF95_MODE_STDBY);		// Set to standby mode
		_mode = STBY;																// Set operating mode to standby 
		bcm2835_delayMicroseconds(DELAY_AFTER_STBY_US);								// Recommended delay before using device after setting mode
	}
}

void VN_RF95::sleep()
{
	if (_mode != SLEEP)
	{
		spiWrite(RF95_REG_01_OP_MODE, RF95_LONG_RANGE_MODE | RF95_MODE_SLEEP);		// Set mode sleep
		_mode = SLEEP;																// Set operating mode to sleep 
		bcm2835_delayMicroseconds(DELAY_AFTER_SLEEP_US);							// Recommended delay before using device after setting mode
	}
}

void VN_RF95::setModeRx()
{
	if (_mode != RX)
	{
		spiWrite(RF95_REG_01_OP_MODE, RF95_LONG_RANGE_MODE | RF95_MODE_RXCONTINUOUS);
		/* If we wanted to enable the DIO pin to go high on packet received interrupt we would uncomment this
		spiWrite(RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on PayloadReady
		*/
		_mode = RX;		
	}
}

void VN_RF95::setModeTx()
{
	if (_mode != TX)
	{
		spiWrite(RF95_REG_01_OP_MODE, RF95_LONG_RANGE_MODE | RF95_MODE_TX);
		/* If we wanted to enable the DIO pin to go high on packet done transmitting interrupt we would uncomment this		
		spiWrite(RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
		*/
		_mode = TX;
	}
}

//SPI STUFF

bool VN_RF95::SPIinit()
{

	if (bcm2835_spi_begin() == 0)
	{
		return false;
	}
	
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default

	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);	  //16 = 15.625MHz on Rpi2, 25MHz on RPI3, 32 = 8mhz

	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
	
	/* 
		BMC Inits the CE0 and CE1 but without setting the pin selection here
		the spi is not correctly initialized, bug in the bmc program 
	*/
	// Initialize the slave select pin
	// On Maple, this must be _after_ spi.begin
	bcm2835_gpio_fsel(_slaveSelectPin, OUTPUT);
	bcm2835_gpio_write(_slaveSelectPin, HIGH);
	
	bcm2835_delay(100);
	return true;
}

uint8_t VN_RF95::spiRead(uint8_t reg)
{
	uint8_t val = 0;

	bcm2835_gpio_write(_slaveSelectPin, LOW);
	bcm2835_spi_transfer(reg & ~SPI_WRITE_MASK); // Send the address with the write mask off
	val = bcm2835_spi_transfer(0);					// The written value is ignored, reg value is read
	bcm2835_gpio_write(_slaveSelectPin, HIGH);

	return val;
}

uint8_t VN_RF95::spiWrite(uint8_t reg, uint8_t val)
{
	uint8_t status = 0;

	bcm2835_gpio_write(_slaveSelectPin, LOW);
	status = bcm2835_spi_transfer(reg | SPI_WRITE_MASK); // Send the address with the write mask on
	bcm2835_spi_transfer(val); // New value follows
	bcm2835_gpio_write(_slaveSelectPin, HIGH);

	return status;
}

uint8_t VN_RF95::spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
	uint8_t status = 0;

	bcm2835_gpio_write(_slaveSelectPin, LOW);

	status = bcm2835_spi_transfer(reg & ~SPI_WRITE_MASK); // Send the start address with the write mask off
	while (len--)
		*dest++ = bcm2835_spi_transfer(0);

	bcm2835_gpio_write(_slaveSelectPin, HIGH);

	return status;
}

uint8_t VN_RF95::spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
	uint8_t status = 0;

	bcm2835_gpio_write(_slaveSelectPin, LOW);
	status = bcm2835_spi_transfer(reg | SPI_WRITE_MASK); // Send the start address with the write mask on
	while (len--)
		bcm2835_spi_transfer(*src++);

	bcm2835_gpio_write(_slaveSelectPin, HIGH);

	return status;
}