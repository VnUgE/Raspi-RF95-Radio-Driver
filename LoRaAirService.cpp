/*
* Version 1.0.4 HAB LoRa radio management Service AKA "LoRaAirService"
* Vaughn Nugent 2019
* Responsible for controlling the on board LoRa radio module using the VN_RF95 library 
* This program relies on the ARM BCM2835 hardware header file for Raspberry Pi Model A, B, B+, the Compute Module, and the Raspberry Pi Zero. 
*/

using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fstream>
#include <iostream>							    //Cout (Debug)
#include <atomic>								//Atomic fields
#include <bcm2835.h>							//BMC Header file for all gpio functions
#include <VN_RF95_Driver.h>  					//VN Custom RF95 Driver 
#include <sys/unistd.h>



#define PATH "/dev/lorastream.bin"				//Input filestream location 
#define OUTPATH "/dev/loraoutstream.bin"		//Output filestream location

// Our Radio Configuration
#define RF_FREQUENCY	915.00					//OPP Freq in MHZ
#define RF_SBW			125000					//Signal Bandwidth
#define RF_SF			12
#define PACKET_SIZE		PAYLOAD_SIZE			//Payload Size

//GPIO config for BCM to use this radio Module
#define OUTPUT BCM2835_GPIO_FSEL_OUTP			//OUTPUT Mode for GPIO pins
#define INPUT BCM2835_GPIO_FSEL_INPT			//INPUT Mode for GPIO pins
#define RF_IRQ_PIN		RPI_V2_GPIO_P1_22		//Physical pin #22 and GPIO 15 as the IRQ pin
#define RF_RST_PIN		RPI_V2_GPIO_P1_15		//spi reset as physical pin #15 or GPIO 10 
#define RF_CS_PIN		RPI_V2_GPIO_P1_24		//spi chip select pin as CE0 or pin #24 or GPIO 18
#define RF_BOOST_PIN	RPI_V2_GPIO_P1_11		//Radio PA Boost Pin 

#define NO_OP_SLEEP		10						//Delays the main thread, this is in miliseconds.

VN_RF95 rf95(RF_CS_PIN);						//Create an instance of the driver


bool debug = false;								//Debug mode. 0 for suppressed output, 1 for detailed output

typedef enum FLAGS : uint8_t
{
	NO_ERR = 0x00, INIT_ERR = 0x05, RX_ERR = 0x10, INVALID_CMD = 0x11, RX_SUCCESS = 0x12, RX_TIMEOUT = 0x13, RX_ID_ERR = 0x14, TX_SUCCESS = 0x20, TX_ERR = 0x21, TX_TIMEOUT = 0x22, FATAL = 0x33
		
	} FLAGS;

std::atomic<FLAGS> _current_flag;				//Atomic flag to be used across threads if needed! 

//Flag for Ctrl-C
volatile sig_atomic_t force_exit = false;
volatile int new_data_rx = 0;

//Function prototypes
void sig_handler(int sig);
void SetFlag(FLAGS flag);
void ReceiveData(int id);
int  StreamHandler();
void PrintOut(string str);
void PrintOut(string str, int val);
void ConfigurePins();

void sig_handler(int sig)
{
	PrintOut("\nExit signaled, stopping service!\n");
	force_exit=true;
}

//Set Flag method will set the current error flag
void SetFlag(FLAGS flag)
{
	_current_flag = flag; 
	PrintOut("DEBUG:Event Raised: ");
	switch (flag)
	{
	case NO_ERR:
	PrintOut("No Error");
		break;
	case INIT_ERR:
		PrintOut("Init Error");
		break;
	case RX_ERR:
		PrintOut("RX Error");
		break;
	case RX_ID_ERR:
		PrintOut("Device ID Mismatch");
		break;
	case INVALID_CMD:
		PrintOut("Invalid command recieved");
		break;
	case RX_SUCCESS:
		PrintOut("Successful Receive");
		break;
	case RX_TIMEOUT:
		PrintOut("Receive Timeout");
		break;
	case TX_SUCCESS:
		PrintOut("Successful Transmission");
		break;
	case TX_ERR:
		PrintOut("Transmit Error");
		break;
	case TX_TIMEOUT:
		PrintOut("Transmit Timeout");
		break;
	case FATAL:
		PrintOut("FATAL ERROR!");			//Call to end the service, we experienced a fatal error
		sig_handler(1);
		break;
	default:
		break;
	}
	PrintOut("\n");
}

//Callback function for receive
void ReceiveData(int id)
{
	uint8_t rx_data[PACKET_SIZE] = { 0 };
	uint8_t length = 0;
	if (!rf95.Receive(rx_data, &length))			// Returns false if there was no data  
	{
		return;
	}
	if (length < 4)									// If the data received doesnt have valid headers
		return;	

	SetFlag(RX_SUCCESS);							//DONE
	
	PrintOut("Bytes Received %d\n", length);
	int i = 0;
	do
	{
		PrintOut("", rx_data[i++]);
	} while (i < length);
	char data[length] = { 0 };
	memcpy(rx_data, data, length);
	ofstream out_stream(OUTPATH, std::ios::binary);
	out_stream.write(data, length);
	out_stream.close();
}

//This function reads the data from the binary stream in order if it exists.
//It deletes the file once the data has been sent. This will cause the executing program to wait until the file is removed until it writes new data
int StreamHandler()
{		
	streampos size;
	char* memblock;

	ifstream file(PATH, ios::in | ios::binary | ios::ate);
	if (file.is_open())
	{
		size = file.tellg();
		memblock = new char[size];
		file.seekg(0, ios::beg);
		file.read(memblock, size);
		file.close();		
		
		int stream_size = size;

		if (stream_size < 4)
			return 0;
		int i = 0;
		uint8_t tx_data[stream_size] = { 0 };
		memcpy(tx_data, memblock, stream_size);
		if (rf95.Send(tx_data, stream_size, false))		//Send our TX_BUFF with async mode off	
			SetFlag(TX_SUCCESS);
		else
			SetFlag(TX_ERR);							//Failed to send
		delete[] memblock;
		rf95.setModeRx();
		remove(PATH);
		return stream_size;
	}	
	return 0;
}

//Functions used to bring all standard output to a single function (Atomic block would be needed if the device is meant to be used across threads, but is necessary to syncronize standard output)
void PrintOut(string str)
{
	if (debug)
	{
		cout << str;
	}
}
void PrintOut(string str, int val)
{
	if (debug)
	{
		cout << str << val;
	}
}

void ConfigurePins()
{

	//If PA Boost is configured
	//bcm2835_gpio_fsel(RF_BOOST_PIN, OUTPUT);
	//bcm2835_gpio_write(RF_BOOST_PIN, HIGH);

	// Pulse a reset on module
	bcm2835_gpio_fsel(RF_RST_PIN, OUTPUT);
	bcm2835_gpio_write(RF_RST_PIN, LOW);
	bcm2835_delay(100);
	bcm2835_gpio_write(RF_RST_PIN, HIGH);
	bcm2835_delay(50);

	return;
}

//Main thread
int main (int argc, const char* argv[] )
{	
	//Handle input argument for debug output
	if (argc > 1)
	{
		std::string s = std::string(argv[1]);
		if (s == "-d" || s == "-D")
		{	
			debug = true;
		}
	}
	PrintOut("LoRa Air Service. Vaughn Nugent - Version 1.1.2\n");
	PrintOut("Driver Version ", VN_RF95_VERSION);
	PrintOut("\n");	
	signal(SIGINT, sig_handler);					//attach signal interrupt to our signal handler	

	if (!bcm2835_init())							//init BCM Module
		return EXIT_FAILURE;
	
	ConfigurePins();
	
	if (!rf95.init())								//Init RF95 driver object with our callback function
	{ 
		PrintOut("\nRF95 module init failed, Please verify wiring/module \n" );
		return EXIT_FAILURE;						//We can't do anything so exit and log the failure
	}

	//At this transmit power, we need to configure the power amplifier boost pin 
	rf95.SetTxPower(23, false);						//Set transmit power		
	rf95.SetFrequency(RF_FREQUENCY);				//Adjust Frequency	
	rf95.SetSpreadingFactor(RF_SF);					//Set LoRa spreading factor
	rf95.SetSignalBandwidth(RF_SBW);				//Set singal bandwidth
	rf95.setModeRx();								//Pre-init rx mode

	PrintOut("Radio driver initialized. LoRa device version: ", rf95.getDeviceVersion());
	PrintOut("\n");

	while(!force_exit)
	{	
		if(_current_flag == FATAL)					//FATAL ERROR FLAG THROWN ! 
			break;		

		ReceiveData(0);

		bcm2835_delay(NO_OP_SLEEP);		

		StreamHandler();							//Lets send some stuff now	
	}	
	rf95.~VN_RF95();								//Call Destructor to reset the radio before we exit
	bcm2835_close();								//Close the BMC
	return EXIT_SUCCESS;  							//Process closed successfully	
}