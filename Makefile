# HAB LoRaAirService
# Vaughn Nugent

CC            = g++ -Iinclude/
CFLAGS        = -Wall -DRASPBERRY_PI -DBCM2835_NO_DELAY_COMPATIBILITY -D__BASEFILE__=\"$*\"
LIBS          = -lbcm2835 -pthread

all: LoRaAirService

LoRaAirService.o: LoRaAirService.cpp 
				$(CC) $(CFLAGS) -c 

VN_RF95_Driver.o: VN_RF95_Driver.cpp
				$(CC) $(CFLAGS) -c
				
LoRaAirService: LoRaAirService.o VN_RF95_Driver.o 
				$(CC) $(LIBS) -o LoRaAirService
				
clean:
				rm -rf *.o LoRaAirService
