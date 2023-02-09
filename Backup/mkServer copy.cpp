#include <stdio.h>
#include <iostream>
#include <string.h>
// ++ for thread function
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
// --

// ++ for socket function
 #include <arpa/inet.h>
#include <netdb.h> 
#include <netinet/in.h> 
#include <sys/socket.h> 


#include <stdint.h>

#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "mkServer.h"
///////////////////////////////////////////////////
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <bcm2835.h>


#define MODE_READ 0
#define MODE_WRITE 1

#define MAX_LEN 32

char wbuf[MAX_LEN];

typedef enum {
    NO_ACTION,
    I2C_BEGIN,
    I2C_END
} i2c_init;

uint8_t  init = NO_ACTION;
uint16_t clk_div = BCM2835_I2C_CLOCK_DIVIDER_148;
uint8_t slave_address = 0x00;
uint32_t len = 0;
uint8_t  modeI2C = MODE_READ;
//-----------------------------------



int file_i2c;
int length;
unsigned char buffer[60] = {0};



//----- OPEN THE I2C BUS -----
int I2C_open()
{
	printf("Trying to open I2C.\n");
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return 1;
	}
	
	int addr = 0x5a;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return 1;
	}
	printf("<<Successfully I2C is opened.>>\n");
	return 0;
}	

//----- READ BYTES -----
void I2C_read()
{
	length = 4;			//<<< Number of bytes to read
	if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read from the i2c bus.\n");
	}
	else
	{
		printf("Data read: %s\n", buffer);
	}
}

//----- WRITE BYTES -----
void I2C_write()
{
	buffer[0] = 0x01;
	buffer[1] = 0x02;
	length = 2;			//<<< Number of bytes to write
	if (write(file_i2c, buffer, length) != length)		
	//write() returns the number of bytes actually written, 
	//if it doesn't match then an error occurred (e.g. no response from the device)
	{
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
	}
}
/////////////////////////////////////////////////////
/////////////////////////////////
#define BAUDRATE 250000 //115200
#define  TICK_FREQ_SQRT_ERROR_COM  28392000 //84000000UL/2.0*0.676;
#define  DIST2STEP 40 // MM TO STPEPS
#define  MICROSTEPPING 1600 // 200 STEPS X 8 MICROSTEP
//////////////////////////////////////////////////////////
// +++ SPI +++
#define SPI_MODE0 0x00  // rest = 0, latch on rise
#define SPI_MODE1 0x01  // rest = 0, latch on fall
#define SPI_MODE2 0x02  // rest = 1, latch on fall
#define SPI_MODE3 0x03  // rest = 1, latch on rise

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
pthread_mutex_t spi_cmd_mutex;
static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *spiDevice = "/dev/spidev0.0";
static uint8_t mode=SPI_MODE0;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay=7;
static int fd;
static speedProfile CMDData;
static uint8_t SPI_CMD;
static uint8_t REVData[26]={0};

///////////////////////////////////


static void SPI_read(uint8_t *REVData)
{
	int ret;
	size_t len = sizeof(CMDData);
   uint8_t tx[len+2]={0,};
	//uint8_t rx[len+2]={0,};

	//memcpy(&tx[1], &CMDData, len);
	tx[0]=0x3C;
	tx[len+1]=0x3B;
	struct spi_ioc_transfer tr;
	memset (&tr, 0, sizeof (tr)) ;
	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)REVData;
	tr.len = len+2;//ARRAY_SIZE(tx);
	tr.delay_usecs = delay;
	tr.speed_hz = speed;
	tr.bits_per_word = bits;
	
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

  
}

static void SPI_write()
{
	int ret;
	size_t len = sizeof(CMDData);
   uint8_t tx[len+2]={0,};
	uint8_t rx[len+2]={0,};

	memcpy(&tx[1], &CMDData, len);
	tx[0]=0x3A;
	tx[len+1]=0x3B;
	struct spi_ioc_transfer tr;
	memset (&tr, 0, sizeof (tr)) ;
	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)rx;
	tr.len = len+2;//ARRAY_SIZE(tx);
	tr.delay_usecs = delay;
	tr.speed_hz = speed;
	tr.bits_per_word = bits;
	
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
   printf("SPI_write, %d\n", ret);
}

int SPI_transfer(uint8_t CMD, uint8_t *rx)
{
	int ret;
	size_t len = sizeof(CMDData);
   uint8_t tx[len+2]={0,};
	//uint8_t rx[len+2]={0,};

	memcpy(&tx[1], &CMDData, len);
	tx[0]=CMD;//[Writing: 0x3A, Reading: 0x3C]
	tx[len+1]=0x3B;
	//if(CMD==0x3A) printf("Write CMD=%x\n", CMD);
	struct spi_ioc_transfer tr;
	memset (&tr, 0, sizeof (tr)) ;
	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)rx;
	tr.len = len+2;//ARRAY_SIZE(tx);
	tr.delay_usecs = delay;
	tr.speed_hz = speed;
	tr.bits_per_word = bits;
	
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
}



// ///////////////////////////////////
// static void SPI_write()
// {
// 	int ret;
// 	size_t len = sizeof(CMDData);
//     uint8_t tx[len]={0,};

// 	memcpy(tx, &CMDData, len);
	
// 	struct spi_ioc_transfer tr;
// 	memset (&tr, 0, sizeof (tr)) ;
// 	tr.tx_buf = (unsigned long)tx;
// 	tr.rx_buf = (unsigned long)NULL;
// 	tr.len = len;//ARRAY_SIZE(tx);
// 	tr.delay_usecs = delay;
// 	tr.speed_hz = speed;
// 	tr.bits_per_word = bits;
// 	puts("SPI_write()");
// 	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
// 	if (ret < 1)
// 		pabort("can't send spi message");
// }
// static void SPI_read(uint8_t *REVData)
// {
// 	int ret;
// 	struct spi_ioc_transfer tr;
// 	memset (&tr, 0, sizeof (tr)) ;
// 	tr.tx_buf = (unsigned long)NULL;
// 	tr.rx_buf = (unsigned long)REVData;
// 	tr.len = 3;//ARRAY_SIZE(tx);
// 	tr.delay_usecs = delay;
// 	tr.speed_hz = speed;
// 	tr.bits_per_word = bits;

// 	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
// 	if (ret < 1)
// 		pabort("can't read spi message");
// }


static void SPI_init(const char *device)
{
	int ret = 0;
	

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
  
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d, fd=%d\n", mode, fd);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
	//char tx[]="G1 X+.2 F140 A600;";
	//G17 S20 A10 C10 D10 T83872 O1
	// CMDData.steps = 0;
	// CMDData.Na = 0;
	// CMDData.Nac = 0;
	// CMDData.NNb = 0;
	// CMDData.Cn_acc0 = 83872;
	// CMDData.dir = 1;
	memset(&CMDData,0,sizeof(CMDData));
	// for(int i=0; i<20; i++)
	//  SPI_write();// flush garbage...


}
/////////////////////////////////////////////////////
using namespace std;
pthread_t tid[3];
int *ptr[3];
int ret1,ret2;
void *thread_waitingClientTCP(void *arg) ;
void *thread_receivingDataTCP(void *socket_desc);
void *thread_receivingDataPSI(void *arg);

bool wirteMsg(int &sock, char * msg, int strLen) ;

static MKServer mkServer;
int main(void)
{
	if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }
      
    // I2C begin if specified    
    if (init == I2C_BEGIN)
    {
      if (!bcm2835_i2c_begin())
      {
        printf("bcm2835_i2c_begin failed. Are you running as root??\n");
	return 1;
      }
    }
	 bcm2835_i2c_setSlaveAddress(slave_address);
    bcm2835_i2c_setClockDivider(clk_div);
    fprintf(stderr, "Clock divider set to: %d\n", clk_div);
    fprintf(stderr, "len set to: %d\n", len);
    fprintf(stderr, "Slave address set to: %d\n", slave_address);   
	 return 0;
//////////////////////////////////////////////////
	//I2C_open();
   SPI_init(spiDevice);
	int err;
	
	err = pthread_create( &tid[0] , NULL ,  thread_waitingClientTCP , NULL);
	if (err != 0)
		printf("\ncan't create thread_waitingClientTCP :[%s]", strerror(err));
	else
		printf("\n thread_waitingClientTCP created successfully\n");
	
	err = pthread_create(&tid[1], NULL, thread_receivingDataPSI, NULL);
	if (err != 0)
		printf("\ncan't create thread_receivingDataPSI :[%s]", strerror(err));
	else
		printf("\n thread_receivingDataPSI created successfully\n");


	printf("I'm already end of main. thread is working...\n");
	////////////////////////////////////////////////////////
	// HOLDING THREAD UNTILL HERE...
	pthread_join(tid[0], (void**)&(ptr[0]));
    pthread_join(tid[1], (void**)&(ptr[1]));
	pthread_join(tid[2], (void**)&(ptr[2]));
	close(fd);
}
////////////////////////////////////////////////////////////
void *thread_receivingDataPSI(void *arg)
{
	while(1)
	{

	}
	return NULL;

	uint8_t REVData[26];
	memset(REVData,0, 26);
	pthread_mutex_lock(&spi_cmd_mutex);
	SPI_CMD=0x3C;// reading...
	pthread_mutex_unlock(&spi_cmd_mutex);
	while(1) {
		
		//SPI_read(REVData);
		pthread_mutex_lock(&spi_cmd_mutex);
		//while(SPI_CMD==)
		SPI_transfer(SPI_CMD, REVData);
		
		if(REVData[0]==0x3A)
		{
			printf("Received: %x, %x, %x\n", REVData[0], REVData[1], REVData[2]);
			memset(REVData,0, 26);
		}
		
		SPI_CMD=0x3C;// reading...
		pthread_mutex_unlock(&spi_cmd_mutex);

	}
}
void *thread_waitingClientTCP(void* arg) 
{ 
    int socket_desc , client_sock , c , *new_sock;
    int option = 1;
	struct sockaddr_in server , client;
	char ipaddress[INET_ADDRSTRLEN];
	
	//Create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
	if (socket_desc == -1)
	{
		printf("Could not create socket");
	}
	puts("Socket created");
	
	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons( 8888 );
	
	//Bind
	if( ::bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
	{
		//print the error message
		perror("bind failed. Error");
		//return 1;
	}
	puts("binding done");
	
	//Listen
	listen(socket_desc , 3);
	
	c = sizeof(struct sockaddr_in);
	
	//Accept and incoming connection
	printf("Waiting for incoming connections...%s, %d\n", INADDR_ANY, socket_desc);
	c = sizeof(struct sockaddr_in);
	// WAITING FOR CONNECTION FROM CLIENT...
	while( (client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c)) )
	{
		puts("Connection accepted");
		

		new_sock = (int *)malloc(1);
		*new_sock = client_sock;
		
        //inet_ntop( AF_INET, &client, ipaddress, INET_ADDRSTRLEN );
		
        printf("client ip address: %s\n", inet_ntoa(client.sin_addr));
		//connection_handler(new_sock);// no thread test...
		// Flush gabage ///////////////////
		memset(&CMDData,0,sizeof(CMDData));
		//SPI_write();
		pthread_mutex_lock(&spi_cmd_mutex);
		SPI_CMD=0x3A;// writing...
		SPI_transfer(SPI_CMD, REVData);
		pthread_mutex_unlock(&spi_cmd_mutex);
		///////////////////////////
		if( pthread_create( &tid[2] , NULL ,  thread_receivingDataTCP , (void*) new_sock) < 0)
		{
			perror("could not create thread");
			//return 1;
		}
		
		//Now join the thread , so that we dont terminate before the thread
		//pthread_join( sniffer_thread , NULL);
		puts("Handler assigned");
	}
	
	if (client_sock < 0)
	{
		perror("accept failed");
		//return 1;
	}
	printf("end of server socket: %d \n", *ptr[0]);
	close(socket_desc);
	if(new_sock) free(new_sock);
	//return 0;

}

void *thread_receivingDataTCP(void *socket_desc)
{
    
	//Get the socket descriptor
	int sock = *(int*)socket_desc;
	int read_size , image_size;
	char message[2000];
	char ch;
	bool res;

    unsigned int  rc, totalcnt=0;
	//Send some messages to the client
	
	
	// sprintf(message, "--- copy from Server.\n");
	// write(sock , message , strlen(message));

	//Receive a message from client
	while( 1 )
	{
			// Read data from the server. 
		if(socket_desc==NULL) return NULL;
		rc = recv(sock, &ch, 1, 0);
		if(rc < 0)
		{
			printf("Server-read() error");
			close(sock);
			break;
		}
		else if (rc == 0)
		{
			printf("Client program has issued a close()");
			close(sock);
			break;
		}
		else if(rc==1)
		{	if(ch==';' || ch=='\n')
			{
				message[totalcnt++] = ';';
				message[totalcnt++] = '\0';
				wirteMsg(sock, message, totalcnt-1);
				//printf(" =====> received: %s from client\n", message);

				char outputPacket[128];
    			if(mkServer.processCmd(message, outputPacket))
				{
					printf(" --> sending %s to MicroController:%d\n", outputPacket, strlen(outputPacket));
					
					pthread_mutex_lock(&spi_cmd_mutex);
					SPI_CMD=0x3A;// writing...
					SPI_transfer(SPI_CMD, REVData);
					pthread_mutex_unlock(&spi_cmd_mutex);
					//SPI_write();
					// uint8_t REVData[25]={0};

					// SPI_transfer(REVData);
					if(REVData[0]==0x3A)
					{
						printf("Received: %x, %x, %x\n", REVData[0], REVData[1], REVData[2]);
					}

				}
				
				

				totalcnt = 0;
			}
			else {
				message[totalcnt++] = ch;
			}
		}
			
			
	}
	
	

	//free(socket_desc);
}
bool wirteMsg(int &sock, char * msg, int strLen) 
{
	//Send command
	socklen_t len;
	int  rc, totalcnt=0;
	char CMDbuf[200], message[1000], temp;
	
	
	if( send(sock , (const char *)msg , strLen , 0)== -1)
	{
		puts("Send failed");
		rc = getsockopt(sock, SOL_SOCKET, SO_ERROR, &temp, &len);
		if(rc == 0)
		{
			sprintf(message,"SO_ERROR was %d\nExit\n", temp);
			puts(message);
		}
		close(sock);
		return (false);
	}
	
	return true;
}

bool MKServer::code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer, code);
  return (strchr_pointer != nullptr);  //Return True if a character was found
}
double  MKServer::code_value()
{
    return (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], nullptr));
}
bool MKServer::processCmd(char * strCmd, char *packet)
{
    //speedProfile outP;

	strcpy(cmdbuffer, strCmd);
    targetDest target;
    if(code_seen('G'))
    {
       if( (int)code_value() == 17)
       {
           target.seen = true;
           if(code_seen('X'))
           {
               target.distance = code_value();
           }
           else target.seen =false;

           if(code_seen('F'))
           {
               target.speed = code_value();
           }
           else target.seen =false;


           if(code_seen('A'))
           {
               target.accel = code_value();
               target.decel = target.accel;
           }
           else target.seen =false;
       }

    }
    if(target.seen)
    {
        //double distance,  double speed, double accel, double decel,  speedProfile& outSpeedProfile
        genSpeedProfile(target, CMDData);
        //char packet[128];
        sprintf(packet, "G17 S%d A%d C%d D%d T%d O%d;", CMDData.steps, CMDData.Na, CMDData.Nac, CMDData.NNb, CMDData.Cn_acc0, CMDData.dir);
        //G17 S[steps] A[Na] C[Nac] D[Nd] T[time0] O[dir]
        //qDebug()<<packet;
        return true;
    }
    return false;
}
void MKServer::genSpeedProfile(targetDest &target,  speedProfile& outSpeedProfile)
{

    double distance = target.distance;
    double speed =  target.speed;
    double accel = target.accel;
    double decel = target.decel;
    // ...OUTPUT...
    // steps: total steps(pluse)
    // Na : pulse counts for acceleration area
    // Nac : pulse counts for acceleration and constant areas
    // NNb : pulse counts for deceleration area
    // Cn_acc0: fisrt period time for 1st pulse of acceleration on Timer.
    // dir: direction

    if(distance<0){
        outSpeedProfile.dir = 0;
        distance*=-1.0;
    }
    else
        outSpeedProfile.dir = 1;

  uint32_t steps = DIST2STEP*distance;
  double  alpha = (2.0*M_PI/MICROSTEPPING);//  % [rad] angle per step
  double two_alpha = 2.0*alpha ;// alpha*2
  double Ttotal=0;

  // % 1. Calcalate Time
  // % Ta = speed/acceleration
  double Ta = speed/accel; //%[sec]
  double Tb = speed/decel; //%[sec]
  double Tc = (steps*alpha - 0.5*accel*Ta*Ta - 0.5*decel*Tb*Tb)/speed;
  if(Tc>0)
  {
      Ttotal = Ta+Tc+Tb;
  }
  else
  {
      Ttotal = Ta+Tb;
  }


  // % 2. Calculate Step
  // % convert Time Zone to Stepper Motor Count
  // % Acceleration Zone
  // % n = speed^2/(2*alpha*accel)
  uint32_t Nb, Nc;
  outSpeedProfile.steps = steps;

  outSpeedProfile.Na = floor(speed*speed/(two_alpha*accel));
  uint32_t Nacc = floor(steps*decel/(accel+decel));


  if(outSpeedProfile.Na<Nacc){
      //%Nb = floor(speed^2/(2*alpha*decel));
      Nb = floor(accel/decel*outSpeedProfile.Na);
      Nc = steps - (outSpeedProfile.Na+Nb);
  }
  else{
      outSpeedProfile.Na = Nacc;
      Nb = steps - Nacc;
      Nc = 0;
  }
  outSpeedProfile.Nac = outSpeedProfile.Na + Nc;
  outSpeedProfile.NNb = Nb;
  outSpeedProfile.Cn_acc0 = uint32_t(TICK_FREQ_SQRT_ERROR_COM*sqrt(two_alpha/accel));

  uint32_t Cn_const_speed = uint32_t(TICK_FREQ_SQRT_ERROR_COM*sqrt(two_alpha/accel)*(sqrt(outSpeedProfile.Na+1)-sqrt(outSpeedProfile.Na)));
  uint32_t Cn_dec0 = uint32_t(TICK_FREQ_SQRT_ERROR_COM*sqrt(two_alpha/decel));

// qDebug()<<"distance: "<<distance<<
//            ", speed: "<<speed<<
//            ", acc: "<<accel<<
//            ", dec: "<<decel<<"\n";

//qDebug()<<"steps: "<<outSpeedProfile.steps<<
//          ", Na: "<<outSpeedProfile.Na<<
//          ", Nac: "<<outSpeedProfile.Nac<<
//          ", NNb: "<<outSpeedProfile.NNb<<
//          ", Cn_acc0: "<<outSpeedProfile.Cn_acc0<<
//          ", dir: "<<outSpeedProfile.dir<<"\n";



}