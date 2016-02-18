/*
 *  ======== main.c ========
 */

//=========================Includes============================//


#include "Robocup_Define_RadioBase.h"
#include "USB_driver.h"
#include "SPI.h"
#include "nRF24L01_driver.h"
#include "circularBuffer.h"
#include "unpacker.h"
#include "testTool.h"

#define RELEASE
//#define DEBUG
//#define HARDWARE_TEST

uint16_t receiveData = 0;
Bool test = false;
char lettre = ' ';
/* Flag used by idle function to check if interrupt occurred */
volatile Bool isrFlag = FALSE;

/*
 *  ======== taskFxn ========
 */

Robot_Handle HandleRobot;

Void SetUp(){

    volatile int status = 0;
    volatile FILE *fid;
    char buff[100];

    HandleRobot.HandleCLK = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    HandleRobot.HandleGPIO = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    HandleRobot.HandleCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    HandleRobot.HandleFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    HandleRobot.HandlePll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    HandleRobot.HandleSCI = SCI_init((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
    HandleRobot.HandleSPI = SPI_init((void *)SPIA_BASE_ADDR, sizeof(SPI_Obj));
    HandleRobot.HandleWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
    HandleRobot.HandlePIE = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    WDOG_disable(HandleRobot.HandleWDog);

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(HandleRobot.HandleCLK, CLK_OscSrc_Internal);

    // Setup the PLL for x10 /2 which will yield 50Mhz = 10Mhz * 10 / 2
    PLL_setup(HandleRobot.HandlePll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);

    CPU_disableGlobalInts(HandleRobot.HandleCpu);
    CPU_clearIntFlags(HandleRobot.HandleCpu);

    // Setup a debug vector table and enable the PIE
    PIE_enable(HandleRobot.HandlePIE);


    // Initialize SCIA
    USB_init(HandleRobot.HandleCLK,HandleRobot.HandleSCI);
    HandleRobot.HandleCB = CB_init();
    CB_flush(&HandleRobot.HandleCB);  //todo change the way we pass the buffer at initialisation
    HandleRobot.HandleUnPacker = UnPacker_init();

    // Initialize SPI
    spi_init(HandleRobot.HandleCLK,HandleRobot.HandleSPI);
    spi_fifo_init(HandleRobot.HandleSPI);
    spi_gpio_init(HandleRobot.HandleGPIO);

    nRFInit(HandleRobot.HandleGPIO,HandleRobot.HandleSPI,GPIO_Number_7,GPIO_Number_6); // 7 = CE et 6 = CSN

    //Redirect STDOUT to SCI
    status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write, SCI_lseek, SCI_unlink, SCI_rename);
    fid = fopen("scia","w");
    freopen("scia:", "w", stdout);
    setvbuf(stdout, buff, _IOFBF, 100);
	//printf("exit Init()\n");
	//fflush(stdout);
    CPU_enableInt(HandleRobot.HandleCpu, CPU_IntNumber_9);
    CPU_enableGlobalInts(HandleRobot.HandleCpu);
#ifdef HARDWARE_TEST
    test_is_SpiDevices_Working_Well();
#endif
}

/*
 *  ======== main ========
 */
Int main(){
    /*
     * use ROV->SysMin to view the characters in the circular buffer
     */
    System_printf("************enter OS************\n\r");
    BIOS_start();    /* does not return */
    return(0);
}

bool flag = false;
uint16_t count=0, res = 15;

Void Round_Robin(){

	flag = Unpacker_parseBuffer(&HandleRobot.HandleUnPacker, &HandleRobot.HandleCB);
	nRF_sendPackets(&HandleRobot.HandleUnPacker);

}


Void Idle(){


#ifdef DEBUG
	//get_test_data(HandleRobot.HandleSCI);
	if( SCI_getRxFifoStatus (HandleRobot.HandleSCI) >= SCI_FifoStatus_1_Word){
		lettre = SCI_getData (HandleRobot.HandleSCI);
		SCI_resetRxFifo(HandleRobot.HandleSCI);
	}
	if(lettre == 'w'){
		flag = CB_write(&HandleRobot.HandleCB, count);
		if (flag == false){
			System_printf("buffer full");
			System_flush();
		}
		else{
		count++;
		}
	}
	if(lettre == 'r'){
		uint16_t out;
		flag = CB_read(&HandleRobot.HandleCB, &out);
		if(flag == false){
			System_printf("buffer vide");
		}
		else{
		System_printf("%d",out);
		}
	}
	if(lettre == 'p'){
		CB_print(&HandleRobot.HandleCB);
	}
	if(lettre == 'f'){
		count = 0;
		CB_flush(&HandleRobot.HandleCB);
	}
	if(lettre == 'q'){
		System_printf("%d", HandleRobot.HandleCB.dataBufferCursorWrite);
		}
	if(lettre == '1'){
		test_Send_Command_To_Player(&HandleRobot.HandleUnPacker, 0, 1);
		System_printf("send packets");
	}
	if(lettre == '0'){
			test_Send_Command_To_Player(&HandleRobot.HandleUnPacker, 0, 0);
			System_printf("Stop Sending packets");
	}

	if(lettre == 'u'){
		Unpacker_cleanPlayerBuffer(&HandleRobot.HandleUnPacker, 1);
		//CB_write(&HandleRobot.HandleCB, FLAGBYTE);
		CB_write(&HandleRobot.HandleCB, 1);
		CB_write(&HandleRobot.HandleCB, 1);
		Unpacker_parseBuffer(&HandleRobot.HandleUnPacker, &HandleRobot.HandleCB);
		CB_write(&HandleRobot.HandleCB, 2);
		CB_write(&HandleRobot.HandleCB, 3);
		//CB_write(&HandleRobot.HandleCB, FLAGBYTE);
		Unpacker_parseBuffer(&HandleRobot.HandleUnPacker, &HandleRobot.HandleCB);
		int i = 0;
		for (i=0; i<3; i++){
			System_printf("%d", HandleRobot.HandleUnPacker.playerBuffer[1][i]);
		}

	}

	lettre = ' ';
#endif
	//System_printf("%x\n\r",SCI_getRXFIFO(HandleRobot.HandleSCI));
	System_flush();
}

Void sci_rx_interupt(){
	// Unroll for faster write
	uint16_t byte = SCI_getData(HandleRobot.HandleSCI);
	CB_write(&HandleRobot.HandleCB, byte);
	byte = SCI_getData(HandleRobot.HandleSCI);
	CB_write(&HandleRobot.HandleCB, byte);

	if(SCI_RxOverflow(HandleRobot.HandleSCI)){
		SCI_resetRxFifo(HandleRobot.HandleSCI);
		SCI_clearRxFifoOvf(HandleRobot.HandleSCI);
	}
	SCI_clearRxFifoInt(HandleRobot.HandleSCI);

}

//SYS/BIOS call this function when system_flush is called
Void System_Output(Char *string, UInt length){
	//send System buffer to serial.
	SCI_write(1, string,length);
}

Void System_10msDelay(uint32_t ms){
	uint32_t i,j = 0;
	for(i = 0;i < ms;i++){
		for(j = 0;j < 10000;j++);
	}
}

Void System_100usDelay(uint32_t us){
	uint32_t i,j = 0;
	for(i = 0;i < us;i++){
		for(j = 0;j < 100;j++);
	}
}

