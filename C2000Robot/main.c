/*
 *  ======== main.c ========
 */

//=========================Includes============================//

#define RELEASE   //RELEASE or DEBUG
#define HARDWARE_TEST
//#define PID_TEST
#define OPEN_LOOP

#include "Robocup_Define.h"
#include "Serial.h"
#include "SPI.h"
#include "pid.h"
#include "nRF24L01_driver.h"
#include "quad_driver.h"
#include "L3GD20_driver.h"
#include "DCMotor_driver.h"
#include "arduino_driver.h"
#include "four_wheel_ctrl.h"
#include "CS_demux.h"
#include "radio_unpacker.h"
#include "testTool.h"
#include "stdio.h"

uint16_t tested = 0;
uint16_t receiveData = 0;
Bool test = false;
char lettre = ' ';
/* Flag used by idle function to check if interrupt occurred */
volatile Bool isrFlag = FALSE;

/*
 *  ======== taskFxn ========
 */
//Robot global variable
Robot_Handle HandleRobot;

//Set up function executed once before entering OS
Void SetUp(){

    volatile int status = 0;
    //volatile FILE *fid;
    char buff[100];

    HandleRobot.HandleCLK = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    HandleRobot.HandleGPIO = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    HandleRobot.HandleCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    HandleRobot.HandleFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    HandleRobot.HandlePll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    HandleRobot.HandleSCI = SCI_init((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
    HandleRobot.HandleSPI = SPI_init((void *)SPIA_BASE_ADDR, sizeof(SPI_Obj));
    HandleRobot.HandleWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
    HandleRobot.HandlePwm1 = PWM_init((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
    HandleRobot.HandlePwm2 = PWM_init((void *)PWM_ePWM2_BASE_ADDR, sizeof(PWM_Obj));
    HandleRobot.HandlePwm3 = PWM_init((void *)PWM_ePWM3_BASE_ADDR, sizeof(PWM_Obj));

    WDOG_disable(HandleRobot.HandleWDog);

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(HandleRobot.HandleCLK, CLK_OscSrc_Internal);

    // Setup the PLL for x10 /2 which will yield 50Mhz = 10Mhz * 10 / 2
    PLL_setup(HandleRobot.HandlePll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);

    CPU_disableGlobalInts(HandleRobot.HandleCpu);
    CPU_clearIntFlags(HandleRobot.HandleCpu);


    // Initialize SCIA
    scia_init(HandleRobot.HandleCLK,HandleRobot.HandleSCI);
    scia_gpio_init(HandleRobot.HandleGPIO);

    // Initialize SPI
    spi_init(HandleRobot.HandleCLK,HandleRobot.HandleSPI);
    spi_fifo_init(HandleRobot.HandleSPI);
    spi_gpio_init(HandleRobot.HandleGPIO);

    demux_Init(GPIO_Number_19, GPIO_Number_12, GPIO_Number_6, CS_0);
    demux_disconnect();

    nRFInit(HandleRobot.HandleGPIO,HandleRobot.HandleSPI,GPIO_Number_7,CS_4); // 7 = CE et 6 = CSN
    HandleRobot.HandleQuad[0] = quad_init(CS_1);
    HandleRobot.HandleQuad[1] = quad_init(CS_2);

    //math float to int conversion initialisation
    robotParam_init();

    gyro_init(CS_5);
    arduino_Init(CS_3);


    /// PWM
    HandleRobot.HandleMotor[0] = dcMotor_init(PWM_1A, GPIO_Number_3);
    HandleRobot.HandleMotor[1] = dcMotor_init(PWM_1B, GPIO_Number_5);
    HandleRobot.HandleMotor[2] = dcMotor_init(PWM_2A, GPIO_Number_32);
    HandleRobot.HandleMotor[3] = dcMotor_init(PWM_3A, GPIO_Number_33);

    HandleRobot.HandlePid[0] = pid_init(PID_P, PID_I, PID_D, _IQ(EPWM_TIMER_TBPRD), _IQ(0));
    HandleRobot.HandlePid[1] = pid_init(PID_P, PID_I, PID_D, _IQ(EPWM_TIMER_TBPRD), _IQ(0));
    HandleRobot.HandlePid[2] = pid_init(PID_P, PID_I, PID_D, _IQ(EPWM_TIMER_TBPRD), _IQ(0));
    HandleRobot.HandlePid[3] = pid_init(PID_P, PID_I, PID_D, _IQ(EPWM_TIMER_TBPRD), _IQ(0));

    HandleRobot.HandlePid[0].term.Ref = _IQ(0);
    HandleRobot.HandlePid[1].term.Ref = _IQ(0);
    HandleRobot.HandlePid[2].term.Ref = _IQ(0);
    HandleRobot.HandlePid[3].term.Ref = _IQ(0);

    status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write, SCI_lseek, SCI_unlink, SCI_rename);
    //fid =
    fopen("scia","w");
    freopen("scia:", "w", stdout);
    setvbuf(stdout, buff, _IOFBF, 100);

#ifdef HARDWARE_TEST
    test_is_SpiDevices_Working_Well();
    test_is_Wheel_System_Working_Well();
    test_is_All_Encoder_Are_Matched_With_Wheels();
    test_is_Directions_Ok();

    /*while(true){
       	unsigned int i,j;
       	for(i = 0; i < 4; i++){
       	    System_printf("Test moteur %d\r\n", i);
              	System_flush();
              	for(j = 0; j < 10; j++)
              		test_is_Wheel_Direction_Left(i);

       	}
     }*/
#endif

    dcMotor_setPWM(&HandleRobot.HandleMotor[0],EPWM_BRAKE);
    dcMotor_setPWM(&HandleRobot.HandleMotor[1],EPWM_BRAKE);
    dcMotor_setPWM(&HandleRobot.HandleMotor[2],EPWM_BRAKE);
    dcMotor_setPWM(&HandleRobot.HandleMotor[3],EPWM_BRAKE);
}
/*
 *  ======== main ========
 */
Int main(){
    /*
     * use ROV->SysMin to view the characters in the circular buffer
     */
    System_printf("************enter OS************\r\n");
    BIOS_start();    /* does not return */
    return(0);
}


/*
 * ======= Test the connection of all device ======
 */

// Decide which type of data to print
typedef enum {
	NO_PRINT,
	RF_COMMAND,
	MOTOR_DATA,
	RF_AND_MOTOR,
	RAW_ENCODER

}printer_states;

printer_states printerState = NO_PRINT;
//This function is executed every 10 ms
void Round_Robin(){

	//***Captor read***  (Read captor first for stability)
	quad_readCounters(&HandleRobot.HandleQuad[0]);
	quad_readCounters(&HandleRobot.HandleQuad[1]);

	/*System_printf("x=%f y=%f theta=%f\r\n",HandleRobot.robotParam.XVelocityCommand.floating
									  ,HandleRobot.robotParam.YVelocityCommand.floating
									  ,HandleRobot.robotParam.ThetaVelocityCommand.floating);*/
#ifdef PID_TEST
	static uint16_t round_robin_count = 0;
	_iq command = _IQ(0);
	if(round_robin_count < 200)
		command = _IQ(0.0);
	else if(round_robin_count < 100 + 200)
		command = _IQ(0.4);
	else if(round_robin_count < 200 + 200)
		command = _IQ(0.4);
	else if(round_robin_count < 300 + 200)
		command = _IQ(0.4);
	else{
		command = _IQ(0);
		dcMotor_setPWM(&HandleRobot.HandleMotor[0], EPWM_BRAKE);
		dcMotor_setPWM(&HandleRobot.HandleMotor[1], EPWM_BRAKE);
		dcMotor_setPWM(&HandleRobot.HandleMotor[2], EPWM_BRAKE);
		dcMotor_setPWM(&HandleRobot.HandleMotor[3], EPWM_BRAKE);
		return;
	}

	// To send a 45 degrees X==Y. (45 degrees => only two wheels running)
	fourWheelCtrl_Update(_IQtoF(command),
						 _IQtoF(command),
						 0.0);
	round_robin_count++;


#ifdef OPEN_LOOP// open loop
		if(round_robin_count < 2){
			System_printf("max motor %d\r\n", dcMotor_getPWM(&HandleRobot.HandleMotor[1]));
			System_printf("Four_wheel_in, Four_wheel_out * 1000, first_wheel_speed(m/s)\n\r");
		}

		float consigne = _IQtoF(command) *  1500.0;

		dcMotor_updatePWM(&HandleRobot.HandleMotor[0], _IQint(_IQ(consigne)));
		dcMotor_updatePWM(&HandleRobot.HandleMotor[1], _IQint(_IQ(0)));
		dcMotor_updatePWM(&HandleRobot.HandleMotor[2], _IQint(_IQ(consigne)));
		dcMotor_updatePWM(&HandleRobot.HandleMotor[3], _IQint(_IQ(0)));

		System_printf("%f,%f,%f\n\r",
				_IQtoF(command),
				consigne,
				_IQtoF(_IQabs(HandleRobot.HandleQuad[1].wheelVelocity[0])));
#else //close loop
		if(round_robin_count < 2){
			System_printf("P%4.f I%4.f D%4.f\r\n", _IQtoF(PID_P)
												 , _IQtoF(PID_I)
												 , _IQtoF(PID_D));

			System_flush();
			System_printf("Four_wheel_in, Four_wheel_out(consigne), error, PID_out(command), first_wheel_speed(m/s)\n\r");
			System_flush();
		}

		pid_update(&HandleRobot.HandlePid[0], _IQabs(HandleRobot.HandleQuad[0].wheelVelocity[1]));
		pid_update(&HandleRobot.HandlePid[2], _IQabs(HandleRobot.HandleQuad[0].wheelVelocity[1]));
		dcMotor_update(&HandleRobot.HandleMotor[0], &HandleRobot.HandlePid[0]);
		dcMotor_update(&HandleRobot.HandleMotor[2], &HandleRobot.HandlePid[2]);

		System_printf("%f,%f,%f,%f,%f\n\r",
				_IQtoF(command),											// Command to four_wheel
				_IQtoF(HandleRobot.HandlePid[0].term.Ref),					// Consign from four_wheel
				_IQtoF(HandleRobot.HandlePid[0].term.Ref - HandleRobot.HandlePid[0].term.Fbk), // Error from pid
				_IQtoF(HandleRobot.HandlePid[0].term.Out),					// Command from pid
				_IQtoF(_IQabs(HandleRobot.HandleQuad[0].wheelVelocity[0]))  // Wheel speed
		);
#endif // OPEN_LOOP


#else

	bool newPacket = false;
	//***Radio Reception***
	newPacket = nRF_Listen(HandleRobot.HandleGPIO,HandleRobot.HandleSPI);

	//***Unpack***
	if(newPacket){
		//System_printf("new packet\r\n");
		unpackBuffer(HandleRF.RXPayload);
	}

	//***Cinetic Model math***
	fourWheelCtrl_Update(HandleRobot.robotParam.XVelocityCommand.floating,
						 HandleRobot.robotParam.YVelocityCommand.floating,
						 HandleRobot.robotParam.ThetaVelocityCommand.floating);


	//***PID maths***
	pid_update(&HandleRobot.HandlePid[0], _IQabs(HandleRobot.HandleQuad[0].wheelVelocity[0]));
	pid_update(&HandleRobot.HandlePid[1], _IQabs(HandleRobot.HandleQuad[0].wheelVelocity[1]));
	pid_update(&HandleRobot.HandlePid[2], _IQabs(HandleRobot.HandleQuad[1].wheelVelocity[0]));
	pid_update(&HandleRobot.HandlePid[3], _IQabs(HandleRobot.HandleQuad[1].wheelVelocity[1]));

	//pid_display(&HandleRobot.HandlePid[0]);

	//***Actuator update***
	dcMotor_update(&HandleRobot.HandleMotor[0],&HandleRobot.HandlePid[0]);
	dcMotor_update(&HandleRobot.HandleMotor[1],&HandleRobot.HandlePid[1]);
	dcMotor_update(&HandleRobot.HandleMotor[2],&HandleRobot.HandlePid[2]);
	dcMotor_update(&HandleRobot.HandleMotor[3],&HandleRobot.HandlePid[3]);


	switch(printerState){
		case RF_COMMAND:
			// X, Y, theta
			System_printf("%f,%f,%f\r\n",
					HandleRobot.robotParam.XVelocityCommand.floating,
					HandleRobot.robotParam.YVelocityCommand.floating,
					HandleRobot.robotParam.ThetaVelocityCommand.floating);
			break;
		case RF_AND_MOTOR:
			System_printf("%f,%f,%f,",
					HandleRobot.robotParam.XVelocityCommand.floating,
					HandleRobot.robotParam.YVelocityCommand.floating,
					HandleRobot.robotParam.ThetaVelocityCommand.floating);
			// Missing break to prevent repetition
		case MOTOR_DATA:
			System_printf("%f,%f,%f,%f\n\r",
					_IQtoF(HandleRobot.HandlePid[0].term.Ref),					// Consign from four_wheel
					_IQtoF(HandleRobot.HandlePid[0].term.Ref - HandleRobot.HandlePid[0].term.Fbk), // Error from pid
					_IQtoF(HandleRobot.HandlePid[0].term.Out),					// Command from pid
					_IQtoF(_IQabs(HandleRobot.HandleQuad[0].wheelVelocity[0])) // Wheel speed
			);
			break;
		case RAW_ENCODER:
			//quad_displayCounters(&HandleRobot.HandleQuad[0]);
			//quad_displayCounters(&HandleRobot.HandleQuad[1]);
			//System_printf("\r\n");
			System_printf("%d, %d, %d, %d\n\r",
					HandleRobot.HandleQuad[0].Count0,
					HandleRobot.HandleQuad[0].Count1,
					HandleRobot.HandleQuad[1].Count0,
					HandleRobot.HandleQuad[1].Count1
			);

			break;
		case NO_PRINT:
			break;
	}

#endif // PID_TEST

}

//This function is executed when roundRobin is not
void Idle(){
	if( SCI_getRxFifoStatus (HandleRobot.HandleSCI) >= SCI_FifoStatus_1_Word){
		lettre = SCI_getData (HandleRobot.HandleSCI);
		SCI_resetRxFifo(HandleRobot.HandleSCI);
	}

	if(lettre == 'h'){
		// Split the string since it's too long for the buffer
		System_printf(
				"Help:\r\n"
				"h : Print help\r\n"
				"x : Disable print\r\n"
				);
		System_flush();
		System_printf(
				"m : Set mode print motor and encoder data\r\n"
				"c : Set mode print RF command\r\n"
				);
		System_flush();
		System_printf(
				"a : Set mode print motor and encoder data and RF command\r\n"
				"e : Set mode print encoder\r\n"
				);
	}
	if(lettre == 'x'){
		printerState = NO_PRINT;
	}
	if(lettre == 'm'){
		printerState = MOTOR_DATA;
		System_printf("PID_consigne(motor0), PID_error, PID_out(command), encoder_speed(m/s)\n\r");
	}
	if(lettre == 'c'){
		printerState = RF_COMMAND;
		System_printf("X, Y, Theta\n\r");
	}
	if(lettre == 'a'){
		printerState = RF_AND_MOTOR;
		System_printf("X, Y, Theta, PID_consigne(motor0), PID_error, PID_out(command), encoder_speed(m/s)\n\r");
	}
	if(lettre == 'e'){
		printerState = RAW_ENCODER;
		System_printf("tick_m0, tick_m1, tick_m2, tick_m3\n\r");
	}

	lettre = ' ';

	System_flush();
}


//SYS/BIOS call this function when system_flush is called
void System_Output(Char *string, UInt length){
	//send System buffer to serial.
	SCI_write(1, string,length);
}

//change values to IQ
void robotParam_init(){

	_iq buf = 0;
	_iq buf2 = 0;
	_iq op = 0;

	HandleRobot.robotParam.wheelRadius = WHEEL_DIAMETER;
	HandleRobot.robotParam.roundRobinTime = RRTIME;
	HandleRobot.robotParam.encoderPPR = ENCODER_PPR;
	op = ONE_ENCODER_PPR;
	buf = _IQdiv(HandleRobot.robotParam.wheelRadius, HandleRobot.robotParam.roundRobinTime);
	buf2 = _IQmpy(buf, PI);
	HandleRobot.robotParam.speedFactor = _IQmpy(buf2, op);
	HandleRobot.robotParam.XVelocityCommand.floating = _IQ(0);
	HandleRobot.robotParam.YVelocityCommand.floating = _IQ(0);
	HandleRobot.robotParam.ThetaVelocityCommand.floating = _IQ(0);
}
