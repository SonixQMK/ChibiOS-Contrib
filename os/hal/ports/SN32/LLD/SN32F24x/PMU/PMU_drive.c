/******************** (C) COPYRIGHT 2013 SONiX *******************************
* COMPANY:			SONiX
* DATE:					2013/12
* AUTHOR:				SA1
* IC:						SN32F240/230/220
* DESCRIPTION:	PMU related functions.
*____________________________________________________________________________
* REVISION	Date				User		Description
* 1.0				2013/12/17	SA1			1. First release
*
*____________________________________________________________________________
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS TIME TO MARKET.
* SONiX SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL 
* DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE
* AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN 
* IN CONNECTION WITH THEIR PRODUCTS.
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN32F240.h>


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/


/*_____ F U N C T I O N S __________________________________________________*/

/***************************************************************************************************
* Function		: PMU_Setting
* Description	: Setting and enter specified Low power mode
* Input				: mode - specified Low power mode (PMU_SLEEP, PMU_DEEP_SLEEP, PMU_DEEP_PWR_DOWN)
* Output			: None
* Return			: None
* Note				: None
****************************************************************************************************/
void PMU_Setting(uint16_t mode)
{	
	if (mode == PMU_DEEP_PWR_DOWN)		//Deep Power Down mode
	{
		//1.	Disable analog IP (ADC, LCD, EHS X'tal, ELS X'tal), External Reset, SWD
		SN_SYS0->EXRSTCTRL = 1;					//Disable External Reset
		SN_SYS0->SWDCTRL = 1;						//Disable SWD
		SN_ADC->ADM_b.ADENB = 0;				//Disable ADC
		SN_LCD->CTRL_b.LCDENB = 0;			//Disable LCD
		SN_SYS0->ANBCTRL_b.EHSEN = 0;		//Disable EHS X'tal
		SN_SYS0->ANBCTRL_b.ELSEN = 0;		//Disable ELS X'tal

		//2. Setup the GPIO status of all GPIO pins on Demand.
		PMU_DPD_GPIO_Setup();

		//3.	 (Optional) Save data to be retained during Deep power-down to the DATA bits in Backup registers
		PMU_Backup_data();

		//4.	Write 0x5A5A0001 to PMU_LATCHCTRL1 register to latch the status of all GPIO pins.
		__PMU_LATCH_GPIO;

		UT_DelayNx10us(5);				//delay > 20us@IHRC=12MHz
	}

	SN_PMU->CTRL = mode;

	__WFI();

	SN_PMU->CTRL = 0x0;
}

/*****************************************************************************
* Function		: PMU_Backup_data
* Description	: Fill in backup value in Bakeup registers.
* Input				: None
* Output			: None
* Return			: None
* Note				: User may manage the backup registers on demand.
*****************************************************************************/
void PMU_Backup_data(void)
{
	//User may modify the following backup registers on demand.
	SN_PMU->BKP0 = 0;
	SN_PMU->BKP1 = 1;
	SN_PMU->BKP2 = 2;
	SN_PMU->BKP3 = 3;
	SN_PMU->BKP4 = 4;
	SN_PMU->BKP5 = 5;
	SN_PMU->BKP6 = 6;
	SN_PMU->BKP7 = 7;
	SN_PMU->BKP8 = 8;
	SN_PMU->BKP9 = 9;
	SN_PMU->BKP10 = 10;
	SN_PMU->BKP11 = 11;
	SN_PMU->BKP12 = 12;
	SN_PMU->BKP13 = 13;
	SN_PMU->BKP14 = 14;
	SN_PMU->BKP15 = 15;
}


/*********************************************************************************
* Function		: PMU_DPD_GPIO_Setup
* Description	: Setup GPIO status before entering DPD mode
* Input				: None
* Output			: None
* Return			: None
* Note				: Setup the GPIO status of all GPIO pins on Demand. The DPDWAKEUP
*								pins which are used to wakeup MCU shall be set as input-pullup 
*								and keep in HIGH level.
*								- LCD hared pin shall be set as input floating if LCD is used!!!
*********************************************************************************/
void PMU_DPD_GPIO_Setup(void)
{
	//Example: Set P0.0~P0.11 as DPD Wakeup pins (input pull-up), other pins are input floating
	SN_GPIO0->MODE = 0x0;
	SN_GPIO0->CFG = 0x0;
	
	SN_GPIO1->MODE = 0x0;
	SN_GPIO1->CFG = 0xAAAAAAAA;
	SN_GPIO2->MODE = 0x0;
	SN_GPIO2->CFG = 0xAAAAAAAA;
	SN_GPIO3->MODE = 0x0;
	SN_GPIO3->CFG = 0xAAAAAAAA;
}
