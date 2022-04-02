#ifndef __SN32F240_PMU_H
#define __SN32F240_PMU_H

/*_____ I N C L U D E S ____________________________________________________*/
#include <stdint.h>


/*_____ D E F I N I T I O N S ______________________________________________*/
#define	PMU_SLEEP						4
#define	PMU_DEEP_SLEEP			2
#define	PMU_DEEP_PWR_DOWN		1


/*_____ M A C R O S ________________________________________________________*/
#define	__PMU_LATCH_GPIO			SN_PMU->LATCHCTRL1 = 0x5A5A0001
#define	__PMU_DELATCH_GPIO		SN_PMU->LATCHCTRL2 = 0x5A5A0001;	while (SN_PMU->LATCHST == 0x1);		\
															SN_PMU->LATCHCTRL2 = 0x5A5A0000;		SN_PMU->LATCHCTRL1 = 0x5A5A0000


/*_____ D E C L A R A T I O N S ____________________________________________*/
void PMU_Setting(uint16_t mode_sel);
void PMU_Backup_data(void);
void PMU_DPD_GPIO_Setup(void);


#endif	/*__SN32F240_PMU_H*/
