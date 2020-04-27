#include "project.h"

CY_ISR(TimerInterrupt)
{
    LEDDrive_Write(!LEDDrive_Read());
    
    Timer_1_ReadStatusRegister();
}

int main(void)
{
    Timer_1_Start();
    
    TimerInterrupt_Start();
    TimerInterrupt_StartEx(TimerInterrupt);
    
    
    Timer_1_WritePeriod(12000);
    
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
