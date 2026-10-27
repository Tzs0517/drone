#include "stm32f4xx.h"                  // Device header
#include "ucos_ii.h"
#include "BSP.h"
#include "app.h"

int main(void)
{
    SystemCoreClockUpdate();
	
		OSInit();

		AppTaskCreate();
	
    OSStart();
	
    while(1){
			
		}
}
