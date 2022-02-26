

#include "BMP280.h"
#include <project.h>
#include <stdio.h>



int main()
{
    

   
    CyGlobalIntEnable;

    BMP280_start();  
    CyDelay(10u);
    BMP280_init();
     
    
   
  
for(;;)
    {
           
        BMP280_init();
        BMP280_readTempC();
        BMP280_readFloatPressure();
        BMP280_readFloatAltitudeMeters();
        
        CyDelay(1000); 
    }
}

/* [] END OF FILE */
