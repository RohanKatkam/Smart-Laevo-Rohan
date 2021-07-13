/* 
*   Header file for ECCX08 crypto chip
*/

#ifndef HEAD_CRYPTO_H_
#define HEAD_CRYPTO_H_

#include <ArduinoECCX08.h>
#include <utility/ECCX08SelfSignedCert.h>

void setupECCX08(){
    if (!ECCX08.begin()) 
        {
            Serial.println("No ECCX08 present!");
            while (1){
                if (ECCX08.begin())
                    break;
                
                delay(1000);
            }
        }

      // reconstruct the self signed cert
    ECCX08SelfSignedCert.beginReconstruction(0, 8);
    ECCX08SelfSignedCert.setCommonName(ECCX08.serialNumber());
    ECCX08SelfSignedCert.endReconstruction();
}

#endif