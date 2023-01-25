#include <EEPROM.h>
int addr_eep = 0;

void setup() {
  // put your setup code here, to run once:
  EEPROM.write(addr_eep, 0);
  int i = 0;
  for(i = 0;i<12;i++)
  {
    EEPROM.write(addr_eep+i+1,0);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
