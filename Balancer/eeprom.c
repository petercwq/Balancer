//
// EEPROM rw Testing
//

#include <avr/eeprom.h>
#include "Arduino.h"

#define EEPROM_START_ADDRESS 0 // Start Address in EEPROM
#define EEPROM_SIZE 1024       // EEPROM size

typedef struct
{
  float pid_p;
  float pid_i;
  float pid_d;
  float turn_speed;
  float move_speed;
  long gyro_pitch_zero; // = -21;
  long gyro_yaw_zero;// = -521;
  long acc_z_zero; // = -400
  uint8_t checksum; // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} conf_t;

conf_t e2_conf;

uint8_t calculate_sum(uint8_t *cb, uint8_t siz)
{
    uint8_t sum = 0x55; // checksum init
    while (--siz)
        sum += *cb++; // calculate checksum (without checksum byte)
    return sum;
}

void loaddefaults()
{
    e2_conf.pid_p = 15;
    e2_conf.pid_i = 1.0;
    e2_conf.pid_d = 5;    
    e2_conf.turn_speed = 30;
    e2_conf.move_speed = 30;
    e2_conf.gyro_pitch_zero = -21;
    e2_conf.gyro_yaw_zero = -521;
    e2_conf.acc_z_zero = -300;
    e2_conf.checksum = calculate_sum((uint8_t *)&e2_conf, sizeof(e2_conf));
}

void save_conf()
{
    e2_conf.checksum = calculate_sum((uint8_t *)&e2_conf, sizeof(e2_conf));
    // write the struct data to EEPROM
    eeprom_write_block((void *)&e2_conf, (void *)EEPROM_START_ADDRESS, sizeof(e2_conf));
}

bool load_conf()
{
    eeprom_read_block((void *)&e2_conf, (void *)EEPROM_START_ADDRESS, sizeof(e2_conf));
    if (calculate_sum((uint8_t *)&e2_conf, sizeof(e2_conf)) != e2_conf.checksum)
    {
        loaddefaults(); // force load defaults
        return false;   // defaults loaded, don't reload constants (EEPROM life saving)
    }
    // read the data struct from EEPROM
    return true; // setting is OK
}

void clear_all()
{
    unsigned char data = 0;
    for (unsigned int i = EEPROM_START_ADDRESS; i < EEPROM_SIZE; i++)
    {
        eeprom_write_byte((uint8_t *)i, data);
    }
}