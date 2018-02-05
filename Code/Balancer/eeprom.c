//
// EEPROM rw Testing
//

#include <avr/eeprom.h>
#include "Arduino.h"
#include "types.h"

#define EEPROM_START_ADDRESS 0 // Start Address in EEPROM
#define EEPROM_SIZE 1024       // EEPROM size

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
    e2_conf.P8 = 10;
    e2_conf.I8 = 0.5;
    e2_conf.D8 = 4;
    for (int i = 0; i < 3; i++)
    {
        e2_conf.acc_zeros[i] = 0;
        e2_conf.gyro_zeros[i] = 0;
    }
    e2_conf.turn_speed = 30;
    e2_conf.move_speed = 50;
    e2_conf.checksum = calculate_sum((uint8_t *)&e2_conf, sizeof(e2_conf));
}

void write()
{
    e2_conf.checksum = calculate_sum((uint8_t *)&e2_conf, sizeof(e2_conf));
    // write the struct data to EEPROM
    eeprom_write_block((void *)&e2_conf, (void *)EEPROM_START_ADDRESS, sizeof(e2_conf));
}

bool read()
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
        eeprom_write(i, data);
    }
}

// // Write an uint value to EEPROM
// void EEPROM_write_short(unsigned int Address, unsigned int Data)
// {
//     unsigned int DataL = Data & 0x00FF;
//     unsigned int DataH = Data >> 8;
//     EEPROM.write(Address, DataL);
//     EEPROM.write(Address + 1, DataH);
// }

// // Read an uint value from EEPROM
// unsigned int EEPROM_read_short(unsigned int Address)
// {
//     unsigned int DataL = EEPROM.read(Address);
//     unsigned int DataH = EEPROM.read(Address + 1);
//     return ((DataH << 8) + DataL);
// }


// void EEPROM_write_block(unsigned char *memory_block, unsigned int start_address, unsigned int block_size)
// {
//     unsigned char Count = 0;
//     for (Count = 0; Count < block_size; Count++)
//     {
//         EEPROM.write(start_address + Count, memory_block[Count]);
//     }
// }

// void EEPROM_read_block(unsigned char *memory_block, unsigned int start_address, unsigned int block_size)
// {
//     unsigned char Count = 0;
//     for (Count = 0; Count < block_size; Count++)
//     {
//         memory_block[Count] = EEPROM.read(start_address + Count);
//         //Serial.println((unsigned int)(memory_block[Count]));   delay(400);
//     }
// }