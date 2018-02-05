#ifndef TYPES_H_
#define TYPES_H_

typedef struct
{
  uint8_t P8;
  uint8_t I8;
  uint8_t D8;
  int16_t acc_zeros[3];
  int16_t gyro_zeros[3];
  uint8_t turn_speed;
  uint8_t move_speed;
  uint8_t checksum; // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} conf_t;

#endif /* TYPES_H_ */