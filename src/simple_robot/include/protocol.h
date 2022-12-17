#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#define HEADER_SOF 0xA5
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#pragma pack(push, 1)
//ZXX
typedef enum
{
  CHASSIS_ODOM_CMD_ID = 0x0101,
  CHASSIS_CTRL_CMD_ID = 0x0102,
  RGB_ID=0x0103,
  RC_ID=0x0104,
  VISION_ID=0x0105
} referee_data_cmd_id_type;

typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef struct
{
  float vx;
  float vy;
  float vw;
}  chassis_odom_info_t;


typedef struct
{
    float vx;
    float vy;
    float vw;
    float yaw;
    float pitch;
}robot_ctrl_info_t;

typedef struct
{

  uint16_t R;
  uint16_t G;
  uint16_t B;

} RGB_info_t;

typedef struct
{
  uint16_t id;
  uint16_t shoot_sta;
  float pitch;
  float yaw;
  float roll;
  float shoot;
} vision_t;

typedef struct
{

  int16_t ch[5];
  char s[2];

}  rc_info_t;


#pragma pack(pop)
#endif //ROBOMASTER_PROTOCOL_H
