#include <stdint.h>
#include <string.h>

#define CAN_HEAD 0xAA
#define CAN_TAIL 0x55
#define CAN_DATA_IN_BYTE 8
#define CAN_LEN_IN_BYTE CAN_DATA_IN_BYTE+2+2+1


typedef struct {
    int id;
    int len;
    uint8_t data[8];
} can_frame;

// typedef struct {
//     uint8_t head; // 0xAA

//     uint8_t dlc; // 数据长度 + 0xC0

//     uint8_t id_low;
//     uint8_t id_high;  

//     uint8_t data[CAN_DATA_IN_BYTE];
//     uint8_t end; // 0x55
// } can_frame_encoded;


// Function to encode a can_frame into a can_frame_encoded
void encode_can_frame(const can_frame *input, uint8_t output[CAN_LEN_IN_BYTE]) {
    output[0] = CAN_HEAD;
    output[1] = 0xC0 + input->len;
    output[2] = input->id & 0xFF;
    output[3] = (input->id >> 8) & 0xFF;
    for (int i = 0; i < CAN_DATA_IN_BYTE; i++)
        output[4 + i] = input->data[i];
    output[CAN_LEN_IN_BYTE - 1] = CAN_TAIL;
}

// Function to decode a can_frame_encoded into a can_frame
void decode_can_frame(uint8_t *input, can_frame *output) {
    output->id = input[2] + (input[3] << 8);
    output->len = input[1] - 0xC0;
    for (int i = 0; i < CAN_DATA_IN_BYTE; i++)
        output->data[i] = input[4 + i];
}