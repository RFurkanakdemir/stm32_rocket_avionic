/*
 * lora_lib.h
 *
 *  Created on: Jan 31, 2023
 *      Author: rfrkn
 */

#ifndef INC_LORA_LIB_H_
#define INC_LORA_LIB_H_



#endif /* INC_LORA_LIB_H_ */

#include <E32Lora.h>


typedef struct dataStruct
{
    float   irtifa;
    float   roket_gps_irtifa;
    float   roket_gps_enlem;
    float   roket_gps_boylam;
    float   gyro_x;
    float   gyro_y;
    float   gyro_z;
    float   acc_x;
    float   acc_y;
    float   acc_z;
    float   angle;
    uint8_t status;

}__attribute__( ( packed ) ) dataStruct_t;


enum NODE_INFORMATION
{
    ROCKET_NODE     = 0xAA,
    ROCKET_NODE_2   = 0xAB,
    PAYLOAD_NODE    = 0xBB,
    GCS_NODE        = 0xCC
};



typedef union dataPaket
{

    struct
    {
        uint32_t        u32_start_header; // 4
        uint8_t         u8_node_information;  // 1
        uint8_t         u8_package_length;    // 1
        dataStruct_t    data;
        uint8_t         u8_crc_data;          // 1
        uint16_t        u16_end_header;   // 2
    }__attribute__( ( packed ) );

    uint8_t u8_array[ sizeof( dataStruct_t ) + ( 4 + 1 + 1 + 1 + 2 ) ];
}dataPaket_t;

typedef struct Variables
{
    uint32_t    telemTimer;
    uint8_t     firstinit;
    uint8_t     u8_buffer[ 180 ];
    uint8_t     u8_counter;
}Variables_t;

enum GCS_COMMANDS
{
    RESEND_TELEMETRY = 0xBA
};
typedef struct GcsPaket
{
    uint32_t        u32_start_header; // 4
    uint8_t         u8_node_information;  // 1
    uint8_t         u8_package_length;    // 1
    uint8_t         u8_package_receiver;
    uint8_t         u8_command;
    uint16_t        u16_end_header;   // 2
}__attribute__( ( packed ) ) GcsPaket_t;


uint8_t check_sum_hesapla( const uint8_t * const ptVeri , const uint8_t START_IDX , const uint8_t END_IDX );
void veriPaketle ( dataPaket_t * const pkt , const  dataStruct_t * const data );
void verileriYolla(uint8_t* ptVeri, const uint8_t veriLength);
void initDataPaket( dataPaket_t * const pkt , const uint8_t WHICH_NODE );
