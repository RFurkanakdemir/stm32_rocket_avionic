/*
 * lora_lib.c
 *
 *  Created on: Jan 31, 2023
 *      Author: rfrkn
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "lora_lib.h"
#include "stm32f1xx.h"



uint8_t check_sum_hesapla( const uint8_t * const ptVeri , const uint8_t START_IDX , const uint8_t END_IDX )
{
    uint32_t check_sum = 0;

    for( uint8_t i = START_IDX ; i < END_IDX ; i++ )
    {
        check_sum += ptVeri[i];
    }
    return ( uint8_t ) ( check_sum % 256 ) ;
}

void veriPaketle ( dataPaket_t * const pkt , const  dataStruct_t * const data )
{
    memcpy ( &( pkt->data ) , data , sizeof( dataStruct_t ) );
    pkt->u8_crc_data = check_sum_hesapla( pkt->u8_array , 4 , sizeof( dataPaket_t )-3 );
}

void verileriYolla(uint8_t* ptVeri, const uint8_t veriLength)
{

	uint8_t status=0;
    status=E32_Transmit(ptVeri, veriLength);
    //status = HAL_UART_Transmit(&_huart, ptVeri, veriLength, 1000);

}

void initDataPaket( dataPaket_t * const pkt , const uint8_t WHICH_NODE )
{
    memset( pkt->u8_array , 0 , sizeof( dataPaket_t ) );
    pkt->u32_start_header       = 0x5254FFFF; // 4
    pkt->u8_node_information    = WHICH_NODE ;  // 1
    pkt->u8_package_length      = sizeof( dataPaket_t );
    pkt->u16_end_header         = 0x0A0D;   // 2
}

