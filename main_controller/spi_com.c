#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif




void test_pattern(int spi_handle){
    uint8_t * received_data = (uint8_t *) malloc(sizeof(uint8_t) * 5);

    uint8_t data[4] = {0x00,0x00,0x00,0x00};
    uint8_t address = 0x17;
    SPI_send(data,spi_handle,received_data);
    int errors = 0;
    for (int i = 0; i < 5; ++i) {
        if(received_data[i] != i){
            errors = 1;
        }
    }
    if(errors == 0){
        fprintf(stderr,"Test 1 OK \n");
    }
    else {
        fprintf(stderr,"Test 1 échoué \n");
    }

}
int initializeSPI(int channel){
    int freq = 500000;
    int spi_handle = spiOpen(channel,freq,0);
    if(spi_handle < 0){
        fprintf(stderr,"Error while SPI handling \n");
    }
    return spi_handle;
}

void SPI_send(uint8_t send_data[4],int spi_handle, uint8_t *received_data){
    uint8_t data[4] = {send_data[0],send_data[1],send_data[2],send_data[3]};
    spiXfer(spi_handle,data,received_data,4);
}

void close_spi(int spi_handle){
    spiClose(spi_handle);
}


void print_data(uint8_t *received_data){
    for (int i = 0; i < 4; ++i) {
        fprintf(stderr,"%d",received_data[i]);
    }
}
