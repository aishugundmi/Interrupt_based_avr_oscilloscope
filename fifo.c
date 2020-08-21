#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>

#include "fifo.h"

uint8_t buff[FIFO_SIZE];
int write_index;
int read_index;
uint8_t unread_count;

int push_to_tx_fifo(uint8_t byte)
{

    int ret = -1;  //init return with error
    //if no space in fifo, return error
    if(fifo_data_available() >= FIFO_SIZE)
    {
        return ret;
    }
    //write data to fifo
    buff[write_index] = byte;

    //increment unread count
    unread_count++;

    //increament write index and check if it is overflowing, if so make it zero
    write_index++;
    if(write_index >= FIFO_SIZE)
    {
        write_index = 0;
    }

    ret = 0;
    return ret;

}

uint8_t pop_tx_fifo(void)
{

	uint8_t byte;

    int ret = -1;  //return -1 if no data to read. If not, copy the uint8_t byte to int variable and return.
                    //so that if the return is not -1, then consider it as a valid data byte,(0 to 0xff),
    if(unread_count == 0)
    {
        return ret;
    }

    byte = buff[read_index];

    read_index++;
    unread_count--;

    if(read_index >= FIFO_SIZE)
    {
        read_index = 0;
    }

    ret = (int)(byte);
  //  uprintf(ret);
    return ret;

}

int fifo_data_available(void)
{
    return unread_count;
}
