#ifndef _FIFO_H_
#define _FIFO_H_

#define FIFO_SIZE 100


int push_to_tx_fifo(uint8_t byte);
uint8_t pop_tx_fifo(void);
int fifo_data_available(void);


#endif // _FIFO_H_
