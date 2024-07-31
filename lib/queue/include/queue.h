#pragma once 

#include "gd32vf103.h"
#include "dma.h"

//#define Q_SIZE 512
#define MEM_SIZE Q_SIZE
#define SIZE_INIT 0
#define HEAD_INIT 0 
#define TAIL_INIT 0
#define STRING_QUEUE_SIZE 32
#define MAX_STRING_SIZE 512

//typedef struct{
//    dma_vector_t values[Q_SIZE];
//    int memorySize;
//    int size;
//    int head;
//    int tail;
//}Queue;

typedef struct{
    char buf[STRING_QUEUE_SIZE][MAX_STRING_SIZE];
    int memorySize;
    int size;
    int head;
    int tail;
    int tail_string;    
}Buf_queue;

void init_q(void);
//int enqueue_vector(dma_vector_t value);
//int dequeue_vector(dma_vector_t *data);
int enqueue_string(char *string);
int dequeue_string(char *string, int num_of_blocks);
int queue_str_len(void);
int empty_string_queue(char *dest, int max_block_size);
int fuseStrings(char *str1, const char *str2);
void clear_queues(void);
//int value_in_buffer(void);