#include "queue.h"
//Queue q;

Buf_queue w;

void init_q(void){
    //q.memorySize = Q_SIZE;
    //q.size       = SIZE_INIT;
    //q.head       = HEAD_INIT;
    //q.tail       = TAIL_INIT;

    w.memorySize  = STRING_QUEUE_SIZE;
    w.size        = SIZE_INIT;
    w.head        = HEAD_INIT;
    w.tail        = TAIL_INIT;
    w.tail_string = TAIL_INIT;
}

//int enqueue_vector(dma_vector_t value){
//    // enqueue a byte onto the queue. returns 1 if success, 0 if it couldn't add it
//	if (q.size == q.memorySize) {
//		return 0; // full
//	}
//	q.values[q.tail].milli = value.milli;
//    q.values[q.tail].seconds = value.seconds;
//    q.values[q.tail].minutes = value.minutes;
//
//    q.values[q.tail].accX  = value.accX;
//    q.values[q.tail].accY  = value.accY;
//    q.values[q.tail].accZ  = value.accZ;
//    q.values[q.tail].gyroX = value.gyroX;
//    q.values[q.tail].gyroY = value.gyroY;
//    q.values[q.tail].gyroZ = value.gyroZ;
//    q.values[q.tail].emg   = value.emg;
//
//	q.tail = (q.tail + 1) % q.memorySize;
//	q.size++;
//
//	return 1;
//}
//
//int dequeue_vector(dma_vector_t *data) {
//    if(q.size == 0){
//        return 0;
//    }
//    // dequeue a values off the queue
//	data->milli    = q.values[q.head].milli;
//    data->seconds  = q.values[q.head].seconds;
//    data->minutes  = q.values[q.head].minutes;
//
//    data->accX     = q.values[q.head].accX;
//    data->accY     = q.values[q.head].accY;
//    data->accZ     = q.values[q.head].accZ;
//    data->gyroX    = q.values[q.head].gyroX;
//    data->gyroY    = q.values[q.head].gyroY;
//    data->gyroZ    = q.values[q.head].gyroZ;
//    data->emg      = q.values[q.head].emg;
//
//	q.head = (q.head + 1) % q.memorySize;
//	q.size--;
//	return 1;
//}

int enqueue_string(char *string) {
    // Check if the queue is already full.
    if (w.size == w.memorySize) {
        return 0; // Return 0 to indicate that the queue is full.
    }

    // Iterate through the characters in the input 'string' and copy them into the queue.
    while (*string != '\0' && w.size != w.memorySize) {
        // Copy the current character from 'string' into the current buffer in the queue.
        w.buf[w.tail][w.tail_string] = *string;
        string++; // Move to the next character in 'string'.
        w.tail_string++; // Move to the next position in the current buffer.

        // If the current buffer is full (512 characters), wrap around to the next buffer.
        if (w.tail_string == MAX_STRING_SIZE - 1) {
            w.buf[w.tail][w.tail_string] = '\0';
            w.tail_string = 0; // Reset the buffer position.
            w.tail = (w.tail + 1) % w.memorySize; // Move to the next buffer.
            w.size++; // Increment the size of the queue.
        }
    }

    return 1; // Return 1 to indicate successful enqueuing.
}

int dequeue_string(char *string, int num_of_blocks) {
    int str_len = 0;
    int blocks_dequeued = 0; // Track the number of blocks dequeued

    // Check if the queue is empty.
    if (w.size == 0) {
        return 0; // Return 0 to indicate that the queue is empty.
    } else if (num_of_blocks > w.size) {
        return -1; // Return -1 to indicate that there are not enough blocks in the queue.
    }

    // Iterate through the queue and concatenate strings until the specified number of blocks are dequeued.
    while (blocks_dequeued < num_of_blocks) {
        // Concatenate the next string from the queue into the 'string' buffer.
        str_len += fuseStrings(string, w.buf[w.head]);
        w.head = (w.head + 1) % w.memorySize; // Move to the next element in the queue.
        w.size--; // Decrement the size of the queue.
        blocks_dequeued++; // Increment the number of blocks dequeued.
    }

    return str_len; // Return the length of the concatenated string.
}

int queue_str_len(void) {
    return w.size; // Return the size of the queue.
}

int empty_string_queue(char *dest, int max_block_size){
    int str_len = 0;
    int blocks_dequeued = 0;
    
    for(int i = max_block_size; i > 0; i--){
        str_len = dequeue_string(dest,i);
        if(str_len > 0){
            blocks_dequeued = i;
            break;
        }
    }
    if(w.size == 0 && blocks_dequeued < max_block_size){
        w.buf[w.head][w.tail_string] = '\0';
        str_len = fuseStrings(dest, w.buf[w.head]);
        w.tail_string = 0;
    }
    return str_len;
}

int fuseStrings(char *str1, const char *str2) {
    char *start = str1;  // Save the starting position of str1
    // Find the end of the first string
    while (*str1 != '\0') {
        str1++;
    }
    // Copy characters from the second string to the end of the first string
    while (*str2 != '\0') {
        *str1 = *str2;
        str1++;
        str2++;
    }
    // Add the null terminator to mark the end of the concatenated string
    *str1 = '\0';
    // Calculate and return the length of the concatenated string
    return str1 - start;
}

void clear_queues(void) {
    // Reset the main queue (q)
    //q.size = SIZE_INIT;
    //q.head = HEAD_INIT;
    //q.tail = TAIL_INIT;

    // Reset the string queue (w)
    w.size = SIZE_INIT;
    w.head = HEAD_INIT;
    w.tail = TAIL_INIT;
    w.tail_string = TAIL_INIT;

    // Clear values in the main queue
    //for (int i = 0; i < Q_SIZE; i++) {
    //    q.values[i].milli = 0;
    //    q.values[i].seconds = 0;
    //    q.values[i].minutes = 0;
    //    q.values[i].accX = 0;
    //    q.values[i].accY = 0;
    //    q.values[i].accZ = 0;
    //    q.values[i].gyroX = 0;
    //    q.values[i].gyroY = 0;
    //    q.values[i].gyroZ = 0;
    //    q.values[i].emg = 0;
    //}

    // Clear values in the string queue
    for (int i = 0; i < STRING_QUEUE_SIZE; i++) {
        for (int j = 0; j < MAX_STRING_SIZE; j++) {
            w.buf[i][j] = '\0';
        }
    }
}


