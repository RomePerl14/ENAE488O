//  By Ben Tebeest

#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include "kilolib.h"

#define MAX_NEIGHBORS 5

// Neighbor tracking
uint8_t global_neighbor_count = 1; // always at least 1
uint8_t global_neighbors[MAX_NEIGHBORS]; // global neighbor array indexed by UID
uint8_t local_neighbors[MAX_NEIGHBORS];  // local neighbor array indexed by UID

// Communication
message_t message;
uint8_t new_message = 0;
uint8_t message_sent = 0;
uint8_t ticks_since_last_update = 0;
uint8_t ticks_per_update = 128; // 32 = 1 second

// color indicators for neighbor count - including self (i.e. always at least 1 neighbor)
uint8_t colors[] = {
    RGB(3,3,3),  //1 - white
    RGB(3,0,0),  //2 - red
    RGB(3,3,0),  //3 - yellow
    RGB(0,3,0),  //4 - green
    RGB(0,0,3)   //5 - blue
};

// array sum function
uint8_t sum_array(uint8_t arr[], size_t size) {
    uint8_t sum = 0;
    for (size_t i = 0; i < size; i++) {
        sum += arr[i];
    }
    return sum;
}

void setup(){
    message.type = NORMAL;

    // initialize global & local neighbor array sorted by UID
    int i;
    for (i = 0; i < MAX_NEIGHBORS; i++){
        if (i == kilo_uid){
            global_neighbors[i] = 1;
            local_neighbors[i] = 1;
        }
        else{
            global_neighbors[i] = 0;
            local_neighbors[i] = 0;
        }
    }

    // store this robots UID and global neighbors in message
    message.data[0] = (uint8_t) kilo_uid;
    message.data[1] = (uint8_t) global_neighbors[0];
    message.data[2] = (uint8_t) global_neighbors[1];
    message.data[3] = (uint8_t) global_neighbors[2];
    message.data[4] = (uint8_t) global_neighbors[3];
    message.data[5] = (uint8_t) global_neighbors[4];

    // zero out the rest of the data
    for(i = 7; i < 9; i++)
    {
      message.data[i] = 0;
    }

    // It's important that the CRC is computed after the data has been set;
    // otherwise it would be wrong.
    message.crc = message_crc(&message);

}


// message transmit callback
message_t *message_tx() {
    // message type always normal
    message.type = NORMAL;

    // store this robots UID and T/F values of it's global neighbor array in message
    message.data[0] = (uint8_t) kilo_uid;
    message.data[1] = (uint8_t) global_neighbors[0];
    message.data[2] = (uint8_t) global_neighbors[1];
    message.data[3] = (uint8_t) global_neighbors[2];
    message.data[4] = (uint8_t) global_neighbors[3];
    message.data[5] = (uint8_t) global_neighbors[4];

    // compute checksum
    message.crc = message_crc(&message);

    // send message
    return &message;
}

// message transmit success callback 
void message_tx_success() {
    // set message tx flag
    message_sent = 1; 
}


void message_rx(message_t *m, distance_measurement_t *d) {

    // check sender UID for updating local neighbors
    uint8_t sender_UID = m->data[0]; 

    // update local & global neighbor array if sender UID not already accounted for
    if (local_neighbors[sender_UID] == 0) {
        local_neighbors[sender_UID] = 1;
        global_neighbors[sender_UID] = 1;
    }

    // store sender's global neighbors in temp array (just for clarity)  
    // and update receiver's global neighbor array if sender has a robot
    // in it's global neighbor array that the receiver does not
    uint8_t sender_global_neighbors[MAX_NEIGHBORS];
    int i;
    for (i=0; i < MAX_NEIGHBORS; i++) {
        sender_global_neighbors[i] = m->data[i+1];
        if (sender_global_neighbors[i] == 1 && global_neighbors[i] != 1) {
            global_neighbors[i] = 1;
        } 
    }

    // message rx flag
    new_message = 1;
}

void set_color_based_on_global_neighbors(uint8_t gn_count) {
    switch(gn_count) {
        case 1: set_color(colors[0]); break;
        case 2: set_color(colors[1]); break;
        case 3: set_color(colors[2]); break;
        case 4: set_color(colors[3]); break;
        case 5: set_color(colors[4]); break;
    }
}

void loop() {
    
        // ***** CHANGE THIS IF SHIT BREAKS ******************
    // periodically re-initialize global & local neighbor arrays 
    // (for re-checking if a previously present neighbor has left)
    if (kilo_ticks - ticks_since_last_update > ticks_per_update) {
        
        int i;
        for (i = 0; i < MAX_NEIGHBORS; i++){
            if (i == kilo_uid) {
                global_neighbors[i] = 1;
                local_neighbors[i] = 1;
            }
            else {
                global_neighbors[i] = 0;
                local_neighbors[i] = 0;
            }
        }
        ticks_since_last_update = kilo_ticks;

        // store this robots UID and global neighbors in message
        message.data[0] = (uint8_t) kilo_uid;
        message.data[1] = (uint8_t) global_neighbors[0];
        message.data[2] = (uint8_t) global_neighbors[1];
        message.data[3] = (uint8_t) global_neighbors[2];
        message.data[4] = (uint8_t) global_neighbors[3];
        message.data[5] = (uint8_t) global_neighbors[4];

        // zero out the rest of the data
        for(i = 7; i < 9; i++)
        {
        message.data[i] = 0;
        }

        // It's important that the CRC is computed after the data has been set;
        // otherwise it would be wrong.
        message.crc = message_crc(&message);
    }
    // ************************************************

    // sum global_neighbors array elements (only either 1 or 0) to get neighbor count
    size_t array_size = sizeof(global_neighbors) / sizeof(global_neighbors[0]); // computes array size by dividing total bytes of array by the bytes in the first index
    global_neighbor_count = sum_array(global_neighbors, array_size);

    // display color based on global neighbor count
    set_color_based_on_global_neighbors(global_neighbor_count);

    // reset message flags
    message_sent = 0;
    new_message = 0;

}


int main()
{
    kilo_init();
    // Register the message_tx callback function.
    kilo_message_tx = message_tx;

    // Register the message_tx_success callback function.
    kilo_message_tx_success = message_tx_success;

    // Register the message_tx_success callback function.
    kilo_message_rx = message_rx;

    kilo_start(setup, loop);
    
    return 0;
}