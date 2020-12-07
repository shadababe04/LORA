/*
 * ev_queue.h
 *
 *  Created on: 15 Feb 2018
 *      Author: TE192184
 */

#ifndef __EV_QUEUE_H__
#define __EV_QUEUE_H__

#include <stdbool.h>
#include <stdio.h>

#define QUEUE_CAPACITY 20

typedef enum {
    EVENT_NO_EVENT,
    EVENT_GOT_RX_DATA,
    EVENT_GOT_RX_TIMEOUT,
    EVENT_GOT_RX_ERROR,
    EVENT_GOT_TX_COMPLETE,
    EVENT_GOT_TX_TIMEOUT,
    EVENT_TIMER_FIRED,
    EVENT_COMP_TRIGGERED,
    EVENT_COMP_TIMER_FIRED
} Queue_Events_t;

typedef struct {
    Queue_Events_t events[QUEUE_CAPACITY];
    size_t head;
    size_t tail;
    size_t size;
} EventQueue;

bool initQueue(EventQueue* pQueue);
bool isEmpty(EventQueue* pQueue);
bool enqueue(EventQueue* pQueue, Queue_Events_t item);
Queue_Events_t dequeue(EventQueue* pQueue);

#endif /* __EV_QUEUE_H__ */
