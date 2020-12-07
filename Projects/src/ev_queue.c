/*
 * ev_queue.c
 *
 *  Created on: 15 Feb 2018
 *      Author: TE192184
 */

#include "ev_queue.h"

bool initQueue(EventQueue* pQueue) {
    if (!pQueue) {
        return false;
    }

    pQueue->head = 0;
    pQueue->tail = 0;
    pQueue->size = 0;
    return true;
}

bool isEmpty(EventQueue* pQueue) {
    return pQueue && pQueue->size == 0;
}

bool enqueue(EventQueue* pQueue, Queue_Events_t item) {
    if (!pQueue || pQueue->size == QUEUE_CAPACITY) // when queue is full
    {
        return false;
    }

    pQueue->events[pQueue->tail] = item;
    pQueue->tail = (pQueue->tail + 1) % QUEUE_CAPACITY;
    pQueue->size++;
    return true;
}

Queue_Events_t dequeue(EventQueue* pQueue) {
    // Return NULL when queue is empty
    // Return (void*)item at the head otherwise.
    Queue_Events_t item;

    if (!pQueue || isEmpty(pQueue)) {
        return EVENT_NO_EVENT;
    }

    item = pQueue->events[pQueue->head];
    pQueue->head = (pQueue->head + 1) % QUEUE_CAPACITY;
    pQueue->size--;
    return item;
}

