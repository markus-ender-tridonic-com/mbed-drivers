/* mbed Microcontroller Library
 * Copyright (c) 2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stddef.h>
#include <stdbool.h>
#include "ticker_api.h"
#include "cmsis.h"

inline bool ticker_timeIsInPeriod(timestamp_t start, timestamp_t time, timestamp_t end);

void ticker_set_handler(const ticker_data_t *const data, ticker_event_handler handler) {
    data->interface->init();

    data->queue->event_handler = handler;
}

void ticker_irq_handler(const ticker_data_t *const data) {
    data->interface->clear_interrupt();

    /* Go through all the pending TimerEvents */
    while (1) {
        if (data->queue->head == NULL) {
            // There are no more TimerEvents left, so disable matches.
            data->interface->disable_interrupt();
            return;
        }

        timestamp_t ticksNow = data->interface->read();		// read actual timer value
        uint32_t diff = data->queue->head->timestamp - ticksNow;
        diff &= TICKER_TIME_MASK;
        if ((diff <= TICKER_FUTURE_TOLERANCE) ||	// also execute event in 'near' future (are otherwise lost because of us-resolution)
            (diff > TICKER_PAST_TOLERANCE)) 		// timestamp just missed (not 'too long' in the past)
        {
            // This events timeout is reached:
            //      point to the following one and execute its handler
            ticker_event_t *p = data->queue->head;
            data->queue->head = data->queue->head->next;

            if (data->queue->event_handler != NULL) {
                (*data->queue->event_handler)(p->id); // NOTE: the handler can set new events
            }
            /* Note: We continue back to examining the head because calling the
             * event handler may have altered the chain of pending events. */
        }
        else {
            // This event and the following ones in the list are in the future:
            //      set it as next interrupt and return
            data->interface->set_interrupt(data->queue->head->timestamp);
            return;
        }
    }
}

void ticker_insert_event(const ticker_data_t *const data, ticker_event_t *obj, timestamp_t timestamp, uint32_t id) {
    /* disable interrupts for the duration of the function */
    __disable_irq();

    timestamp_t actCnt = data->interface->read();		// read actual timer counter

    /* reduce actual timer count because it counts up (us-resolution)
     * while periodic events are executed and re-inserted;
     * this can be a problem if timeout-counts are close together  */
    actCnt -= TICKER_EXPECT_ISR_DELAY;
    timestamp &= TICKER_TIME_MASK;						// limit timeout value acc. used timer

    // initialise our data
    obj->timestamp = timestamp;
    obj->id = id;

    /* Go through the list until we either reach the end, or find
       an element this should come before (which is possibly the
       head). */
    ticker_event_t *prev = NULL, *p = data->queue->head;
    while (p != NULL) {
        /* check if we come before p */
        if(ticker_timeIsInPeriod(actCnt, timestamp, p->timestamp)) {
            break;
        }
        /* go to the next element */
        prev = p;
        p = p->next;
    }
    /* if prev is NULL we're at the head */
    if (prev == NULL) {
        data->queue->head = obj;
        data->interface->set_interrupt(timestamp);
    } else {
        prev->next = obj;
    }
    /* if we're at the end p will be NULL, which is correct */
    obj->next = p;

    __enable_irq();
}

void ticker_remove_event(const ticker_data_t *const data, ticker_event_t *obj) {
    __disable_irq();

    // remove this object from the list
    if (data->queue->head == obj) {
        // first in the list, so just drop me
        data->queue->head = obj->next;
        if (data->queue->head == NULL) {
            data->interface->disable_interrupt();
        } else {
            data->interface->set_interrupt(data->queue->head->timestamp);
        }
    } else {
        // find the object before me, then drop me
        ticker_event_t* p = data->queue->head;
        while (p != NULL) {
            if (p->next == obj) {
                p->next = obj->next;
                break;
            }
            p = p->next;
        }
    }

    __enable_irq();
}

timestamp_t ticker_read(const ticker_data_t *const data)
{
    return data->interface->read();
}

//                                            actCnt        insertCnt          nextCnt
inline bool ticker_timeIsInPeriod(timestamp_t start, timestamp_t time, timestamp_t end)
{
    // Taking care to handle wrapping:
    //   Case (A.1)
    //                       A    I   N
    //      0 ---------------|----|---|-- 0xf
    //
    //   Case (A.2): this case also allows S==T==E
    //         N                 A    I
    //      0 -|-----------------|----|-- 0xf
    //
    //   Case (B)
    //         I   N                 A
    //      0 -|---|-----------------|--- 0xf
    //
    if((time >= start && ( time < end ||              // (A.1)
                          start >= end)) ||           // (A.2)
        (time < start && end < start && end > time))  // (B)
    {
        return true;
    }
    return false;
}
