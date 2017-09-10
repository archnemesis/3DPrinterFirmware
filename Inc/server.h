/*
 * server.h
 *
 *  Created on: Jul 22, 2017
 *      Author: Robin
 */

#ifndef SERVER_H_
#define SERVER_H_

#include "FreeRTOS.h"
#include "queue.h"

#define CONNECT_MESSAGE	"ARMCTRL V1.0\n"

extern QueueHandle_t incomingQueue;

void server_thread(void const * argument);

/**
 * Place data on the ougoing queue to be sent
 * to all connected clients.
 */
void server_write(const char * string);

#endif /* SERVER_H_ */
