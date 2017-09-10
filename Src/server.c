/*
 * server.c
 *
 *  Created on: Jul 22, 2017
 *      Author: Robin
 */

#include "server.h"
#include "sockets.h"
#include "cmsis_os.h"

#include <string.h>

QueueHandle_t incomingQueue;
QueueHandle_t outgoingQueue;

void server_write(const char * string)
{
	char *str = pvPortMalloc(strlen(string) + 1);
	memset(str, 0, strlen(string) + 1);
	memcpy(str, string, strlen(string));
	xQueueSendToBack(outgoingQueue, (void *)&str, portMAX_DELAY);
}

void server_thread(void const * argument)
{
	(void)argument;

	incomingQueue = xQueueCreate(10, sizeof(char *));
	outgoingQueue = xQueueCreate(10, sizeof(char *));

	int socket_fd;
	struct sockaddr_in sa, ra;
	struct fd_set master_set;
	struct fd_set working_set;
	struct fd_set writing_set;
	struct timeval timeout;
	int max_fd, new_fd, ready_fd;
	int on = 1;
	int rc = 0;
	int i;
	uint32_t opt;
	char buffer[128];

	socket_fd = socket(PF_INET, SOCK_STREAM, 0);

	if (socket_fd < 0) {
		__BKPT();
	}

	opt = fcntl(socket_fd, F_GETFL, 0);
	opt |= O_NONBLOCK;
	fcntl(socket_fd, F_SETFL, opt);

	memset(&sa, 0, sizeof(struct sockaddr_in));
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_ANY);
	sa.sin_len = sizeof(sa);
	sa.sin_port = htons(100);

	if (bind(socket_fd, (struct sockaddr *)&sa, sizeof(sa)) == -1) {
		close(socket_fd);
		__BKPT();
	}

	if (listen(socket_fd, 1) == -1) {
		close(socket_fd);
		__BKPT();
	}

	FD_ZERO(&master_set);
	max_fd = socket_fd;
	FD_SET(socket_fd, &master_set);

	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	while (1)
	{
		memcpy(&working_set, &master_set, sizeof(master_set));
		memcpy(&writing_set, &master_set, sizeof(master_set));
		rc = select(max_fd + 1, &working_set, &writing_set, NULL, &timeout);

		if (rc < 0) {
			__BKPT();
		}

		if (rc == 0) {
			continue;
		}

		ready_fd = max_fd;
		for (i = 0; i <= ready_fd; i++) {
			if (i == socket_fd) {
				if (FD_ISSET(i, &working_set)) {
					do {
						on = sizeof(ra);
						new_fd = accept(socket_fd, (struct sockaddr *)&ra, (socklen_t *)&on);
						if (new_fd < 0) {
							if (errno != EWOULDBLOCK) {
								__BKPT();
							}
							break;
						}

						/* add socket to master set */
						FD_SET(new_fd, &master_set);

						/* keep track of the max */
						if (new_fd > max_fd) {
							max_fd = new_fd;
						}

						server_write("Welcome to the thing!\r\n");
					} while (new_fd != -1);
				}
			}
			else {
				if (FD_ISSET(i, &working_set)) {
					rc = recv(i, buffer, sizeof(buffer), 0);

					if (rc <= 0) {
						if (errno != EWOULDBLOCK) {
							close(i);
							FD_CLR(i, &master_set);
							if (i == max_fd) {
								while (FD_ISSET(max_fd, &master_set) == 0) {
									max_fd -= 1;
								}
							}
						}
					}
					else {
						if (uxQueueSpacesAvailable(incomingQueue) && rc > 0) {
							char *msg = pvPortMalloc(rc + 1);
							memset(msg, 0, rc + 1);
							strncpy(msg, buffer, rc);
							xQueueSendToBack(incomingQueue, (const void *)&msg, 0);
							memset(buffer, 0, sizeof(buffer));
						}
					}
				}
				if (FD_ISSET(i, &writing_set)) {
					char *msg;
					if (xQueueReceive(outgoingQueue, (void *)&msg, 0)) {
						if (send(i, msg, strlen(msg), 0) < 0) {
							close(i);
							FD_CLR(i, &master_set);
							if (i == max_fd) {
								while (FD_ISSET(max_fd, &master_set) == 0) {
									max_fd -= 1;
								}
							}
						}
						vPortFree(msg);
					}
				}
			}
		}

		osDelay(10);
	}
}
