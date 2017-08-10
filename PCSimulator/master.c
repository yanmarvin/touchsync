/*
 * This programs simulates a TouchSync master on a PC
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

#define _PCSIM_
#include "../touchsync.h"

#define PI 3.1415926535

int stop_sampling = 0; // used by the main thread to notify the sampling thread to stop
timestamp_t t0;  // store starting timestamp
pthread_rwlock_t lock_rw = PTHREAD_RWLOCK_INITIALIZER; // read/write lock for the circular buffer in touchsync.h

// get a timestamp in microsecond
timestamp_t get_local_clock_us() {
	struct timeval tp;
	timestamp_t us;
	gettimeofday(&tp, NULL);
	us = tp.tv_sec * 1000000;
	us += tp.tv_usec;
	return us;
};

// get a relative timestamp in microsecond
timestamp_t get_relative_clock_us() {
	struct timeval tp;
	timestamp_t us;
	gettimeofday(&tp, NULL);
	us = tp.tv_sec * 1000000;
	us += tp.tv_usec;
	us -= t0;
	return us;
};

// sampling thread: it simulates periodic SEP sensor sampling and pushes each sample to the circular buffer
// arg: read/write lock
void* sample(void *arg)
{
	timestamp_t t;
	double t_;
	sigval_t v;
	pthread_rwlock_t *_lock_rw = (pthread_rwlock_t *)arg;

	while (1)
	{
		t = get_relative_clock_us();
		t_ = ((double)t) / 1000000; // convert to seconds
		v = (sigval_t)(512 * sin(2 * PI * 50 * t_) + 512); // generate a new sample within the range [0,1024]

		pthread_rwlock_wrlock(_lock_rw); // acquire write lock of the circular buffer
		buf_add(t, v); // push the new sample to the circular buffer
		pthread_rwlock_unlock(_lock_rw); // release write lock

		usleep(3000); // sleep for 3 milliseconds to simulate 333Hz sampling
		if (stop_sampling) break;
	}

	return NULL;
};

int main(int argc, char *argv[])
{
	int socket_desc, client_sock, c, read_size;
	struct sockaddr_in server, client;
	char client_message[2000];

	// record start timestamp
	t0 = get_local_clock_us();

	// create the sensor sampling thread
	pthread_t sensor_thread;
	if (pthread_create(&sensor_thread, NULL, sample, &lock_rw)) {
		fprintf(stderr, "error creating threat\n");
		return 1;
	}

	// create socket
	socket_desc = socket(AF_INET, SOCK_STREAM, 0);
	if (socket_desc == -1)
	{
		fprintf(stderr, "Could not create socket");
		return 1;
	}
	// prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(8888);
	// bind
	if (bind(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0)
	{
		fprintf(stderr, "bind failed. Error");
		return 1;
	}
	// listen
	listen(socket_desc, 3);
	// accept incoming connection
	c = sizeof(struct sockaddr_in);
	client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
	if (client_sock < 0)
	{
		fprintf(stderr, "accept failed");
		return 1;
	}
	// connection established

	// receive the request message from slave
	while ((read_size = recv(client_sock, client_message, 2000, 0)) > 0)
	{
		t2 = get_relative_clock_us(); // timestamp the received request packet

		usleep(10000); // simulate 10 ms processing delay

		strcpy(client_message, "reply1"); // create a reply1 message
		t3 = get_relative_clock_us(); // timestamp the transmission of the reply1 packet
		write(client_sock, client_message, strlen(client_message));
		recv(client_sock, client_message, 2000, 0); // receive ack to reply1

		send_reply1_done(); // use the routine provided by touchsync.h

		sprintf(client_message, "%lu,%lu,%lu,%lu", t2, t3, phi2, phi3); // create a reply2 message containing t2, t3, phi2, phi3
		write(client_sock, client_message, strlen(client_message)); // transmit the reply2 message
	}

	if (read_size == 0) // client disconnected
	{
		fflush(stdout);
	}
	else if (read_size == -1)
	{
		fprintf(stderr, "Failed to receive request message.\n");
	}

	stop_sampling = 1; // notify the sampling thread to stop
	// wait for the sampling thread to finish
	if (pthread_join(sensor_thread, NULL)) {
		fprintf(stderr, "Error joining thread.\n");
		return 1;
	}

	return 0;
};
