/*
 * This program simulates a TouchSync client on a PC
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

timestamp_t t0; // store starting timestamp

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
	int sock;
	struct sockaddr_in server;
	char message[1000], server_reply[2000];
	offset_t offset; // clock offset from the master

	// record start timestamp
	t0 = get_local_clock_us();

	// create the sensor sampling thread
	pthread_t sensor_thread;
	if (pthread_create(&sensor_thread, NULL, sample, &lock_rw)) {
		fprintf(stderr, "error creating threat\n");
		return 1;
	}
	usleep(1000000); // wait until the circular buffer has been filled with some data

	// create socket
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == -1)
	{
		fprintf(stderr, "Could not create socket\n");
		return 1;
	}
	server.sin_addr.s_addr = inet_addr("127.0.0.1");
	server.sin_family = AF_INET;
	server.sin_port = htons(8888);
	// connect to remote server
	if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0)
	{
		fprintf(stderr, "connect failed. Error\n");
		return 1;
	}
	// connection established

	while (1)
	{
		strcpy(message, "request"); // create a request message
		t1 = get_relative_clock_us(); // timestamp the transmission of the request message
		usleep(rand() % 100000); // simulate a random packet transmission delay that is uniformly distributed within [0,100ms]
		if (send(sock, message, strlen(message), 0) < 0)
		{
			fprintf(stderr, "send request failed.");
			close(sock);
			return 1;
		}

		// receive the reply1 packet
		if (recv(sock, server_reply, 2000, 0) < 0)
		{
			fprintf(stderr, "receive reply1 failed");
			close(sock);
			return 1;
		}
		usleep(rand() % 100000); // simulate a random packet transmission delay that is unformly distributed within [0,100ms]
		t4 = get_relative_clock_us(); // tiemstamp the reception of the reply1 message	  
		strcpy(message, "ack");
		send(sock, message, strlen(message), 0); // send an ack message
		usleep(60000);
		on_receive_reply1();

		// receive the reply2 packet
		if (recv(sock, server_reply, 2000, 0) < 0)
		{
			fprintf(stderr, "receive reply2 failed");
			close(sock);
			return 1;
		}

		// parse the reply2 packet
		sscanf(server_reply, "%lu,%lu,%lu,%lu", &t2, &t3, &phi2, &phi3);

		// use the routine provided by the touchsync.h
		// if the integer ambiguity problem can be solved, stop
		if (on_receive_reply2(&offset)) break;

		usleep(1000000); // wait one second before the next synchronization session
	}

	close(sock);

	printf("offset=%d\n", (int)offset);

	stop_sampling = 1; // notify the sampling thread to stop
	// wait for the sampling thread to finish
	if (pthread_join(sensor_thread, NULL)) {
		fprintf(stderr, "Error joining thread.\n");
		return 1;
	}

	return 0;
};
