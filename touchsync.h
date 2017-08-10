#ifndef TOUCHSYNC_H
#define TOUCHSYNC_H

#define BUF_SIZE 2048 // size of the circular buffer to store SEP samples
#define WORKING_BUF_SIZE 1024 // size of the working buffer to store a selected SEP signal segment
typedef unsigned long timestamp_t; // timestamp data type
typedef int sigval_t; // SEP signal value data type
typedef signed long offset_t; // clock offset data type

#define PI 3.1415926535
#define SIGNAL_T 20000
#define INF 65535
#define PRECISION_TAG 1000000 // for Z1 it is 32768
#define ENABLE_PLL 0 // turn on/off PLL. 1: on; 0: off

// some global variables for solving ambiguity equation
#define MAX_IJ 20  // define the maximum i+j
#define MAX_EPSILON_ERROR 3000
#define IS_PRIOR_KNOWLEDGE_USED 1 // whether use the assymetry knowledge of transmission
offset_t delta_q,delta_solution[MAX_IJ], delta_candidate[MAX_IJ];
timestamp_t tau_solution[MAX_IJ], tau_candidate[MAX_IJ], theta_q, theta_p, tau_q, rtt;
int solve_count = 0; // record the number of solutions
int candidate_count = 0; // record the number of solutions
int session_count = 0; // record the number of sessions
int ijscale;

timestamp_t t1, t2, t3, t4; // synchronization packet timestamps (see paper)
timestamp_t phi1, phi2, phi3, phi4; // elapsed time from the last impulse (see paper)

timestamp_t sep_timestamp[BUF_SIZE]; // circular buffer to store timestamps of SEP samples
sigval_t sep_signal[BUF_SIZE]; // circular buffer to store SEP sample values
int buf_index = 0; // the circular bufffer position to be written

#ifdef _PCSIM_ // if this header is used on PC
extern pthread_rwlock_t lock_rw; // for locking the circular buffer to avoid read/write conflict
#endif

////////////BPF related

int SamplingRate = 333;



//////////////PLL related
struct STATE {
	double tau1, tau2;
	double w0;
	int UB, K0;
	double t;
	double uc;
	double phi2;
	int Q;
	int u1;
	double uf;
};

struct PLL_PARAMETER {
	double wn;
	double zeta;
	double K;
	double v0;
	double v1;
	double v2;
};
/////////////////////////

//void bpf_core_double(int x) {
//	y = b[0] * x + z[0];
//	// calcualte the recursive equations
//	z[1] = b[2] * x_prev - a[2] * y_prev; // z[1, m - 1]
//	z[0] = z[1] - a[1] * y;  // z[0, m]
//}

/*
* signal processing main function
* t: timestamp
* v: sample value
* zcr: all zcr timestamps
*/
void EZCR(timestamp_t * t, sigval_t * v, timestamp_t * zcr, uint16_t len, uint16_t * zcr_len) {

	uint16_t i, j;
	/* Rising edge detection */
	// Should be global, the signal processing is real-time
	uint8_t negative_cur = 1;
	uint8_t negative_prev = 1;


	float timestamp_prev = 0.0;

	/* Zero-Crossing Detection */
	timestamp_t zcr_timestamp[150] = { 0 };



	/* BPF related */
	// Init
	// 727Hz, [45Hz,55Hz], butter
	// float BPF_b[] ={0.04143578, 0., -0.04143578};
	// float BPF_a[] = {1., -1.74262779, 0.91712843};

	// 364Hz, [45Hz,55Hz], butter
	//float BPF_b[] = { 0.07963244, 0., -0.7963244 };
	//float BPF_a[] = { 1., -1.2011403, 0.84073512 };

	// 333Hz, [45Hz,55Hz], butter
	float BPF_b[] = { 0.0864434747516911, 0.0, -0.0864434747516911 };
	float BPF_a[] = { 1, -1.0773459643074454, 0.8271130504966180 };

	float BPF_z[2] = { 0.0 }; /* size = max(len(a),len(b))*/
	float y_prev = 0.0; // current filtered results, the latest filtered results
	uint16_t x_prev = 0; // the input data, the previous input data
	float ybpf[WORKING_BUF_SIZE] = { 0.0 }; /*Store the filtered data*/
	float ybpf_prev = 0.0;

	/* PLL related*/
	timestamp_t prev_u2_timestamp = 0;
	timestamp_t prev_zcr_timestamp = 0;
	float phi = 0.0, last_phi = 0.0, phi_prev = 0.0;
	float phi_hat = 0.0, phi_hat_prev = 0.0, phi_hat_prev2 = 0.0;
	float delta_phi;
	float zcr_interval_current;
	uint16_t k, cnt2 = 0; // 512 maximum for uint8_t
	uint8_t cnt_compensateClock; // 512 maximum
	float compensateClockCycle = 0.02;
	float PLL_v0 = 0.0, PLL_v1 = 0.0, PLL_v2 = 0.0;
	//float PLL_a0;
	float PLL_a1, PLL_a2;
	float PLL_b0, PLL_b1, PLL_b2;
	//struct PLL_PARAMETER * pll_para = (struct PLL_PARAMETER *)malloc(sizeof(struct PLL_PARAMETER));
	struct PLL_PARAMETER pll_para[1];

	// BPF processing
	//currentTicksH = call LocalTime.get();
	//printfz1("BPF %lu\n", currentTicksH);
	/* band-pass filter IIR
	* input v: original sample
	* output ybpf: filtered sample
	*/
	for (i = 0; i < len; i++) {
		ybpf[i] = BPF_b[0] * v[i] + BPF_z[0];
		// calcualte the recursive equations
		BPF_z[1] = BPF_b[2] * x_prev - BPF_a[2] * y_prev; // z[1, m - 1]
		BPF_z[0] = BPF_z[1] - BPF_a[1] * ybpf[i];  // z[0, m]
		x_prev = v[i];
		y_prev = ybpf[i];
	}

	///////// Overhead monitorint (Z1) ////////
	// currentTicksH = call LocalTime.get();
	// printfz1("ZCR %lu\n", currentTicksH);
	//////////////////////////////////////////

	// uint8_t deadzone = SAMPLING_RATE * 0.0075
	//         fs = 333Hz, deadzone = 2
	j = 0;
	for (i = 0; i < len; i++) {
		//printf("ybpf[%d] = %f\n", i, ybpf[i]);
		if (ybpf[i] > 0.0) {
			negative_cur = 0;
			if (negative_prev == 1) {
				// linear interpolation
				//printf("%d\n",i);
				zcr_timestamp[j] = timestamp_prev + ybpf_prev * ((t[i] - t[0]) * 1.0 - timestamp_prev) / (ybpf_prev - ybpf[i]);
				//printf("[%u]\tBPF %luticks\t%luticks\n", j, zcr_timestamp[j], (zcr_timestamp[j] - prev_zcr_timestamp));
				prev_zcr_timestamp = zcr_timestamp[j]; // should be comment if the above printf is disabled
				i = i + 2; // use deadzone to avoid jitter
				j++;
			}
		}
		else {
			negative_cur = 1;
			timestamp_prev = t[i] - t[0];
			ybpf_prev = ybpf[i];
		}
		negative_prev = negative_cur;
	}

	if (ENABLE_PLL) {
		// PLL parameter init
		// This part could be replaced once all the paramters are finalized
		pll_para->wn = 0.03;
		pll_para->zeta = 0.9;
		pll_para->K = 1000.0;
		pll_para->v0 = 0.0;
		pll_para->v1 = 0.0;
		pll_para->v2 = 0.0;

		// PLL processing
		//currentTicksH = call LocalTime.get();
		//printfz1("PLL %lu\n", currentTicksH);
		/* if ZCR found in this batch */
        /* If more than 1 ZCRs are found in this batch*/
		if (j > 1) {
			/* assign the timestamp for each zero crossing point */
			// note on the definition of 20ms
			// create the 20ms ZCR

			float tau1 = 0.0, tau2 = 0.0;

			// PLL parameter init
			// This part could be replaced once all the paramters are finalized
			// compute the coefficients
			tau1 = pll_para->K / pll_para->wn / pll_para->wn;
			tau2 = 2 * pll_para->zeta / pll_para->wn;

			PLL_b0 = 4 * pll_para->K / tau1*(1 + tau2 / 2);
			PLL_b1 = 8 * pll_para->K / tau1;
			PLL_b2 = 4 * pll_para->K / tau1 * (1 - tau2 / 2);
			//PLL_a0 = 1.0;
			PLL_a1 = -2.0;
			PLL_a2 = 1.0;

			// PLL processing
			for (i = 1; i < j; i++) {
				phi = ((zcr_timestamp[i] - zcr_timestamp[1]) * 1.0 / PRECISION_TAG); // potentional last digit precision loss


				last_phi = phi; // push

				delta_phi = phi - phi_hat;
				zcr_interval_current = phi - phi_prev;

				/////////////////////ZCR compensation//////////////////////////////////////
				if (zcr_interval_current > 0.025) {
					cnt_compensateClock = (phi - phi_prev) / 0.02; // use integer part only
					phi = phi_prev;
					k = 0;
					while (k < cnt_compensateClock) {
						phi = phi + compensateClockCycle;
						delta_phi = phi - phi_hat;

						//printf("%u\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t", cnt2, phi, (phi - phi_prev), phi_hat, (phi_hat - phi_hat_prev2), ((phi_hat - phi_hat_prev2) - (phi - phi_prev)), delta_phi);
						zcr[cnt2] = phi_hat * PRECISION_TAG + zcr_timestamp[1];

						//printf("PLL %luus\t%luus\n", zcr[cnt2], (zcr[cnt2] - prev_u2_timestamp));
                        prev_u2_timestamp = zcr[cnt2]; // should be comment if corresponding printf is/are disabled
                        //fprintf(fpOutput, "%u\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\n", cnt2, phi, (phi - phi_prev), phi_hat, (phi_hat - phi_hat_prev2), ((phi_hat - phi_hat_prev2) - (phi - phi_prev)), delta_phi);

						cnt2++;

						//////// PLL computing
						// advance buffer
						PLL_v2 = PLL_v1;  // shift center register to upper register
						PLL_v1 = PLL_v0;  // shift lower register to center register
										  // compute new lower register
						PLL_v0 = delta_phi - PLL_v1*PLL_a1 - PLL_v2*PLL_a2;
						// compute new output
						phi_hat = PLL_v0*PLL_b0 + PLL_v1*PLL_b1 + PLL_v2*PLL_b2;

						phi_prev = phi;
						phi_hat_prev2 = phi_hat_prev; // should be comment if corresponding printf is/are disabled
						phi_hat_prev = phi_hat;
						///////////////////////

						k++;
					}
				}
				////////////////// -END- ZCR compensation  ////////////////////////////


				phi = last_phi; // pop

                //printf("%u\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t", cnt2, phi, (phi - phi_prev), phi_hat, (phi_hat - phi_hat_prev2), ((phi_hat - phi_hat_prev2) - (phi - phi_prev)), delta_phi);

				//zcr[cnt2] = (phi_hat * PRECISION_TAG + zcr_timestamp[1]) * 1000 / PRECISION_TAG;
				//printfz1("PLL %lums\t%lums\n", zcr[cnt2], (zcr[cnt2] - prev_u2_timestamp));

				// output ticks
				zcr[cnt2] = phi_hat * PRECISION_TAG + zcr_timestamp[1];
				//printf("t[0] = %ld\tsum=%ld\n",t[0],(phi_hat*PRECISION_TAG + zcr_timestamp[1] + t[0]));
				printf("PLL %ldticks\t%ldticks\n", zcr[cnt2], (zcr[cnt2] - prev_u2_timestamp));
				prev_u2_timestamp = zcr[cnt2]; // should be comment if corresponding printf is/are disabled
				cnt2 = cnt2 + 1;

				//////// PLL computing
				// advance buffer
				PLL_v2 = PLL_v1;  // shift center register to upper register
				PLL_v1 = PLL_v0;  // shift lower register to center register
								  // compute new lower register
				PLL_v0 = delta_phi - PLL_v1*PLL_a1 - PLL_v2*PLL_a2;
				// compute new output
				phi_hat = PLL_v0*PLL_b0 + PLL_v1*PLL_b1 + PLL_v2*PLL_b2;

				phi_prev = phi;
				phi_hat_prev2 = phi_hat_prev; // should be comment if corresponding printf is/are disabled
				phi_hat_prev = phi_hat;
				///////////////////////
			}
			//////// -End- PLL processing //////////////////////
		}
		///////////// -END- PLL and missing ZCR compensation //////////////////

		*zcr_len = cnt2;
	}
	else {
		for (i = 0; i < j; i++) {
			zcr[i] = zcr_timestamp[i];
		}
		*zcr_len = j;
	}


}


/// <summary>
/// Scale the difference of two phi,i.e. phi2-phi1, to [0,20ms]
/// </summary>
/// <param name="inphi1">First Phi</param>
/// <param name="inphi2">Second Phi</param>
/// <returns>the scaled phi2-phi2</returns>
int Scale2T(timestamp_t inphi1, timestamp_t inphi2)
{
	int diff = (int)(offset_t)(inphi2 - inphi1);
	//if (inphi1 >= inphi2)
	//	diff = inphi2 - inphi1;
	//else
	//{
	//	diff = inphi1 - inphi2;
	//	diff = -diff;
	//}
	while (diff >= SIGNAL_T) diff -= SIGNAL_T;
	while (diff < 0) diff += SIGNAL_T;
	return diff;
}



/*
* add a SEP sample to the circular buffer
* t: timestamp
* v: sample value
*/
void buf_add(timestamp_t t, sigval_t v)
{
	sep_timestamp[buf_index] = t;
	sep_signal[buf_index] = v;
	buf_index++;
	if (buf_index == BUF_SIZE) buf_index = 0;
}

/*
* find a segment of SEP signal from the circular buffer
* start_time: start time of the segment
* end_time: end time of the segment
* t: output buffer for timestamps
* v: output buffer for SEP sample values
* len: available space of the output buffers
* outlen: length of the output buffers
* return:
*   -1: the available space of the output buffers is not sufficient
*   1: success
*/
int buf_query(timestamp_t start_time, timestamp_t end_time, timestamp_t t[], sigval_t v[], int len, int *outlen)
{
	int i = buf_index; // start searching from the oldest sample in the circular buffer
	unsigned char stop = 0;
	*outlen = 0;
	while (1)
	{
		if (sep_timestamp[i] >= start_time && sep_timestamp[i] <= end_time)
		{
			if (*outlen == len) return -1; // space of the output buffers is not sufficient
			t[*outlen] = sep_timestamp[i];
			v[*outlen] = sep_signal[i];
			(*outlen)++;
		}
		i++;
		if (i == BUF_SIZE) i = 0;

		// search until the newest sample in the circular buffer
		if (buf_index == 0)
		{
			if (i == (BUF_SIZE - 1)) stop = 1;
		}
		else
		{
			if (i == (buf_index - 1)) stop = 1;
		}
		if (stop) break;
	}
	return 1;
};

/*
* Function to be used by the master when the reply1 packet is received
* This function assumes that t2 and t3 have been updated. It will use t2 and t3
* to find a signal segment and compute the phi2 and phi3.
* return:
*   1: success
*/
int send_reply1_done()
{
	printf("\n======= send_reply1_done =========\n");
	printf("t2=%lu, t3=%lu\n", t2, t3);
	timestamp_t t[WORKING_BUF_SIZE]; // working buffer to store timestamps
	sigval_t v[WORKING_BUF_SIZE]; // working buffer to store SEP sample values
	timestamp_t zcr[WORKING_BUF_SIZE];
	int len;
	int i;
	uint16_t zcr_len = 0;

	// on PC, acquire read lock of the circular buffer
#ifdef _PCSIM_
	pthread_rwlock_rdlock(&lock_rw);
#endif

	// in TinyOS, use atomic operation to avoid read/write conflict of the circular buffer
#ifdef _TINYOS_
	atomic{
#endif
		// select a SEP signal segment according to t2 and t3
		buf_query(t2 - 60000, t3 + 60000, t, v, WORKING_BUF_SIZE, &len);
	printf("length of working buff:%d\n",len);
	// on PC, release read lock
#ifdef _PCSIM_
	pthread_rwlock_unlock(&lock_rw);
#endif

#ifdef _TINYOS_
	}
#endif

		//for (i = 0; i < len; i++)
		//{
		//	printf("%lu\t%d\n", t[i], v[i]);
		//}
		//printf("\n");

		// add signal processing algorithms here
		// input are t & v
		// output is zcr
	EZCR(t, v, zcr, len, &zcr_len);
	//printf("ZCR length:%d\n", zcr_len);
	//printf("ZCR:\n");
	//for (i = 0; i < zcr_len; i++)
	//	printf("%lu\n", zcr[i] + t[0]);

	// compute phi2 and phi3
	phi2 = INF;
	phi3 = INF;
	for (i = 1; i < zcr_len; i++)
	{
		printf("t2=%lu, t3=%lu\t", t2, t3);
		printf("zcr[i - 1] + t[0]:%lu,\tzcr[i] + t[0]:%lu\n", zcr[i - 1] + t[0], zcr[i] + t[0]);
		if (phi2 == INF)
		{
			if ((t2 >= zcr[i - 1] + t[0]) && (t2 < zcr[i] + t[0]))
			{
				phi2 = t2 - zcr[i - 1] - t[0];
				printf("phi2:%lu\n", phi2);
			}
		}
		if (phi3 == INF)
		{
			if ((t3 >= zcr[i - 1] + t[0]) && (t3 < zcr[i] + t[0]))
			{
				phi3 = t3 - zcr[i - 1] - t[0];
				printf("phi3:%lu\n", phi3);
			}
		}
	}
	printf("phi2:%lu ,phi3:%lu\n", phi2, phi3);

	return 1;
};

/*
 * Function to be used by the slave when the reply1 packet is received
 * This function assumes that t1 and t4 have been updated. It will use t1 and t4
 * to find a signal segment and compute the phi1 and phi4.
 * return:
 *   1: success
 */
int on_receive_reply1()
{
	timestamp_t t[WORKING_BUF_SIZE]; // working buffer to store timestamps
	sigval_t v[WORKING_BUF_SIZE]; // working buffer to store SEP sample values
	timestamp_t zcr[WORKING_BUF_SIZE];
	int len;
	int i;
	uint16_t zcr_len = 0;
	printf("\n======= on_receive_reply1 =========\n");
	printf("t1=%lu, t4=%lu\n", t1, t4);

	// on PC, acquire read lock of the circular buffer
#ifdef _PCSIM_
	pthread_rwlock_rdlock(&lock_rw);
#endif

	// in TinyOS, use atomic operation to avoid read/write conflict of the circular buffer
#ifdef _TINYOS_
	atomic{
#endif

		// select a SEP signal segment according to t1 and t4
		buf_query(t1 - 60000, t4 + 60000, t, v, BUF_SIZE, &len);
	//for (i = 0; i < len; i++)
	//{
	//	printf("%lu\t%d\n", t[i], v[i]);
	//}
	//printf("\n");
	// on PC, release read lock
#ifdef _PCSIM_
	pthread_rwlock_unlock(&lock_rw);
#endif

#ifdef _TINYOS_
	}
#endif

		//for (i = 0; i < len; i++)
		//{
		//	printf("%lu\t%d\n", t[i], v[i]);
		//}

		// add signal processing algorithms here
	EZCR(t, v, zcr, len, &zcr_len);
	//printf("ZCR length:%d\n", zcr_len);
	//printf("ZCR:\n");
	//for (i = 0; i < zcr_len; i++)
	//	printf("%lu\n", zcr[i] + t[0]);

	// compute phi1 and phi4
	phi1 = INF;
	phi4 = INF;
	for (i = 1; i < zcr_len; i++)
	{
		if (phi1 == INF)
		{
			if ((t1 >= t[0] + zcr[i - 1]) && (t1 < t[0] + zcr[i]))
			{
				phi1 = t1 - t[0] - zcr[i - 1];
				printf("phi1:%lu\n", phi1);
			}
		}
		if (phi4 == INF)
		{
			if ((t4 >= zcr[i - 1] + t[0]) && (t4 < zcr[i] + t[0]))
			{
				phi4 = t4 - t[0] - zcr[i - 1];
				printf("phi4:%lu\n", phi4);
			}
		}
	}
	return 1;
}


// return: 0 means no convergence; 1 means convergence

/*
* Function to be used by the slave when the reply2 packet is received
* This function assumes that t1, t2, t3, t4, phi1, phi2, phi3, and phi4 have been updated.
* offset: if the integer ambiguity can be solved, this variable gives the estimated clock offset
* return:
*   0: the integer ambiguity cannot be solved yet
*   1: the integer ambiguity can be solved
*/
int on_receive_reply2(offset_t *offset)
{
	int i, j;

	printf("======== on_receive_reply2 ========\n");
	session_count++;
	printf("session no.:%d\n", session_count);
	printf("solve_count:%d\n", solve_count);
	printf("t1=%lu, t2=%lu, t3=%lu, t4=%lu\n", t1, t2, t3, t4);
	printf("phi1=%lu, phi2=%lu, phi3=%lu, phi4=%lu\n", phi1, phi2, phi3, phi4);
	printf("RTT=%lu\n", (t4 - t1) - (t3 - t2));

	if (phi1 == INF || phi2 == INF || phi3 == INF || phi4 == INF) return 0;

	// start finding ambigurity equation solution
	// calc theta_q and theta_p
	theta_q = Scale2T(phi1, phi2);
	theta_p = Scale2T(phi3, phi4);
	printf("theta_q=%lu, theta_p=%lu\n", theta_q, theta_p);
	// calc Round Trip Time
	rtt = (t4 - t1) - (t3 - t2);

	// calc the scale of i+j
	ijscale = ceil((rtt - theta_q - theta_p) / SIGNAL_T);
	printf("ijscale=%d\n", ijscale);
	if (solve_count == 0)
		for (i = 0; i < MAX_IJ; i++) delta_solution[i] = INF;

	candidate_count = 0;
	for (i = 0; i <= ijscale; i++)
	{
		tau_q = theta_q + SIGNAL_T*i;
		//tau_p = theta_p + SIGNAL_T*(ijscale - i);
		delta_q = t1 - t2 + tau_q;
		//delta_q = t2 - t1;
		//delta_q = -delta_q;
		//delta_q += tau_q;
		
		//delta_p = t4 - (t3 + tau_p);
		printf("i=%d  tau_q=%lu  delta_q=%ld\n", i, tau_q, delta_q);
		// Start finding if there is any intersection to previous solution sets.
		if (solve_count == 0)
		{
			// if it is the first solution we get
			delta_candidate[i] = delta_q;
			tau_candidate[i] = tau_q;
			candidate_count++;
		}
		else
		{
			for (j = 0; j < solve_count; j++)
			{
				if (abs(delta_solution[j] - delta_q) < MAX_EPSILON_ERROR) // check whether intersect or not
				{
					delta_candidate[candidate_count] = (delta_solution[j] + delta_q) / 2; // if yes, use the average as the intersected solution
					tau_candidate[candidate_count] = (tau_solution[j] + tau_q) / 2;
					candidate_count++;
					break;
				}
			}
		}
	}
	printf("candidate number:%d\n", candidate_count);
	
	if (candidate_count == 0) return 0;
	
	// after the whole search, update the solution sets with the intersected candidates for further calculation.
	for (i = 0; i < candidate_count; i++)
	{
		delta_solution[i] = delta_candidate[i];
		tau_solution[i] = tau_candidate[i];
		printf("delta_candidate:%ld\n", delta_candidate[i]);
	}

	solve_count = candidate_count;
	
	// check whether converges or not
	if (solve_count == 1)
	{
		printf("!!!!!!!!!!!!!! only 1 solution remaining !!!!!!!!!!!!!!!!!!\n");
		*offset = delta_solution[0];
		return 1;
	}
	if (IS_PRIOR_KNOWLEDGE_USED && solve_count == 2)
	{
		printf("!!!!!!!!!!!!!! only 2 solutions remaining !!!!!!!!!!!!!!!!!!\n");
		if ((tau_solution[0] < rtt - tau_solution[0]) && (tau_solution[1] > rtt - tau_solution[1]))
			*offset = delta_solution[0];
		else if ((tau_solution[0] > rtt - tau_solution[0]) && (tau_solution[1] < rtt - tau_solution[1]))
			*offset = delta_solution[1];
		else
			return 0;
		return 1;
	}
	
	return 0;
};


#endif
