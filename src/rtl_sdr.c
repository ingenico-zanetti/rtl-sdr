/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include "getopt/getopt.h"
#endif

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#define DEFAULT_SAMPLE_RATE		2048000
#define DEFAULT_BUF_LENGTH		(16 * 16384)
#define MINIMAL_BUF_LENGTH		512
#define MAXIMAL_BUF_LENGTH		(256 * 16384)

#define DBFS_MIN (-480)

// Look Up Table in 10*dBFS
static int dbfs_lut[256][256];

static void dbfs_lut_init(void){
	int i, q;
	for(i = 0 ; i < 256 ; i++){
		int centered_i = i - 158;
		int centered_i_squared = centered_i * centered_i;
		for(q = 0 ; q < 256 ; q++){
			int centered_q = q - 128;
			int mag = centered_i_squared + (centered_q * centered_q);
			int dbfs = DBFS_MIN; // if mag is 0
			if(mag > 0){
				dbfs = 100 * logf((float)mag / 16384.0); // 0dBFS => <||I||=128, ||Q||=0> or <||I||=0, ||Q||=128> ; ||I|| & ||Q|| = 128 => 3dBFS
			}
			dbfs_lut[i][q] = dbfs;
		}
	}
}

static int do_exit = 0;
static uint64_t bytes_to_read = 0;
static rtlsdr_dev_t *dev = NULL;

void usage(void)
{
	fprintf(stderr,
		"rtl_sdr, an I/Q recorder for RTL2832 based DVB-T receivers\n\n"
		"Usage:\t -f frequency_to_tune_to [Hz]\n"
		"\t[-s samplerate (default: 2048000 Hz)]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-b output_block_size (default: 16 * 16384)]\n"
		"\t[-n number of samples to read (default: 0, infinite)]\n"
		"\t[-S force sync output (default: async)]\n"
		"\t[-D enable direct sampling (default: off)]\n"
		"\tfilename (a '-' dumps samples to stdout)\n\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	signal(SIGPIPE, SIG_IGN);
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dev);
}
#endif

#define MAX_HZ (5)
static int target_dbfs = 0; // in tenth of dBFS
static int current_gain = 0; // in tenth of dB
static uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
static int max_samples = 0;
static int sample_index = 0;
static int max_dbfs = DBFS_MIN;
#include <sys/eventfd.h>
int max_event = -1;
int delta_gain = 0;
int pid_somme = 0;

int gain_count = 0;
int gains[100];
int gain_index = 1;

static void initialize_gains(void){
	int i;
	gain_count = rtlsdr_get_tuner_gains(dev, NULL);
	fprintf(stderr, "Supported gain values (%d): ", gain_count);

	gain_count = rtlsdr_get_tuner_gains(dev, gains);
	for (i = 0; i < gain_count; i++)
		fprintf(stderr, "%.1f ", gains[i] / 10.0);
	fprintf(stderr, "\n");
	gain_index = (gain_count / 2);
}

void update_gain_pid(int delta){
	int old_pid_somme = pid_somme;
	pid_somme += delta;
	// fprintf(stderr, "%s:somme+delta %d+%d=>%d" "\n", __func__, old_pid_somme, delta, pid_somme);
}

static void signal_event(int fd){
	uint64_t u = 1ULL;
	write(fd, &u, sizeof(u));
}

static void clear_event(int fd){
	uint64_t u = 1ULL;
	read(fd, &u, sizeof(u));
}

static int create_event(void){
	return(eventfd(0, EFD_NONBLOCK));
}

static void *set_new_gain_thread(void *unused){
	fprintf(stderr, "%s started !" "\n", __func__);
	do{
		fd_set wait;
		struct timeval timeout;
		int selected = 0;
		FD_ZERO(&wait);
		FD_SET(max_event, &wait);
		timeout.tv_usec = 0;
		timeout.tv_sec = 1;
		selected = select(max_event + 1, &wait, NULL, NULL, &timeout);
		if(selected > 0){
			int new_gain_index = gain_index;
			clear_event(max_event);
			if(pid_somme > 300){
				pid_somme -= 300;
				new_gain_index++;
			}
			if(pid_somme < -300){
				pid_somme += 300;
				new_gain_index--;
			}
			// Ensure new_gain_index points to a usable gain
			if(new_gain_index < 1){
				new_gain_index = 1;
			}else if(new_gain_index > (gain_count - 1)){
				new_gain_index = (gain_count - 1);
			}
			if(new_gain_index != gain_index){
				current_gain =  gains[new_gain_index];
				// fprintf(stderr, "%s: target_dbfs = %d, gain_index:%d=>%d (gains:%d=>%d) ; ", __func__,  target_dbfs, gain_index, new_gain_index, gains[gain_index], gains[new_gain_index]);
				verbose_gain_set(dev, current_gain);
				gain_index = new_gain_index;
			}
		}
	}while(0 == do_exit);
	return(unused);
}

static void max_thread_init(void){
	pthread_t max_thread_id;
	max_event = create_event();
	pthread_create(&max_thread_id, NULL, set_new_gain_thread, NULL);
}


static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	if (ctx) {
		if (do_exit)
			return;

		if ((bytes_to_read > 0) && (bytes_to_read < (uint64_t)len)) {
			len = bytes_to_read;
			do_exit = 1;
			rtlsdr_cancel_async(dev);
		}
		if(target_dbfs < 0) {
			/* Compute MAX for sample_rate / MAX_HZ IQ samples */
			int n = len / 2;
			while(n--){
				unsigned char *p = buf;
				unsigned char i, q;
				int level;
				if(sample_index > 0){
					sample_index--;
				}else{
					delta_gain = target_dbfs - max_dbfs;
					update_gain_pid(delta_gain);
					signal_event(max_event);
					// fprintf(stderr, "%s: target_dbfs=%d, max_dbfs=%d, delta=%d" "\n", __func__, target_dbfs, max_dbfs, delta_gain);
					
					sample_index = max_samples - 1;
					max_dbfs = DBFS_MIN;
				}
				i = *p++;
				q = *p++;
				level = dbfs_lut[i][q];
				if(level > max_dbfs){
					max_dbfs = level;
				}

			}
		}
		if (fwrite(buf, 1, len, (FILE*)ctx) != len) {
			fprintf(stderr, "Short write, samples lost, exiting!\n");
			rtlsdr_cancel_async(dev);
		}

		if (bytes_to_read > 0)
			bytes_to_read -= (uint64_t)len;
	}
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filename = NULL;
	int n_read;
	int r, opt;
	int gain = 0;
	int ppm_error = 0;
	int direct_sampling = 0;
	int sync_mode = 0;
	FILE *file;
	uint8_t *buffer;
	int dev_index = 0;
	int dev_given = 0;
	uint32_t frequency = 100000000;
	uint32_t out_block_size = DEFAULT_BUF_LENGTH;

	while ((opt = getopt(argc, argv, "d:f:g:s:b:n:p:SD")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			frequency = (uint32_t)atofs(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'b':
			out_block_size = (uint32_t)atof(optarg);
			break;
		case 'n':
			bytes_to_read = (uint64_t)atof(optarg) * 2;
			break;
		case 'S':
			sync_mode = 1;
			break;
		case 'D':
			direct_sampling = 1;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		usage();
	} else {
		filename = argv[optind];
	}

	if(out_block_size < MINIMAL_BUF_LENGTH ||
	   out_block_size > MAXIMAL_BUF_LENGTH ){
		fprintf(stderr,
			"Output block size wrong value, falling back to default\n");
		fprintf(stderr,
			"Minimal length: %u\n", MINIMAL_BUF_LENGTH);
		fprintf(stderr,
			"Maximal length: %u\n", MAXIMAL_BUF_LENGTH);
		out_block_size = DEFAULT_BUF_LENGTH;
	}

	buffer = malloc(out_block_size * sizeof(uint8_t));

	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	/* Set direct sampling */
        if (direct_sampling)
                verbose_direct_sampling(dev, 2);

	/* Set the sample rate */
	verbose_set_sample_rate(dev, samp_rate);

	/* Set the frequency */
	verbose_set_frequency(dev, frequency);

	if (0 == gain) {
		 /* Enable automatic gain */
		verbose_auto_gain(dev);
	} else {
		if(gain > 0){
			/* Enable manual gain */
			gain = nearest_gain(dev, gain);
			verbose_gain_set(dev, gain);
		}else{
			/* Enable manual AGC */
			/* target is in tenth of dBFS */
			initialize_gains();
			current_gain = gains[gain_index];
			verbose_gain_set(dev, current_gain);
			target_dbfs = gain;
			max_samples = samp_rate / MAX_HZ;
			sample_index = max_samples;
			dbfs_lut_init();
			max_thread_init();
		}
	}

	verbose_ppm_set(dev, ppm_error);

	if(strcmp(filename, "-") == 0) { /* Write samples to stdout */
		file = stdout;
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
	} else {
		file = fopen(filename, "wb");
		if (!file) {
			fprintf(stderr, "Failed to open %s\n", filename);
			goto out;
		}
	}

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);

	if (sync_mode) {
		fprintf(stderr, "Reading samples in sync mode...\n");
		while (!do_exit) {
			r = rtlsdr_read_sync(dev, buffer, out_block_size, &n_read);
			if (r < 0) {
				fprintf(stderr, "WARNING: sync read failed.\n");
				break;
			}

			if ((bytes_to_read > 0) && (bytes_to_read < (uint32_t)n_read)) {
				n_read = bytes_to_read;
				do_exit = 1;
			}

			if (fwrite(buffer, 1, n_read, file) != (size_t)n_read) {
				fprintf(stderr, "Short write, samples lost, exiting!\n");
				break;
			}

			if ((uint32_t)n_read < out_block_size) {
				fprintf(stderr, "Short read, samples lost, exiting!\n");
				break;
			}

			if (bytes_to_read > 0)
				bytes_to_read -= n_read;
		}
	} else {
		fprintf(stderr, "Reading samples in async mode...\n");
		r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)file,
				      0, out_block_size);
	}

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	if (file != stdout)
		fclose(file);

	rtlsdr_close(dev);
	free (buffer);
out:
	return r >= 0 ? r : -r;
}
