#include "libfreenect.h"
#include "libfreenect_registration.h"
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#ifdef _WIN32
#include <direct.h>
#define make_dir(p) _mkdir(p)
#else
#include <sys/stat.h>
#define make_dir(p) mkdir(p, 0777)
#endif

#define W 640
#define H 480
#define DEPTH_BYTES (W * H * 2)
#define RGB_BYTES (W * H * 3)
#define QCAP 128
#define WARMUP_MIN 60.0
#define DRIFT_EVERY_S 60.0
#define DEFAULT_SNAP 120
#define PATH_MAX_LEN 1024

typedef struct {
	char type;
	double host;
	uint32_t ts;
	int bytes;
	FILE *index;
	char dir[PATH_MAX_LEN];
	uint8_t *buf;
} item;

typedef struct {
	char dir[PATH_MAX_LEN];
	FILE *index;
	int open;
} capture;

enum { IDLE, SNAP, REC };

static item queue[QCAP];
static int q_head, q_len;
static pthread_mutex_t q_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t q_cv = PTHREAD_COND_INITIALIZER;
static int writer_done;

static pthread_mutex_t st_mx = PTHREAD_MUTEX_INITIALIZER;
static int mode = IDLE;
static int depth_left, rgb_left;
static char label[256];
static capture cap, drift;

static volatile sig_atomic_t running = 1;
static char session_dir[PATH_MAX_LEN];
static char *device_json;
static double t_start;
static FILE *log_fp, *drift_csv;
static pthread_mutex_t log_mx = PTHREAD_MUTEX_INITIALIZER;
static uint16_t raw_to_mm[2048];
static freenect_raw_tilt_state accel;
static int have_accel;

static unsigned long n_dropped, n_lost_lines, n_gaps[2];
static uint32_t last_ts[2];
static double nominal_step[2];
static uint32_t steps[2][31];
static int n_steps[2];
static double last_drift;
static double last_centre_mm;

static double now(void)
{
	struct timespec t;
	timespec_get(&t, TIME_UTC);
	return t.tv_sec + t.tv_nsec / 1e9;
}

static double minutes(void) { return (now() - t_start) / 60.0; }

static void logf_(const char *fmt, ...)
{
	va_list ap;
	pthread_mutex_lock(&log_mx);
	fprintf(log_fp, "%.3f %7.2fmin ", now(), minutes());
	va_start(ap, fmt);
	vfprintf(log_fp, fmt, ap);
	va_end(ap);
	fputc('\n', log_fp);
	fflush(log_fp);
	pthread_mutex_unlock(&log_mx);
}

static void on_log(freenect_context *ctx, freenect_loglevel level, const char *msg)
{
	(void)ctx;
	size_t n = strlen(msg);
	if (log_fp)
		logf_("[freenect %d] %.*s", (int)level, (int)(n && msg[n - 1] == '\n' ? n - 1 : n), msg);
	if (strstr(msg, "Lost") || strstr(msg, "resync") || strstr(msg, "Expected")) {
		n_lost_lines++;
		pthread_mutex_lock(&st_mx);
		int busy = mode != IDLE;
		pthread_mutex_unlock(&st_mx);
		if (busy)
			printf("  ! packet loss during capture: %s", msg);
	}
}

static int cmp_u16(const void *a, const void *b)
{
	return (int)*(const uint16_t *)a - (int)*(const uint16_t *)b;
}

static int centre_median(const uint16_t *d, int *valid)
{
	static uint16_t v[40 * 40];
	int n = 0;
	for (int y = H / 2 - 20; y < H / 2 + 20; y++)
		for (int x = W / 2 - 20; x < W / 2 + 20; x++)
			if (d[y * W + x] < 2047)
				v[n++] = d[y * W + x];
	*valid = n;
	if (!n)
		return -1;
	qsort(v, n, sizeof v[0], cmp_u16);
	return v[n / 2];
}

static void enqueue(char type, uint32_t ts, const void *data, int bytes, capture *c)
{
	pthread_mutex_lock(&q_mx);
	if (q_len == QCAP) {
		pthread_mutex_unlock(&q_mx);
		n_dropped++;
		logf_("queue full: dropped %c frame ts=%u (disk too slow)", type, ts);
		printf("  ! dropped a frame: disk not keeping up\n");
		return;
	}
	item *it = &queue[(q_head + q_len) % QCAP];
	it->type = type;
	it->host = now();
	it->ts = ts;
	it->bytes = bytes;
	it->index = c->index;
	snprintf(it->dir, sizeof it->dir, "%s", c->dir);
	if (bytes)
		memcpy(it->buf, data, bytes);
	q_len++;
	pthread_cond_signal(&q_cv);
	pthread_mutex_unlock(&q_mx);
}

static void write_item(item *it)
{
	if (it->type == 'x') {
		fclose(it->index);
		return;
	}
	const char *ext = it->type == 'd' ? "pgm" : it->type == 'r' ? "ppm" : "dump";
	char name[128], path[PATH_MAX_LEN + 128];
	snprintf(name, sizeof name, "%c-%f-%u.%s", it->type, it->host, it->ts, ext);
	snprintf(path, sizeof path, "%s/%s", it->dir, name);
	FILE *fp = fopen(path, "wb");
	if (!fp) {
		logf_("cannot write %s: %s", path, strerror(errno));
		printf("  ! cannot write %s\n", path);
		return;
	}
	if (it->type == 'd')
		fprintf(fp, "P5 %d %d 65535\n", W, H);
	else if (it->type == 'r')
		fprintf(fp, "P6 %d %d 255\n", W, H);
	fwrite(it->buf, it->bytes, 1, fp);
	fclose(fp);
	fprintf(it->index, "%s\n", name);
	fflush(it->index);
}

static void *writer(void *arg)
{
	(void)arg;
	static item local;
	local.buf = malloc(RGB_BYTES);
	for (;;) {
		pthread_mutex_lock(&q_mx);
		while (!q_len && !writer_done)
			pthread_cond_wait(&q_cv, &q_mx);
		if (!q_len && writer_done) {
			pthread_mutex_unlock(&q_mx);
			break;
		}
		item *it = &queue[q_head];
		uint8_t *tmp = local.buf;
		local = *it;
		it->buf = tmp;
		q_head = (q_head + 1) % QCAP;
		q_len--;
		pthread_mutex_unlock(&q_mx);
		write_item(&local);
	}
	free(local.buf);
	return NULL;
}

static void write_text(const char *path, const char *text)
{
	FILE *fp = fopen(path, "wb");
	if (fp) {
		fputs(text, fp);
		fclose(fp);
	}
}

static int open_capture(capture *c, const char *dir)
{
	snprintf(c->dir, sizeof c->dir, "%s", dir);
	if (make_dir(dir) != 0)
		return -1;
	char p[PATH_MAX_LEN + 32];
	snprintf(p, sizeof p, "%s/INDEX.txt", dir);
	c->index = fopen(p, "wb");
	if (!c->index)
		return -1;
	snprintf(p, sizeof p, "%s/device.json", dir);
	write_text(p, device_json);
	c->open = 1;
	return 0;
}

static void close_capture(capture *c)
{
	if (c->open)
		enqueue('x', 0, NULL, 0, c);
	c->open = 0;
}

static void gap_check(int s, uint32_t ts)
{
	if (last_ts[s]) {
		uint32_t step = ts - last_ts[s];
		if (n_steps[s] < 31) {
			steps[s][n_steps[s]++] = step;
			if (n_steps[s] == 31) {
				uint32_t sorted[31];
				memcpy(sorted, steps[s], sizeof sorted);
				for (int i = 0; i < 31; i++)
					for (int j = i + 1; j < 31; j++)
						if (sorted[j] < sorted[i]) {
							uint32_t t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t;
						}
				nominal_step[s] = sorted[15];
				logf_("%s nominal timestamp step %.0f", s ? "rgb" : "depth", nominal_step[s]);
			}
		} else if (step > 1.5 * nominal_step[s]) {
			n_gaps[s]++;
			logf_("%s timestamp gap: step %u = %.1f frames", s ? "rgb" : "depth", step, step / nominal_step[s]);
		}
	}
	last_ts[s] = ts;
}

static void finish_snap_locked(void)
{
	if (mode == SNAP && depth_left <= 0 && rgb_left <= 0) {
		close_capture(&cap);
		mode = IDLE;
		logf_("snap %s done", label);
		printf("  snap %s done\n> ", label);
		fflush(stdout);
	}
}

static void depth_cb(freenect_device *dev, void *data, uint32_t ts)
{
	(void)dev;
	gap_check(0, ts);
	double t = now();
	if (t - last_drift >= DRIFT_EVERY_S) {
		int valid, raw = centre_median(data, &valid);
		last_centre_mm = raw >= 0 ? raw_to_mm[raw] : -1;
		fprintf(drift_csv, "%.3f,%.3f,%d,%.0f,%d,%d,%d,%d\n", t, minutes(), raw, last_centre_mm, valid,
		        have_accel ? accel.accelerometer_x : 0, have_accel ? accel.accelerometer_y : 0,
		        have_accel ? accel.accelerometer_z : 0);
		fflush(drift_csv);
		enqueue('d', ts, data, DEPTH_BYTES, &drift);
		last_drift = t;
	}
	pthread_mutex_lock(&st_mx);
	if (mode == REC || (mode == SNAP && depth_left > 0)) {
		enqueue('d', ts, data, DEPTH_BYTES, &cap);
		depth_left--;
		finish_snap_locked();
	}
	pthread_mutex_unlock(&st_mx);
}

static void rgb_cb(freenect_device *dev, void *data, uint32_t ts)
{
	(void)dev;
	gap_check(1, ts);
	pthread_mutex_lock(&st_mx);
	if (mode == REC || (mode == SNAP && rgb_left > 0)) {
		enqueue('r', ts, data, RGB_BYTES, &cap);
		rgb_left--;
		finish_snap_locked();
	}
	pthread_mutex_unlock(&st_mx);
}

static void write_meta(const char *dir, const char *kind, int n)
{
	char p[PATH_MAX_LEN + 32], buf[1024];
	snprintf(p, sizeof p, "%s/meta.json", dir);
	snprintf(buf, sizeof buf,
	         "{\n  \"label\": \"%s\",\n  \"kind\": \"%s\",\n  \"frames_requested\": %d,\n"
	         "  \"started_unix\": %.3f,\n  \"minutes_since_stream_start\": %.2f,\n"
	         "  \"depth_format\": \"FREENECT_DEPTH_11BIT raw, little-endian PGM\",\n"
	         "  \"accel_raw\": [%d, %d, %d]\n}\n",
	         label, kind, n, now(), minutes(), accel.accelerometer_x, accel.accelerometer_y,
	         accel.accelerometer_z);
	write_text(p, buf);
}

static void start_capture(int kind, const char *name, int n)
{
	pthread_mutex_lock(&st_mx);
	if (mode != IDLE) {
		pthread_mutex_unlock(&st_mx);
		printf("  busy with %s; 'stop' first\n", label);
		return;
	}
	char dir[PATH_MAX_LEN];
	snprintf(dir, sizeof dir, "%s/%s", session_dir, name);
	if (open_capture(&cap, dir) != 0) {
		pthread_mutex_unlock(&st_mx);
		printf("  cannot create %s (exists already?) - raw data is never overwritten\n", dir);
		return;
	}
	snprintf(label, sizeof label, "%s", name);
	write_meta(dir, kind == SNAP ? "snap" : "rec", kind == SNAP ? n : -1);
	if (minutes() < WARMUP_MIN)
		printf("  warning: only %.0f of %.0f warm-up minutes\n", minutes(), WARMUP_MIN);
	depth_left = rgb_left = n;
	mode = kind;
	logf_("%s %s start (n=%d)", kind == SNAP ? "snap" : "rec", name, n);
	pthread_mutex_unlock(&st_mx);
	printf("  %s %s started\n", kind == SNAP ? "snap" : "rec", name);
}

static void stop_capture(void)
{
	pthread_mutex_lock(&st_mx);
	if (mode == IDLE)
		printf("  nothing to stop\n");
	else {
		close_capture(&cap);
		logf_("%s stopped", label);
		printf("  %s stopped\n", label);
		mode = IDLE;
	}
	pthread_mutex_unlock(&st_mx);
}

static void status(void)
{
	pthread_mutex_lock(&q_mx);
	int ql = q_len;
	pthread_mutex_unlock(&q_mx);
	printf("  streaming %.1f min (warm-up %s) | centre %.0f mm | queue %d/%d | dropped %lu | "
	       "loss msgs %lu | gaps depth %lu rgb %lu | %s\n",
	       minutes(), minutes() >= WARMUP_MIN ? "done" : "NOT done", last_centre_mm, ql, QCAP, n_dropped,
	       n_lost_lines, n_gaps[0], n_gaps[1], mode == IDLE ? "idle" : label);
}

static void *commands(void *arg)
{
	(void)arg;
	char line[512], cmd[32], name[256];
	printf("commands: snap <label> [n] | rec <label> | stop | status | quit\n> ");
	fflush(stdout);
	while (running && fgets(line, sizeof line, stdin)) {
		int n = DEFAULT_SNAP;
		int got = sscanf(line, "%31s %255s %d", cmd, name, &n);
		if (got < 1) {
		} else if (!strcmp(cmd, "snap") && got >= 2 && n > 0)
			start_capture(SNAP, name, n);
		else if (!strcmp(cmd, "rec") && got >= 2)
			start_capture(REC, name, 0);
		else if (!strcmp(cmd, "stop"))
			stop_capture();
		else if (!strcmp(cmd, "status"))
			status();
		else if (!strcmp(cmd, "quit") || !strcmp(cmd, "exit"))
			break;
		else
			printf("  ? snap <label> [n] | rec <label> | stop | status | quit\n");
		printf("> ");
		fflush(stdout);
	}
	running = 0;
	return NULL;
}

static void on_sigint(int s)
{
	(void)s;
	running = 0;
}

static char *registration_json(freenect_registration *r)
{
	freenect_reg_info *i = &r->reg_info;
	char *s = malloc(4096);
	snprintf(s, 4096,
	         "{\n\t\"reg_info\": {\n"
	         "\t\t\"ax\": %d, \"bx\": %d, \"cx\": %d, \"dx\": %d,\n"
	         "\t\t\"ay\": %d, \"by\": %d, \"cy\": %d, \"dy\": %d,\n"
	         "\t\t\"dx_start\": %d, \"dy_start\": %d,\n"
	         "\t\t\"dx_beta_start\": %d, \"dy_beta_start\": %d,\n"
	         "\t\t\"dx_beta_inc\": %d, \"dy_beta_inc\": %d,\n"
	         "\t\t\"dxdx_start\": %d, \"dxdy_start\": %d, \"dydx_start\": %d, \"dydy_start\": %d,\n"
	         "\t\t\"dxdxdx_start\": %d, \"dydxdx_start\": %d, \"dxdxdy_start\": %d, \"dydxdy_start\": %d,\n"
	         "\t\t\"dydydx_start\": %d, \"dydydy_start\": %d,\n"
	         "\t\t\"pad_info\": {\"start_lines\": %u, \"end_lines\": %u, \"cropping_lines\": %u},\n"
	         "\t\t\"zero_plane_info\": {\"dcmos_emitter_distance\": %.9g, \"dcmos_rcmos_distance\": %.9g, "
	         "\"reference_distance\": %.9g, \"reference_pixel_size\": %.9g}\n"
	         "\t},\n\t\"const_shift\": %.9g\n}\n",
	         i->ax, i->bx, i->cx, i->dx, i->ay, i->by, i->cy, i->dy, i->dx_start, i->dy_start,
	         i->dx_beta_start, i->dy_beta_start, i->dx_beta_inc, i->dy_beta_inc, i->dxdx_start,
	         i->dxdy_start, i->dydx_start, i->dydy_start, i->dxdxdx_start, i->dydxdx_start,
	         i->dxdxdy_start, i->dydxdy_start, i->dydydx_start, i->dydydy_start,
	         r->reg_pad_info.start_lines, r->reg_pad_info.end_lines, r->reg_pad_info.cropping_lines,
	         r->zero_plane_info.dcmos_emitter_dist, r->zero_plane_info.dcmos_rcmos_dist,
	         r->zero_plane_info.reference_distance, r->zero_plane_info.reference_pixel_size,
	         r->const_shift);
	return s;
}

int main(int argc, char **argv)
{
	if (argc != 2 || argv[1][0] == '-') {
		printf("usage: freenect-session <new session dir>\n"
		       "Streams depth (11-bit raw) and RGB continuously until 'quit'. Logs centre depth\n"
		       "and one frame per minute to drift/, and records labelled fakenect-format\n"
		       "captures on command.\n");
		return 1;
	}
	freenect_context *ctx;
	freenect_device *dev;
	if (freenect_init(&ctx, NULL)) {
		printf("freenect_init failed\n");
		return 1;
	}
	freenect_set_log_callback(ctx, on_log);
	freenect_set_log_level(ctx, FREENECT_LOG_INFO);
	freenect_select_subdevices(ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
	if (freenect_open_device(ctx, &dev, 0)) {
		printf("no Kinect found\n");
		freenect_shutdown(ctx);
		return 1;
	}

	snprintf(session_dir, sizeof session_dir, "%s", argv[1]);
	if (make_dir(session_dir) != 0) {
		printf("cannot create %s (exists already?) - use a new directory\n", session_dir);
		return 1;
	}
	char p[PATH_MAX_LEN + 32];
	snprintf(p, sizeof p, "%s/session.log", session_dir);
	log_fp = fopen(p, "w");
	snprintf(p, sizeof p, "%s/drift.csv", session_dir);
	drift_csv = fopen(p, "w");
	if (!log_fp || !drift_csv) {
		printf("cannot write into %s\n", session_dir);
		return 1;
	}
	fprintf(drift_csv, "unix,minutes,centre_raw_median,centre_mm,centre_valid_px,accel_x,accel_y,accel_z\n");

	for (int i = 0; i < QCAP; i++)
		queue[i].buf = malloc(RGB_BYTES);

	freenect_registration reg = freenect_copy_registration(dev);
	memcpy(raw_to_mm, reg.raw_to_mm_shift, sizeof raw_to_mm);
	device_json = registration_json(&reg);
	freenect_destroy_registration(&reg);
	snprintf(p, sizeof p, "%s/device.json", session_dir);
	write_text(p, device_json);
	snprintf(p, sizeof p, "%s/drift", session_dir);
	if (open_capture(&drift, p) != 0) {
		printf("cannot create %s\n", p);
		return 1;
	}

	pthread_t wt, ct;
	pthread_create(&wt, NULL, writer, NULL);

	freenect_set_depth_mode(dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
	freenect_set_video_mode(dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
	freenect_set_depth_callback(dev, depth_cb);
	freenect_set_video_callback(dev, rgb_cb);
	freenect_start_depth(dev);
	freenect_start_video(dev);
	t_start = now();
	last_drift = t_start - DRIFT_EVERY_S;
	logf_("streams started");
	printf("streaming into %s. Leave it running: %.0f min warm-up before measuring captures.\n",
	       session_dir, WARMUP_MIN);

	signal(SIGINT, on_sigint);
	pthread_create(&ct, NULL, commands, NULL);

	double last_accel = 0;
	while (running) {
		struct timeval tv = {0, 100000};
		if (freenect_process_events_timeout(ctx, &tv) < 0) {
			logf_("freenect_process_events failed: device lost");
			printf("\n! Kinect lost (USB error). Session data so far is saved.\n");
			break;
		}
		double t = now();
		if (t - last_accel >= 1.0) {
			if (freenect_update_tilt_state(dev) == 0) {
				accel = *freenect_get_tilt_state(dev);
				have_accel = 1;
				pthread_mutex_lock(&st_mx);
				if (mode != IDLE)
					enqueue('a', last_ts[0], &accel, sizeof accel, &cap);
				pthread_mutex_unlock(&st_mx);
			}
			last_accel = t;
		}
	}
	running = 0;

	freenect_stop_depth(dev);
	freenect_stop_video(dev);
	pthread_mutex_lock(&st_mx);
	if (mode != IDLE)
		close_capture(&cap);
	mode = IDLE;
	pthread_mutex_unlock(&st_mx);
	close_capture(&drift);
	pthread_mutex_lock(&q_mx);
	writer_done = 1;
	pthread_cond_signal(&q_cv);
	pthread_mutex_unlock(&q_mx);
	pthread_join(wt, NULL);
	logf_("session end: dropped %lu, loss msgs %lu, gaps depth %lu rgb %lu", n_dropped, n_lost_lines,
	      n_gaps[0], n_gaps[1]);
	printf("done: dropped %lu, loss msgs %lu, gaps depth %lu rgb %lu\n", n_dropped, n_lost_lines, n_gaps[0],
	       n_gaps[1]);
	fclose(drift_csv);
	fclose(log_fp);
	freenect_close_device(dev);
	freenect_shutdown(ctx);
	return 0;
}
