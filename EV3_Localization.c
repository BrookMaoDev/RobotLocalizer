/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization

 This file provides the implementation of all the functionality required for the
 EV3 robot localization project. Please read through this file carefully, and
 note the sections where you must implement functionality for your bot.

 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are
 expected to develop high wuality, clean code. Test your code extensively with
 valgrind, and make sure its memory management is clean.

 In a nutshell, the starter code provides:

 * Reading a map from an input image (in .ppm format). The map is bordered with
 red, must have black streets with yellow intersections, and buildings must be
 either blue, green, or be left white (no building).

 * Setting up an array with map information which contains, for each
 intersection, the colours of the buildings around it in ** CLOCKWISE ** order
 from the top-left.

 * Initialization of the EV3 robot (opening a socket and setting up the
 communication between your laptop and your bot)

 What you must implement:

 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot
 must not wander outside the map (though of course it's possible parts of the
 robot will leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to
 determine its location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different
 intersections in a sequence that allows it to achieve reliable localization

 * Basic path planning - once the robot has found its location, it must drive
 toward a user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---

  The starter code provides a skeleton for implementing a sensor calibration
 routine, it is called when the code receives -1  -1 as target coordinates. The
 goal of this function should be to gather informatin about what the sensor
 reads for different colours under the particular map/room illumination/battery
 level conditions you are working on - it's entirely up to you how you want to
 do this, but note that careful calibration would make your work much easier, by
 allowing your robot to more robustly (and with fewer mistakes) interpret the
 sensor data into colours.

   --> The code will exit after calibration without running localization (no
 target!) SO - your calibration code must *save* the calibration information
 into a file, and you have to add code to main() to read and use this
            calibration data yourselves.

 What you need to understand thoroughly in order to complete this project:

 * The histogram localization method as discussed in lecture. The general steps
 of probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and
 unreliable, you have to handle this smartly

 * Robot control with feedback - your robot does not perform exact motions, you
 can assume there will be error and drift, your code has to handle this.

 * The robot control API you will use to get your robot to move, and to acquire
   sensor data. Please see the API directory and read through the header files
 and attached documentation

 Starter code:
 F. Estrada, 2018 - for CSC C85

*/

#include <stdio.h>
#include <time.h>

#include "EV3_Localization.h"

// Stuff we define goes inside here
#define SIMULATE /* comment this line out to run on the real robot */

#ifdef SIMULATE
#define BT_open(id) 0
#define BT_close() ((void)0)

/* Reads one line of user input into buf, discarding anything past its end so
 * an over-long line doesn't spill into the next prompt. Returns 0 on EOF. */
static int read_line(char *buf, int size) {
  fflush(stdout);
  if (fgets(buf, size, stdin) == NULL)
    return 0;
  if (strchr(buf, '\n') == NULL) {
    int c;
    while ((c = getchar()) != '\n' && c != EOF)
      ;
  }
  return 1;
}

static int letter_to_colour(char c) {
  switch (c) {
  case 'g':
  case 'G':
    return 3;
  case 'b':
  case 'B':
    return 2;
  case 'w':
  case 'W':
    return 6;
  default:
    return -1;
  }
}
#endif

#define SENSOR_PORT PORT_3
#define LEFT_MOTOR MOTOR_D
#define RIGHT_MOTOR MOTOR_A

#define VERBOSE 1
#define DEBUG_SENSOR 0

#define NUM_COLORS 7 /* index 0 unused, 1..6 are real colours */
#define AVG_SAMPLES 10

static const char *colour_names[NUM_COLORS] = {
    "", "black", "blue", "green", "yellow", "red", "white"};

typedef struct {
  double r, g, b;
} rgb_t;

static rgb_t centroid[NUM_COLORS] = {
    {0, 0, 0},       /* 0 unused */
    {0, 0, 0},       /* 1 black  */
    {0, 0, 255},     /* 2 blue   */
    {0, 255, 0},     /* 3 green  */
    {255, 255, 0},   /* 4 yellow */
    {255, 0, 0},     /* 5 red    */
    {255, 255, 255}, /* 6 white  */
};

static int read_rgb(rgb_t *v) {
  int r = 0, g = 0, b = 0, a = 0;
  if (BT_read_colour_RGBraw_NXT(SENSOR_PORT, &r, &g, &b, &a) != 1) {
    fprintf(stderr, "read_rgb: read failed on port %d\n", SENSOR_PORT);
    return 0;
  }
  /* BT_read_colour_RGBraw_NXT returns each channel minus the 4th "ambient"
   * value, which on our sensor is often larger than the channel itself and
   * swings with room light, giving negative, unstable readings. Adding it
   * back recovers the sensor's own r,g,b, which are positive and consistent
   * between spots of the same colour. */
  v->r = r + a;
  v->g = g + a;
  v->b = b + a;
  if (DEBUG_SENSOR)
    fprintf(stderr, "read_rgb: R=%.0f G=%.0f B=%.0f (ambient=%d)\n", v->r, v->g,
            v->b, a);
  return 1;
}

static rgb_t read_rgb_avg(int n) {
  rgb_t sum = {0, 0, 0};
  int ok = 0;
  for (int i = 0; i < n; i++) {
    rgb_t v;
    if (read_rgb(&v)) {
      sum.r += v.r;
      sum.g += v.g;
      sum.b += v.b;
      ok++;
    }
  }
  if (ok > 0) {
    sum.r /= ok;
    sum.g /= ok;
    sum.b /= ok;
  }
  if (DEBUG_SENSOR)
    fprintf(
        stderr,
        "read_rgb_avg: %d of %d reads succeeded, avg R=%.1f G=%.1f B=%.1f\n",
        ok, n, sum.r, sum.g, sum.b);
  return sum;
}

/* Colour classifier: white-balance -> HSV -> point in the HSV cone -> nearest
 * calibrated centroid. Raw readings are 16-bit sensor counts (ambient already
 * subtracted), so only values relative to the white calibration mean much. */
#define WHITE 6
#define V_WEIGHT 1.0 /* weight of brightness vs. colour in the distance */
#define COLOUR_BIT(c) (1 << (c))
#define BUILDING_MASK (COLOUR_BIT(2) | COLOUR_BIT(3) | COLOUR_BIT(WHITE))
#define ALL_COLOURS_MASK 0x7E /* colours 1..6 */
#define CALIB_SPOTS 3         /* spots averaged per colour when calibrating */
#define TEST_SPOTS 5          /* spots per colour in the accuracy test */
#define ACCURACY_MIN 0.5
#define ACCURACY_MAX 0.95 /* 1.0 would let one bad scan zero out the truth */

typedef struct {
  double h, s, v;
} hsv_t;

static double centroid_feat[NUM_COLORS][3]; /* centroids in cone space */

/* Standard RGB to HSV: h in degrees [0, 360), s in [0, 1], v = max channel. */
static hsv_t rgb_to_hsv(rgb_t c) {
  double max = fmax(c.r, fmax(c.g, c.b));
  double min = fmin(c.r, fmin(c.g, c.b));
  double chroma = max - min;
  hsv_t out = {0, 0, max};
  if (max > 0)
    out.s = chroma / max;
  if (chroma > 0) {
    if (max == c.r)
      out.h = 60 * fmod((c.g - c.b) / chroma, 6);
    else if (max == c.g)
      out.h = 60 * ((c.b - c.r) / chroma + 2);
    else
      out.h = 60 * ((c.r - c.g) / chroma + 4);
    if (out.h < 0)
      out.h += 360;
  }
  return out;
}

/* White-balances a raw reading against the white centroid (so white reads
 * (1,1,1) and the sensor's uneven channel sensitivity cancels out), then maps
 * it into an HSV cylinder: (S cos H, S sin H, V). The radius is saturation,
 * not chroma (S*V), so a colour read dimmer than calibrated (sensor half
 * over an edge, or a bit higher off the paper) keeps its hue: with chroma, a
 * dim yellow at the black/yellow edge classified as green. */
static void colour_features(rgb_t raw, double f[3]) {
  const rgb_t *w = &centroid[WHITE];
  rgb_t balanced = {w->r > 0 ? raw.r / w->r : 0, w->g > 0 ? raw.g / w->g : 0,
                    w->b > 0 ? raw.b / w->b : 0};
  hsv_t hsv = rgb_to_hsv(balanced);
  double h = hsv.h * M_PI / 180.0;
  f[0] = hsv.s * cos(h);
  f[1] = hsv.s * sin(h);
  f[2] = V_WEIGHT * hsv.v;
}

/* Recomputes centroid_feat; call whenever centroid[] changes. */
static void update_centroid_features(void) {
  for (int c = 1; c < NUM_COLORS; c++)
    colour_features(centroid[c], centroid_feat[c]);
  for (int c = 1; c < NUM_COLORS; c++)
    if (c != WHITE &&
        centroid[c].r + centroid[c].g + centroid[c].b >
            centroid[WHITE].r + centroid[WHITE].g + centroid[WHITE].b)
      fprintf(stderr,
              "Warning: %s reads brighter than white - white calibration is "
              "probably bad (sensor off the paper or too high?)\n",
              colour_names[c]);
}

/* Returns the colour (1..6) allowed by mask whose centroid is nearest to the
 * reading in cone space, or -1 if mask allows nothing. */
static int classify_colour(rgb_t raw, int mask) {
  double f[3], best_dist = INFINITY;
  int best = -1;
  colour_features(raw, f);
  for (int c = 1; c < NUM_COLORS; c++) {
    if (!(mask & COLOUR_BIT(c)))
      continue;
    double dist = 0;
    for (int k = 0; k < 3; k++)
      dist += (f[k] - centroid_feat[c][k]) * (f[k] - centroid_feat[c][k]);
    if (dist < best_dist) {
      best_dist = dist;
      best = c;
    }
  }
  return best;
}

static double clamp_accuracy(double a) {
  return fmin(ACCURACY_MAX, fmax(ACCURACY_MIN, a));
}

/* Prints each centroid's raw RGB, HSV (after white balance), and cone point. */
static void print_colour_table(void) {
  printf("%-7s %8s %8s %8s | %6s %5s %5s | %6s %6s %6s\n", "colour", "R", "G",
         "B", "H", "S", "V", "x", "y", "z");
  for (int c = 1; c < NUM_COLORS; c++) {
    const rgb_t *w = &centroid[WHITE];
    rgb_t bal = {centroid[c].r / w->r, centroid[c].g / w->g,
                 centroid[c].b / w->b};
    hsv_t hsv = rgb_to_hsv(bal);
    printf("%-7s %8.1f %8.1f %8.1f | %6.1f %5.2f %5.2f | %6.2f %6.2f %6.2f\n",
           colour_names[c], centroid[c].r, centroid[c].g, centroid[c].b, hsv.h,
           hsv.s, hsv.v, centroid_feat[c][0], centroid_feat[c][1],
           centroid_feat[c][2]);
  }
}

/* Waits for the user to press Enter, discarding anything typed first. */
static void wait_for_enter(void) {
  int ch;
  while ((ch = getchar()) != '\n' && ch != EOF)
    ;
}

/* Sounds. Each colour has its own pitch, low (black) to high (white), so a
 * classification can be heard without looking at the screen. */
#define SOUND 1         /* set to 0 to silence the robot */
#define SOUND_VOLUME 30 /* 0..63 */
#define COLOUR_TONE_MS 150

/* Plays n notes of {frequency Hz, duration ms}. In simulation, just logs the
 * label, since there is no EV3 to play it. */
static void play_tones(const int (*notes)[2], int n, const char *label) {
  if (!SOUND)
    return;
#ifdef SIMULATE
  (void)notes;
  (void)n;
  if (VERBOSE)
    printf("[sound: %s]\n", label);
#else
  int tones[50][3];
  memset(tones, -1, sizeof(tones)); /* -1 marks the end of the sequence */
  for (int i = 0; i < n && i < 49; i++) {
    tones[i][0] = notes[i][0];
    tones[i][1] = notes[i][1];
    tones[i][2] = SOUND_VOLUME;
  }
  if (BT_play_tone_sequence(tones) != 0)
    fprintf(stderr, "play_tones: could not play %s\n", label);
#endif
}

static void play_colour_tone(int colour) {
  static const int freq[NUM_COLORS] = {0, 220, 330, 440, 660, 880, 1320};
  if (colour < 1 || colour >= NUM_COLORS)
    return;
  const int note[1][2] = {{freq[colour], COLOUR_TONE_MS}};
  play_tones(note, 1, colour_names[colour]);
}

/* Rising arpeggio: first time the robot is confident of its location. */
static void play_confident_sound(void) {
  static const int notes[3][2] = {{523, 120}, {659, 120}, {784, 250}};
  play_tones(notes, 3, "confident");
}

/* Fanfare: reached the destination. */
static void play_arrival_sound(void) {
  static const int notes[5][2] = {
      {523, 150}, {659, 150}, {784, 150}, {1047, 300}, {784, 500}};
  play_tones(notes, 5, "arrived");
}

/* Street following. The sensor sits ~10 cm ahead of the wheels, so spinning
 * in place sweeps it across a wide arc - a small spin is enough to find the
 * street again after drifting off it. Turns are timed (no gyro), so the
 * *_MS values below need tuning on the real map. */
#define BLACK 1
#define YELLOW 4
#define RED 5
#define DRIVE_POWER 15    /* slow forward speed while on the street */
#define SWEEP_POWER 20    /* spin speed while searching; 10 barely moved */
#define SWEEP_STEP_MS 300 /* sweep pass n spins for n * SWEEP_STEP_MS */
#define SWEEP_PASSES 8    /* give up after this many passes */
#define CONFIRM_READS 2   /* consecutive reads needed to trust a colour */
#define OFF_STREET_READS                                                       \
  3 /* ...to decide we left the street: the black/yellow                       \
       edge reads as green for ~2 reads on arrival */
#define MAX_DRIVE_READS 3000 /* safety cap on sensor reads per drive */

/* Intersection scan and turns. Each building on Map1 is a small square in
 * the corner next to its intersection (~1.3 to ~6 cm from the centre along
 * each street, ~5 cm from the centre to the middle), with a white gap and
 * then the next intersection's building beyond it. Spinning in place swings
 * the sensor in a ~10 cm circle around the axle. With the axle over the
 * centre, that circle passes outside all 4 of this intersection's buildings
 * and only crosses the gaps and the neighbours' buildings. Driving less
 * before the spin doesn't help: the circle keeps its 10 cm radius and just
 * moves backward.
 *
 * So the scan reads 2 buildings at a time instead. It parks the sensor on
 * the street beside the middle of the rear pair (then the front pair) and
 * sweeps right then left. Each building is the first thing the sensor
 * reaches after it leaves the street. See scan_intersection().
 *
 * Those parking spots are found from the yellow's edges, not by timing a
 * drive from a standstill: a short drive at low power is mostly motor
 * spin-up, so its distance changes a lot with battery and floor. Only the
 * last bit past the edge is timed, and the robot is already moving then.
 *
 * The robot can reach the intersection crooked, since drive_along_street()
 * steers by putting the sensor back on the street, not by straightening the
 * robot, and a crooked drive across the yellow puts the sensor beside it.
 * Centring the sensor only straightens the robot when the axle is on the
 * street's centre line too, and driving ~10 cm (the sensor's distance ahead
 * of the axle) brings the axle to where the sensor was. So before the front
 * pair, the scan drives the axle onto the centre and straightens up there.
 *
 * Turns still spin with the axle over the centre and count street crossings
 * instead of timing the spin, so they stay right however fast the robot
 * actually turns (battery, floor, slip). */
#define CENTER_DRIVE_MS                                                        \
  1600 /* drive time at DRIVE_POWER moving the axle from                       \
          where the sensor hit yellow to the centre (~12 cm) */
#define PAST_YELLOW_MS                                                         \
  250 /* keep driving this long after the sensor leaves the yellow, so it      \
         stops ~2 cm past the edge, level with the middle of the buildings     \
         on that side (they span ~0 to ~4 cm past the edge) */
#define FRONT_TO_CENTRE_MS                                                     \
  1000 /* drive time from the front-scan spot to the axle over the centre      \
          (~7 cm); turns tolerate ~1-2 cm of error here */
#define YELLOW_TIMEOUT_MS                                                      \
  2000 /* give up if the yellow edge never shows; the longest search is ~9 cm  \
          (~1.2 s), and this caps how far a missed edge lets the robot run */
#define BUILDING_VOTE_STREETS                                                  \
  0.7 /* after leaving the street, vote for this many street-widths of spin    \
         (~10-15 deg); a building is ~2 street-widths across on the sweep, so  \
         the votes stay inside it */
#define STEP_SCAN                                                              \
  0 /* 1: pause at each stage of the scan until Enter is pressed, to check     \
       where the sensor is when tuning PAST_YELLOW_MS etc. */
#define SCAN_POWER 20 /* spin speed while scanning; 8 barely moved */
#define SCAN_DEG_PER_MS                                                        \
  0.03125 /* rough spin rate at SCAN_POWER, only used by spin_test() */
#define STREET_MS_GUESS                                                        \
  280 /* time to spin the sensor across one street at SCAN_POWER, used until   \
         the first real crossing is measured; a run on 2026-10-22 measured     \
         ~270 ms */
#define SPIN_COAST_MS                                                          \
  0 /* how long the robot keeps turning after a stop command; raise it if      \
       centring on a street keeps ending past centre in the spin direction */
#define SCAN_TIMEOUT_MS 30000

#ifndef SIMULATE
static double now_ms(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec * 1000.0 + t.tv_nsec / 1e6;
}

/* Reads one sample and classifies it as any of the six colours. Returns -1
 * if the read failed. */
static int read_colour(void) {
  rgb_t v;
  if (!read_rgb(&v))
    return -1;
  return classify_colour(v, ALL_COLOURS_MASK);
}

static void stop_motors(void) {
  BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
}

/* Spins in place (dir -1 = left/counter-clockwise, +1 = right/clockwise) for
 * up to ms milliseconds, polling the sensor the whole time. Stops as soon as
 * it sees a colour in stop_mask and returns it; returns 0 if time ran out and
 * -1 on a read failure. */
static int spin_until(int dir, double ms, int stop_mask) {
  int c = 0;
  double end = now_ms() + ms;
  BT_turn(LEFT_MOTOR, dir * SWEEP_POWER, RIGHT_MOTOR, -dir * SWEEP_POWER);
  while (now_ms() < end) {
    c = read_colour();
    if (c < 0 || stop_mask & COLOUR_BIT(c))
      break;
    c = 0;
  }
  stop_motors();
  return c;
}

/* Sweeps the sensor side to side with growing width until it lands on black
 * or yellow: pass n spins n * SWEEP_STEP_MS, alternating sides, so the sensor
 * visits offsets -1, +1, -2, +2, ... steps around where it started. The first
 * pass goes toward whichever side found the street last time, since the
 * robot usually drifts off the same side again. Returns the colour found, 0
 * if nothing was found, -1 on a read failure. */
static int find_black(void) {
  static int first_dir = -1;
  int dir = first_dir;
  for (int n = 1; n <= SWEEP_PASSES; n++, dir = -dir) {
    if (VERBOSE)
      printf("Sweeping %s for %d ms\n", dir < 0 ? "left" : "right",
             n * SWEEP_STEP_MS);
    int c = spin_until(dir, n * SWEEP_STEP_MS,
                       COLOUR_BIT(BLACK) | COLOUR_BIT(YELLOW));
    if (c != 0) {
      if (c > 0)
        first_dir = dir;
      return c;
    }
  }
  return 0;
}

/* Takes one reading to confirm the sensor responds, and reports what it sees.
 * Returns 1 if the read worked. */
static int check_sensor(void) {
  rgb_t v;
  if (!read_rgb(&v))
    return 0;
  printf("Sensor check: R=%.0f G=%.0f B=%.0f -> %s\n", v.r, v.g, v.b,
         colour_names[classify_colour(v, ALL_COLOURS_MASK)]);
  return 1;
}

static void sleep_ms(double ms) {
  struct timespec t = {(time_t)(ms / 1000), (long)(fmod(ms, 1000) * 1e6)};
  nanosleep(&t, NULL);
}

/* Drives straight at power for ms milliseconds, then brakes. */
static void drive_for(int power, double ms) {
  BT_drive(LEFT_MOTOR, RIGHT_MOTOR, power);
  sleep_ms(ms);
  stop_motors();
}

/* How long the sensor takes to cross one street while spinning. Starts as
 * STREET_MS_GUESS; the first full crossing measured replaces it, and later
 * ones are averaged in. */
static double street_ms = STREET_MS_GUESS;

/* Records one measured street crossing (first black read to first read off
 * it, so the CONFIRM_READS delay cancels out). */
static void record_crossing(double ms) {
  static int measured = 0;
  street_ms = measured ? 0.5 * street_ms + 0.5 * ms : ms;
  measured = 1;
}

/* Spins in place (dir +1 = clockwise/right, -1 = counter-clockwise/left)
 * until the sensor has entered n streets, then turns on until it's centred
 * on the n-th one: half a street-width of spin from where the sensor first
 * read black there. Entering or leaving a street only counts once
 * CONFIRM_READS reads in a row agree, so one misread can't add a street, but
 * the timing uses the first read of each run, since confirming takes a few
 * reads' worth of turning.
 *
 * Returns 1 on success, 0 on a sensor failure or timeout. */
static int spin_streets(int dir, int n) {
  int c = read_colour();
  if (c < 0)
    return 0;
  int on_street = c == BLACK, flip = 0, entered = 0;
  double edge_at = 0, entered_at = 0, give_up = now_ms() + SCAN_TIMEOUT_MS;

  BT_turn(LEFT_MOTOR, dir * SCAN_POWER, RIGHT_MOTOR, -dir * SCAN_POWER);
  while (entered < n) {
    c = read_colour();
    if (c < 0 || now_ms() > give_up) {
      stop_motors();
      fprintf(stderr, "spin_streets: %s after %d of %d streets\n",
              c < 0 ? "sensor read failed" : "timed out", entered, n);
      return 0;
    }
    if ((c == BLACK) == on_street) {
      flip = 0;
      continue;
    }
    if (flip++ == 0)
      edge_at = now_ms();
    if (flip >= CONFIRM_READS) {
      on_street = !on_street;
      flip = 0;
      if (on_street) {
        entered++;
        entered_at = edge_at;
      } else if (entered_at > 0) {
        record_crossing(edge_at - entered_at);
      }
    }
  }
  double left = street_ms / 2 - (now_ms() - entered_at) - SPIN_COAST_MS;
  if (left > 0)
    sleep_ms(left);
  stop_motors();
  return 1;
}

/* Spins in place (dir +1 = clockwise/right, -1 = counter-clockwise/left)
 * until the sensor leaves a street, then keeps spinning for
 * BUILDING_VOTE_STREETS street-widths (timed with the measured street_ms, so
 * it covers the same angle at any spin speed), voting on every reading, and
 * stops with the sensor still over the building. If the sensor starts off the
 * street (over the other side's building, after the previous sweep), it first
 * crosses the street, and that crossing updates street_ms. Leaving or
 * entering only counts once CONFIRM_READS reads in a row agree, so the blurry
 * readings at the street edge don't vote.
 *
 * Returns the most-voted building colour (blue, green, or white), or -1 on a
 * sensor failure, a timeout, or if no reading was a building colour. */
static int sweep_building(int dir) {
  int votes[NUM_COLORS] = {0};
  int c = read_colour();
  if (c < 0)
    return -1;
  int on_street = c == BLACK, flip = 0, voting = 0;
  double edge_at = 0, entered_at = 0, vote_start = 0, vote_end = 0;
  double give_up = now_ms() + SCAN_TIMEOUT_MS;

  BT_turn(LEFT_MOTOR, dir * SCAN_POWER, RIGHT_MOTOR, -dir * SCAN_POWER);
  while (!voting || now_ms() < vote_end) {
    c = read_colour();
    if (c < 0 || now_ms() > give_up) {
      stop_motors();
      fprintf(stderr, "sweep_building: %s\n",
              c < 0 ? "sensor read failed"
                    : "timed out looking for the street");
      return -1;
    }
    if (voting) {
      votes[c]++;
      continue;
    }
    if ((c == BLACK) == on_street) {
      flip = 0;
      continue;
    }
    if (flip++ == 0)
      edge_at = now_ms();
    if (flip >= CONFIRM_READS) {
      on_street = !on_street;
      flip = 0;
      if (on_street) {
        entered_at = edge_at;
      } else {
        if (entered_at > 0)
          record_crossing(edge_at - entered_at);
        voting = 1;
        vote_start = now_ms();
        vote_end = vote_start + BUILDING_VOTE_STREETS * street_ms;
      }
    }
  }
  stop_motors();

  int best = -1;
  for (int k = 1; k < NUM_COLORS; k++)
    if (BUILDING_MASK & COLOUR_BIT(k) && votes[k] > 0 &&
        (best < 0 || votes[k] > votes[best]))
      best = k;
  if (VERBOSE)
    printf("sweep %-5s: black %d, blue %d, green %d, yellow %d, red %d, "
           "white %d in %.0f ms -> %s\n",
           dir > 0 ? "right" : "left", votes[BLACK], votes[2], votes[3],
           votes[YELLOW], votes[RED], votes[WHITE], now_ms() - vote_start,
           best > 0 ? colour_names[best] : "no building colour");
  return best;
}

/* Reads the two buildings either side of the street the sensor is on: sweeps
 * right, then left across the street, then back right until it's centred on
 * the street again. Returns 1 on success. */
static int sweep_pair(int *right, int *left) {
  *right = sweep_building(+1);
  if (*right < 0)
    return 0;
  *left = sweep_building(-1);
  if (*left < 0)
    return 0;
  return spin_streets(+1, 1);
}

/* With STEP_SCAN on, stops the scan until Enter so the sensor's position can
 * be checked by eye. */
static void scan_pause(const char *where) {
  if (!STEP_SCAN)
    return;
  printf("Scan paused: %s. Press Enter to continue\n", where);
  wait_for_enter();
}

/* Drives straight (power's sign picks forward/back) until the sensor reaches
 * the yellow, and stops there. With cross set, it keeps going until the
 * sensor has also left the yellow, then PAST_YELLOW_MS more, and stops; if
 * the sensor starts on the yellow, it only has to leave it. Both edges only
 * count once CONFIRM_READS reads in a row agree. Returns 1 on success, 0 on
 * a sensor failure or if the edge never shows. */
static int drive_to_yellow(int power, int cross) {
  int c = read_colour();
  if (c < 0)
    return 0;
  int on_yellow = c == YELLOW, seen_yellow = on_yellow, flip = 0;
  double give_up = now_ms() + YELLOW_TIMEOUT_MS;

  BT_drive(LEFT_MOTOR, RIGHT_MOTOR, power);
  while (!seen_yellow || (cross && on_yellow)) {
    c = read_colour();
    if (c < 0 || now_ms() > give_up) {
      stop_motors();
      fprintf(stderr, "drive_to_yellow: %s\n",
              c < 0         ? "sensor read failed"
              : seen_yellow ? "never left the yellow"
                            : "never reached the yellow");
      return 0;
    }
    if ((c == YELLOW) == on_yellow) {
      flip = 0;
    } else if (++flip >= CONFIRM_READS) {
      on_yellow = !on_yellow;
      flip = 0;
      seen_yellow |= on_yellow;
    }
  }
  if (cross)
    sleep_ms(PAST_YELLOW_MS);
  stop_motors();
  return 1;
}

/* Checks the sensor is on the street, sweeping to find it if not. Returns 1
 * if it is. */
static int check_on_street(const char *where) {
  if (read_colour() == BLACK || find_black() == BLACK)
    return 1;
  fprintf(stderr, "scan_intersection: sensor not on the street %s\n", where);
  return 0;
}

/* Spins in place until the sensor has left the street (CONFIRM_READS
 * non-black reads in a row), then stops. Returns 1 on success. */
static int spin_off_street(int dir) {
  int off = 0;
  double give_up = now_ms() + SCAN_TIMEOUT_MS;
  BT_turn(LEFT_MOTOR, dir * SCAN_POWER, RIGHT_MOTOR, -dir * SCAN_POWER);
  while (off < CONFIRM_READS) {
    int c = read_colour();
    if (c < 0 || now_ms() > give_up) {
      stop_motors();
      fprintf(stderr, "spin_off_street: %s\n",
              c < 0 ? "sensor read failed" : "timed out");
      return 0;
    }
    off = c == BLACK ? 0 : off + 1;
  }
  stop_motors();
  return 1;
}

/* With the axle over the intersection centre, turns the robot until the
 * sensor is centred on the street ahead: spins right off the street, then
 * back left until centred on it. Axle and sensor both on the street's centre
 * line means the robot points straight along it, however crooked it arrived.
 * Returns 1 on success. */
static int align_with_street(void) {
  return check_on_street("after centring - check CENTER_DRIVE_MS") &&
         spin_off_street(+1) && spin_streets(-1, 1);
}

/* Run with target -2 -2. Spins at SCAN_POWER for as long as the code thinks
 * one full turn takes; watch how far the robot actually turns, then correct
 * SCAN_DEG_PER_MS using the printed formula. */
static void spin_test(void) {
  double ms = 360.0 / SCAN_DEG_PER_MS;
  printf("Spin test: mark the robot's heading, then press Enter. It will spin "
         "clockwise at power %d for %.0f ms (should be exactly 360 deg).\n",
         SCAN_POWER, ms);
  wait_for_enter();
  BT_turn(LEFT_MOTOR, SCAN_POWER, RIGHT_MOTOR, -SCAN_POWER);
  sleep_ms(ms);
  stop_motors();
  printf("If it turned A degrees instead of 360, set\n"
         "  SCAN_DEG_PER_MS = %.4f * A / 360\n",
         SCAN_DEG_PER_MS);
}
#endif
// And ends here

int map[400][4]; // This holds the representation of the map, up to 20x20
                 // intersections, raster ordered, 4 building colours per
                 // intersection.
int sx, sy;      // Size of the map (number of intersections along x and y)
static inline int coords_to_idx(int x, int y) { return x + y * sx; }
double beliefs[400][4]; // Beliefs for each location and motion direction

// Localization helpers (ours) - these need the map/beliefs globals above.

/* P(the sensor reports a building's colour correctly). A placeholder until
 * calibration measures the real value; calibration can overwrite it. */
static double colour_accuracy = 0.85;
#define BOUNDARY_ACCURACY                                                      \
  0.95                       /* P(a drive ends on red iff we left the map)     \
                              */
#define SPAWN_FRACTION 0.01  /* belief for newly entered states, x uniform */
#define CONFIDENCE_RATIO 3.0 /* best state must beat the runner-up by this */
#define MAX_LOCALIZATION_STEPS 100
/* Chance of a random move while not confident. Without it, a tie between two
 * look-alike states can make the robot shuttle between them forever. */
#define EXPLORE_PROB 0.3

static const int dir_dx[4] = {0, 1, 0, -1}; /* UP, RIGHT, DOWN, LEFT */
static const int dir_dy[4] = {-1, 0, 1, 0};
static const char *dir_names[4] = {"UP", "RIGHT", "DOWN", "LEFT"};
/* Indexed by number of clockwise quarter turns */
static const char *turn_names[4] = {"go straight", "turn right", "turn around",
                                    "turn left"};

static int in_map(int x, int y) { return x >= 0 && x < sx && y >= 0 && y < sy; }

/* Prints the beliefs as four sx-by-sy grids side by side, one per heading. */
static void print_beliefs(const char *label) {
  if (!VERBOSE)
    return;
  printf("---- beliefs %s ----\n", label);
  for (int d = 0; d < 4; d++)
    printf("%-*s", sx * 10 + 3, dir_names[d]);
  printf("\n");
  for (int y = 0; y < sy; y++) {
    for (int d = 0; d < 4; d++) {
      for (int x = 0; x < sx; x++)
        printf("%10.3e", beliefs[coords_to_idx(x, y)][d]);
      printf("   ");
    }
    printf("\n");
  }
}

/* Rescales beliefs to sum to 1. Falls back to uniform if everything hit 0. */
static void normalize_beliefs(void) {
  double sum = 0;
  for (int i = 0; i < sx * sy; i++)
    for (int d = 0; d < 4; d++)
      sum += beliefs[i][d];
  for (int i = 0; i < sx * sy; i++)
    for (int d = 0; d < 4; d++)
      beliefs[i][d] = sum > 0 ? beliefs[i][d] / sum : 1.0 / (sx * sy * 4);
}

/* Returns the largest belief and stores its state; *second gets the
 * runner-up belief. */
static double best_state(int *bx, int *by, int *bd, double *second) {
  double best = -1;
  *second = -1;
  for (int y = 0; y < sy; y++)
    for (int x = 0; x < sx; x++)
      for (int d = 0; d < 4; d++) {
        double b = beliefs[coords_to_idx(x, y)][d];
        if (b > best) {
          *second = best;
          best = b;
          *bx = x;
          *by = y;
          *bd = d;
        } else if (b > *second) {
          *second = b;
        }
      }
  return best;
}

/* Sensor update: weights every state by acc^matches * (1-acc)^mismatches.
 * The reading is robot-relative, so facing direction d (d clockwise quarter
 * turns from UP) means robot corner k sees map corner (k + d) % 4. */
static void sensor_update(const int reading[4]) {
  for (int i = 0; i < sx * sy; i++)
    for (int d = 0; d < 4; d++) {
      int matches = 0;
      for (int k = 0; k < 4; k++)
        if (reading[k] == map[i][(k + d) % 4])
          matches++;
      beliefs[i][d] *= pow(colour_accuracy, matches) *
                       pow(1.0 - colour_accuracy, 4 - matches);
    }
}

/* Motion update for turning in place by quarter_turns clockwise (1 = right,
 * 2 = about-face, 3 = left). Position is unchanged; only the heading rotates.
 */
static void turn_update(int quarter_turns) {
  for (int i = 0; i < sx * sy; i++) {
    double old[4];
    memcpy(old, beliefs[i], sizeof(old));
    for (int d = 0; d < 4; d++)
      beliefs[i][(d + quarter_turns) % 4] = old[d];
  }
}

/* Motion update for driving one block and arriving at a yellow intersection.
 * Each state takes the belief of the state one block behind it. States that
 * would have driven off the map are dropped (we'd have seen red), and states
 * with nothing behind them get a small non-zero value so we can recover. */
static void forward_update(void) {
  static double next[400][4];
  double spawn = SPAWN_FRACTION / (double)(sx * sy * 4);
  for (int y = 0; y < sy; y++)
    for (int x = 0; x < sx; x++)
      for (int d = 0; d < 4; d++) {
        int px = x - dir_dx[d], py = y - dir_dy[d];
        next[coords_to_idx(x, y)][d] =
            in_map(px, py) ? beliefs[coords_to_idx(px, py)][d] : spawn;
      }
  memcpy(beliefs, next, sizeof(next));
}

/* Sensing update for a drive that ended on red, applied once the robot has
 * turned around (before the motion update for the drive back): only states
 * with the map edge directly behind them are consistent with having bounced
 * off it. */
static void boundary_update(void) {
  for (int y = 0; y < sy; y++)
    for (int x = 0; x < sx; x++)
      for (int d = 0; d < 4; d++) {
        int edge_behind = !in_map(x - dir_dx[d], y - dir_dy[d]);
        beliefs[coords_to_idx(x, y)][d] *=
            edge_behind ? BOUNDARY_ACCURACY : 1.0 - BOUNDARY_ACCURACY;
      }
}

/* Physically turns the robot quarter_turns clockwise. Returns 1 on success. */
static int turn_robot(int quarter_turns) {
  if (quarter_turns == 3)
    return turn_at_intersection(1);
  for (int t = 0; t < quarter_turns; t++)
    if (!turn_at_intersection(0))
      return 0;
  return 1;
}

/* Drives to the next intersection and applies the matching belief update.
 * If the drive ends on red, the robot turns around and drives back, ending
 * one block in from the edge intersection it left. from_intersection is 0
 * for the very first drive, when the robot started somewhere on a street and
 * there is no previous state to shift. Returns 1 if the robot ended at an
 * intersection, 0 on failure. */
static int advance_to_intersection(int from_intersection) {
  int result = drive_along_street();
  if (result == 4) {
    if (VERBOSE)
      printf("Drive ended on yellow (intersection)\n");
    if (from_intersection)
      forward_update();
    return 1;
  }
  if (result != 5)
    return 0;

  if (VERBOSE)
    printf("Drive ended on red (boundary), turning around\n");
  if (!turn_robot(2))
    return 0;
  turn_update(2);
  if (drive_along_street() != 4)
    return 0;
  /* Red is only 2-4 cm past where the sensor starts after leaving the edge
   * intersection, so after turning around there the sensor is already past
   * that intersection's yellow, and the drive back ends one block further
   * in. So: weight states that were at the edge facing in, then shift them
   * one block forward. */
  boundary_update();
  forward_update();
  return 1;
}

/* Motion update for a recovery: the robot ended at the same intersection or
 * a neighbouring one, facing any way. Spreads each intersection's belief
 * evenly over itself and its on-map neighbours, and over all 4 headings, so
 * what we knew before still counts but no longer pins down the state. */
static void lost_update(void) {
  static double next[400][4];
  double spawn = SPAWN_FRACTION / (double)(sx * sy * 4);
  for (int i = 0; i < sx * sy; i++)
    for (int d = 0; d < 4; d++)
      next[i][d] = spawn;
  for (int y = 0; y < sy; y++)
    for (int x = 0; x < sx; x++) {
      double total = 0;
      for (int d = 0; d < 4; d++)
        total += beliefs[coords_to_idx(x, y)][d];
      int targets[5], n = 0;
      targets[n++] = coords_to_idx(x, y);
      for (int d = 0; d < 4; d++)
        if (in_map(x + dir_dx[d], y + dir_dy[d]))
          targets[n++] = coords_to_idx(x + dir_dx[d], y + dir_dy[d]);
      for (int t = 0; t < n; t++)
        for (int d = 0; d < 4; d++)
          next[targets[t]][d] += total / (n * 4);
    }
  memcpy(beliefs, next, sizeof(next));
}

/* Gets the robot going again after a failed scan, turn, or drive: finds the
 * street, drives to the next intersection, and applies lost_update(), since
 * we don't know which way the failure left it. Returns 1 once it is at an
 * intersection again. */
static int recover(void) {
  printf("Recovering: finding the street and driving to the next "
         "intersection\n");
  if (!find_street() || !advance_to_intersection(0)) {
    fprintf(stderr, "recover: failed, will try again\n");
    return 0;
  }
  lost_update();
  normalize_beliefs();
  print_beliefs("after recovery (normalized)");
  return 1;
}

/* Picks how many clockwise quarter turns to make before driving one block,
 * assuming the robot is at (x,y) facing d. Prefers going straight, then right,
 * then left, then turning around, taking the first move that gets closer to
 * the destination. If already there, takes the first move that stays on the
 * map, to keep gathering evidence. While not confident, makes a random
 * on-map move with probability EXPLORE_PROB instead. */
static int choose_action(int x, int y, int d, int dest_x, int dest_y,
                         int confident) {
  static const int order[4] = {0, 1, 3, 2};
  int dist = abs(dest_x - x) + abs(dest_y - y);
  if (!confident && rand() < EXPLORE_PROB * RAND_MAX) {
    int legal[4], n = 0;
    for (int t = 0; t < 4; t++)
      if (in_map(x + dir_dx[(d + t) % 4], y + dir_dy[(d + t) % 4]))
        legal[n++] = t;
    if (n > 0) {
      if (VERBOSE)
        printf("Exploring with a random move\n");
      return legal[rand() % n];
    }
  }
  for (int o = 0; o < 4; o++) {
    int nd = (d + order[o]) % 4;
    int nx = x + dir_dx[nd], ny = y + dir_dy[nd];
    if (!in_map(nx, ny))
      continue;
    if (dist == 0 || abs(dest_x - nx) + abs(dest_y - ny) < dist)
      return order[o];
  }
  return 2; /* only reachable on a 1x1 map */
}

int main(int argc, char *argv[]) {
  char mapname[1024];
  int dest_x, dest_y, rx, ry;
  unsigned char *map_image;

  memset(&map[0][0], 0, 400 * 4 * sizeof(int));
  sx = 0;
  sy = 0;

  if (argc < 4) {
    fprintf(stderr, "Usage: EV3_Localization map_name dest_x dest_y\n");
    fprintf(stderr, "    map_name - should correspond to a properly formatted "
                    ".ppm map image\n");
    fprintf(stderr, "    dest_x, dest_y - target location for the bot within "
                    "the map, -1 -1 calls calibration routine\n");
    exit(1);
  }
  strcpy(&mapname[0], argv[1]);
  dest_x = atoi(argv[2]);
  dest_y = atoi(argv[3]);

  // Open a socket to the EV3 for remote controlling the bot. Opened once here
  // so both calibration and localization share the same connection.
  if (BT_open(HEXKEY) != 0) {
    fprintf(stderr, "Unable to open comm socket to the EV3, make sure the EV3 "
                    "kit is powered on, and that the\n");
    fprintf(stderr,
            " hex key for the EV3 matches the one in EV3_Localization.h\n");
    exit(1);
  }
#ifndef SIMULATE
  rgb_t v;
  read_rgb(&v); /* first read after connecting returns NaN; discard it */
#endif

  if (dest_x == -1 && dest_y == -1) {
    calibrate_sensor();
    BT_close();
    exit(1);
  }
#ifndef SIMULATE
  if (dest_x == -2 && dest_y == -2) { /* ours: measure the spin rate */
    spin_test();
    BT_close();
    exit(0);
  }
#endif

  /******************************************************************************************************************
   * OPTIONAL TO DO: If you added code for sensor calibration, add just below
   * this comment block any code needed to read your calibration data for use in
   * your localization code. Skip this if you are not using calibration
   * ****************************************************************************************************************/
  load_calibration();
  // Your code for reading any calibration information should not go below this
  // line //

  map_image = readPPMimage(&mapname[0], &rx, &ry);
  if (map_image == NULL) {
    fprintf(stderr, "Unable to open specified map image\n");
    BT_close();
    exit(1);
  }

  if (parse_map(map_image, rx, ry) == 0) {
    printf("sx: %d", sx);
    printf("sy: %d", sy);
    fprintf(stderr, "Unable to parse input image map. Make sure the image is "
                    "properly formatted\n");
    free(map_image);
    BT_close();
    exit(1);
  }

  if (dest_x < 0 || dest_x >= sx || dest_y < 0 || dest_y >= sy) {
    fprintf(stderr, "Destination location is outside of the map\n");
    free(map_image);
    BT_close();
    exit(1);
  }

  // Initialize beliefs - uniform probability for each location and direction
  for (int j = 0; j < sy; j++)
    for (int i = 0; i < sx; i++) {
      beliefs[i + (j * sx)][0] = 1.0 / (double)(sx * sy * 4);
      beliefs[i + (j * sx)][1] = 1.0 / (double)(sx * sy * 4);
      beliefs[i + (j * sx)][2] = 1.0 / (double)(sx * sy * 4);
      beliefs[i + (j * sx)][3] = 1.0 / (double)(sx * sy * 4);
    }

#ifndef SIMULATE
  if (!check_sensor()) {
    fprintf(stderr, "Colour sensor is not responding on port %d\n",
            SENSOR_PORT + 1);
    BT_close();
    free(map_image);
    exit(1);
  }
#endif

  fprintf(stderr, "All set, ready to go!\n");

  /*******************************************************************************************************************************
   *
   *  TO DO - Implement the main localization loop, this loop will have the
   * robot explore the map, scanning intersections and updating beliefs in the
   * beliefs array until a single location/direction is determined to be the
   * correct one.
   *
   *          The beliefs array contains one row per intersection (recall that
   * the number of intersections in the map_image is given by sx, sy, and that
   * the map[][] array contains the colour indices of buildings around each
   * intersection. Indexing into the map[][] and beliefs[][] arrays is by raster
   * order, so for an intersection at i,j (with 0<=i<=sx-1 and 0<=j<=sy-1),
   * index=i+(j*sx)
   *
   *          In the beliefs[][] array, you need to keep track of 4 values per
   * intersection, these correspond to the belief the robot is at that specific
   * intersection, moving in one of the 4 possible directions as follows:
   *
   *          beliefs[i][0] <---- belief the robot is at intersection with index
   * i, facing UP beliefs[i][1] <---- belief the robot is at intersection with
   * index i, facing RIGHT beliefs[i][2] <---- belief the robot is at
   * intersection with index i, facing DOWN beliefs[i][3] <---- belief the robot
   * is at intersection with index i, facing LEFT
   *
   *          Initially, all of these beliefs have uniform, equal probability.
   * Your robot must scan intersections and update belief values based on
   * agreement between what the robot sensed, and the colours in the map.
   *
   *          You have two main tasks these are organized into two major
   * functions:
   *
   *          robot_localization()    <---- Runs the localization loop until the
   * robot's location is found go_to_target()          <---- After localization
   * is achieved, takes the bot to the specified map location
   *
   *          The target location, read from the command line, is left in
   * dest_x, dest_y
   *
   *          Here in main(), you have to call these two functions as
   * appropriate. But keep in mind that it is always possible that even if your
   * bot managed to find its location, it can become lost again while driving to
   * the target location, or it may be the initial localization was wrong and
   * the robot ends up in an unexpected place - a very solid implementation
   * should give your robot the ability to determine it's lost and needs to run
   * localization again.
   *
   *******************************************************************************************************************************/

  // HERE - write code to call robot_localization() and go_to_target() as
  // needed, any additional logic required to get the
  //        robot to complete its task should be here.
  // robot_localization() keeps localizing while it drives toward the
  // destination, so it also does go_to_target()'s job.
  int robot_x, robot_y, direction;
  if (robot_localization(dest_x, dest_y, &robot_x, &robot_y, &direction))
    fprintf(stderr, "Done: robot at (%d,%d) facing %s\n", robot_x, robot_y,
            dir_names[direction]);
  else
    fprintf(stderr, "Localization did not reach the destination\n");

  // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
  BT_close();
  free(map_image);
  exit(0);
}

int find_street(void) {
  /*
   * This function gets your robot onto a street, wherever it is placed on the
   * map. You can do this in many ways, but think about what is the most
   * effective and reliable way to detect a street and stop your robot once it's
   * on it.
   *
   * You can use the return value to indicate success or failure, or to inform
   * the rest of your code of the state of your bot after calling this function
   */
  if (VERBOSE) {
    printf("find_street called\n");
  }
#ifdef SIMULATE
  return 1;
#else
  /* Minimal version: assumes the robot was placed close to a street, so a
   * sweep reaches it without driving anywhere. */
  int c = read_colour();
  if (c == BLACK || c == YELLOW)
    return 1;
  return find_black() > 0;
#endif
}

/* Returns 4 if the drive ended on yellow (intersection), 5 if on red
 * (boundary), 0 on failure. */
int drive_along_street(void) {
  /*
   * This function drives your bot along a street, making sure it stays on the
   * street without straying to other pars of the map. It stops at an
   * intersection.
   *
   * You can implement this in many ways, including a controlled (PID for
   * example), a neural network trained to track and follow streets, or a
   * carefully coded process of scanning and moving. It's up to you, feel free
   * to consult your TA or the course instructor for help carrying out your
   * plan.
   *
   * You can use the return value to indicate success or failure, or to inform
   * the rest of your code of the state of your bot after calling this function.
   */
#ifdef SIMULATE
  char buf[64] = {0};
  printf("drive_along_street called. Did it end on (y)ellow or (r)ed? ");
  if (!read_line(buf, sizeof(buf)))
    return 0;
  if (buf[0] == 'y' || buf[0] == 'Y')
    return 4;
  if (buf[0] == 'r' || buf[0] == 'R')
    return 5;
  return 0;
#else
  /*
   * Polls the sensor in a loop while driving slowly:
   *  - black: on the street, keep driving straight.
   *  - yellow/red: stop once CONFIRM_READS reads in a row agree, so a single
   *    misread doesn't end the drive. Yellow is ignored until we have left
   *    the intersection we started on (the sensor starts over it).
   *  - anything else, CONFIRM_READS times in a row: we drifted off the
   *    street, so stop and sweep side to side until the sensor finds black.
   */
  int left_start = 0; /* have we driven off the starting yellow yet? */
  int last = 0, count = 0, driving = 0;
  for (int i = 0; i < MAX_DRIVE_READS; i++) {
    int c = read_colour();
    if (c < 0)
      break;
    count = c == last ? count + 1 : 1;
    last = c;
    if (c != YELLOW)
      left_start = 1;

    int stop_colour = c == RED || (c == YELLOW && left_start);
    if (stop_colour && count >= CONFIRM_READS) {
      stop_motors();
      if (VERBOSE)
        printf("drive_along_street: stopped on %s\n", colour_names[c]);
      return c;
    }
    if (c == BLACK || c == YELLOW || count < OFF_STREET_READS) {
      /* On the street (or still confirming): keep going */
      if (!driving) {
        BT_drive(LEFT_MOTOR, RIGHT_MOTOR, DRIVE_POWER);
        driving = 1;
      }
      continue;
    }

    if (VERBOSE)
      printf("drive_along_street: off the street (%s), searching\n",
             colour_names[c]);
    stop_motors();
    driving = 0;
    if (find_black() <= 0) {
      fprintf(stderr, "drive_along_street: lost the street\n");
      return 0;
    }
    last = count = 0;
  }
  stop_motors();
  fprintf(stderr, "drive_along_street: sensor failed or drive took too long\n");
  return 0;
#endif
}

/* Returns 1 on success, 0 on failure. Colours are 2 blue, 3 green, 6 white. */
int scan_intersection(int *tl, int *tr, int *br, int *bl) {
  /*
   * This function carries out the intersection scan - the bot should
   * (obviously) be placed at an intersection for this, and the specific set of
   * actions will depend on how you designed your bot and its sensor. Whatever
   * the process, you should make sure the intersection scan is reliable - i.e.
   * the positioning of the sensor is reliably over the buildings it needs to
   * read, repeatably, and as the robot moves over the map.
   *
   * Use the APIs sensor reading calls to poll the sensors. You need to remember
   * that sensor readings are noisy and unreliable so * YOU HAVE TO IMPLEMENT
   * SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
   *
   * Recall your lectures on sensor and noise management, and implement a
   * strategy that makes sense. Document your process in the code below so your
   * TA can quickly understand how it works.
   *
   * Once your bot has read the colours at the intersection, it must return them
   * using the provided pointers to 4 integer variables:
   *
   * tl - top left building colour
   * tr - top right building colour
   * br - bottom right building colour
   * bl - bottom left building colour
   *
   * The function's return value can be used to indicate success or failure, or
   * to notify your code of the bot's state after this call.
   */

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

#ifdef SIMULATE
  char buf[64] = {0};
  /* Colours are relative to the robot's heading (tl = front-left), exactly as
   * the real sensor would see them - not relative to the map's "up". */
  printf("scan_intersection called. Enter tl tr br bl relative to the robot's "
         "heading as 4 letters from g/b/w, e.g. gbgw: ");
  if (!read_line(buf, sizeof(buf)))
    return 0;
  *tl = letter_to_colour(buf[0]);
  *tr = letter_to_colour(buf[1]);
  *br = letter_to_colour(buf[2]);
  *bl = letter_to_colour(buf[3]);
  if (*tl < 0 || *tr < 0 || *br < 0 || *bl < 0)
    return 0;
  return 1;
#else
  /*
   * The buildings sit right beside the intersection, too close for one full
   * spin to read them (see the comment above PAST_YELLOW_MS). Instead the
   * robot reads them in two pairs from spots along the street, each placed
   * from a yellow edge the sensor sees, with a stop at the centre to
   * straighten up in between (see the comment above CENTER_DRIVE_MS).
   *
   *  1. drive_along_street() stops with the sensor just onto the yellow.
   *     Reverse until it leaves the yellow, and PAST_YELLOW_MS more (~2 cm):
   *     the sensor is on the street we came in on, level with the middle of
   *     the two rear buildings.
   *  2. Sweep right until the sensor leaves the street: the first thing past
   *     the street edge is the back-right building, so the readings over
   *     the next BUILDING_VOTE_STREETS street-widths of spin vote on its
   *     colour. Then sweep left across the street to vote on back-left the
   *     same way, and come back to centre on the street.
   *  3. Drive forward until the sensor is back onto the yellow - the spot
   *     drive_along_street() stopped at - then CENTER_DRIVE_MS to put the
   *     axle over the centre, and centre the sensor on the street ahead
   *     (align_with_street()) so the robot points along the street.
   *  4. Reverse until the sensor reaches the yellow's far edge, then drive
   *     forward off it and PAST_YELLOW_MS more: the sensor is on the street
   *     ahead, level with the middle of the front buildings. Sweep the same
   *     way.
   *  5. Drive FRONT_TO_CENTRE_MS to put the axle back over the centre, where
   *     turn_at_intersection() expects it.
   *
   * Voting only right after leaving the street keeps the sensor well inside
   * the building it's reading (~30 deg of sweep), and well away from the
   * white gap and the next intersection's building beyond it. The majority
   * vote outvotes single misreads.
   */
  *tl = *tr = *br = *bl = -1;

  scan_pause("sensor should be just onto the yellow");
  if (!drive_to_yellow(-DRIVE_POWER, 1))
    return 0;
  scan_pause("sensor should be beside the middle of the rear buildings");
  if (!check_on_street("beside the rear buildings - check PAST_YELLOW_MS") ||
      !sweep_pair(br, bl))
    return 0;
  scan_pause("rear done, sensor should be centred on the street");
  if (!drive_to_yellow(DRIVE_POWER, 0))
    return 0;
  drive_for(DRIVE_POWER, CENTER_DRIVE_MS);
  if (!align_with_street())
    return 0;
  scan_pause("axle should be over the centre, robot straight along the street");
  if (!drive_to_yellow(-DRIVE_POWER, 0) || !drive_to_yellow(DRIVE_POWER, 1))
    return 0;
  scan_pause("sensor should be beside the middle of the front buildings");
  if (!check_on_street("beside the front buildings - check PAST_YELLOW_MS") ||
      !sweep_pair(tr, tl))
    return 0;
  scan_pause("front done, sensor should be centred on the street");
  drive_for(DRIVE_POWER, FRONT_TO_CENTRE_MS);
  scan_pause("axle should be over the centre again");
  if (!check_on_street("after centring - check FRONT_TO_CENTRE_MS"))
    return 0;

  printf("Scanned (robot-relative tl tr br bl): %s %s %s %s\n",
         colour_names[*tl], colour_names[*tr], colour_names[*br],
         colour_names[*bl]);
  return 1;
#endif
}

int turn_at_intersection(int turn_direction) {
  /*
   * This function is used to have the robot turn either left or right at an
   * intersection (obviously your bot can not just drive forward!).
   *
   * If turn_direction=0, turn right, else if turn_direction=1, turn left.
   *
   * You're free to implement this in any way you like, but it should reliably
   * leave your bot facing the correct direction and on a street it can follow.
   *
   * You can use the return value to indicate success or failure, or to inform
   * your code of the state of the bot
   */
  if (VERBOSE) {
    printf("turn_at_intersection called: turning %s\n",
           turn_direction == 0 ? "right" : "left");
  }
#ifdef SIMULATE
  return 1;
#else
  /* The robot is centred on the intersection after scanning, with the sensor
   * on the street ahead. A 90 degree turn is "spin until the sensor reaches
   * the next street" - counted, not timed, like the scan. */
  return spin_streets(turn_direction == 0 ? +1 : -1, 1);
#endif
}

int robot_localization(int dest_x, int dest_y, int *robot_x, int *robot_y,
                       int *direction) {
  /*  This function implements the main robot localization process. You have to
   * write all code that will control the robot and get it to carry out the
   * actions required to achieve localization.
   *
   *  Localization process:
   *
   *  - Find the street, and drive along the street toward an intersection
   *  - Scan the colours of buildings around the intersection
   *  - Update the beliefs in the beliefs[][] array according to the sensor
   * measurements and the map data
   *  - Repeat the process until a single intersection/facing direction is
   * distintly more likely than all the rest
   *
   *  * We have provided headers for the following functions:
   *
   *  find_street()
   *  drive_along_street()
   *  scan_intersection()
   *  turn_at_intersection()
   *
   *  You *do not* have to use them, and can write your own to organize your
   * robot's work as you like, they are provided as a suggestion.
   *
   *  Note that *your bot must explore* the map to achieve reliable
   * localization, this means your intersection scanning strategy should not
   * rely exclusively on moving forward, but should include turning and
   * exploring other streets than the one your bot was initially placed on.
   *
   *  For each of the control functions, however, you will need to use the EV3
   * API, so be sure to become familiar with it.
   *
   *  In terms of sensor management - the API allows you to read colours either
   * as indexed values or RGB, it's up to you which one to use, and how to
   * interpret the noisy, unreliable data you're likely to get from the sensor
   *  in order to update beliefs.
   *
   *  HOWEVER: *** YOU must document clearly both in comments within this
   * function, and in your report, how the sensor is used to read colour data,
   * and how the beliefs are updated based on the sensor readings.
   *
   *  DO NOT FORGET - Beliefs should always remain normalized to be a
   * probability distribution, that means the sum of beliefs over all
   * intersections and facing directions must be 1 at all times.
   *
   *  The function receives as input pointers to three integer values, these
   * will be used to store the estimated robot's location and facing direction.
   * The direction is specified as: 0 - UP 1 - RIGHT 2 - BOTTOM 3 - LEFT
   *
   *  The function's return value is 1 if localization was successful, and 0
   * otherwise.
   */

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

  /*
   * Our approach (changed signature: also takes the destination). Localization
   * never stops, so instead of localizing and then handing off to
   * go_to_target(), each step drives toward the destination from the most
   * likely state, which both explores the map and makes progress. Each step:
   *
   *  1. Sense: scan the 4 buildings (robot-relative), weight each state by
   *     acc^matches * (1-acc)^mismatches after rotating the reading into map
   *     orientation for that state's heading, then normalize.
   *  2. Stop if the best state beats the runner-up by CONFIDENCE_RATIO and is
   *     the destination.
   *  3. Act: turn 0/1/2/3 quarter turns toward the destination (rotating the
   *     headings in beliefs[][] to match), then drive one block. Arriving on
   *     yellow shifts beliefs one block forward; arriving on red means we
   *     were at the edge facing out, so the robot turns around, beliefs are
   *     weighted by whether the edge is behind each state, and the drive
   *     back (which passes the edge intersection and ends one block in)
   *     shifts them one block forward. Normalize, then repeat.
   *
   * Returns 1 with the final state once confident and at the destination, 0
   * on failure or after MAX_LOCALIZATION_STEPS steps.
   */
  int reading[4], bx, by, bd;
  int was_ever_confident = 0; /* the confident sound plays only once */
  double best, second;

  *robot_x = -1;
  *robot_y = -1;
  *direction = -1;
  srand(time(NULL));

  /* Never give up on a failed scan, turn, or drive: get back to the street,
   * drive to the next intersection, and spread the beliefs to match how
   * little we know about that move (see lost_update()). */
  int need_recovery = !find_street() || !advance_to_intersection(0);
  if (!need_recovery) {
    normalize_beliefs();
    print_beliefs("initial (normalized)");
  }

  for (int step = 0; step < MAX_LOCALIZATION_STEPS; step++) {
    if (VERBOSE)
      printf("\n===== Localization step %d =====\n", step);

    if (need_recovery) {
      need_recovery = !recover();
      continue;
    }

    // Sense
    if (!scan_intersection(&reading[0], &reading[1], &reading[2],
                           &reading[3])) {
      fprintf(stderr, "robot_localization: intersection scan failed\n");
      need_recovery = 1;
      continue;
    }
    if (VERBOSE)
      printf("Scan (robot-relative tl tr br bl): %s %s %s %s\n",
             colour_names[reading[0]], colour_names[reading[1]],
             colour_names[reading[2]], colour_names[reading[3]]);
    sensor_update(reading);
    print_beliefs("after sensing (raw)");
    normalize_beliefs();
    print_beliefs("after sensing (normalized)");

    // Check confidence
    best = best_state(&bx, &by, &bd, &second);
    if (VERBOSE)
      printf("Most likely: (%d,%d) facing %s, belief %.4f, %.2fx runner-up\n",
             bx, by, dir_names[bd], best, best / second);
    int confident = best >= CONFIDENCE_RATIO * second;
    if (confident) {
      printf("Confident we are at (%d,%d) facing %s (belief %.4f, %.2fx "
             "runner-up)\n",
             bx, by, dir_names[bd], best, best / second);
      if (!was_ever_confident) {
        play_confident_sound();
        was_ever_confident = 1;
      }
      if (bx == dest_x && by == dest_y) {
        printf("Reached destination (%d,%d)\n", dest_x, dest_y);
        play_arrival_sound();
        *robot_x = bx;
        *robot_y = by;
        *direction = bd;
        return 1;
      }
    }

    // Act
    int turns = choose_action(bx, by, bd, dest_x, dest_y, confident);
    if (VERBOSE)
      printf("Action: %s, then drive one block\n", turn_names[turns]);
    if (!turn_robot(turns)) {
      fprintf(stderr, "robot_localization: turn failed\n");
      need_recovery = 1;
      continue;
    }
    turn_update(turns);
    if (!advance_to_intersection(1)) {
      fprintf(stderr, "robot_localization: could not reach an intersection\n");
      need_recovery = 1;
      continue;
    }
    print_beliefs("after motion (raw)");
    normalize_beliefs();
    print_beliefs("after motion (normalized)");
  }

  fprintf(stderr, "robot_localization: gave up after %d steps\n",
          MAX_LOCALIZATION_STEPS);
  return 0;
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x,
                 int target_y) {
  /*
   * This function is called once localization has been successful, it performs
   * the actions required to take the robot from its current location to the
   * specified target location.
   *
   * You have to write the code required to carry out this task - once again,
   * you can use the function headers provided, or write your own code to
   * control the bot, but document your process carefully in the comments below
   * so your TA can easily understand how everything works.
   *
   * Your code should be able to determine if the robot has gotten lost (or if
   * localization was incorrect), and your bot should be able to recover.
   *
   * Inputs - The robot's current location x,y (the intersection coordinates,
   * not image pixel coordinates) The target's intersection location
   *
   * Return values: 1 if successful (the bot reached its target destination), 0
   * otherwise
   */

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  return (0);
}

void calibrate_sensor(void) {
  /*
   * This function is called when the program is started with -1  -1 for the
   * target location.
   *
   * You DO NOT NEED TO IMPLEMENT ANYTHING HERE - but it is strongly recommended
   * as good calibration will make sensor readings more reliable and will make
   * your code more resistent to changes in illumination, map quality, or
   * battery level.
   *
   * The principle is - Your code should allow you to sample the different
   * colours in the map, and store representative values that will help you
   * figure out what colours the sensor is reading given the current conditions.
   *
   * Inputs - None
   * Return values - None - your code has to save the calibration information to
   * a file, for later use (see in main())
   *
   * How to do this part is up to you, but feel free to talk with your TA and
   * instructor about it!
   */

  /************************************************************************************************************************
   *   OIPTIONAL TO DO  -   Complete this function
   ***********************************************************************************************************************/
  fprintf(stderr, "Calibration function called!\n");

#ifdef SIMULATE
  /* No real socket is opened in simulation, so sensor reads would crash. */
  fprintf(stderr, "Calibration needs the real robot; disable SIMULATE\n");
  return;
#endif

  /* The EV3 connection is opened (and closed) by main(). */

  /* 1. Centroids: average AVG_SAMPLES reads at each of CALIB_SPOTS different
   *    spots per colour, so the centroid reflects position variation too. */
  for (int c = 1; c < NUM_COLORS; c++) {
    rgb_t sum = {0, 0, 0};
    for (int s = 0; s < CALIB_SPOTS; s++) {
      printf("Place sensor over %s (spot %d of %d), then press Enter\n",
             colour_names[c], s + 1, CALIB_SPOTS);
      wait_for_enter();
      rgb_t r = read_rgb_avg(AVG_SAMPLES);
      play_colour_tone(c); /* reading taken; centroids aren't ready to
                              classify against yet */
      sum.r += r.r;
      sum.g += r.g;
      sum.b += r.b;
    }
    centroid[c].r = sum.r / CALIB_SPOTS;
    centroid[c].g = sum.g / CALIB_SPOTS;
    centroid[c].b = sum.b / CALIB_SPOTS;
  }
  update_centroid_features();
  print_colour_table();

  /* 2. Accuracy test on fresh spots of every colour. Each spot takes
   *    AVG_SAMPLES single reads classified against all six colours - the way
   *    drive_along_street and scan_intersection classify - and the spot's
   *    majority vote stands in for one scan of that building. The building
   *    vote hit rate becomes colour_accuracy in the localization model. */
  int confusion[NUM_COLORS][NUM_COLORS] = {{0}}; /* single reads */
  int read_correct = 0, read_total = 0, vote_correct = 0, vote_total = 0;
  for (int c = 1; c < NUM_COLORS; c++)
    for (int s = 0; s < TEST_SPOTS; s++) {
      printf("Accuracy test: place sensor over a NEW %s spot (%d of %d), then "
             "press Enter\n",
             colour_names[c], s + 1, TEST_SPOTS);
      wait_for_enter();
      int votes[NUM_COLORS] = {0};
      for (int i = 0; i < AVG_SAMPLES; i++) {
        rgb_t v;
        if (!read_rgb(&v))
          continue;
        int got = classify_colour(v, ALL_COLOURS_MASK);
        votes[got]++;
        confusion[c][got]++;
        read_correct += got == c;
        read_total++;
      }
      int best = 1;
      for (int k = 2; k < NUM_COLORS; k++)
        if (votes[k] > votes[best])
          best = k;
      printf("  %d of %d reads were %s, majority %s\n", votes[c], AVG_SAMPLES,
             colour_names[c], colour_names[best]);
      play_colour_tone(best);
      if (BUILDING_MASK & COLOUR_BIT(c)) {
        vote_correct += best == c;
        vote_total++;
      }
    }

  printf("\nSingle-read confusion (rows = true, columns = classified):\n%-7s",
         "");
  for (int k = 1; k < NUM_COLORS; k++)
    printf(" %6s", colour_names[k]);
  printf(" %8s\n", "correct");
  for (int c = 1; c < NUM_COLORS; c++) {
    int row = 0;
    printf("%-7s", colour_names[c]);
    for (int k = 1; k < NUM_COLORS; k++) {
      printf(" %6d", confusion[c][k]);
      row += confusion[c][k];
    }
    printf(" %7.1f%%\n", row > 0 ? 100.0 * confusion[c][c] / row : 0.0);
  }
  printf("Single reads correct: %d/%d = %.3f\n", read_correct, read_total,
         read_total > 0 ? (double)read_correct / read_total : 0.0);

  colour_accuracy = clamp_accuracy((double)vote_correct / vote_total);
  printf("Building spot votes correct: %d/%d = %.3f, stored as accuracy %.3f\n",
         vote_correct, vote_total, (double)vote_correct / vote_total,
         colour_accuracy);

  /* 3. Save: one raw RGB centroid per line (colours 1..6), then accuracy. */
  FILE *f = fopen("calibration.txt", "w");
  if (f == NULL) {
    fprintf(stderr, "Unable to write calibration.txt\n");
    return;
  }
  for (int c = 1; c < NUM_COLORS; c++)
    fprintf(f, "%f %f %f\n", centroid[c].r, centroid[c].g, centroid[c].b);
  fprintf(f, "%f\n", colour_accuracy);
  fclose(f);
}

/* Reads calibration.txt: 6 lines of raw RGB centroids (colours 1..6), then an
 * optional accuracy line. Older 6-line files keep the default accuracy. */
void load_calibration(void) {
  FILE *f = fopen("calibration.txt", "r");
  if (f == NULL) {
    fprintf(stderr, "No calibration.txt found, using ideal map colours\n");
    update_centroid_features();
    return;
  }
  rgb_t loaded[NUM_COLORS];
  int ok = 1;
  for (int i = 1; i < NUM_COLORS && ok; i++)
    ok =
        fscanf(f, "%lf %lf %lf", &loaded[i].r, &loaded[i].g, &loaded[i].b) == 3;
  double accuracy;
  int has_accuracy = ok && fscanf(f, "%lf", &accuracy) == 1;
  fclose(f);
  if (!ok) {
    fprintf(stderr, "calibration.txt is incomplete, using ideal map colours\n");
    update_centroid_features();
    return;
  }
  for (int i = 1; i < NUM_COLORS; i++)
    centroid[i] = loaded[i];
  if (has_accuracy)
    colour_accuracy = clamp_accuracy(accuracy);
  update_centroid_features();
  if (VERBOSE) {
    print_colour_table();
    printf("Colour accuracy: %.3f%s\n", colour_accuracy,
           has_accuracy ? "" : " (default, not in calibration.txt)");
  }
}

int parse_map(unsigned char *map_img, int rx, int ry) {
  /*
    This function takes an input image map array, and two integers that specify
    the image size. It attempts to parse this image into a representation of the
    map in the image. The size and resolution of the map image should not affect
    the parsing (i.e. you can make your own maps without worrying about the
    exact position of intersections, roads, buildings, etc.).

    However, this function requires:

    * White background for the image  [255 255 255]
    * Red borders around the map  [255 0 0]
    * Black roads  [0 0 0]
    * Yellow intersections  [255 255 0]
    * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white
    [255 255 255] (any other colour values are ignored - so you can add markings
    if you like, those will not affect parsing)

    The image must be a properly formated .ppm image, see readPPMimage below for
    details of the format. The GIMP image editor saves properly formatted .ppm
    images, as does the imagemagick image processing suite.

    The map representation is read into the map array, with each row in the
    array corrsponding to one intersection, in raster order, that is, for a map
    with k intersections along its width:

     (row index for the intersection)

     0     1     2    3 ......   k-1

     k    k+1   k+2  ........

     Each row will then contain the colour values for buildings around the
    intersection clockwise from top-left, that is


     top-left               top-right

             intersection

     bottom-left           bottom-right

     So, for the first intersection (at row 0 in the map array)
     map[0][0] <---- colour for the top-left building
     map[0][1] <---- colour for the top-right building
     map[0][2] <---- colour for the bottom-right building
     map[0][3] <---- colour for the bottom-left building

     Color values for map locations are defined as follows (this agrees with
    what the EV3 sensor returns in indexed-colour-reading mode):

     1 -  Black
     2 -  Blue
     3 -  Green
     4 -  Yellow
     5 -  Red
     6 -  White

     If you find a 0, that means you're trying to access an intersection that is
    not on the map! Also note that in practice, because of how the map is
    defined, you should find only Green, Blue, or White around a given
    intersection.

     The map size (the number of intersections along the horizontal and vertical
    directions) is updated and left in the global variables sx and sy.

     Feel free to create your own maps for testing (you'll have to print them to
    a reasonable size to use with your bot).

  */

  int last3[3];
  int x, y;
  unsigned char R, G, B;
  int ix, iy;
  int bx, by, dx, dy, wx, wy; // Intersection geometry parameters
  int tgl;
  int idx;

  ix = iy = 0; // Index to identify the current intersection

  // Determine the spacing and size of intersections in the map
  tgl = 0;
  for (int i = 0; i < rx; i++) {
    for (int j = 0; j < ry; j++) {
      R = *(map_img + ((i + (j * rx)) * 3));
      G = *(map_img + ((i + (j * rx)) * 3) + 1);
      B = *(map_img + ((i + (j * rx)) * 3) + 2);
      if (R == 255 && G == 255 && B == 0) {
        // First intersection, top-left pixel. Scan right to find width and
        // spacing
        bx = i; // Anchor for intersection locations
        by = j;
        for (int k = i; k < rx;
             k++) // Find width and horizontal distance to next intersection
        {
          R = *(map_img + ((k + (by * rx)) * 3));
          G = *(map_img + ((k + (by * rx)) * 3) + 1);
          B = *(map_img + ((k + (by * rx)) * 3) + 2);
          if (tgl == 0 && (R != 255 || G != 255 || B != 0)) {
            tgl = 1;
            wx = k - i;
          }
          if (tgl == 1 && R == 255 && G == 255 && B == 0) {
            tgl = 2;
            dx = k - i;
          }
        }
        for (int k = j; k < ry;
             k++) // Find height and vertical distance to next intersection
        {
          R = *(map_img + ((bx + (k * rx)) * 3));
          G = *(map_img + ((bx + (k * rx)) * 3) + 1);
          B = *(map_img + ((bx + (k * rx)) * 3) + 2);
          if (tgl == 2 && (R != 255 || G != 255 || B != 0)) {
            tgl = 3;
            wy = k - j;
          }
          if (tgl == 3 && R == 255 && G == 255 && B == 0) {
            tgl = 4;
            dy = k - j;
          }
        }

        if (tgl != 4) {
          fprintf(stderr, "Unable to determine intersection geometry!\n");
          return (0);
        } else
          break;
      }
    }
    if (tgl == 4)
      break;
  }
  fprintf(stderr,
          "Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, "
          "horiz_distance=%d, vertical_distance=%d\n",
          bx, by, wx, wy, dx, dy);

  sx = 0;
  for (int i = bx + (wx / 2); i < rx; i += dx) {
    R = *(map_img + ((i + (by * rx)) * 3));
    G = *(map_img + ((i + (by * rx)) * 3) + 1);
    B = *(map_img + ((i + (by * rx)) * 3) + 2);
    if (R == 255 && G == 255 && B == 0)
      sx++;
  }

  sy = 0;
  for (int j = by + (wy / 2); j < ry; j += dy) {
    R = *(map_img + ((bx + (j * rx)) * 3));
    G = *(map_img + ((bx + (j * rx)) * 3) + 1);
    B = *(map_img + ((bx + (j * rx)) * 3) + 2);
    if (R == 255 && G == 255 && B == 0)
      sy++;
  }

  fprintf(stderr,
          "Map size: Number of horizontal intersections=%d, number of vertical "
          "intersections=%d\n",
          sx, sy);

  // Scan for building colours around each intersection
  idx = 0;
  for (int j = 0; j < sy; j++)
    for (int i = 0; i < sx; i++) {
      x = bx + (i * dx) + (wx / 2);
      y = by + (j * dy) + (wy / 2);

      fprintf(stderr, "Intersection location: %d, %d\n", x, y);
      // Top-left
      x -= wx;
      y -= wy;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][0] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][0] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][0] = 6;
      else
        fprintf(stderr,
                "Colour is not valid for intersection %d,%d, Top-Left "
                "RGB=%d,%d,%d\n",
                i, j, R, G, B);

      // Top-right
      x += 2 * wx;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][1] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][1] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][1] = 6;
      else
        fprintf(stderr,
                "Colour is not valid for intersection %d,%d, Top-Right "
                "RGB=%d,%d,%d\n",
                i, j, R, G, B);

      // Bottom-right
      y += 2 * wy;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][2] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][2] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][2] = 6;
      else
        fprintf(stderr,
                "Colour is not valid for intersection %d,%d, Bottom-Right "
                "RGB=%d,%d,%d\n",
                i, j, R, G, B);

      // Bottom-left
      x -= 2 * wx;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][3] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][3] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][3] = 6;
      else
        fprintf(stderr,
                "Colour is not valid for intersection %d,%d, Bottom-Left "
                "RGB=%d,%d,%d\n",
                i, j, R, G, B);

      fprintf(stderr, "Colours for this intersection: %d, %d, %d, %d\n",
              map[idx][0], map[idx][1], map[idx][2], map[idx][3]);

      idx++;
    }

  return (1);
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry) {
  // Reads an image from a .ppm file. A .ppm file is a very simple image
  // representation format with a text header followed by the binary RGB data at
  // 24bits per pixel. The header has the following form:
  //
  // P6
  // # One or more comment lines preceded by '#'
  // 340 200
  // 255
  //
  // The first line 'P6' is the .ppm format identifier, this is followed by one
  // or more lines with comments, typically used to inidicate which program
  // generated the .ppm file. After the comments, a line with two integer values
  // specifies the image resolution as number of pixels in x and number of
  // pixels in y. The final line of the header stores the maximum value for
  // pixels in the image, usually 255. After this last header line, binary data
  // stores the RGB values for each pixel in row-major order. Each pixel
  // requires 3 bytes ordered R, G, and B.
  //
  // NOTE: Windows file handling is rather crotchetty. You may have to change
  // the
  //       way this file is accessed if the images are being corrupted on read
  //       on Windows.
  //

  FILE *f;
  unsigned char *im;
  char line[1024];
  int i;
  unsigned char *tmp;
  double *fRGB;

  im = NULL;
  f = fopen(filename, "rb+");
  if (f == NULL) {
    fprintf(stderr,
            "Unable to open file %s for reading, please check name and path\n",
            filename);
    return (NULL);
  }
  fgets(&line[0], 1000, f);
  if (strcmp(&line[0], "P6\n") != 0) {
    fprintf(stderr, "Wrong file format, not a .ppm file or header end-of-line "
                    "characters missing\n");
    fclose(f);
    return (NULL);
  }
  fprintf(stderr, "%s\n", line);
  // Skip over comments
  fgets(&line[0], 511, f);
  while (line[0] == '#') {
    fprintf(stderr, "%s", line);
    fgets(&line[0], 511, f);
  }
  sscanf(&line[0], "%d %d\n", rx, ry); // Read image size
  fprintf(stderr, "nx=%d, ny=%d\n\n", *rx, *ry);

  fgets(&line[0], 9, f); // Read the remaining header line
  fprintf(stderr, "%s\n", line);
  im = (unsigned char *)calloc((*rx) * (*ry) * 3, sizeof(unsigned char));
  if (im == NULL) {
    fprintf(stderr, "Out of memory allocating space for image\n");
    fclose(f);
    return (NULL);
  }
  fread(im, (*rx) * (*ry) * 3 * sizeof(unsigned char), 1, f);
  fclose(f);

  return (im);
}
