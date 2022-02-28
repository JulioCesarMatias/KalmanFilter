#include <Arduino.h>

/*
 * This file is part of EmuFlight.
 *
 * EmuFlight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * EmuFlight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#define LOOP_RATE 1000 // 1Khz Loop

#define MAX_KALMAN_WINDOW_SIZE 65

#define VARIANCE_SCALE 1.0f

uint8_t imuf_w = 32;
uint16_t imuf_q = 10000;

typedef struct pt1Filter_s
{
  float state;
  float k;
} pt1Filter_t;

typedef struct kalman_s
{
  float q;     //process noise covariance
  float r;     //measurement noise covariance
  float p;     //estimation error covariance matrix
  float k;     //kalman gain
  float x;     //state
  float lastX; //previous state
  float e;
  float Var;
  uint16_t windex;
  float Window[MAX_KALMAN_WINDOW_SIZE];
  float varianceWindow[MAX_KALMAN_WINDOW_SIZE];
  float SumMean;
  float Mean;
  float SumVar;
  float inverseN;
  uint16_t w;

  pt1Filter_t kFilter;
} kalman_t;

kalman_t kalmanFilterState;

float pt1FilterGain(uint16_t f_cut, float dT)
{
  const float RC = 0.5f / (3.14159265358979323846f * f_cut);
  return dT / (RC + dT);
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
  filter->state = filter->state + filter->k * (input - filter->state);
  return filter->state;
}

void pt1FilterInit(pt1Filter_t *filter, float k)
{
  filter->state = 0.0f;
  filter->k = k;
}

static void init_kalman(kalman_t *filter, float q)
{
  memset(filter, 0, sizeof(kalman_t));
  filter->q = q * 0.0001f; //add multiplier to make tuning easier
  filter->r = 88.0f;       //seeding R at 88.0f
  filter->p = 30.0f;       //seeding P at 30.0f
  filter->e = 1.0f;
  filter->w = imuf_w;
  filter->inverseN = 1.0f / (float)(filter->w);

  pt1FilterInit(&filter->kFilter, pt1FilterGain(50, LOOP_RATE * 1e-6f));
}

void update_kalman_covariance(kalman_t *kalmanState, float rate)
{
  if (kalmanState->w < 3)
  {
    return;
  }

  kalmanState->Window[kalmanState->windex] = rate;
  kalmanState->SumMean += kalmanState->Window[kalmanState->windex];

  float varianceElement = kalmanState->Window[kalmanState->windex] - kalmanState->Mean;
  varianceElement = varianceElement * varianceElement;
  kalmanState->SumVar += varianceElement;
  kalmanState->varianceWindow[kalmanState->windex] = varianceElement;
  kalmanState->windex++;

  if (kalmanState->windex > kalmanState->w)
  {
    kalmanState->windex = 0;
  }

  kalmanState->SumMean -= kalmanState->Window[kalmanState->windex];
  kalmanState->SumVar -= kalmanState->varianceWindow[kalmanState->windex];

  //New mean
  kalmanState->Mean = kalmanState->SumMean * kalmanState->inverseN;
  kalmanState->Var = kalmanState->SumVar * kalmanState->inverseN;

  kalmanState->r = sqrtf(kalmanState->Var) * VARIANCE_SCALE;
}

float kalman_process(kalman_t *kalmanState, float input)
{
  if (kalmanState->w < 3)
  {
    return input;
  }

  //project the state ahead using acceleration
  kalmanState->x += (kalmanState->x - kalmanState->lastX) * kalmanState->k;

  kalmanState->lastX = kalmanState->x;

  float e = constrain(kalmanState->r / 45.0f + 0.005f, 0.005f, 0.9f);
  //make the 1 a configurable value for testing purposes
  e = -powf(e - 1.0f, 2) * 0.7f + (e - 1.0f) * (1.0f - 0.7f) + 1.0f;
  kalmanState->e = e;

  //prediction update
  kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);

  //measurement update
  kalmanState->k = kalmanState->p / (kalmanState->p + 10.0f);
  kalmanState->k = pt1FilterApply(&kalmanState->kFilter, kalmanState->k);
  kalmanState->x += kalmanState->k * (input - kalmanState->x);
  kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;

  return kalmanState->x;
}

void setup()
{
  Serial.begin(115200);
  init_kalman(&kalmanFilterState, imuf_q);
}

void loop()
{
  int16_t analog_read = analogRead(0);
  Serial.print(analog_read);
  Serial.print(" ");
  Serial.println((int16_t)kalman_process(&kalmanFilterState, analog_read));

  update_kalman_covariance(&kalmanFilterState, analog_read);

  delay(1); // 1Khz loop
}