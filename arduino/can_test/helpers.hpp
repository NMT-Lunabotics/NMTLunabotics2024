#ifndef HELPERS_H
#define HELPERS_H

#include "arduino_lib.hpp"

class PID {
private:
  float error;
  float prev_error;
  float derivative;
  float integral;

  float p, i, d;

public:
  PID(float p, float i, float d) : p(p), i(i), d(d) {
    error = 0;
    prev_error = 0;
    derivative = 0;
    integral = 0;
  }

  float update(float error) {
    derivative = error - prev_error;
    integral += error;
    prev_error = error;
    return p * error + i * integral + d * derivative;
  }

  void resetIntegral() { integral = 0; }
};

class Median {
private:
  int history_size;
  int *history;
  int current_idx;

public:
  Median(int history_size) : history_size(history_size) {
    history = new int[history_size];
    current_idx = 0;
    memset(history, 0, sizeof(history));
  }

  ~Median() { delete[] history; }

  int update(int new_val) {
    history[current_idx] = new_val;
    current_idx++;
    current_idx %= history_size;

    int sorted[history_size];
    memcpy(sorted, history, history_size * sizeof(history[0]));

    qsort(sorted, history_size, sizeof(sorted[0]),
          [](const void *a, const void *b) {
            if (*(int *)a > *(int *)b)
              return 1;
            else if (*(int *)a < *(int *)b)
              return -1;
            return 0;
          });

    if (history_size % 2 == 1)
      return (sorted[history_size / 2] + sorted[history_size / 2 + 1]) / 2;
    else
      return sorted[history_size / 2];
  }
};

#endif
