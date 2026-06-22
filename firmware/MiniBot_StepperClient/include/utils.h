#ifndef __UTILS_H__
#define __UTILS_H__

#include <Arduino.h>
#include <math.h>

template <size_t N> class RollingAverage {
public:
  RollingAverage() : index(0), count(0), sum(0.0f) {
    for (size_t i = 0; i < N; ++i)
      values[i] = 0.0f;
  }

  void add(float val) {
    if (count < N) {
      sum += val;
      values[index] = val;
      ++count;
    } else {
      sum -= values[index];
      sum += val;
      values[index] = val;
    }
    index = (index + 1) % N;
  }

  float avg() const { return (count > 0) ? (sum / count) : 0.0f; }

  void reset() {
    index = 0;
    count = 0;
    sum = 0.0f;
    for (size_t i = 0; i < N; ++i)
      values[i] = 0.0f;
  }

private:
  float values[N];
  size_t index;
  size_t count;
  float sum;
};

#endif // __UTILS_H__
