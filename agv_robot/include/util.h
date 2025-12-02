#ifndef AGV_ROBOT_UTIL_H
#define AGV_ROBOT_UTIL_H
#include <mutex>

template <typename T>
class DataWithMutex {
 public:
  DataWithMutex(T b = static_cast<T>(0)) : data(b) {}
  DataWithMutex(const DataWithMutex &another_obj) {
    another_obj.mx.lock();
    mx.lock();
    data = another_obj.data;
    mx.unlock();
    another_obj.mx.unlock();
  }
  operator T() {
    mx.lock();
    T result = data;
    mx.unlock();
    return result;
  }
  DataWithMutex &operator=(const T &another_data) {
    mx.lock();
    data = another_data;
    mx.unlock();
    return *this;
  }
  DataWithMutex &operator=(const DataWithMutex &another_obj) {
    if (this != &another_obj) {
      another_obj.mx.lock();
      mx.lock();
      data = another_obj.data;
      mx.unlock();
      another_obj.mx.unlock();
    }
    return *this;
  }
  T operator++(int) {
    mx.lock();
    T result = data++;
    mx.unlock();
    return result;
  }
  T operator++() {
    mx.lock();
    T result = ++data;
    mx.unlock();
    return result;
  }
  T operator--(int) {
    mx.lock();
    T result = data--;
    mx.unlock();
    return result;
  }
  T operator--() {
    mx.lock();
    T result = --data;
    mx.unlock();
    return result;
  }
  //    T* operator&(){return &data;}//not safe
  DataWithMutex &operator+=(T another_data) {
    mx.lock();
    data += another_data;
    mx.unlock();
    return *this;
  }
  DataWithMutex &operator/=(T another_data) {
    mx.lock();
    data /= another_data;
    mx.unlock();
    return *this;
  }
  DataWithMutex &operator-=(T another_data) {
    mx.lock();
    data -= another_data;
    mx.unlock();
    return *this;
  }
  DataWithMutex &operator*=(T another_data) {
    mx.lock();
    data *= another_data;
    mx.unlock();
    return *this;
  }

 private:
  T data;
  mutable std::mutex mx;
};

// 数据类型转化 float  转化为 unsigned char
// src 要转化的数据
// des 目标数据
// width 输入数据的宽
// height 输入数据的高
int convertF1ToU1(float *src, unsigned char *des, int width, int height) {
  int i, j, temp;
  for (i = 0; i < height; i++) {
    for (j = 0; j < width; j++) {
      temp = (int)(src[i * width + j]);
      des[i * width + j] = (unsigned char)temp;
    }
  }
  return 0;
}
// 数据类型转化 unsigned char 转化为 float
int convertU1ToF1(unsigned char *src, float *des, int width, int height) {
  int i, j;
  for (i = 0; i < height; i++) {
    for (j = 0; j < width; j++) {
      des[i * width + j] = (float)src[i * width + j];
    }
  }
  return 0;
}
#endif  // AGV_ROBOT_UTIL_H