/*
 * utils.h
 *
 *  Created on: 30 Sep 2017
 *      Author: rdjondo
 */

#ifndef UTILS_H_
#define UTILS_H_


/**
 * Vector point double buffer
 * */

template<class T>
class DoubleBuffer {
private:
  std::mutex idxMutex;
  T buf[2];
  T empty;
  int busyIdx; /* Locked buffer reading */
  int lastWriteIdx; /* Last updated buffer */
public:
  DoubleBuffer();
  void update(const T & a);
  const T get();
};

template<class T>
DoubleBuffer<T>::DoubleBuffer() :
    busyIdx(-1), lastWriteIdx(-1) {
  empty.clear();
}

template<class T>
void DoubleBuffer<T>::update(const T & a) {
  idxMutex.lock();
  int writeIdx = !lastWriteIdx;
  if (busyIdx!=-1) {
    writeIdx = !busyIdx;
  } else{
    busyIdx = writeIdx;
  }
  idxMutex.unlock();

  /* Write buffer */
  buf[writeIdx] = a;

  idxMutex.lock();
  if (busyIdx==writeIdx) {
    busyIdx=-1;
  }
  lastWriteIdx = writeIdx; /* New data */
  idxMutex.unlock();

}

template<class T>
const T DoubleBuffer<T>::get() {
  idxMutex.lock();
  int readIdx = lastWriteIdx;
  if (busyIdx!=-1) {
    readIdx = !busyIdx;
  }else{
    busyIdx = readIdx;
  }
  idxMutex.unlock();

  /** If no write since last read return empty vector */
  if(lastWriteIdx==-1){
    return empty;
  }

  /* Write a copy buffer out */
  T buf_out(buf[readIdx]);

  idxMutex.lock();
  if (busyIdx==readIdx) {
    busyIdx=-1;
  }
  lastWriteIdx = -1;
  idxMutex.unlock();

  return buf_out;
}


extern void logWaypoints(int max_loops, const VectorPoints & next_vals, DoubleBuffer<VectorPoints> & log);

extern std::string hasData(std::string s);


extern void loadMap(VectorPoints &map_waypoints, std::vector<double> &map_waypoints_s,
    std::vector<double> &map_waypoints_dx,
    std::vector<double> &map_waypoints_dy);

#endif /* UTILS_H_ */
