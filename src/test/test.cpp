/*
 * test.cpp
 *
 *  Created on: Oct 5, 2017
 *      Author: Raphael
 */

#include <cmath>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <random>
#include <vector>
#include <atomic>
#include <functional> // For ref wrapper

#include "json.hpp"
#include "points.h"
#include "utils.h"
#include "trajectory.h"

using namespace std;

// Idea create a client thead that consumes waypoints
// and returns to the producer thread (server) a list of
// past waypoints as well as the current one.

// Need to check whether the list of past waypoints
// is really what is left to consume
// 
void consume_waypoints(VectorPoints next_vals, VectorPoints & next_out) {
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(1, 10);
  int dice_roll = distribution(generator); // generates number in the range 1..6

}

int main() {
  atomic_bool ready(false);
  int max_loops = 5;
  VectorPoints next_vals;
  std::thread logging_thread(logWaypoints, std::ref(ready), max_loops,
      std::cref(next_vals));
  return 0;
}
