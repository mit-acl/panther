#include <iostream>
#include "Hungarian.h"
#include <chrono>

int main(void)
{
  // please use "-std=c++11" for this initialization of vector.
  vector<vector<double> > costMatrix = {
    { 10, 19, 8, 15, 0 }, { 10, 18, 7, 17, 0 }, { 13, 16, 9, 14, 0 }, { 12, 19, 8, 18, 0 }
  };

  HungarianAlgorithm HungAlgo;
  vector<int> assignment;

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

  double cost = HungAlgo.Solve(costMatrix, assignment);

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  std::cout << "Time difference = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.0 << "[ms]"
            << std::endl;

  for (unsigned int x = 0; x < costMatrix.size(); x++)
    std::cout << x << "," << assignment[x] << "\t";

  std::cout << "\ncost: " << cost << std::endl;

  return 0;
}
