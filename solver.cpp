// Taken (and modified) from https://github.com/casadi/casadi/blob/752695adf24e0d4c904cc0bbeeedbcb73025559d/docs/examples/cplusplus/cmake_pkgconfig/casadi_demo.cpp

// Run first the casadi_demo.m file to generate the file my_function.casadi

#include <casadi/casadi.hpp>
// #include <ctime>
// #include <fstream>
#include <iomanip>
#include <iostream>

using namespace casadi;
using namespace std;

int main() {
  // Definitions of DM, DMVector,... are in dm_fwd.hpp

  // ///////////////////////// EXAMPLE OF HOW TO CALL a .casadi file
  Function tmp = Function::load("./my_function.casadi");
  std::vector<DM> result =
      tmp(std::vector<DM>{3, std::vector<double>{0.1, 0.2, 0}});
  std::cout << result << std::endl;
  //////////////////////////

  return 0;
}