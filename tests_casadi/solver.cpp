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
//   std::vector<DM> result =
//       tmp(std::vector<DM>{3, std::vector<double>{0.1, 0.2, 0}});

  
  //Conversion DM <--> Eigen:  https://github.com/casadi/casadi/issues/2563
  
  
  std::map<std::string, DM> map_arguments;
  map_arguments["theta_FOV_deg"]=80.0;
  map_arguments["p0"]=std::vector<double>{-4.0, 0.0, 0};
  map_arguments["v0"]=std::vector<double>{0.0, 0.0, 0};
  map_arguments["a0"]=std::vector<double>{0.0, 0.0, 0};
  map_arguments["pf"]=std::vector<double>{4.0, 0.0, 0};
  map_arguments["vf"]=std::vector<double>{0.0, 0.0, 0};
  map_arguments["af"]=std::vector<double>{0.0, 0.0, 0};
  map_arguments["y0"]=0.0;
  map_arguments["ydot0"]=0.0;
  map_arguments["all_w_fe"]=DM::ones(3,15);
  map_arguments["c_jerk"]=0.0;
  map_arguments["c_yaw"]=0.0;
  map_arguments["c_vel_isInFOV"]=1.0;
  map_arguments["all_nd"]=DM::rand(4,0);
  
  std::map<std::string, DM> result =tmp(map_arguments);
  
  std::cout << result << std::endl;
  //////////////////////////

  return 0;
}