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

  // ///////////////////////// EXAMPLE OF HOW TO CALL A FUNCTION
  // // Inspired from line 77 of
  // http://docs.ros.org/en/melodic/api/pinocchio/html/casadi-basic_8cpp_source.html
  // auto x = SX::sym("x", 1, 1);
  // auto y = SX::sym("y", 1, 1);
  // casadi::Function fun("fun", std::vector<SX>{x, y},
  //                      std::vector<SX>{x + y, x - y});
  // std::cout << "fun = " << fun << std::endl;
  // auto res = fun(std::vector<SX>{1, 2});
  // std::cout << "fun(x, y)=" << res << std::endl;
  // /////////////////////////

  return 0;
}

/////// OLD STUFF
// auto opti = casadi::Opti();

// auto x = opti.variable();
// auto y = opti.variable();
// auto z = opti.variable();

// opti.minimize(x * x + 100 * z * z);
// opti.subject_to(z + (1 - x) * (1 - x) - y == 0);

// Dict opts = {{"ipopt.linear_solver", "mumps"},
//              {"ipopt.tol", 1e-10}};  //{"ipopt.linear_solver", "ma27"},
//              mumps

// opti.solver("ipopt", opts);
// auto sol = opti.solve();

// std::cout << sol.value(x) << ":" << sol.value(y) << std::endl;

// Function solver = nlpsol("solver", "ipopt", "./../example.so");

// // Bounds and initial guess
// std::map<std::string, DM> arg, res;
// arg["lbx"] = -DM::inf();
// arg["ubx"] = DM::inf();
// // arg["lbg"] = -DM::inf();
// // arg["ubg"] = DM::inf();
// // arg["x0"] = 0;

// // Solve the NLP
// res = solver(arg);  // solver(arg);

// // Print solution
// cout << "-----" << endl;
// cout << "objective at solution = " << res.at("f") << endl;
// cout << "primal solution = " << res.at("x") << endl;
// cout << "dual solution (x) = " << res.at("lam_x") << endl;
// cout << "dual solution (g) = " << res.at("lam_g") << endl;

// return 0;

// Optimization variables
// MX x = MX::sym("x", 2);

// // Objective
// MX f = x(0) * x(0) + x(1) * x(1);

// // Constraints
// MX g = x(0) + x(1) - 10;

// // Create an NLP solver instance
// Function solver = nlpsol("solver", "ipopt", {{"x", x}, {"f", f}, {"g",
// g}});

// // Generate C code for the NLP functions
// solver.generate_dependencies("nlp2.c");

// // Just-in-time compilation?
// // bool jit = false;
// // if (jit) {
// //   // Create a new NLP solver instance using just-in-time compilation
// //   solver = nlpsol("solver", "ipopt", "nlp.c");
// // } else {
// // Compile the c-code
// std::cout << "Compiling the code!" << std::endl;
// int flag = system("gcc -fPIC -shared -O0 nlp.c -o nlp2.so");
// std::cout << "Flag= " << flag << std::endl;
// casadi_assert(flag == 0, "Compilation failed");
/////////////////////////////////////
// // Create a new NLP solver instance from the compiled code
// Function solver = nlpsol("solver", "ipopt", "./../minimal_example.so");
// // }

// // Bounds and initial guess
// std::map<std::string, DM> arg, res;
// arg["lbx"] = -DM::inf();
// arg["ubx"] = DM::inf();
// arg["lbg"] = -DM::inf();
// arg["ubg"] = DM::inf();
// arg["x0"] = 4.5;

// // Solve the NLP
// res = solver(arg);

// // Print solution
// cout << "-----" << endl;
// cout << "objective at solution = " << res.at("f") << endl;
// cout << "primal solution = " << res.at("x") << endl;
// cout << "dual solution (x) = " << res.at("lam_x") << endl;
// cout << "dual solution (g) = " << res.at("lam_g") << endl;

// std::string novale = "./../my_function.casadi";

///////////////////OLD STUFF

// Function f = Function("f", {x, y}, {x + y});

// std::map<std::string, DM> arg, res;

// // Solve NLP
// arg["lbx"] = nl.x_lb;
// arg["ubx"] = nl.x_ub;
// arg["lbg"] = nl.g_lb;
// arg["ubg"] = nl.g_ub;
// arg["x0"] = nl.x_init;
// res = solver(arg);
// res = solver(arg);
// for (auto&& s : res) {
//   std::cout << s.first << ": " << std::vector<double>(s.second) <<
//   std::endl;
// }

// auto z = f(std::vector<MX>{3, 4});
// std::cout << z << std::endl;

// Convert to sxfunction to decrease overhead
// Function vfcn_sx = tmp.expand("tmp");