/* ----------------------------------------------------------------------------
 * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "utils.hpp"
#include "exprtk.hpp"

int main()
{
  mt::PieceWisePol pwp;
  pwp.times.push_back(0.0);
  pwp.times.push_back(1.0);
  pwp.times.push_back(2.0);
  pwp.times.push_back(3.0);

  for (auto tmp : pwp.times)
  {
    std::cout << tmp << std::endl;
  }

  Eigen::Matrix<double, 4, 1> coeffs;
  coeffs << 1.0, 2.0, 3.0, 4.0;
  pwp.all_coeff_x.push_back(coeffs);
  pwp.all_coeff_y.push_back(coeffs);
  pwp.all_coeff_z.push_back(coeffs);
  coeffs << 5.0, 6.0, 7.0, 8.0;
  pwp.all_coeff_x.push_back(coeffs);
  pwp.all_coeff_y.push_back(coeffs);
  pwp.all_coeff_z.push_back(coeffs);
  coeffs << 9.0, 10.0, 11.0, 12.0;
  pwp.all_coeff_x.push_back(coeffs);
  pwp.all_coeff_y.push_back(coeffs);
  pwp.all_coeff_z.push_back(coeffs);

  std::vector<std::string> s = pieceWisePol2String(pwp);

  // std::cout << "The string is " << std::endl;
  // std::cout << s << std::endl;

  // Check if it leads the the good result
  typedef exprtk::symbol_table<double> symbol_table_t;
  typedef exprtk::expression<double> expression_t;
  typedef exprtk::parser<double> parser_t;

  double t;

  symbol_table_t symbol_table;
  symbol_table.add_variable("t", t);

  std::vector<expression_t> expressions(3);
  expressions[0].register_symbol_table(symbol_table);
  expressions[1].register_symbol_table(symbol_table);
  expressions[2].register_symbol_table(symbol_table);
  /*  expression_t expression_x;
    expression_x.register_symbol_table(symbol_table);
    expression_t expression_x;
    expression_x.register_symbol_table(symbol_table);
    expression_t expression_x;
    expression_x.register_symbol_table(symbol_table);*/

  parser_t parser;

  for (int i = 0; i < s.size(); i++)
  {
    parser.compile(s[i], expressions[i]);
  }

  /*  if (!parser.compile(s[0], expressions[0]) || !parser.compile(s, expressions[1]) || !parser.compile(s,
    expressions[2]))
    {
      printf("Compilation error...\n");
      return 0;
    }*/

  for (t = 0; t < 5; t = t + 0.2001)
  {
    std::cout << "*****t= " << t << std::endl;
    std::cout << "result with the string= " << expressions[0].value() << ", " << expressions[1].value() << ", "
              << expressions[2].value() << std::endl;
    std::cout << "result with the eval()= " << pwp.eval(t).transpose() << std::endl;
  }

  // Test composing to pwp

  mt::PieceWisePol pwpB;
  pwpB.times.push_back(2.5);
  pwpB.times.push_back(3.5);
  pwpB.times.push_back(4.5);
  pwpB.times.push_back(5.5);

  for (auto tmp : pwpB.times)
  {
    std::cout << tmp << std::endl;
  }

  coeffs << 13.0, 14.0, 15.0, 16.0;
  pwpB.all_coeff_x.push_back(coeffs);
  pwpB.all_coeff_y.push_back(coeffs);
  pwpB.all_coeff_z.push_back(coeffs);
  coeffs << 17.0, 18.0, 19.0, 20.0;
  pwpB.all_coeff_x.push_back(coeffs);
  pwpB.all_coeff_y.push_back(coeffs);
  pwpB.all_coeff_z.push_back(coeffs);
  coeffs << 21.0, 22.0, 23.0, 24.0;
  pwpB.all_coeff_x.push_back(coeffs);
  pwpB.all_coeff_y.push_back(coeffs);
  pwpB.all_coeff_z.push_back(coeffs);

  for (t = 0; t < 5; t = t + 0.4)
  {
    mt::PieceWisePol pwpAB = composePieceWisePol(t, -0.1, pwp, pwpB);
    std::cout << "==================" << std::endl;
    std::cout << "t= " << t << std::endl;
    std::cout << "==================" << std::endl;
    std::cout << "Polynomial A: " << std::endl;
    pwp.print();
    std::cout << "==================" << std::endl;
    std::cout << "Polynomial B: " << std::endl;
    pwpB.print();
    std::cout << "==================" << std::endl;
    std::cout << "Polynomial AB: " << std::endl;
    pwpAB.print();
  }

  std::cout << "Test linear transformation..." << std::endl;
  ////////////////// Test linear transformation
  Eigen::VectorXd coeff_p(5);
  Eigen::VectorXd coeff_q(5);
  double a = 7.0;
  double b = 3.0;
  coeff_p << 5.0, 1.0, 2.0, 3.0, 6.0;
  linearTransformPoly(coeff_p, coeff_q, a, b);

  std::cout << "coeff_p= " << coeff_p.transpose() << std::endl;
  std::cout << "a= " << a << ", b=" << b << std::endl;
  std::cout << "coeff of q(at +b)= " << coeff_q.transpose() << std::endl;

  // The previous result should be the same as this one in matlab:
  // coeff_p=[5.0, 1.0, 2.0, 3.0, 6.0]';
  // syms t real; T=(t.^[(numel(coeff_p)-1):-1:0])';
  // p=coeff_p'*T; a=7.0; b=3.0; q=subs(p,t,a*t+b);
  // coeffs(q,'All')

  std::cout << "Test scale/shift poly..." << std::endl;
  changeDomPoly(coeff_p, 3.0, 50.0, coeff_q, 1.0, 7.0);
  std::cout << "coeff_q= " << coeff_q.transpose() << std::endl;

  // The previous result should be the same as this one in matlab:

  // tp1=3.0; tp2=50; tq1=1.0; tq2=7.0;
  // syms a b
  // s=solve([tp1==a*tq1+b, tp2==a*tq2+b],[a,b])

  // coeff_p=[5.0, 1.0, 2.0, 3.0, 6.0]';
  // syms t real; T=(t.^[(numel(coeff_p)-1):-1:0])';
  // p=coeff_p'*T;
  // q=subs(p,t,s.a*t+s.b);

  // assert(subs(p, t,tp1) - subs(q, t,tq1) ==0)
  // assert(subs(p, t,tp2) - subs(q, t,tq2) ==0)

  // vpa(coeffs(q,'All'),6)
  {
    std::cout << "Test probMultivariateNormalDist..." << std::endl;

    Eigen::Vector3d a(0.1, 0.6, 0.3);
    Eigen::Vector3d b(0.3, 0.9, 2.3);
    Eigen::Vector3d mu(0.2, 0.3, 0.4);
    Eigen::Vector3d std_deviation(5.0, 3.0, 1.0);
    double result = probMultivariateNormalDist(a, b, mu, std_deviation);
    std::cout << "result= " << result << std::endl;

    // The previous result should be the same as this one in matlab:
    //     clc; clear;
    // mu=[0.2, 0.3, 0.4]'; std_deviation=[5.0, 3.0, 1.0];
    // Sigma=diag(std_deviation.^2);
    // a=[0.1, 0.6, 0.3]'; b=[0.3, 0.9, 2.3]';
    // mvncdf(b, mu, Sigma) - mvncdf(a, mu, Sigma)
  }

  return 0;
}