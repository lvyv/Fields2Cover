//=============================================================================
//    Test for Asymmetric Turning Radius Reeds-Shepp Curves
//=============================================================================

#include "fields2cover.h"
#include <iostream>

int main() {
  std::cout << "=== Test: Asymmetric Turning Radius Reeds-Shepp Curves ===" << std::endl;

  // Create a random field
  f2c::Random rand(42);
  F2CCells cells = rand.generateRandField(1e4, 5).getField();

  // Create robot with asymmetric turning radii
  F2CRobot robot(2.0, 2.0);  // width=2.0m, coverage_width=6.0m

  // Set asymmetric turning radii
  robot.setMinTurningRadiusLeft(10.0);   // Left turn: 2m radius (tighter)
  robot.setMinTurningRadiusRight(1.0);  // Right turn: 4m radius (wider)
  // robot.setMaxDiffCurv(0.1);

  std::cout << "Robot configuration:" << std::endl;
  std::cout << "  Width: " << robot.getWidth() << " m" << std::endl;
  std::cout << "  Coverage Width: " << robot.getCovWidth() << " m" << std::endl;
  std::cout << "  Left Turn Radius: " << robot.getMinTurningRadiusLeft() << " m" << std::endl;
  std::cout << "  Right Turn Radius: " << robot.getMinTurningRadiusRight() << " m" << std::endl;
  std::cout << "  Left Curvature: " << robot.getMaxCurvLeft() << " 1/m" << std::endl;
  std::cout << "  Right Curvature: " << robot.getMaxCurvRight() << " 1/m" << std::endl;

  // Generate headlands
  f2c::hg::ConstHL const_hl;
  F2CCells no_hl = const_hl.generateHeadlands(cells, 3.0 * robot.getWidth());

  // Generate swaths
  f2c::sg::BruteForce bf;
  F2CSwaths swaths = bf.generateSwaths(M_PI, robot.getCovWidth(), no_hl.getGeometry(0));

  // Sort swaths
  f2c::rp::SnakeOrder snake_sorter;
  swaths = snake_sorter.genSortedSwaths(swaths);

  f2c::pp::PathPlanning path_planner;

  // Test 1: Standard symmetric Reeds-Shepp (using left radius for both)
  std::cout << "\n=== Test 1: Standard Reeds-Shepp (symmetric, radius=2m) ===" << std::endl;
  F2CRobot robot_sym(2.0, 6.0);
  robot_sym.setMinTurningRadius(2.0);
  robot_sym.setMaxDiffCurv(1);



  f2c::pp::ReedsSheppCurves reeds_shepp_sym;
  F2CPath path_sym = path_planner.planPath(robot_sym, swaths, reeds_shepp_sym);
  std::cout << "  Path length: " << path_sym.length() << " m" << std::endl;

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(path_sym);
  f2c::Visualizer::plot(swaths);
  f2c::Visualizer::save("test_RS_symmetric.png");
  std::cout << "  Saved: test_RS_symmetric.png" << std::endl;

  // Test 2: Asymmetric Reeds-Shepp
  /*
  std::cout << "\n=== Test 2: Asymmetric Reeds-Shepp (left=2m, right=4m) ===" << std::endl;
  f2c::pp::ReedsSheppCurvesAsym reeds_shepp_asym;
  F2CPath path_asym = path_planner.planPath(robot, swaths, reeds_shepp_asym);
  std::cout << "  Path length: " << path_asym.length() << " m" << std::endl;
  */
  
  f2c::pp::DubinsCurves dubins_asym;
  F2CPath path_asym = path_planner.planPath(robot, swaths, dubins_asym);

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(path_asym);
  f2c::Visualizer::plot(swaths);
  f2c::Visualizer::save("test_RS_asymmetric.png");
  std::cout << "  Saved: test_RS_asymmetric.png" << std::endl;

  // Test 3: Simple turn comparison
  std::cout << "\n=== Test 3: Simple Turn Comparison ===" << std::endl;

  // Create a simple scenario with two parallel swaths
  F2CLineString line1, line2;
  line1.addPoint(0, 0);
  line1.addPoint(20, 0);
  line2.addPoint(0, 10);
  line2.addPoint(20, 10);

  F2CSwath swath1(line1);
  F2CSwath swath2(line2);
  F2CSwaths simple_swaths;
  simple_swaths.push_back(swath1);
  simple_swaths.push_back(swath2);

  // Symmetric turn
  F2CPath simple_path_sym = path_planner.planPath(robot_sym, simple_swaths, reeds_shepp_sym);
  std::cout << "  Symmetric path length: " << simple_path_sym.length() << " m" << std::endl;

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(simple_path_sym);
  f2c::Visualizer::plot(simple_swaths);
  f2c::Visualizer::save("test_simple_turn_symmetric.png");
  std::cout << "  Saved: test_simple_turn_symmetric.png" << std::endl;

  // Asymmetric turn
  // F2CPath simple_path_asym = path_planner.planPath(robot, simple_swaths, reeds_shepp_asym);
  F2CPath simple_path_asym = path_planner.planPath(robot, simple_swaths, dubins_asym);
  std::cout << "  Asymmetric path length: " << simple_path_asym.length() << " m" << std::endl;

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(simple_path_asym);
  f2c::Visualizer::plot(simple_swaths);
  f2c::Visualizer::save("test_simple_turn_asymmetric.png");
  std::cout << "  Saved: test_simple_turn_asymmetric.png" << std::endl;

  std::cout << "\n=== All tests completed! ===" << std::endl;

  return 0;
}
