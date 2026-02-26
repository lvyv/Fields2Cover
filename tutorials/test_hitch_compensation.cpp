//=============================================================================
//    Test for Hitch Compensation in Path Planning
//=============================================================================

#include "fields2cover.h"
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string sanitizeDoubleForFileName(double value) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << value;
  std::string text = oss.str();
  for (auto& c : text) {
    if (c == '.') {
      c = '_';
    }else if (c == '-') {
      c = 'm';
    }
  }
  return text;
}

std::string normalizeMode(std::string mode) {
  std::transform(mode.begin(), mode.end(), mode.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return mode;
}

F2CSwaths sortSwathsByMode(const F2CSwaths& swaths, const std::string& mode_raw) {
  const std::string mode = normalizeMode(mode_raw);
  if (mode == "snake") {
    f2c::rp::SnakeOrder sorter;
    return sorter.genSortedSwaths(swaths);
  }
  if (mode == "boustrophedon" || mode == "ox" || mode == "niugeng") {
    f2c::rp::BoustrophedonOrder sorter;
    return sorter.genSortedSwaths(swaths);
  }
  if (mode == "spiral") {
    f2c::rp::SpiralOrder sorter;
    return sorter.genSortedSwaths(swaths);
  }

  throw std::invalid_argument(
      "Unsupported mode: " + mode_raw +
      ". Use one of: snake, boustrophedon, spiral");
}

void printPathStatesSummary(const F2CPath& path, const std::string& label) {
  std::cout << "\n--- " << label << " ---" << std::endl;
  std::cout << "Path length: " << path.length() << " m" << std::endl;

  std::cout << std::left
            << std::setw(6) << "Idx"
            << std::setw(14) << "Type(num)"
            << std::setw(16) << "Dir(num)"
            << std::setw(24) << "Point(x,y)"
            << std::setw(12) << "Angle(rad)"
            << std::setw(10) << "Len"
            << std::endl;

  const auto& states = path.getStates();
  const size_t max_rows = std::min<size_t>(states.size(), 40);

  for (size_t i = 0; i < max_rows; ++i) {
    const auto& s = states[i];
    std::ostringstream type_ss, dir_ss;
    type_ss << s.type << "(" << static_cast<int>(s.type) << ")";
    dir_ss << s.dir << "(" << static_cast<int>(s.dir) << ")";

    std::ostringstream point_ss;
    point_ss << "(" << std::fixed << std::setprecision(2)
             << s.point.getX() << ", " << s.point.getY() << ")";

    std::cout << std::left
              << std::setw(6) << i
              << std::setw(14) << type_ss.str()
              << std::setw(16) << dir_ss.str()
              << std::setw(24) << point_ss.str()
              << std::setw(12) << std::fixed << std::setprecision(3) << s.angle
              << std::setw(10) << std::fixed << std::setprecision(3) << s.len
              << std::endl;
  }

  if (states.size() > max_rows) {
    std::cout << "... (" << (states.size() - max_rows)
              << " more states omitted)" << std::endl;
  }
}

}// namespace

int main(int argc, char** argv) {
  // Usage:
  //   ./test_hitch_compensation <D1> <mode> <turn_model>
  // Example:
  //   ./test_hitch_compensation 5 boustrophedon rs
  double D1 = 5.0;
  if (argc > 1) {
    D1 = std::stod(argv[1]);
  }

  std::string mode = "snake";
  if (argc > 2) {
    mode = argv[2];
  }
  mode = normalizeMode(mode);

  std::string turn_model = "dubins";
  if (argc > 3) {
    turn_model = argv[3];
  }
  turn_model = normalizeMode(turn_model);

  if (turn_model != "dubins" && turn_model != "rs" && turn_model != "reeds_shepp") {
    throw std::invalid_argument(
        "Unsupported turn model: " + turn_model +
        ". Use one of: dubins, rs, reeds_shepp");
  }

  std::cout << "=== Test: Hitch Compensation with Full Field ===" << std::endl;
  std::cout << "Using D1 (hitch offset) = " << D1 << " m" << std::endl;
  std::cout << "Route mode = " << mode << std::endl;
  std::cout << "Turn model = " << turn_model << std::endl;
  std::cout << "PathState meaning (from PathState.h):" << std::endl;
  std::cout << "  PathSectionType: SWATH=1, TURN=2, HL_SWATH=3" << std::endl;
  std::cout << "  PathDirection:   FORWARD=1, BACKWARD=-1" << std::endl;

  // Build a complete test field
  F2CCells cells(F2CCell(F2CLinearRing({
      F2CPoint(0.0, 0.0),
      F2CPoint(60.0, 0.0),
      F2CPoint(60.0, 40.0),
      F2CPoint(0.0, 40.0),
      F2CPoint(0.0, 0.0)
  })));

  F2CRobot robot_base(2.0, 6.0);
  robot_base.setMinTurningRadius(2.0);
  robot_base.setMaxDiffCurv(0.1);
  robot_base.setCruiseVel(1.0);
  robot_base.setTurnVel(0.5);
  robot_base.setHitchOffset(0.0);

  F2CRobot robot_hitch = robot_base;
  robot_hitch.setHitchOffset(D1);

  // Standard flow used by the project tutorials
  f2c::hg::ConstHL const_hl;
  F2CCells no_hl = const_hl.generateHeadlands(cells, 3.0 * robot_hitch.getWidth());

  f2c::sg::BruteForce bf;
  F2CSwaths raw_swaths = bf.generateSwaths(M_PI / 2.0, robot_hitch.getCovWidth(), no_hl.getGeometry(0));
  F2CSwaths swaths = sortSwathsByMode(raw_swaths, mode);

  f2c::pp::PathPlanning planner;

  // Baseline (D1=0) and compensated path (D1=user input)
  F2CPath path_base;
  F2CPath path_hitch;
  if (turn_model == "dubins") {
    f2c::pp::DubinsCurves dubins;
    path_base = planner.planPath(robot_base, swaths, dubins);
    path_hitch = planner.planPath(robot_hitch, swaths, dubins);
  }else {
    f2c::pp::ReedsSheppCurves rs;
    path_base = planner.planPath(robot_base, swaths, rs);
    path_hitch = planner.planPath(robot_hitch, swaths, rs);
  }

  printPathStatesSummary(path_base,
      "Baseline path (D1 = 0.0, mode = " + mode + ", turn = " + turn_model + ")");
  printPathStatesSummary(path_hitch,
      "Compensated path (D1 = user input, mode = " + mode + ", turn = " + turn_model + ")");

  std::cout << "\nLength difference (compensated - baseline): "
            << (path_hitch.length() - path_base.length()) << " m" << std::endl;

  // Save images for direct visual comparison
  const std::string d1_tag = sanitizeDoubleForFileName(D1);

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(swaths);
  f2c::Visualizer::plot(path_base);
  f2c::Visualizer::save("test_hitch_compensation_" + mode + "_" + turn_model + "_baseline_D1_0_00.png");

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(swaths);
  f2c::Visualizer::plot(path_hitch);
  f2c::Visualizer::save("test_hitch_compensation_" + mode + "_D1_" + d1_tag + ".png");

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(swaths);
  f2c::Visualizer::plot(path_base);
  f2c::Visualizer::plot(path_hitch);
  f2c::Visualizer::save("test_hitch_compensation_" + mode + "_compare_D1_" + d1_tag + ".png");

  std::cout << "Saved images:" << std::endl;
  std::cout << "  - test_hitch_compensation_" << mode << "_baseline_D1_0_00.png" << std::endl;
  std::cout << "  - test_hitch_compensation_" << mode << "_D1_" << d1_tag << ".png" << std::endl;
  std::cout << "  - test_hitch_compensation_" << mode << "_compare_D1_" << d1_tag << ".png" << std::endl;

  std::cout << "\nDone. Try modes: snake, boustrophedon, spiral" << std::endl;
  return 0;
}
