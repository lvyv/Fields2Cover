//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#include <steering_functions/utilities/utilities.hpp>
#include "fields2cover/path_planning/path_planning.h"

namespace f2c::pp {

F2CPath PathPlanning::planPath(const F2CRobot& robot,
    const F2CRoute& route, TurningBase& turn, double discretization_step) {
  F2CPath path;
  for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
    auto prev_swaths = (i >0) ? route.getSwaths(i-1) : F2CSwaths();
    path += planPathForConnection(robot,
        prev_swaths, route.getConnection(i), route.getSwaths(i), turn);
    path += planPath(robot, route.getSwaths(i), turn, discretization_step);
  }
  if (route.sizeConnections() > route.sizeVectorSwaths()) {
    path += planPathForConnection(robot,
      route.getLastSwaths(), route.getLastConnection(), F2CSwaths(), turn);
  }
  return path;
}

F2CPath PathPlanning::planPath(const F2CRobot& robot,
    const F2CSwaths& swaths, TurningBase& turn, double discretization_step) {
  // D1: 农具(犁具)相对机体中心的后向距离。
  // - 用户外部传入：robot.setHitchOffset(D1)
  // - 用户不传(默认 0)：保持原始行为，不做补偿
  const double D1 = robot.getHitchOffset();
  // D2/D3: 转弯前“前进回正 + 倒退回到转弯点”的距离。
  // - D2：右转时使用（顺时针）
  // - D3：左转时使用（逆时针）
  // - 用户外部传入：
  //     robot.setHitchStraightenDistRight(D2)
  //     robot.setHitchStraightenDistLeft(D3)
  // - 用户不传(默认 0)：保持原始行为，不做补偿
  const double D2 = robot.getHitchStraightenDistRight();
  const double D3 = robot.getHitchStraightenDistLeft();

  F2CPath path;
  if (swaths.size() > 1) {
    for (size_t i = 0; i < swaths.size()-1; ++i) {
      // 1) 当前耕作直线路径(swath)
      path.appendSwath(swaths[i], robot.getCruiseVel());

      // 2) 地头补偿：前进 D1 继续作业，抬犁倒退 D1 回到地头点
      // 目的：避免“机体中心到达地头就转弯”导致犁具末端 D1 漏耕。
      if (D1 > 1e-6) {
        const double ang_out = swaths[i].getOutAngle();
        const F2CPoint p_end = swaths[i].endPoint();
        const F2CPoint p_ext(
          p_end.getX() + D1 * cos(ang_out),
          p_end.getY() + D1 * sin(ang_out));

        // (2.1) 前进 D1：仍然是作业段(SWATH)
        path.addState(p_end, ang_out, D1,
          f2c::types::PathDirection::FORWARD,
          f2c::types::PathSectionType::SWATH,
          robot.getCruiseVel());

        // (2.2) 抬犁倒退 D1：非作业段，用 TURN 类型标记
        path.addState(p_ext, ang_out, D1,
          f2c::types::PathDirection::BACKWARD,
          f2c::types::PathSectionType::TURN,
          robot.getTurnVel());
      }

      // 3) 转弯段：从 swath[i] 末端连接到下一条 swath 的起点
      // 注意：上面补偿结束后机体回到了原 swath.endPoint()，因此这里仍然用 endPoint()。
      F2CPath turn_path = turn.createTurn(robot,
          swaths[i].endPoint(), swaths[i].getOutAngle(),
          swaths[i + 1].startPoint(), swaths[i + 1].getInAngle());
      turn_path.discretize(discretization_step);

      // 2.5) 转弯到 H 点后再做 D2/D3 前进+回退：
      // H 点近似为转弯轨迹上偏离首尾连线最远的点（圆弧最高点），
      // 不需要用户传入 H，只根据 turn_path 自动计算。
      {
        const double ang_out = swaths[i].getOutAngle();
        const double ang_in_next = swaths[i + 1].getInAngle();
        const double delta = atan2(sin(ang_in_next - ang_out), cos(ang_in_next - ang_out));
        const bool is_right_turn = (delta < 0.0);
        const double D = is_right_turn ? D2 : D3;

        if (D > 1e-6 && turn_path.size() > 0) {
          const F2CPoint p_start = swaths[i].endPoint();
          const F2CPoint p_end_next = swaths[i + 1].startPoint();

          // 找到离首尾连线最远的点，视为 H。
          size_t idx_h = 0;
          double max_dist = -1.0;
          for (size_t k = 0; k < turn_path.size(); ++k) {
            double d = fabs(turn_path[k].point.signedDistance2Segment(p_start, p_end_next));
            if (d > max_dist) {
              max_dist = d;
              idx_h = k;
            }
          }

          const F2CPoint H = turn_path[idx_h].point;
          const double ang_H = turn_path[idx_h].angle;
          // 在 H 点沿转弯轨迹切线方向前进 D，再回退 D（与圆相切）
          const double ang_tangent = ang_H;
          const F2CPoint p_fwd(
            H.getX() + D * cos(ang_tangent),
            H.getY() + D * sin(ang_tangent));

          F2CPath new_turn;
          // 先保留转弯到 H 的轨迹
          for (size_t k = 0; k <= idx_h; ++k) {
            new_turn.addState(turn_path[k]);
          }
          // 在 H 点前进 D（抬犁）
          new_turn.addState(H, ang_tangent, D,
            f2c::types::PathDirection::FORWARD,
            f2c::types::PathSectionType::TURN,
            robot.getTurnVel());
          // 再从 I 点回退 D 回到 H 点
          new_turn.addState(p_fwd, ang_tangent, D,
            f2c::types::PathDirection::BACKWARD,
            f2c::types::PathSectionType::TURN,
            robot.getTurnVel());
          // 继续从 H 之后的原始转弯轨迹
          for (size_t k = idx_h + 1; k < turn_path.size(); ++k) {
            new_turn.addState(turn_path[k]);
          }

          turn_path = std::move(new_turn);
        }
      }

      // 转弯过程中抬犁：转弯段统一标记为 TURN
      turn_path.setTurnType();
      path += turn_path;
    }
  }

  if (swaths.size() > 0) {
    // 最后一条 swath：同样追加补偿，否则最后一条末端仍可能漏耕
    path.appendSwath(swaths.back(), robot.getCruiseVel());

    if (D1 > 1e-6) {
      const double ang_out = swaths.back().getOutAngle();
      const F2CPoint p_end = swaths.back().endPoint();
      const F2CPoint p_ext(
        p_end.getX() + D1 * cos(ang_out),
        p_end.getY() + D1 * sin(ang_out));

      path.addState(p_end, ang_out, D1,
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH,
        robot.getCruiseVel());

      path.addState(p_ext, ang_out, D1,
        f2c::types::PathDirection::BACKWARD,
        f2c::types::PathSectionType::TURN,
        robot.getTurnVel());
    }
  }

  return path;
}

F2CPath PathPlanning::planPathForConnection(const F2CRobot& robot,
    const F2CSwaths& s1,
    const F2CMultiPoint& mp,
    const F2CSwaths& s2,
    TurningBase& turn) {
  F2CPoint p1, p2;
  double ang1, ang2;

  if (s1.size() > 0) {
    p1 = s1.back().endPoint();
    ang1 = s1.back().getOutAngle();
  } else if (mp.size() > 0) {
    p1 = mp[0];
    ang1 = mp.getOutAngle(0);
  } else {
    return {};
  }
  if (s2.size() > 0) {
    p2 = s2[0].startPoint();
    ang2 = s2[0].getInAngle();
  } else if (mp.size() > 0) {
    p2 = mp.getLastPoint();
    ang2 = mp.getInAngle(mp.size()-1);
  } else {
    return {};
  }
  return planPathForConnection(robot, p1, ang1, mp, p2, ang2, turn);
}

F2CPath PathPlanning::planPathForConnection(const F2CRobot& robot,
    const F2CPoint& p1, double ang1,
    const F2CMultiPoint& mp,
    const F2CPoint& p2, double ang2,
    TurningBase& turn) {
  auto v_con = simplifyConnection(robot,
      p1, ang1, mp, p2, ang2);

  F2CPath path;
  for (int i = 1; i < v_con.size(); ++i) {
    auto t = turn.createTurn(robot,
        v_con[i-1].first, v_con[i-1].second,
        v_con[i].first, v_con[i].second);
    // 连接段同样属于转弯/机动：抬犁
    t.setTurnType();
    path += t;
  }
  return path;
}



double PathPlanning::getSmoothTurningRadius(const F2CRobot& robot) {
  double x, y, ang, k;
  end_of_clothoid(0.0, 0.0, 0.0, 0.0, robot.getMaxDiffCurv(), 1.0,
      robot.getMaxCurv() / robot.getMaxDiffCurv(),
      &x, &y, &ang, &k);
  double xi = x - sin(ang) / robot.getMaxCurv();
  double yi = y + cos(ang) / robot.getMaxCurv();
  return sqrt(xi*xi + yi*yi);
}

std::vector<std::pair<F2CPoint, double>> PathPlanning::simplifyConnection(
    const F2CRobot& robot,
    const F2CPoint& p1, double ang1,
    const F2CMultiPoint& mp,
    const F2CPoint& p2, double ang2) {
  const double safe_dist = getSmoothTurningRadius(robot);
  std::vector<std::pair<F2CPoint, double>> path;
  path.emplace_back(p1, ang1);

  if (p1.distance(p2) < 6.0 * safe_dist || mp.size() < 2) {
    path.emplace_back(p2, ang2);
    return path;
  }

  std::vector<F2CPoint> vp;
  for (int i = 1; i < mp.size() - 1; ++i) {
    double ang_in  = (mp[i] - mp[i-1]).getAngleFromPoint();
    double ang_out = (mp[i+1] - mp[i]).getAngleFromPoint();
    if (fabs(ang_in - ang_out) > 0.1) {
      vp.emplace_back(mp[i]);
    }
  }

  if (vp.size() < 2) {
    path.emplace_back(p2, ang2);
    return path;
  }

  for (int i = 1; i < vp.size() - 1; ++i) {
    double dist_in  = vp[i].distance(vp[i-1]);
    double dist_out  = vp[i].distance(vp[i+1]);
    if (dist_in == 0.0 || dist_out == 0.0) {
      continue;
    }
    double d_in  = min(0.5 * dist_in,  safe_dist);
    double d_out  = min(0.5 * dist_out,  safe_dist);
    F2CPoint p_in =  (vp[i-1] - vp[i]) * (d_in  / dist_in)  + vp[i];
    F2CPoint p_out = (vp[i+1] - vp[i]) * (d_out / dist_out) + vp[i];
    if (p_in.distance(path.back().first) > 3.0 * safe_dist &&
        p_in.distance(p1)                > 3.0 * safe_dist &&
        p_in.distance(p2)                > 3.0 * safe_dist) {
      double ang_in   = (vp[i  ] - vp[i-1]).getAngleFromPoint();
      path.emplace_back(p_in, ang_in);
    }
    if (p_out.distance(vp[i+1]) > 3.0 * safe_dist &&
        p_out.distance(p1)                > 3.0 * safe_dist &&
        p_out.distance(p2)                > 3.0 * safe_dist &&
        p_out.distance(p_in)              > 3.0 * safe_dist) {
      double ang_out  = (vp[i+1] - vp[i]).getAngleFromPoint();
      path.emplace_back(p_out, ang_out);
    }
  }
  path.emplace_back(p2, ang2);
  return path;
}

}  // namespace f2c::pp
