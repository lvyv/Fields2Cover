//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#pragma once
#ifndef FIELDS2COVER_TYPES_PATHSTATE_H_
#define FIELDS2COVER_TYPES_PATHSTATE_H_

#include "fields2cover/types/Point.h"

namespace f2c::types {

enum class PathSectionType {
  SWATH = 1,
  TURN = 2,
  HL_SWATH = 3,
};

enum class PathDirection {
  FORWARD = 1,
  BACKWARD = -1,
  // STOP = 0,
};

// 在头文件中声明操作符重载
std::ostream& operator<<(std::ostream& os, PathSectionType type);
std::ostream& operator<<(std::ostream& os, PathDirection dir);

struct PathState {
  Point point {0, 0, 0};
  double angle {0.0};
  double len {0.0};  // >= 0
  PathDirection dir {PathDirection::FORWARD};
  PathSectionType type {PathSectionType::SWATH};
  double velocity {1.0};

 public:
  Point atEnd() const;
};


}  // namespace f2c::types

#endif  //  FIELDS2COVER_TYPES_PATHSTATE_H_
