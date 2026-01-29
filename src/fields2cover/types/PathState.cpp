//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#include "fields2cover/types/PathState.h"

namespace f2c::types {

Point PathState::atEnd() const {
  return point.getPointFromAngle(angle, len * static_cast<double>(dir));
}

// 重载 PathSectionType 的 << 操作符
std::ostream& operator<<(std::ostream& os, PathSectionType type) {
    switch (type) {
        case PathSectionType::SWATH:
            os << "SWATH";
            break;
        case PathSectionType::TURN:
            os << "TURN";
            break;
        case PathSectionType::HL_SWATH:
            os << "HL_SWATH";
            break;
        default:
            os << "Unknown PathSectionType(" << static_cast<int>(type) << ")";
    }
    return os;
}

// 重载 PathDirection 的 << 操作符
std::ostream& operator<<(std::ostream& os, PathDirection dir) {
    switch (dir) {
        case PathDirection::FORWARD:
            os << "FORWARD";
            break;
        case PathDirection::BACKWARD:
            os << "BACKWARD";
            break;
        default:
            os << "Unknown PathDirection(" << static_cast<int>(dir) << ")";
    }
    return os;
}

}  // namespace f2c::types

