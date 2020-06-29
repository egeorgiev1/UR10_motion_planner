#pragma once

#include <array>
#include <vector>

#include "util.h"

class INearestNeighbour {
public:
    // to be associated with another vector that contains the actual ConfigSpacePoints in the corresponding indexes
    virtual void addPoint(intmax_t index, std::array<real_t, 3> position) = 0;
    
    // returns an array of indexes
    virtual std::vector<intmax_t> getInRadius(std::array<real_t, 3> origin, real_t radius) = 0;

     // returns an array of indexes
    virtual std::vector<intmax_t> getClosestNPoints(std::array<real_t, 3> origin, intmax_t count) = 0;

    virtual ~INearestNeighbour() = default;
};
