#pragma once

#include <array>
#include <vector>
#include <map>

#include "i_nearest_neighbour.h"
#include "util.h"

class NaiveNearestNeighbour : public INearestNeighbour {
private:
    std::map<intmax_t, std::array<real_t, 3>> _index;
public:
    // to be associated with another vector that contains the actual ConfigSpacePoints in the corresponding indexes
    void addPoint(intmax_t index, std::array<real_t, 3> position) override;
    
    // returns an array of indexes
    std::vector<intmax_t> getInRadius(std::array<real_t, 3> origin, real_t radius) override;

     // returns an array of indexes
    std::vector<intmax_t> getClosestNPoints(std::array<real_t, 3> origin, intmax_t count) override;
};
