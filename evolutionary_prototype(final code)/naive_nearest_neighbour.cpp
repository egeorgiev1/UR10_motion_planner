#include <queue>

#include "naive_nearest_neighbour.h"

#include "util.h"

void NaiveNearestNeighbour::addPoint(intmax_t index, std::array<real_t, 3> position) {
    _index[index] = position;
}

std::vector<intmax_t> NaiveNearestNeighbour::getInRadius(
    std::array<real_t, 3> origin,
    real_t radius
) {
    std::vector<intmax_t> result;

    for(const auto& position : _index) {
        if(euclidean_distance(origin, position.second) <= radius) {
            result.push_back(position.first);
        }
    }

    return result;
}

std::vector<intmax_t> NaiveNearestNeighbour::getClosestNPoints(
    std::array<real_t, 3> origin,
    intmax_t count
) {
    // Create min-heap with a custom comparator
    auto comparator = [origin](
        decltype(_index)::value_type left,
        decltype(_index)::value_type right
    ) {
        return euclidean_distance(left.second, origin) > euclidean_distance(right.second, origin);
    };

    // TODO: decltype cannot work, cuz value_type.first is const!!!
    std::priority_queue<
        std::pair<intmax_t, std::array<real_t, 3>>,
        std::vector<
            std::pair<intmax_t, std::array<real_t, 3>>
        >,
        decltype(comparator)
    > min_heap(comparator);

    // Populate priority queue
    for(const auto& position : _index) {
        min_heap.push(position);
    }

    // Pop up to 'count' many elements from the priority queue to
    // construct the result
    std::vector<intmax_t> result;

    while(count > 0 && !min_heap.empty()) {
        auto current_closest = min_heap.top();

        result.push_back(current_closest.first);
        min_heap.pop();

        count--;
    }

    return result;
}
