#pragma once

#include <vector>
#include <functional>

template<typename TNotifyParameter>
class Observable {
private:
    std::vector<
        std::function<void(TNotifyParameter)>
    > _handlers;

protected:
    void _notify(TNotifyParameter param) {
        for(auto& handler : _handlers) {
            handler(param);
        }
    }

public:
    void register_handler(std::function<void(TNotifyParameter)> handler) {
        _handlers.push_back(handler);
    }
};