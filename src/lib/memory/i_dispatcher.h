#pragma once

#include "i_device.h"

namespace NMemory {

class IDispatcher : public IDevice {
public:
    virtual void AddDevice(IDevice* device, TAddressRange range) = 0;
};

} // namespace NMemory
