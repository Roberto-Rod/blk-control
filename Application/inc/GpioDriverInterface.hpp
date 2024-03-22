#pragma once

#include <utility>

namespace mercury
{
    namespace blackstar
    {
        class GpioDriverInterface
        {
        public:
            virtual ~GpioDriverInterface() = default;

            virtual bool read() = 0;
            virtual void write(const bool state) = 0;
        };
    }
}
