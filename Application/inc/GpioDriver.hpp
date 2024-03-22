#pragma once

#include "Application/inc/GpioDriverInterface.hpp"

#include <utility>

namespace mercury
{
    namespace blackstar
    {
        template<typename TR, typename TW>
        class GpioDriver : public GpioDriverInterface
        {
        public:
            GpioDriver(TR&& readFunction, TW&& writeFunction)
                : m_readFunction { std::forward<TR&>(readFunction) },
                  m_writeFunction { std::forward<TW&>(writeFunction) }
            {
            }
            ~GpioDriver() = default;

            bool read() override
            {
                return m_readFunction();
            }

            void write(const bool state) override
            {
                m_writeFunction(state);
            }

        private:
            TR& m_readFunction;
            TW& m_writeFunction;
        };
    }
}
