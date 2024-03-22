#pragma once

#ifdef __cplusplus
extern "C" [[noreturn]]
#endif
void controllerTaskRun();

#ifdef __cplusplus

#include "Application/inc/Bsp.hpp"

namespace mercury
{
    namespace blackstar
    {
        class ControllerTask
        {
        public:
            ControllerTask() = default;
            [[noreturn]] void run();

            static constexpr auto k_initialisedSignal { 0x01 };

        private:

        };
    }
}
#endif // __cplusplus
