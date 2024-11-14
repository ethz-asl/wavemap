#ifndef WAVEMAP_CORE_UTILS_PROFILE_PROFILER_INTERFACE_H_
#define WAVEMAP_CORE_UTILS_PROFILE_PROFILER_INTERFACE_H_

#ifdef TRACY_ENABLE

#include <tracy/Tracy.hpp>

#define ProfilerZoneScoped ZoneScoped
#define ProfilerZoneScopedN(x) ZoneScopedN(x)

#define ProfilerFrameMark FrameMark
#define ProfilerFrameMarkNamed(x) FrameMarkNamed(x)

#define ProfilerPlot(x, y) TracyPlot(x, y)

#define ProfilerSetThreadName(x) tracy::SetThreadName(x)

#else

#define ProfilerZoneScoped
#define ProfilerZoneScopedN(x)

#define ProfilerFrameMark
#define ProfilerFrameMarkNamed(x)

#define ProfilerPlot(x, y)

#define ProfilerSetThreadName(x)

#endif

#endif  // WAVEMAP_CORE_UTILS_PROFILE_PROFILER_INTERFACE_H_
