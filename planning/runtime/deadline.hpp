#pragma once

#include <sys/time.h>

#include <cmath>
#include <stdexcept>

namespace mpb::planning {

// A hard per-solve deadline for both backends. SIGALRM terminates the isolated
// trial process; the Python runner records a right-censored timeout
// observation. VAMP has no native wall-time cancellation hook. Never use this
// in a GUI process.
class SolveDeadline {
 public:
  explicit SolveDeadline(double seconds) {
    itimerval timer{};
    timer.it_value.tv_sec = static_cast<long>(seconds);
    timer.it_value.tv_usec =
        static_cast<long>((seconds - std::floor(seconds)) * 1000000);
    if (setitimer(ITIMER_REAL, &timer, nullptr)) {
      throw std::runtime_error("Cannot arm planning timeout");
    }
  }
  ~SolveDeadline() {
    itimerval timer{};
    setitimer(ITIMER_REAL, &timer, nullptr);
  }
  SolveDeadline(const SolveDeadline&) = delete;
  SolveDeadline& operator=(const SolveDeadline&) = delete;
};

}  // namespace mpb::planning
