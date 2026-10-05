#include <chrono>

namespace JPS
{
class Timer
{
    typedef std::chrono::high_resolution_clock high_resolution_clock;
    typedef std::chrono::milliseconds milliseconds;
    typedef std::chrono::microseconds microseconds;

public:
    explicit Timer(bool run = false)
    {
        if (run)
            Reset();
    }
    void Reset() { _start = high_resolution_clock::now(); }
    milliseconds Elapsed() const
    {
        return std::chrono::duration_cast<milliseconds>(
            high_resolution_clock::now() - _start);
    }
    /// Elapsed time in microseconds (integer). Elapsed() above truncates to
    /// whole milliseconds, which is useless for sub-ms plans.
    long long ElapsedUs() const
    {
        return std::chrono::duration_cast<microseconds>(
                   high_resolution_clock::now() - _start)
            .count();
    }
    /// Elapsed time in milliseconds as a double (microsecond precision).
    double ElapsedMs() const { return ElapsedUs() / 1000.0; }

private:
    high_resolution_clock::time_point _start;
};
} // namespace JPS
