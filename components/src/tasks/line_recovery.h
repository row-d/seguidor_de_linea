#ifndef LINE_RECOVERY_H
#define LINE_RECOVERY_H

// Bounded recovery without advancing away from the track.
class LineRecovery
{
    bool searching = false;
    unsigned long started_ms = 0;
    int last_direction = -1;

public:
    void observe(int error)
    {
        searching = false;
        if (error < -40) last_direction = -1;
        else if (error > 40) last_direction = 1;
    }

    void reset() { searching = false; }

    // 0: timed out, -1: rotate left, +1: rotate right.
    int direction(unsigned long now, unsigned int switch_ms, unsigned int timeout_ms)
    {
        if (!searching)
        {
            searching = true;
            started_ms = now;
        }
        const unsigned long elapsed = now - started_ms;
        if (elapsed >= timeout_ms) return 0;
        return ((elapsed / switch_ms) % 2 == 0) ? last_direction : -last_direction;
    }
};
#endif
