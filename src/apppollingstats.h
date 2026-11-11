/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Statistics of the polling rate test ("Measure Polling Rate", next to the COM port).

    Same rules as the WebApp's js/app/polling-test.js (class PollingStats), so the two Apps give
    the same numbers. Every HID report of the gun (absolute mouse, Run mode) gives one time in ms:

        AppPollingStats stats;
        stats.AddReport(timeMs);        // each HID report
        stats.Sample(nowMs);            // about every display frame: graph, min and max
        stats.Snap(nowMs);              // current, average (last 3 s), min and max (graph)

    A gun held still sends no new aim: a gap longer than PAUSE_MS ends a "burst" of movement and
    is not counted, so the numbers stay those of the movement. No Qt, only the standard library.

    Copyright (C) 2025  Team OpenFIRE

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef APPPOLLINGSTATS_H
#define APPPOLLINGSTATS_H

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <vector>

class AppPollingStats
{
public:
    static constexpr int CAMERA_FPS = 209;          // OpenFIRECameraProfile.h: DFRobot and PAJ7025
    static constexpr double PAUSE_MS = 60;          // longer gap: the gun stopped or left the window
    static constexpr double WINDOW_MS = 500;        // "current" rate: reports of the last half second
    static constexpr double AVERAGE_MS = 3000;      // average: the last 3 s of movement
    static constexpr double SETTLE_MS = 150;        // a burst needs this much movement before it is shown
    static constexpr double SAMPLE_MS = 100;        // one point of the graph / of min and max
    static constexpr int HISTORY_S = 10;            // seconds of movement shown by the graph
    static constexpr int HISTORY_SAMPLES = HISTORY_S * 1000 / (int)SAMPLE_MS;

    // Average and graph (Hz): green from 180 Hz (about 15% under the 209 fps of the camera),
    // orange from 60 Hz (the refresh of an ordinary display), red below. Indicative thresholds.
    static constexpr double AVERAGE_OK_HZ = 180;
    static constexpr double AVERAGE_WARN_HZ = 60;
    enum Level_e { levelNone = 0, levelOk, levelWarn, levelBad };

    struct Sample_s {
        double hz;
        bool full;          // the window covered enough movement: it counts for min and max
        bool resumed;       // first point after a pause
    };

    struct Snapshot_s {
        bool moving = false;
        std::optional<double> current;
        std::optional<double> average;      // last 3 s of movement
        std::optional<double> min;          // last 10 s of movement, like the graph
        std::optional<double> max;
        long reports = 0;
    };

    struct Piece_s {
        double fromIndex, fromHz, toIndex, toHz;
    };

    AppPollingStats() { Reset(); }

    void Reset()
    {
        reports = 0;
        count = 0;
        hasLast = false;
        last = 0;
        recent.clear();
        window.clear();
        windowSum = 0;
        samples.clear();
        hasLastSample = false;
        lastSample = 0;
        resume = false;
    }

    /// @brief      One HID report at time (ms). Times that go back or repeat are ignored.
    void AddReport(const double &time)
    {
        if(!std::isfinite(time)) return;
        if(hasLast && time <= last) return;
        reports++;
        if(!hasLast || time - last > PAUSE_MS) {
            // First report of a burst: the gap before it is a pause, not an interval.
            recent.clear();
        } else AddInterval(time - last);
        last = time;
        hasLast = true;
        recent.push_back(time);
        while(!recent.empty() && recent.front() < time - WINDOW_MS)
            recent.pop_front();
    }

    /// @brief      Whether the gun is sending reports now.
    bool Moving(const double &now) const
    {
        return hasLast && now - last <= PAUSE_MS * 2;
    }

    /// @brief      Rate of the reports of the last WINDOW_MS of the current burst (Hz), none if too few.
    std::optional<double> Current(const double &now) const
    {
        if(!Moving(now) || recent.size() < 3) return std::nullopt;
        // The first reports of a burst are too few for a rate (it would jump up and down).
        const double span = recent.back() - recent.front();
        if(span < SETTLE_MS) return std::nullopt;
        return (recent.size() - 1) * 1000.0 / span;
    }

    /// @brief      About every display frame: one point every SAMPLE_MS while moving. While the
    ///             gun is still nothing is added, so the graph stops and goes on afterwards.
    void Sample(const double &now)
    {
        const std::optional<double> hz = Current(now);
        if(!hz) {
            if(hasLastSample) resume = true;    // next point: after a pause
            hasLastSample = false;
            return;
        }
        if(hasLastSample && now - lastSample < SAMPLE_MS) return;
        lastSample = now;
        hasLastSample = true;
        // Only windows that cover enough of the movement count for min and max.
        const bool full = recent.back() - recent.front() >= WINDOW_MS * 0.6;
        samples.push_back({ *hz, full, resume && !samples.empty() });
        resume = false;
        while((int)samples.size() > HISTORY_SAMPLES)
            samples.pop_front();
    }

    Snapshot_s Snap(const double &now) const
    {
        Snapshot_s s;
        s.moving = Moving(now);
        s.current = Current(now);
        if(!window.empty() && windowSum > 0)
            s.average = 1000.0 * window.size() / windowSum;
        // Lowest and highest rate of the graph (last 10 s of movement): a peak or a drop goes
        // away once it leaves the graph, e.g. a mouse used before the lightgun.
        for(const Sample_s &sample : samples) {
            if(!sample.full) continue;
            s.min = s.min ? std::min(*s.min, sample.hz) : sample.hz;
            s.max = s.max ? std::max(*s.max, sample.hz) : sample.hz;
        }
        s.reports = reports;
        return s;
    }

    const std::deque<Sample_s> &Samples() const { return samples; }
    long Count() const { return count; }

    static Level_e AverageLevel(const std::optional<double> &hz)
    {
        if(!hz || !std::isfinite(*hz)) return levelNone;
        return *hz >= AVERAGE_OK_HZ ? levelOk : *hz >= AVERAGE_WARN_HZ ? levelWarn : levelBad;
    }

    /// @brief      Splits the stretch (i0, hz0)-(i1, hz1) where it crosses the colour thresholds:
    ///             each piece has one colour only, whatever the scale of the graph.
    static std::vector<Piece_s> SplitAtThresholds(const double &i0, const double &hz0, const double &i1, const double &hz1)
    {
        std::vector<double> cuts;
        for(const double limit : { AVERAGE_OK_HZ, AVERAGE_WARN_HZ })
            if((hz0 - limit) * (hz1 - limit) < 0)
                cuts.push_back((limit - hz0) / (hz1 - hz0));
        std::sort(cuts.begin(), cuts.end());
        std::vector<std::pair<double, double>> points { { i0, hz0 } };
        for(const double f : cuts)
            points.push_back({ i0 + (i1 - i0) * f, hz0 + (hz1 - hz0) * f });
        points.push_back({ i1, hz1 });
        std::vector<Piece_s> pieces;
        for(size_t k = 1; k < points.size(); ++k)
            pieces.push_back({ points[k-1].first, points[k-1].second, points[k].first, points[k].second });
        return pieces;
    }

private:
    void AddInterval(const double &interval)
    {
        count++;
        window.push_back(interval);
        windowSum += interval;
        // Only time spent moving fills the window: a pause does not push old reports out.
        while(window.size() > 1 && windowSum - window.front() >= AVERAGE_MS) {
            windowSum -= window.front();
            window.pop_front();
        }
    }

    long reports;                   // all the reports, also the first of each burst
    long count;                     // intervals ever counted (within the bursts)
    bool hasLast;
    double last;                    // time of the last report
    std::deque<double> recent;      // report times of the last WINDOW_MS, for "current"
    std::deque<double> window;      // intervals of the last AVERAGE_MS of movement
    double windowSum;
    std::deque<Sample_s> samples;   // every SAMPLE_MS of movement: the graph
    bool hasLastSample;
    double lastSample;
    bool resume;
};

#endif // APPPOLLINGSTATS_H
