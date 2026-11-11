/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Time of every HID report of the mice, for the polling rate test (Windows: Raw Input).

    Windows joins the mouse moves into one WM_MOUSEMOVE when the App is busy, and Qt drops the
    older ones too, so the events of Qt can't count the reports of a gun at 209 Hz. Raw Input
    (WM_INPUT) gives every report instead, like getCoalescedEvents() in the browser. The reports
    are read by a thread of its own with a hidden message-only window, and timed there with the
    performance counter, so a slow redraw of the test window does not move the times.

    A report counts when the pointer moves: a relative mouse with a delta, an absolute one (the
    gun) at a new position; a report with only buttons is not an aim.

        AppPollingRawInput raw;
        raw.Start();                    // false where Raw Input is missing (not Windows)
        raw.Take(times);                // report times since the last call (ms, NowMs() clock)
        raw.Stop();

    No Qt, only Win32 and the standard library.

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

#ifndef APPPOLLINGRAWINPUT_H
#define APPPOLLINGRAWINPUT_H

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

class AppPollingRawInput
{
public:
    AppPollingRawInput() = default;
    ~AppPollingRawInput() { Stop(); }
    AppPollingRawInput(const AppPollingRawInput &) = delete;
    AppPollingRawInput &operator=(const AppPollingRawInput &) = delete;

    /// @brief      Whether this system gives every report (Windows only).
    static bool Available();

    /// @brief      Monotonic clock of the report times (ms).
    static double NowMs();

    /// @brief      Starts reading the reports; false if not possible (they are then not counted).
    bool Start();

    void Stop();

    bool Running() const { return running; }

    /// @brief      Moves the report times read since the last call into times (appended).
    void Take(std::vector<double> &times);

    /// @brief      Called by the thread of the reports (Windows).
    void Add(const double &time);

private:
    static constexpr size_t MAX_PENDING = 20000;    // nobody reads them: the oldest go

    bool running = false;
    std::thread thread;
    std::atomic<void *> window { nullptr };         // HWND of the message-only window
    std::mutex lock;
    std::vector<double> pending;
};

#endif // APPPOLLINGRAWINPUT_H
