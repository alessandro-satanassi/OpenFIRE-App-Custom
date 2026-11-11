/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Time of every HID report of the mice, for the polling rate test (Windows: Raw Input).

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

#include "apppollingrawinput.h"

#include <chrono>
#include <future>
#include <map>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

void AppPollingRawInput::Take(std::vector<double> &times)
{
    std::lock_guard<std::mutex> guard(lock);
    times.insert(times.end(), pending.begin(), pending.end());
    pending.clear();
}

void AppPollingRawInput::Add(const double &time)
{
    std::lock_guard<std::mutex> guard(lock);
    if(pending.size() >= MAX_PENDING)
        pending.erase(pending.begin(), pending.begin() + MAX_PENDING / 2);
    pending.push_back(time);
}

#ifdef _WIN32

namespace {

constexpr UINT WM_POLLING_QUIT = WM_APP + 1;
const wchar_t WINDOW_CLASS[] = L"OpenFIREappPollingRawInput";

struct Thread_s {
    AppPollingRawInput *owner;
    std::map<HANDLE, std::pair<LONG, LONG>> absolute;   // last position of each absolute pointer
    std::vector<BYTE> buffer;
};

LRESULT CALLBACK RawWindowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    if(message == WM_INPUT) {
        // Timed first: reading the report does not delay it.
        const double time = AppPollingRawInput::NowMs();
        Thread_s *state = reinterpret_cast<Thread_s *>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        UINT size = 0;
        if(state && GetRawInputData(reinterpret_cast<HRAWINPUT>(lParam), RID_INPUT, nullptr, &size, sizeof(RAWINPUTHEADER)) == 0 && size) {
            if(state->buffer.size() < size)
                state->buffer.resize(size);
            if(GetRawInputData(reinterpret_cast<HRAWINPUT>(lParam), RID_INPUT, state->buffer.data(), &size, sizeof(RAWINPUTHEADER)) == size) {
                const RAWINPUT *input = reinterpret_cast<const RAWINPUT *>(state->buffer.data());
                if(input->header.dwType == RIM_TYPEMOUSE) {
                    const RAWMOUSE &mouse = input->data.mouse;
                    bool moved;
                    if(mouse.usFlags & MOUSE_MOVE_ABSOLUTE) {
                        // The gun: a report at the same position is not a new aim.
                        auto &position = state->absolute[input->header.hDevice];
                        moved = position.first != mouse.lLastX || position.second != mouse.lLastY;
                        position = { mouse.lLastX, mouse.lLastY };
                    } else moved = mouse.lLastX != 0 || mouse.lLastY != 0;
                    if(moved)
                        state->owner->Add(time);
                }
            }
        }
        // Raw Input wants DefWindowProc for its cleanup.
        return DefWindowProcW(hwnd, message, wParam, lParam);
    }
    if(message == WM_POLLING_QUIT) {
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, message, wParam, lParam);
}

} // namespace

bool AppPollingRawInput::Available()
{
    return true;
}

double AppPollingRawInput::NowMs()
{
    static const double period = [] {
        LARGE_INTEGER frequency;
        QueryPerformanceFrequency(&frequency);
        return 1000.0 / (double)frequency.QuadPart;
    }();
    LARGE_INTEGER counter;
    QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart * period;
}

bool AppPollingRawInput::Start()
{
    if(running)
        return true;
    {
        std::lock_guard<std::mutex> guard(lock);
        pending.clear();
    }

    std::promise<bool> started;
    std::future<bool> result = started.get_future();
    thread = std::thread([this, &started]() {
        HINSTANCE instance = GetModuleHandleW(nullptr);
        WNDCLASSW windowClass = {};
        windowClass.lpfnWndProc = RawWindowProc;
        windowClass.hInstance = instance;
        windowClass.lpszClassName = WINDOW_CLASS;
        RegisterClassW(&windowClass);       // already there after the first test: fine

        HWND hwnd = CreateWindowExW(0, WINDOW_CLASS, L"", 0, 0, 0, 0, 0, HWND_MESSAGE, nullptr, instance, nullptr);
        if(!hwnd) {
            started.set_value(false);
            return;
        }
        Thread_s state { this, {}, {} };
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(&state));

        // Generic desktop / mouse. INPUTSINK: also when the hidden window is not in the
        // foreground (it never is); the test window decides when the reports count.
        RAWINPUTDEVICE device = { 0x01, 0x02, RIDEV_INPUTSINK, hwnd };
        if(!RegisterRawInputDevices(&device, 1, sizeof(device))) {
            DestroyWindow(hwnd);
            started.set_value(false);
            return;
        }
        window = hwnd;
        started.set_value(true);

        MSG message;
        while(GetMessageW(&message, nullptr, 0, 0) > 0)
            DispatchMessageW(&message);

        RAWINPUTDEVICE remove = { 0x01, 0x02, RIDEV_REMOVE, nullptr };
        RegisterRawInputDevices(&remove, 1, sizeof(remove));
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, 0);
        DestroyWindow(hwnd);
        window = nullptr;
    });

    running = result.get();
    if(!running && thread.joinable())
        thread.join();
    return running;
}

void AppPollingRawInput::Stop()
{
    if(!running)
        return;
    running = false;
    if(void *hwnd = window.load())
        PostMessageW(static_cast<HWND>(hwnd), WM_POLLING_QUIT, 0, 0);
    if(thread.joinable())
        thread.join();
}

#else

// Elsewhere the test window uses the mouse events of Qt (the rate can come out lower).

bool AppPollingRawInput::Available()
{
    return false;
}

double AppPollingRawInput::NowMs()
{
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

bool AppPollingRawInput::Start()
{
    return false;
}

void AppPollingRawInput::Stop()
{
    running = false;
}

#endif
