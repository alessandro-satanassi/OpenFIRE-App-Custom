/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Lens refinement of the fitted camera ("Refine lens", Gun Settings > camera).

    Port of the WebApp's js/core/lensfit.js: same data collection, same fit, same choices,
    so that the App and the WebApp give the same values for the same views.

    The gun streams the four tracker vertices (sLensSample) while the user sweeps it over the
    whole screen. Seen by a real camera, every frame is a rigid view of the same flat LED
    frame: with the right lens values each view must be explained by a rotation and a position
    of the gun. The fit looks for the lens values (K1, K2 and the focal ratio) together with the
    unknown LED shape (aspect, skew, keystone) and the pose of every frame, minimising the
    reprojection error (bundle adjustment, Schur complement on the per-frame poses).

    Which values are refined depends on the data only, never on the camera model:
    record values -> K1 -> K1 + K2 -> K1 + K2 + focal ratio, each step taken only when it
    lowers the error significantly and its values are well determined; the curve must be
    physically valid (positive, radial order kept) over the whole field.

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

#ifndef APPLENSFIT_H
#define APPLENSFIT_H

#include <QByteArray>
#include <QString>

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

namespace LensFit {

using Point = std::array<double, 2>;
using Frame = std::array<Point, 4>;         // tracker vertices in perimeter order, camera units
using Mat3 = std::array<std::array<double, 3>, 3>;

/// @brief  Lens values of one group of sLensInfo (camera record, or values in use)
struct Lens {
    double k1 = 0, k2 = 0, max = 0, min = 0, focal = 0;
};

/// @brief  sLensInfo: camera coordinate space, camera record values and values in use
struct Info {
    int resX = 0, resY = 0;
    Lens record, active;
};

/// @brief  sLensSample: the four tracker vertices of one frame
struct Sample {
    bool diamond = false;
    bool complete = false;      // strict checks met: the frame can be used for the fit
    std::array<Point, 4> points{};
};

/// @brief  Refined lens values; an empty value is the camera record's value
struct Values {
    std::optional<double> k1, k2, focal;
};

std::optional<Info> ParseInfo(const QByteArray &payload);
std::optional<Sample> ParseSample(const QByteArray &payload);

/// @brief  Settings encoding: float bits, OF_Const::lensValueUnset for an empty value
uint32_t ToBits(const std::optional<double> &value);
std::optional<double> FromBits(const uint32_t &bits);

/// @brief  Same check as the firmware (OpenFIRECamera::LensCurveValid)
bool CurveValid(double k1, double k2);

enum Tiers_e {
    TierRecord = 0,
    TierK1,
    TierK1K2,
    TierK1K2Focal,
    TierCount
};

struct Result {
    bool ok = false;
    QString reason;
    int tier = TierRecord;
    Values values;                  // empty = camera record value
    double rmsRecord = 0, rms = 0;
    std::optional<double> rmsActive;
    bool activeCustom = false;      // the values in use were refined earlier
    bool keepActive = false;        // the new values do not beat the values in use
    bool hasNear = false;           // record kept: closest values valid and well determined
    int nearTier = TierRecord;
    Values near;
    int validated = -1;             // check on held-out views: -1 not run, 0 failed, 1 passed
    double qObs = 0;
    int frames = 0, dropped = 0;
    struct FitSummary {
        bool valid = false;
        double rms = 0;
        std::array<double, 7> all{};
    };
    std::array<FitSummary, TierCount> fits{};
};

/// @brief  Shared with a running Solve() (another thread): progress 0..1 and cancel request
struct Control {
    std::atomic<double> progress{0};
    std::atomic<bool> cancel{false};
};

/// @brief  Fits the lens values. Empty when cancelled through control.
std::optional<Result> Solve(const Info &info, const std::vector<Frame> &frames, Control *control = nullptr);

// ----- Data collection ------------------------------------------------------------------

constexpr int SCREEN_COLS = 6, SCREEN_ROWS = 4;     // cells to paint inside the screen
constexpr int MIN_FRAMES = 250;
// Frames per screen cell: a screen all green also holds the frames the fit needs.
constexpr int SCREEN_TARGET = (MIN_FRAMES + SCREEN_COLS * SCREEN_ROWS - 1) / (SCREEN_COLS * SCREEN_ROWS);
// Camera field: rings of the normalised radius (lower bound of each ring; the last has no end).
constexpr int FIELD_RING_COUNT = 5;
constexpr std::array<double, FIELD_RING_COUNT> FIELD_RINGS = {0, 0.3, 0.5, 0.65, 0.8};
constexpr int FIELD_TARGET = 12;                    // LED points per ring
constexpr double ROLL_TARGET = 8 * 3.14159265358979323846 / 180;   // tilt reached on each side
constexpr int ROLL_FRAMES = 15;                     // frames beyond the tilt, each side
constexpr double AIM_INSET = 0.07;                  // guide targets at least 7% inside the screen edges
constexpr int MAX_FRAMES = 900;
constexpr int CELL_CAP = 12;                        // frames kept per screen cell and size band

struct Progress {
    double total = 0, screen = 0, field = 0, roll = 0, rollLeft = 0, rollRight = 0, frames = 0;
    bool done = false;
    int count = 0;
};

/// @brief  What the user has to do now (see Collector::GuideNow)
struct Guide {
    int step = 1;
    Progress progress;
    bool hasTarget = false;
    Point target{};             // step 2: screen point to aim at (screen fraction)
    bool closer = false, farther = false;
    int cell = -1;
    double roll = 0, left = 0, right = 0, limit = ROLL_TARGET;  // step 3
};

class Collector
{
public:
    explicit Collector(const Info &info);

    /// @brief  Adds a sLensSample; returns true when the frame was kept for the fit
    bool Add(const Sample &sample);

    Progress ProgressNow() const;

    /// @brief  1 paint the screen; 2 bring the LEDs to the edges of the camera view;
    ///         3 tilt the gun left and right; 4 done
    Guide GuideNow();

    /// @brief  Ring of the camera field of a point (camera units)
    int Ring(const Point &p) const;

    const std::vector<Frame> &Frames() const { return frames; }

    std::array<int, SCREEN_COLS * SCREEN_ROWS> screen{};
    std::array<int, FIELD_RING_COUNT> field{};
    int rollLeft = 0, rollRight = 0;
    bool hasAim = false;
    Point aim{};                // last aim, as a screen fraction
    bool hasLastPoints = false;
    Frame lastPoints{};         // last vertices (camera units, perimeter order)
    bool lastComplete = false;

private:
    struct Meta {
        long long key = 0;
        int cell = -1;
        std::array<int, 4> cells{};
        int roll = 0;
    };
    struct Live {
        Frame pts{};
        Mat3 Hi{};
        bool diamond = false;
        double roll = 0;
    };
    struct Target {
        Point aim{};
        double dist = 0;
        int cell = -1;
    };

    void Keep(const Frame &pts, const Meta &meta);
    void Count(const Meta &meta, const int &d);
    bool Adds(const Meta &meta) const;
    int Redundant(const long long *onlyKey) const;
    void Remove(const int &i);
    std::optional<Target> FieldTarget();
    std::optional<Target> Reach(const int &ring, const int &dir) const;

    Info info;
    double cx = 0, cy = 0;
    std::vector<Frame> frames;
    std::vector<Meta> meta;     // what each kept frame added (parallel to frames)
    std::map<long long, int> buckets;
    bool hasLast = false;
    Frame last{};
    bool hasLive = false;
    Live live;                  // last valid view, for the guide
    int guideCell = -1;         // camera field cell the guide is pointing at
};

} // namespace LensFit

#endif // APPLENSFIT_H
