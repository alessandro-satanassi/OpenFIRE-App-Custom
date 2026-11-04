/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    IR emitter placement planner (Help > Open IR Emitter Alignment Assistant).

    Port of the WebApp's js/core/emitter-layout.js: same geometry, same checks, same choices,
    so that the App and the WebApp suggest the same placement for the same setup.
    Geometry and estimates only: nothing is sent to the lightgun.
    The camera is a record (field of view, lens values, sensor size), never a model-specific
    algorithm. An add-on wide converter is described by its conversion factor combined with
    the native field of view.

    Assistente posizionamento emettitori IR: porting di js/core/emitter-layout.js della WebApp,
    stessa geometria, stessi controlli, stesse scelte. Solo stime: nulla viene inviato alla pistola.

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

#ifndef APPEMITTERLAYOUT_H
#define APPEMITTERLAYOUT_H

#include <optional>
#include <vector>

namespace EmitterLayout {

/// @brief  Optics of one camera record (WebApp boards/OpenFIREshared.js, cameraOptics): field of
///         view from the camera specifications, lens values and sizes of the firmware camera record
struct Camera {
    int model = 0;              // OF_Const::cameraModel_e
    const char *label = "";
    double horizontal = 0, vertical = 0;    // native field of view, degrees
    double tolerance = 0;                   // share of the field of view kept as a reserve
    bool addOnLens = true;                  // an add-on converter can be fitted (false: already wide)
    double coordinateX = 0, coordinateY = 0;
    double sensorX = 0, sensorY = 0;
    double k1 = 0, k2 = 0, max = 0, min = 0, focal = 0;
};

/// @brief  The camera records, in the order of OF_Const::cameraModel_e
const std::vector<Camera> &Cameras();
/// @brief  Record of a camera model, nullptr when unknown
const Camera *CameraByModel(int model);

/// @brief  Refined lens values of the fitted camera (settings CamLensK1/K2/Focal); empty = record value
struct Custom {
    std::optional<double> k1, k2, focal;
};

/// @brief  Optics used by the planner: the record, refined values or converter applied
struct Optics : Camera {
    bool estimated = false;     // an add-on converter factor is applied
    bool refined = false;       // refined lens values of the App are applied
    double tx = 0, ty = 0, sideGain = 0;
    bool operator==(const Optics &o) const;
};

struct Settings {
    double inches = 55, aspect = 16.0 / 9.0, distance = 2;
    bool diamond = false;
};

struct Dimensions {
    double width = 0, height = 0;   // metres
};

enum Status { StatusGood = 0, StatusLimited, StatusClose };

struct Result {
    double ratio = 0;               // base / height of the Square rectangle (screen aspect for Diamond)
    Status status = StatusGood;
    double width = 0, height = 0;   // screen size, metres
    Settings settings;
    Optics optics;
    double base = 0;                // horizontal LED spacing, metres
    std::optional<double> minDistance;  // empty: more than 10 m (or not asked)
    std::optional<double> moveTo;       // for fuller LED coverage
};

constexpr double DEFAULT_RATIO = 0.711;
constexpr double MAX_DISTANCE = 10;
constexpr double DISTANCE_STEP = .1;
constexpr double STEP_FORWARD = .9;
constexpr double FAR_STEP = .85;
constexpr double FAR_HEIGHTS = 4;

/// @brief  Clamped settings, like the WebApp (20..120", aspect 1..4, 0.3..10 m)
Settings Clamp(const Settings &input);
Dimensions ScreenSize(double inches, double aspect);
/// @brief  false when the values are not usable (missing optics, converter out of 0.3..1)
bool MakeOptics(const Camera &record, double factor, const Custom *custom, Optics &out);
/// @brief  Rectangles checked for this setup (Square), or the screen aspect (Diamond)
std::vector<double> Candidates(const Settings &s, double distance = 0);
/// @brief  Suggested placement; false when the optics are not usable
bool Calculate(const Settings &input, const Camera &record, double factor, const Custom *custom,
               bool withDistance, Result &out);

/// @brief  Up rounding to the distance step (0.1 m)
double DistanceCeil(double value);

} // namespace EmitterLayout

#endif // APPEMITTERLAYOUT_H
