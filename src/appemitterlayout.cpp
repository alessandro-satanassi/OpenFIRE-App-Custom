/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    IR emitter placement planner: port of the WebApp's js/core/emitter-layout.js.

    Every function follows its WebApp counterpart line by line (same constants, same order of
    the operations), so that both Apps give the same results.
    Ogni funzione segue la sua controparte della WebApp riga per riga (stesse costanti, stesso
    ordine delle operazioni), cosi' le due App danno gli stessi risultati.

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

#include "appemitterlayout.h"
#include "applensfit.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <string>

namespace EmitterLayout {

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double R = PI / 180;
constexpr double INF = std::numeric_limits<double>::infinity();

using Vec3 = std::array<double, 3>;
using Point = std::array<double, 2>;
using Quad = std::array<Point, 4>;

struct Roll { double c, sn; int spare; };

const std::array<Roll, 7> &PlayRolls()
{
    static const std::array<Roll, 7> rolls = [] {
        std::array<Roll, 7> list{};
        const int degrees[7] = {-15, -10, -5, 0, 5, 10, 15};
        for(int i = 0; i < 7; ++i)
            list[i] = {std::cos(degrees[i] * R), std::sin(degrees[i] * R), degrees[i] ? 1 : 0};
        return list;
    }();
    return rolls;
}

double Clampd(double x, double a, double b) { return std::max(a, std::min(b, x)); }
double Dot(const Vec3 &a, const Vec3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
Vec3 Norm(const Vec3 &v) { const double n = std::hypot(v[0], v[1], v[2]); return {v[0] / n, v[1] / n, v[2] / n}; }
Vec3 Cross(const Vec3 &a, const Vec3 &b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
// Math.round of the WebApp: halves go up
double JsRound(double x) { return std::floor(x + 0.5); }

struct Pose {
    Vec3 eye{}, f{}, right{}, down{};
    int spare = 0;
};

std::vector<Pose> Poses(const Settings &s, double distance)
{
    const Dimensions d = ScreenSize(s.inches, s.aspect);
    const double w = d.width, h = d.height;
    // One play profile: side moves up to 20% of the screen width, the gun held straight (all
    // four LEDs) or rolled up to 15 degrees: then one LED may leave the view, the firmware
    // rebuilds it from the other three (spare). Larger rolls are rare: not checked.
    const double sides[3] = {-w * .2, 0, w * .2};
    const double ups[2] = {-h * .1, h * .1};
    const double aims[3] = {-.5, 0, .5};
    std::vector<Pose> list;
    list.reserve(3 * 2 * 3 * 3 * 7);
    for(const double side : sides) for(const double up : ups) for(const double ax : aims) for(const double ay : aims) {
        const Vec3 f = Norm({ax * w - side, ay * h - up, distance}), right = Norm({f[2], 0, -f[0]}), down = Cross(f, right);
        for(const Roll &roll : PlayRolls()) {
            Pose p;
            p.eye = {side, up, -distance};
            p.f = f;
            p.spare = roll.spare;
            for(int i = 0; i < 3; ++i) {
                p.right[i] = right[i] * roll.c + down[i] * roll.sn;
                p.down[i] = down[i] * roll.c - right[i] * roll.sn;
            }
            list.push_back(p);
        }
    }
    return list;
}

std::array<Vec3, 4> Leds(const Settings &s, double ratio)
{
    const Dimensions d = ScreenSize(s.inches, s.aspect);
    const double w = d.width, h = d.height;
    if(s.diamond)
        return {{{0, -h / 2, 0}, {w / 2, 0, 0}, {0, h / 2, 0}, {-w / 2, 0, 0}}};
    return {{{-ratio * h / 2, -h / 2, 0}, {ratio * h / 2, -h / 2, 0}, {-ratio * h / 2, h / 2, 0}, {ratio * h / 2, h / 2, 0}}};
}

Point Project(const Optics &o, const Pose &pose, const Vec3 &led)
{
    const Vec3 v = {led[0] - pose.eye[0], led[1] - pose.eye[1], led[2] - pose.eye[2]};
    const double z = Dot(v, pose.f);
    if(!(z > 0)) return {1e6, 1e6};
    double x = Dot(v, pose.right) / (z * o.tx) * o.sideGain;
    double y = Dot(v, pose.down) / (z * o.ty) * o.sideGain;
    const double r = std::hypot(x, y);
    if(r != 0 && (o.k1 != 0 || o.k2 != 0)) {
        double lo = 0, hi = std::max(2.0, r / o.min + 1);
        for(int n = 0; n < 18; ++n) {
            const double m = (lo + hi) / 2, q = m * m, gain = Clampd(1 + o.k1 * q + o.k2 * q * q, o.min, o.max);
            if(m * gain < r) lo = m; else hi = m;
        }
        const double scale = (lo + hi) / (2 * r);
        x *= scale;
        y *= scale;
    }
    return {x, y};
}

// Test the radial boundary directly for the distance search (no inversion).
bool Fits(const Settings &s, const Optics &o, double ratio, const std::vector<Pose> &scenePoses, double border = .05)
{
    const std::array<Vec3, 4> points = Leds(s, ratio);
    const double limit = 1 - border;
    for(const Pose &pose : scenePoses) {
        int out = 0;
        for(const Vec3 &led : points) {
            const Vec3 v = {led[0] - pose.eye[0], led[1] - pose.eye[1], led[2] - pose.eye[2]};
            const double z = Dot(v, pose.f);
            bool bad = !(z > 0);
            if(!bad) {
                const double x = Dot(v, pose.right) / (z * o.tx) * o.sideGain, y = Dot(v, pose.down) / (z * o.ty) * o.sideGain;
                const double m = std::max(std::fabs(x), std::fabs(y));
                if(m != 0) {
                    const double r = (x * x + y * y) * limit * limit / (m * m);
                    const double gain = Clampd(1 + o.k1 * r + o.k2 * r * r, o.min, o.max);
                    bad = m > limit * gain;
                }
            }
            if(bad && ++out > pose.spare) return false;
        }
    }
    return true;
}

// Pointing sensitivity in screen-height units, using the firmware radial map.
std::optional<Point> Pointer(const Optics &o, double ratio, const Quad &raw)
{
    Quad q;
    for(int i = 0; i < 4; ++i) {
        const double x = raw[i][0], y = raw[i][1], r = x * x + y * y;
        const double g = Clampd(1 + o.k1 * r + o.k2 * r * r, o.min, o.max);
        q[i] = {x * g, y * g};
    }
    const double x0 = q[0][0], y0 = q[0][1], x1 = q[1][0], y1 = q[1][1];
    const double x3 = q[2][0], y3 = q[2][1], x2 = q[3][0], y2 = q[3][1];
    const double dx1 = x1 - x2, dy1 = y1 - y2, dx2 = x3 - x2, dy2 = y3 - y2, sx = x0 - x1 + x2 - x3, sy = y0 - y1 + y2 - y3;
    const double det = dx1 * dy2 - dx2 * dy1;
    if(std::fabs(det) < 1e-12) return std::nullopt;
    const double g = (sx * dy2 - dx2 * sy) / det, h = (dx1 * sy - sx * dy1) / det;
    const double a = x1 - x0 + g * x1, b = x3 - x0 + h * x3, c = x0, d = y1 - y0 + g * y1, e = y3 - y0 + h * y3, f = y0;
    const double den = a * e - b * d;
    if(std::fabs(den) < 1e-12) return std::nullopt;
    return Point{((b * f - c * e) / den - .5) * ratio, (c * d - a * f) / den - .5};
}

// Approximate hidden points with the same virtual-camera geometry as Anchor.
struct Plane {
    bool identity = true;
    double focal = 0;
    std::array<double, 9> r{};
    std::optional<Point> To(const Point &p) const
    {
        if(identity) return p;
        const double x = p[0], y = p[1];
        const double z = r[6] * x + r[7] * y + r[8] * focal;
        if(!(z > 0)) return std::nullopt;
        return Point{(r[0] * x + r[1] * y + r[2] * focal) * focal / z, (r[3] * x + r[4] * y + r[5] * focal) * focal / z};
    }
    std::optional<Point> From(const Point &p) const
    {
        if(identity) return p;
        const double x = p[0], y = p[1];
        const double z = r[2] * x + r[5] * y + r[8] * focal;
        if(!(z > 0)) return std::nullopt;
        return Point{(r[0] * x + r[3] * y + r[6] * focal) * focal / z, (r[1] * x + r[4] * y + r[7] * focal) * focal / z};
    }
};

Plane AnchorPlane(const std::vector<Point> &points, double focal)
{
    Plane plane;
    if(!(focal > 0)) return plane;
    Vec3 mean = {0, 0, 0};
    for(const Point &p : points) {
        const double n = std::hypot(p[0], p[1], focal);
        mean[0] += p[0] / n;
        mean[1] += p[1] / n;
        mean[2] += focal / n;
    }
    const Vec3 m = Norm(mean);
    const double vx = m[1], vy = -m[0], c = 1 / (1 + m[2]);
    plane.identity = false;
    plane.focal = focal;
    plane.r = {1 - vy * vy * c, vx * vy * c, vy, vx * vy * c, 1 - vx * vx * c, -vx, -vy, vx, 1 - (vx * vx + vy * vy) * c};
    return plane;
}

// Keep the complete reference fixed while perturbing current measurements.
// An empty function: the reconstruction is not possible (WebApp: null).
using Rebuild = std::function<std::optional<Quad>(const Quad &)>;

Rebuild Reconstruction(const Optics &o, const Quad &reference, const std::vector<int> &visible, double focal)
{
    if(visible.size() == 4) return [](const Quad &current) { return std::optional<Quad>(current); };
    if(visible.empty()) return [reference](const Quad &) { return std::optional<Quad>(reference); };
    const auto sensor = [&o](const Point &p) { return Point{p[0] * o.sensorX / 2, p[1] * o.sensorY / 2}; };
    std::vector<Point> old;
    for(const int i : visible) old.push_back(sensor(reference[i]));
    if(visible.size() == 1) {
        const int v0 = visible[0];
        const Point old0 = old[0];
        const double sx = o.sensorX, sy = o.sensorY;
        return [=](const Quad &current) {
            const Point now = {current[v0][0] * sx / 2, current[v0][1] * sy / 2};
            const double dx = now[0] - old0[0], dy = now[1] - old0[1];
            Quad result;
            for(int i = 0; i < 4; ++i)
                result[i] = {reference[i][0] + 2 * dx / sx, reference[i][1] + 2 * dy / sy};
            return std::optional<Quad>(result);
        };
    }
    const Plane P = AnchorPlane(old, focal);
    std::vector<Point> p;
    for(const Point &x : old) {
        const auto t = P.To(x);
        if(!t) return Rebuild();
        p.push_back(*t);
    }
    const bool two = visible.size() == 2;
    const Point a = {p[1][0] - p[0][0], p[1][1] - p[0][1]};
    const Point b = two ? Point{-a[1], a[0]} : Point{p[2][0] - p[0][0], p[2][1] - p[0][1]};
    const double det = a[0] * b[1] - a[1] * b[0];
    if(std::fabs(det) < 1e-12) return Rebuild();
    struct Hidden { int i; double s, t; };
    std::vector<Hidden> hidden;
    for(int i = 0; i < 4; ++i) {
        if(std::find(visible.begin(), visible.end(), i) != visible.end()) continue;
        const auto h = P.To(sensor(reference[i]));
        if(!h) return Rebuild();
        const double x = (*h)[0] - p[0][0], y = (*h)[1] - p[0][1];
        hidden.push_back({i, (x * b[1] - y * b[0]) / det, (a[0] * y - a[1] * x) / det});
    }
    const double sx = o.sensorX, sy = o.sensorY;
    return [=](const Quad &current) -> std::optional<Quad> {
        std::vector<Point> now;
        for(const int i : visible) now.push_back({current[i][0] * sx / 2, current[i][1] * sy / 2});
        const Plane Q = AnchorPlane(now, focal);
        std::vector<Point> q;
        for(const Point &x : now) {
            const auto t = Q.To(x);
            if(!t) return std::nullopt;
            q.push_back(*t);
        }
        const Point u = {q[1][0] - q[0][0], q[1][1] - q[0][1]};
        const Point v = two ? Point{-u[1], u[0]} : Point{q[2][0] - q[0][0], q[2][1] - q[0][1]};
        Quad result = current;
        for(const Hidden &hd : hidden) {
            const auto h = Q.From({q[0][0] + hd.s * u[0] + hd.t * v[0], q[0][1] + hd.s * u[1] + hd.t * v[1]});
            if(!h) return std::nullopt;
            result[hd.i] = {2 * (*h)[0] / sx, 2 * (*h)[1] / sy};
        }
        return result;
    };
}

struct Score {
    double fine = INF, coarse = INF, worstFine = INF, worstCoarse = INF, full = 0;
    double Bound(int k) const { return k == 0 ? fine : k == 1 ? coarse : k == 2 ? worstFine : worstCoarse; }
};

bool Inside(const Point &p) { return std::max(std::fabs(p[0]), std::fabs(p[1])) <= 1; }

// A geometric index, not a measured pointing error. It combines reconstruction bias and
// sensitivity at camera-output and sensor-pixel scales, in mean and worst poses.
// No camera IDs, fitted per-model weights or calculations in the firmware loop.
Score Precision(const Settings &s, const Optics &o, double ratio, double distance)
{
    const Dimensions dim = ScreenSize(s.inches, s.aspect);
    const double width = dim.width, height = dim.height;
    const std::array<Vec3, 4> points = Leds(s, ratio);
    const double focal = o.sensorX * .5 * o.focal;
    const double resolution[2] = {o.coordinateX != 0 ? o.coordinateX : o.sensorX, o.coordinateY != 0 ? o.coordinateY : o.sensorY};
    const Score invalid;
    double fine = 0, coarse = 0, worstFine = 0, worstCoarse = 0, count = 0, full = 0;
    const int sweepSteps = (int)std::ceil(std::log2(std::max(resolution[0], resolution[1])));
    // Radial optics and centred LEDs are mirror symmetric: the same average needs one height
    // and two lateral positions (the mirrored side counts twice).
    for(const double depth : {distance, distance * STEP_FORWARD}) for(const double side : {0.0, .2}) for(const double up : {.1}) {
        const double weight = side != 0 ? 2 : 1;
        Pose centrePose;
        centrePose.eye = {side * width, up * height, -depth};
        centrePose.f = Norm({-centrePose.eye[0], -centrePose.eye[1], depth});
        centrePose.right = Norm({centrePose.f[2], 0, -centrePose.f[0]});
        centrePose.down = Cross(centrePose.f, centrePose.right);
        const Vec3 eye = centrePose.eye;
        Quad centre;
        for(int i = 0; i < 4; ++i) centre[i] = Project(o, centrePose, points[i]);
        const double targets[5][2] = {{0, 0}, {-.5, -.5}, {.5, -.5}, {-.5, .5}, {.5, .5}};
        for(const auto &target : targets) for(const double roll : {-15.0, 0.0, 15.0}) {
            const double ax = target[0], ay = target[1];
            const auto view = [&](double t) {
                const Vec3 f = Norm({t * ax * width - eye[0], t * ay * height - eye[1], depth});
                const Vec3 base = Norm({f[2], 0, -f[0]}), vertical = Cross(f, base);
                const double c = std::cos(t * roll * R), sn = std::sin(t * roll * R);
                Pose pose;
                pose.eye = eye;
                pose.f = f;
                for(int i = 0; i < 3; ++i) {
                    pose.right[i] = base[i] * c + vertical[i] * sn;
                    pose.down[i] = vertical[i] * c - base[i] * sn;
                }
                Quad q;
                for(int i = 0; i < 4; ++i) q[i] = Project(o, pose, points[i]);
                return q;
            };
            Quad raw = view(1);
            std::vector<int> visible;
            for(int i = 0; i < 4; ++i) if(Inside(raw[i])) visible.push_back(i);
            Quad ref = centre;
            // A nearby complete view along a centre-to-target sweep.
            if(visible.size() < 4 && std::all_of(centre.begin(), centre.end(), Inside)) {
                double lo = 0, hi = 1;
                for(int n = 0; n < sweepSteps; ++n) {
                    const double mid = (lo + hi) / 2;
                    const Quad q = view(mid);
                    if(std::all_of(q.begin(), q.end(), Inside)) { lo = mid; ref = q; }
                    else hi = mid;
                }
            }
            const Rebuild reconstruct = Reconstruction(o, ref, visible, focal);
            const auto evaluate = [&](const Quad &r) -> std::optional<Point> {
                if(!reconstruct) return std::nullopt;
                const auto quad = reconstruct(r);
                if(!quad) return std::nullopt;
                return Pointer(o, ratio, *quad);
            };
            const auto q = evaluate(raw);
            if(!q) return invalid;
            double viewFine = std::pow((*q)[0] - ax * s.aspect, 2) + std::pow((*q)[1] - ay, 2), viewCoarse = viewFine;
            if(visible.size() == 4) full += weight;
            for(const int i : visible) for(int axis = 0; axis < 2; ++axis) {
                const double value = raw[i][axis], epsilon = 1 / resolution[axis];
                raw[i][axis] = value + epsilon;
                const auto a = evaluate(raw);
                raw[i][axis] = value - epsilon;
                const auto b = evaluate(raw);
                raw[i][axis] = value;
                if(!a || !b) return invalid;
                // Uniform quantisation variance: step squared / 12.
                const double delta = (std::pow((*a)[0] - (*b)[0], 2) + std::pow((*a)[1] - (*b)[1], 2)) / 12;
                viewFine += delta;
                viewCoarse += delta * std::pow(resolution[axis] / (axis ? o.sensorY : o.sensorX), 2);
            }
            fine += weight * viewFine;
            coarse += weight * viewCoarse;
            worstFine = std::max(worstFine, viewFine);
            worstCoarse = std::max(worstCoarse, viewCoarse);
            count += weight;
        }
    }
    Score result;
    result.fine = fine / count;
    result.coarse = coarse / count;
    result.worstFine = worstFine;
    result.worstCoarse = worstCoarse;
    result.full = full / count;
    return result;
}

struct Row {
    double ratio = 0, usable = 0, good = 0, at = 0;
};

struct Step {
    double at = 0;
    int row = 0;
};

struct Plan {
    std::vector<Row> rows;
    int first = 0;
    std::vector<Step> steps;
    std::optional<double> green;
};

// The progression is cached once per setup (like the WebApp: the last setup only).
struct PlanKey {
    double inches = 0, aspect = 0;
    bool diamond = false;
    Optics optics;
    bool operator==(const PlanKey &k) const
    {
        return inches == k.inches && aspect == k.aspect && diamond == k.diamond && optics == k.optics;
    }
};

const Plan &DistancePlan(const Settings &s, const Optics &o)
{
    static bool cached = false;
    static PlanKey planKey;
    static Plan cachedPlan;
    const PlanKey key = {s.inches, s.aspect, s.diamond, o};
    if(cached && key == planKey) return cachedPlan;

    const double height = ScreenSize(s.inches, s.aspect).height;
    const std::vector<double> immediate = Candidates(s, 0);
    // Reuse the identical search endpoints for every rectangle.
    const std::vector<Pose> nearPoses = Poses(s, .3), farPoses = Poses(s, MAX_DISTANCE);

    Plan plan;
    for(const double ratio : Candidates(s, MAX_DISTANCE)) {
        // Physical limits first; the FAR rule is applied apart, not divided by STEP_FORWARD.
        const auto limit = [&](double border) {
            const auto accepts = [&](double d) {
                if(d == .3) return Fits(s, o, ratio, nearPoses, border);
                if(d == MAX_DISTANCE) return Fits(s, o, ratio, farPoses, border);
                return Fits(s, o, ratio, Poses(s, d), border);
            };
            if(!accepts(MAX_DISTANCE)) return INF;
            double lo = .3, hi = MAX_DISTANCE;
            if(accepts(lo)) return lo;
            for(int i = 0; i < 16; ++i) {
                const double mid = (lo + hi) / 2;
                if(accepts(mid)) hi = mid; else lo = mid;
            }
            return hi;
        };
        const bool now = std::find(immediate.begin(), immediate.end(), ratio) != immediate.end();
        const double gate = now ? 0 : FAR_HEIGHTS * height / FAR_STEP;
        Row row;
        row.ratio = ratio;
        row.usable = std::max(limit(0), gate);
        row.good = std::max(limit(.05) / STEP_FORWARD, gate);
        row.at = ratio <= DEFAULT_RATIO ? row.usable : row.good;
        plan.rows.push_back(row);
    }
    std::vector<Row> &rows = plan.rows;
    int reference = 0;
    for(int i = 1; i < (int)rows.size(); ++i) {
        const Row &a = rows[reference], &b = rows[i];
        if(b.usable < a.usable || (b.usable == a.usable && b.ratio > a.ratio)) reference = i;
    }
    double firstGood = rows[reference].good;
    for(Row &row : rows) if(row.ratio > rows[reference].ratio) {
        if(row.at > firstGood) row.at = row.good;
        firstGood = std::min(firstGood, row.good);
    }

    std::map<std::pair<int, double>, Score> scoreCache;
    const auto score = [&](int row, double distance) -> const Score & {
        const auto entry = std::make_pair(row, distance);
        auto it = scoreCache.find(entry);
        if(it == scoreCache.end())
            it = scoreCache.emplace(entry, Precision(s, o, rows[row].ratio, distance)).first;
        return it->second;
    };
    // Preserve the reference when the model is ambiguous. A preferred layout must improve both
    // mean and worst poses and keep the first green control step; visibility breaks precision ties.
    const auto choose = [&](int baseline, const std::vector<int> &eligible, double distance) {
        std::vector<int> alternatives;
        for(const int r : eligible)
            if(r != baseline && DistanceCeil(rows[r].good) <= DistanceCeil(rows[baseline].good))
                alternatives.push_back(r);
        if(alternatives.empty()) return baseline;
        const Score ref = score(baseline, distance);
        int best = baseline;
        double bestRegret = 1;
        for(const int row : alternatives) {
            const Score q = score(row, distance);
            bool all = true;
            for(int k = 0; k < 4; ++k) if(!(q.Bound(k) <= ref.Bound(k))) all = false;
            if(!std::isfinite(q.coarse) || !all) continue;
            double regret = -INF;
            for(int k = 0; k < 4; ++k) {
                const double r = ref.Bound(k);
                regret = std::max(regret, q.Bound(k) / (std::isfinite(r) && r > 0 ? r : 1));
            }
            if(regret < bestRegret || (regret == bestRegret && q.full > score(best, distance).full)) {
                best = row;
                bestRegret = regret;
            }
        }
        return best;
    };
    const double firstDistance = std::min(MAX_DISTANCE, DistanceCeil(rows[reference].usable));
    int baseline = reference;
    for(int i = 0; i < (int)rows.size(); ++i)
        if(rows[i].ratio > rows[baseline].ratio && rows[i].at <= firstDistance) baseline = i;
    std::vector<int> initial, all;
    for(int i = 0; i < (int)rows.size(); ++i) {
        all.push_back(i);
        if(rows[i].usable <= firstDistance) initial.push_back(i);
    }
    plan.first = choose(baseline, initial.empty() ? all : initial, firstDistance);
    int selected = plan.first;
    // Only wider successors; no dependence on the slider history.
    const int lastStep = (int)JsRound(MAX_DISTANCE / DISTANCE_STEP);
    for(int n = (int)JsRound(firstDistance / DISTANCE_STEP) + 1; n <= lastStep; ++n) {
        const double distance = n / (1 / DISTANCE_STEP);
        int base = selected;
        for(int i = 0; i < (int)rows.size(); ++i)
            if(rows[i].ratio > rows[base].ratio && rows[i].at <= distance) base = i;
        std::vector<int> eligible;
        for(int i = 0; i < (int)rows.size(); ++i)
            if(rows[i].ratio >= rows[selected].ratio && (i == selected || rows[i].at <= distance)) eligible.push_back(i);
        const int next = choose(base, eligible, distance);
        if(next != selected) {
            plan.steps.push_back({distance, next});
            selected = next;
        }
    }
    // First green distance of this same progression (the ideal-distance button finds the same value).
    for(int n = (int)JsRound(.3 / DISTANCE_STEP); !plan.green && n <= lastStep; ++n) {
        const double distance = n / (1 / DISTANCE_STEP);
        int row = plan.first;
        for(const Step &step : plan.steps) if(step.at <= distance) row = step.row;
        if(distance >= rows[row].good) plan.green = distance;
    }
    planKey = key;
    cachedPlan = std::move(plan);
    cached = true;
    return cachedPlan;
}

} // namespace


// Camera records of the firmware (lib/OpenFIRECamera) with the fields of view of
// boards/camera-optics.json, exactly as the WebApp build puts them in OpenFIREshared.js.
const std::vector<Camera> &Cameras()
{
    static const std::vector<Camera> cameras = {
        {0, "DFRobot / Wii Cam",  33,    23,    0,    true,  1024.0, 768.0,  128.0, 96.0, 0.006,     0.0,      1.2, 0.8, 3.375983255123055},
        {1, "PixArt PAJ7025R2",   38.3,  38.3,  0.05, true,  4096.0, 4096.0, 98.0,  98.0, 0.0,       0.0,      1.2, 0.8, 2.8796037665217264},
        {2, "PixArt PAJ7025R3",   111.3, 111.3, 0.05, false, 4096.0, 4096.0, 98.0,  98.0, -0.128946, 0.155087, 1.5, 0.8, 0.7012987012987013},
    };
    return cameras;
}

const Camera *CameraByModel(int model)
{
    for(const Camera &c : Cameras()) if(c.model == model) return &c;
    return nullptr;
}

bool Optics::operator==(const Optics &o) const
{
    return model == o.model && horizontal == o.horizontal && vertical == o.vertical && tolerance == o.tolerance &&
           addOnLens == o.addOnLens && coordinateX == o.coordinateX && coordinateY == o.coordinateY &&
           sensorX == o.sensorX && sensorY == o.sensorY && k1 == o.k1 && k2 == o.k2 && max == o.max && min == o.min &&
           focal == o.focal && estimated == o.estimated && refined == o.refined && tx == o.tx && ty == o.ty &&
           sideGain == o.sideGain;
}

double DistanceCeil(double value)
{
    return std::ceil(value * (1 / DISTANCE_STEP)) / (1 / DISTANCE_STEP);
}

Dimensions ScreenSize(double inches, double aspect)
{
    return {inches * .0254 * aspect / std::sqrt(1 + aspect * aspect), inches * .0254 / std::sqrt(1 + aspect * aspect)};
}

Settings Clamp(const Settings &input)
{
    const auto finite = [](double v, double a, double b, double fallback) { return std::isfinite(v) ? Clampd(v, a, b) : fallback; };
    Settings s;
    s.inches = finite(input.inches, 20, 120, 55);
    s.aspect = finite(input.aspect, 1, 4, 16.0 / 9.0);
    s.distance = finite(input.distance, .3, MAX_DISTANCE, 2);
    s.diamond = input.diamond;
    return s;
}

bool MakeOptics(const Camera &record, double factor, const Custom *custom, Optics &out)
{
    if(!(record.horizontal > 0 && record.vertical > 0 && record.focal > 0)) return false;   // missing camera optics
    Optics o;
    static_cast<Camera &>(o) = record;
    o.estimated = factor != 1;
    o.refined = false;
    if(custom && factor == 1) {
        const double k1 = custom->k1 && std::isfinite(*custom->k1) ? *custom->k1 : record.k1;
        const double k2 = custom->k2 && std::isfinite(*custom->k2) ? *custom->k2 : record.k2;
        if((custom->k1 && std::isfinite(*custom->k1)) || (custom->k2 && std::isfinite(*custom->k2))) {
            double curveMin = 0, curveMax = 0;
            if(LensFit::CurveRange(k1, k2, curveMin, curveMax)) {
                o.k1 = k1;
                o.k2 = k2;
                o.min = std::min(o.min, curveMin);
                o.max = std::max(o.max, curveMax);
                o.refined = true;
            }
        }
        if(custom->focal && std::isfinite(*custom->focal) && *custom->focal >= record.focal * .5 && *custom->focal <= record.focal * 2) {
            o.focal = *custom->focal;
            o.refined = true;
        }
        const double nativeGain = 1 + record.k1 + record.k2, gain = 1 + o.k1 + o.k2;
        const double scale = gain / nativeGain * record.focal / o.focal;
        if(!(scale > 0)) return false;   // invalid refined optics
        o.horizontal = 2 * std::atan(std::tan(record.horizontal * R / 2) * scale) / R;
        o.vertical = 2 * std::atan(std::tan(record.vertical * R / 2) * scale) / R;
    }
    if(factor != 1) {
        if(!(factor >= .3 && factor <= 1)) return false;   // invalid converter
        o.horizontal = 2 * std::atan(std::tan(record.horizontal * R / 2) / factor) / R;
        o.vertical = 2 * std::atan(std::tan(record.vertical * R / 2) / factor) / R;
        o.focal *= factor;
        o.k1 = o.k2 = 0;   // ideal converter model; the factor describes the combined field of view
    }
    o.tx = std::tan(o.horizontal * (1 - o.tolerance) * R / 2);
    o.ty = std::tan(o.vertical * (1 - o.tolerance) * R / 2);
    o.sideGain = 1 + o.k1 + o.k2;
    out = o;
    return true;
}

std::vector<double> Candidates(const Settings &s, double distance)
{
    if(s.diamond) return {s.aspect};
    const bool far = distance * FAR_STEP >= FAR_HEIGHTS * ScreenSize(s.inches, s.aspect).height;
    const double list[] = {DEFAULT_RATIO, .8, .85, 1.3, 1.4, 1.5, 1.6, 1.68, s.aspect};
    std::vector<double> result;
    for(const double r : list) {
        if(!(r <= s.aspect && (r <= .85 || r >= 1.5 || (r == s.aspect && r >= 1.3) || (far && (r == 1.3 || r == 1.4)))))
            continue;
        if(std::find(result.begin(), result.end(), r) != result.end()) continue;   // first occurrence only
        result.push_back(r);
    }
    return result;
}

bool Calculate(const Settings &input, const Camera &record, double factor, const Custom *custom,
               bool withDistance, Result &out)
{
    const Settings s = Clamp(input);
    Optics o;
    if(!MakeOptics(record, factor, custom, o)) return false;
    const Dimensions dim = ScreenSize(s.inches, s.aspect);
    const Plan &plan = DistancePlan(s, o);
    // A red result keeps the next useful placement. The colour and distance labels still refer
    // to the actual input and to this same displayed rectangle.
    int chosen = plan.first;
    for(const Step &step : plan.steps) if(step.at <= s.distance) chosen = step.row;
    const Row &row = plan.rows[chosen];
    Result r;
    r.ratio = row.ratio;
    r.status = s.distance < row.usable ? StatusClose : s.distance < row.good ? StatusLimited : StatusGood;
    r.width = dim.width;
    r.height = dim.height;
    r.settings = s;
    r.optics = o;
    r.base = s.diamond ? dim.width : row.ratio * dim.height;
    // Green: the displayed layout's own limit. Otherwise the first green distance: farther back
    // the progression may already show wider LEDs.
    if(withDistance)
        r.minDistance = r.status == StatusGood ? std::optional<double>(DistanceCeil(row.good)) : plan.green;
    if(r.status == StatusClose && row.usable <= MAX_DISTANCE)
        r.moveTo = DistanceCeil(row.usable);
    out = r;
    return true;
}

} // namespace EmitterLayout
