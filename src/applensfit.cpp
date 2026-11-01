/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Lens refinement of the fitted camera: port of the WebApp's js/core/lensfit.js.

    The lens model is the firmware's (OpenFIRE_Perspective::applyLensCorrection):
        nx = (x - cx) / cx, ny = (y - cy) / cy, q = nx^2 + ny^2
        corrected = c + (p - c) * (1 + K1 q + K2 q^2)        (clamped to the record's limits)
    The firmware does not know the camera: it sends its coordinate space and the camera record
    values (sLensInfo), and only reads the refined values back from settings.

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

#include "applensfit.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>

namespace LensFit {

namespace {

// Firmware OF_Const::lensValueUnset: all ones (a NaN) = the camera record's value.
constexpr uint32_t LENS_UNSET = 0xFFFFFFFF;
constexpr double PI = 3.14159265358979323846;
constexpr double INF = std::numeric_limits<double>::infinity();
const double NaN = std::numeric_limits<double>::quiet_NaN();

using Vec = std::vector<double>;
using Mat = std::vector<Vec>;
using All = std::array<double, 7>;       // [k1, k2, focalScale, a, s, kx, ky]
using Pose = std::array<double, 6>;      // rotation vector, translation
using Free = std::array<bool, 7>;

constexpr int G_K1 = 0, G_K2 = 1, G_F = 2;

/// @brief  Thrown at the points where the WebApp gives the page back (cancel requested)
struct Cancelled {};

struct Ctx {
    Control *control = nullptr;
    int stage = 0;
    int stages = 1;
    void Yield() const {
        if(control != nullptr && control->cancel.load())
            throw Cancelled();
    }
    void Done() {
        if(control != nullptr)
            control->progress.store(std::min(1.0, double(++stage) / stages));
    }
};

int32_t ReadInt32(const QByteArray &payload, const int &offset)
{
    // Little endian on the wire, as every value of the serial protocol.
    const uint8_t *b = reinterpret_cast<const uint8_t*>(payload.constData() + offset);
    const uint32_t v = uint32_t(b[0]) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16) | (uint32_t(b[3]) << 24);
    int32_t s;
    memcpy(&s, &v, sizeof(s));
    return s;
}

double ReadFloat32(const QByteArray &payload, const int &offset)
{
    const uint32_t v = (uint32_t)ReadInt32(payload, offset);
    float f;
    memcpy(&f, &v, sizeof(f));
    return f;
}

// Same check as the firmware (OpenFIRECamera::LensCurveValid): over q = 0..2 the factor
// 1 + K1 q + K2 q^2 stays positive and the corrected radius grows with the radius,
// 1 + 3 K1 q + 5 K2 q^2 > 0. Both are parabolas: their extremes are at the ends or at the
// vertex -b/(2c) when inside, so the check is exact. The values are checked as the firmware
// stores them (float32), with a small margin so that the firmware never refuses a curve accepted here.
constexpr double CURVE_MARGIN = 1e-3;

void ParabolaRange(const double &b, const double &c, double &min, double &max)
{
    const double end = 1 + 2 * b + 4 * c;
    min = std::min(1.0, end);
    max = std::max(1.0, end);
    if(c != 0) {
        const double q = -b / (2 * c);
        if(q > 0 && q < 2) {
            const double v = 1 + b * q + c * q * q;
            min = std::min(min, v);
            max = std::max(max, v);
        }
    }
}

// ----- Small linear algebra -------------------------------------------------------------

std::optional<Vec> SolveLinear(const Mat &A, const Vec &b)
{
    // Gaussian elimination with partial pivoting; A is n x n (rows).
    const size_t n = b.size();
    Mat M(n, Vec(n + 1));
    for(size_t i = 0; i < n; ++i) {
        for(size_t k = 0; k < n; ++k) M[i][k] = A[i][k];
        M[i][n] = b[i];
    }
    for(size_t c = 0; c < n; ++c) {
        size_t p = c;
        for(size_t r = c + 1; r < n; ++r) if(std::fabs(M[r][c]) > std::fabs(M[p][c])) p = r;
        if(!(std::fabs(M[p][c]) > 1e-300)) return std::nullopt;
        if(p != c) std::swap(M[p], M[c]);
        for(size_t r = c + 1; r < n; ++r) {
            const double f = M[r][c] / M[c][c];
            if(f == 0) continue;
            for(size_t k = c; k <= n; ++k) M[r][k] -= f * M[c][k];
        }
    }
    Vec x(n, 0);
    for(size_t r = n; r-- > 0;) {
        double s = M[r][n];
        for(size_t k = r + 1; k < n; ++k) s -= M[r][k] * x[k];
        x[r] = s / M[r][r];
    }
    for(const double &v : x) if(!std::isfinite(v)) return std::nullopt;
    return x;
}

std::optional<Mat> Invert(const Mat &A)
{
    // Gauss-Jordan with partial pivoting, one pass for all columns.
    const size_t n = A.size();
    Mat M(n, Vec(n));
    Mat I(n, Vec(n, 0));
    for(size_t r = 0; r < n; ++r) {
        for(size_t k = 0; k < n; ++k) M[r][k] = A[r][k];
        I[r][r] = 1;
    }
    for(size_t c = 0; c < n; ++c) {
        size_t p = c;
        for(size_t r = c + 1; r < n; ++r) if(std::fabs(M[r][c]) > std::fabs(M[p][c])) p = r;
        if(!(std::fabs(M[p][c]) > 1e-300)) return std::nullopt;
        if(p != c) { std::swap(M[p], M[c]); std::swap(I[p], I[c]); }
        for(size_t r = c + 1; r < n; ++r) {
            const double f = M[r][c] / M[c][c];
            if(f == 0) continue;
            for(size_t k = c; k < n; ++k) M[r][k] -= f * M[c][k];
            for(size_t k = 0; k < n; ++k) I[r][k] -= f * I[c][k];
        }
    }
    for(size_t r = n; r-- > 0;) {
        for(size_t k = r + 1; k < n; ++k) {
            const double f = M[r][k];
            if(f) for(size_t m = 0; m < n; ++m) I[r][m] -= f * I[k][m];
        }
        const double d = M[r][r];
        for(size_t m = 0; m < n; ++m) I[r][m] /= d;
    }
    for(const Vec &row : I) for(const double &v : row) if(!std::isfinite(v)) return std::nullopt;
    return I;
}

Mat3 Mul3(const Mat3 &A, const Mat3 &B)
{
    Mat3 C{};
    for(int i = 0; i < 3; ++i) for(int j = 0; j < 3; ++j)
        C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
    return C;
}

std::optional<Mat3> Inv3(const Mat3 &m)
{
    const double a = m[0][0], b = m[0][1], c = m[0][2];
    const double d = m[1][0], e = m[1][1], f = m[1][2];
    const double g = m[2][0], h = m[2][1], i = m[2][2];
    const double A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    const double det = a * A + b * B + c * C;
    if(!(std::fabs(det) > 1e-300)) return std::nullopt;
    const double s = 1 / det;
    return Mat3{{{A * s, -(b * i - c * h) * s, (b * f - c * e) * s},
                 {B * s, (a * i - c * g) * s, -(a * f - c * d) * s},
                 {C * s, -(a * h - b * g) * s, (a * e - b * d) * s}}};
}

/// @brief  Unit square (0,0) (1,0) (1,1) (0,1) -> quad q[0..3] (perimeter order). Heckbert.
std::optional<Mat3> SquareToQuad(const Frame &q)
{
    const double x0 = q[0][0], y0 = q[0][1], x1 = q[1][0], y1 = q[1][1];
    const double x2 = q[2][0], y2 = q[2][1], x3 = q[3][0], y3 = q[3][1];
    const double sx = x0 - x1 + x2 - x3, sy = y0 - y1 + y2 - y3;
    const double dx1 = x1 - x2, dx2 = x3 - x2, dy1 = y1 - y2, dy2 = y3 - y2;
    const double den = dx1 * dy2 - dx2 * dy1;
    if(!(std::fabs(den) > 1e-300)) return std::nullopt;
    const double g = (sx * dy2 - dx2 * sy) / den, h = (dx1 * sy - sx * dy1) / den;
    return Mat3{{{x1 - x0 + g * x1, x3 - x0 + h * x3, x0},
                 {y1 - y0 + g * y1, y3 - y0 + h * y3, y0},
                 {g, h, 1}}};
}

Mat3 Rotation(const Pose &v)
{
    // Rodrigues: rotation vector -> matrix.
    const double th = std::hypot(v[0], v[1], v[2]);
    if(th < 1e-12) return Mat3{{{1, -v[2], v[1]}, {v[2], 1, -v[0]}, {-v[1], v[0], 1}}};
    const double k0 = v[0] / th, k1 = v[1] / th, k2 = v[2] / th;
    const double c = std::cos(th), s = std::sin(th), t = 1 - c;
    return Mat3{{{c + k0 * k0 * t, k0 * k1 * t - k2 * s, k0 * k2 * t + k1 * s},
                 {k1 * k0 * t + k2 * s, c + k1 * k1 * t, k1 * k2 * t - k0 * s},
                 {k2 * k0 * t - k1 * s, k2 * k1 * t + k0 * s, c + k2 * k2 * t}}};
}

std::array<double, 3> RotationVector(const Mat3 &R)
{
    const double c = std::max(-1.0, std::min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2));
    const double th = std::acos(c);
    const double w[3] = {R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]};
    if(th < 1e-9) return {w[0] / 2, w[1] / 2, w[2] / 2};
    if(PI - th < 1e-6) {
        // Near 180 degrees: axis from the diagonal.
        double ax[3] = {std::sqrt(std::max(0.0, (R[0][0] + 1) / 2)), std::sqrt(std::max(0.0, (R[1][1] + 1) / 2)),
                        std::sqrt(std::max(0.0, (R[2][2] + 1) / 2))};
        if(R[0][1] < 0) ax[1] = -ax[1];
        if(R[0][2] < 0) ax[2] = -ax[2];
        return {ax[0] * th, ax[1] * th, ax[2] * th};
    }
    const double s = th / (2 * std::sin(th));
    return {w[0] * s, w[1] * s, w[2] * s};
}

std::optional<Mat3> Orthonormalise(const Mat3 &R)
{
    // Polar decomposition by averaging with the inverse transpose (converges in a few steps).
    Mat3 M = R;
    for(int it = 0; it < 30; ++it) {
        const std::optional<Mat3> inv = Inv3(M);
        if(!inv) return std::nullopt;
        Mat3 next{};
        double d = 0;
        for(int i = 0; i < 3; ++i) for(int j = 0; j < 3; ++j) {
            next[i][j] = 0.5 * (M[i][j] + (*inv)[j][i]);
            d = std::max(d, std::fabs(next[i][j] - M[i][j]));
        }
        M = next;
        if(d < 1e-12) break;
    }
    return M;
}

double SumSquares(const double *r, const size_t &n)
{
    double s = 0;
    for(size_t i = 0; i < n; ++i) s += r[i] * r[i];
    return s;
}

// ----- Model --------------------------------------------------------------------------

constexpr int SQUARE_PERIMETER[4] = {0, 1, 3, 2};    // Square tracker order TL, TR, BL, BR
constexpr int DIAMOND_PERIMETER[4] = {0, 1, 2, 3};   // Diamond: top, right, bottom, left
constexpr double UNIT[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};

struct Model {
    double cx, cy, f0;
    Lens record;

    explicit Model(const Info &info)
        : cx(info.resX / 2.0), cy(info.resY / 2.0), f0(info.resX / 2.0 * info.record.focal), record(info.record) {}

    /// @brief  Lens-corrected point (unclamped)
    Point Correct(const Point &p, const double &k1, const double &k2) const {
        const double dx = p[0] - cx, dy = p[1] - cy;
        const double nx = dx / cx, ny = dy / cy, q = nx * nx + ny * ny;
        const double g = 1 + k1 * q + k2 * q * q;
        return {cx + dx * g, cy + dy * g};
    }

    Frame CorrectFrame(const Frame &f, const double &k1, const double &k2) const {
        Frame q;
        for(int j = 0; j < 4; ++j) q[j] = Correct(f[j], k1, k2);
        return q;
    }

    Mat3 Kinv(const double &focalScale) const {
        const double f = f0 * focalScale;
        return Mat3{{{1 / f, 0, -cx / f}, {0, 1 / f, -cy / f}, {0, 0, 1}}};
    }
};

Mat3 ShapeMatrix(const All &all)
{
    // LED frame: unit square -> plane, gauge fixed by vertex 0 at the origin and vertex 1 on +x.
    return Mat3{{{1, all[4], 0}, {0, all[3], 0}, {all[5], all[6], 1}}};
}

std::array<Point, 4> ShapePoints(const All &all)
{
    const Mat3 A = ShapeMatrix(all);
    std::array<Point, 4> out;
    for(int j = 0; j < 4; ++j) {
        const double u = UNIT[j][0], v = UNIT[j][1];
        const double w = A[2][0] * u + A[2][1] * v + 1;
        out[j] = {(A[0][0] * u + A[0][1] * v) / w, (A[1][1] * v) / w};
    }
    return out;
}

All Expand(const Free &free, const Vec &values, const All &fixed)
{
    All all = fixed;
    size_t n = 0;
    for(int i = 0; i < 7; ++i) if(free[i]) all[i] = values[n++];
    return all;
}

std::optional<Vec> GeometricResiduals(const Model &model, const std::vector<Frame> &frames, const All &all)
{
    const Mat3 kinv = model.Kinv(all[G_F]);
    const std::optional<Mat3> ainv = Inv3(ShapeMatrix(all));
    if(!ainv) return std::nullopt;
    Vec out(frames.size() * 2);
    for(size_t i = 0; i < frames.size(); ++i) {
        const std::optional<Mat3> G = SquareToQuad(model.CorrectFrame(frames[i], all[G_K1], all[G_K2]));
        if(!G) return std::nullopt;
        const Mat3 M = Mul3(Mul3(kinv, *G), *ainv);
        const double m1[3] = {M[0][0], M[1][0], M[2][0]}, m2[3] = {M[0][1], M[1][1], M[2][1]};
        const double n1 = std::hypot(m1[0], m1[1], m1[2]), n2 = std::hypot(m2[0], m2[1], m2[2]);
        out[2 * i] = (m1[0] * m2[0] + m1[1] * m2[1] + m1[2] * m2[2]) / (n1 * n2);
        out[2 * i + 1] = (n1 - n2) / (n1 + n2);
    }
    return out;
}

struct LmResult {
    Vec x;
    double cost;
};

/// @brief  Dense Levenberg-Marquardt with forward-difference Jacobian (few parameters)
std::optional<LmResult> Levenberg(const std::function<std::optional<Vec>(const Vec &)> &fn,
                                  const Vec &x0, const Vec &steps, const int &iterations, const Ctx &ctx)
{
    Vec x = x0;
    std::optional<Vec> r = fn(x);
    if(!r) return std::nullopt;
    double cost = SumSquares(r->data(), r->size());
    double lambda = 1e-3;
    const size_t n = x.size();
    for(int it = 0; it < iterations; ++it) {
        ctx.Yield();
        std::vector<Vec> J;
        for(size_t j = 0; j < n; ++j) {
            Vec xp = x;
            xp[j] += steps[j];
            const std::optional<Vec> rp = fn(xp);
            if(!rp) return LmResult{x, cost};
            Vec col(r->size());
            for(size_t k = 0; k < r->size(); ++k) col[k] = ((*rp)[k] - (*r)[k]) / steps[j];
            J.push_back(std::move(col));
        }
        Mat H(n, Vec(n, 0));
        Vec g(n, 0);
        for(size_t a = 0; a < n; ++a) {
            double s = 0;
            for(size_t k = 0; k < r->size(); ++k) s += J[a][k] * (*r)[k];
            g[a] = s;
            for(size_t b = 0; b <= a; ++b) {
                double h = 0;
                for(size_t k = 0; k < r->size(); ++k) h += J[a][k] * J[b][k];
                H[a][b] = h;
            }
        }
        for(size_t a = 0; a < n; ++a) for(size_t b = a + 1; b < n; ++b) H[a][b] = H[b][a];
        bool improved = false;
        for(int tries = 0; tries < 10 && !improved; ++tries) {
            Mat D = H;
            for(size_t a = 0; a < n; ++a) D[a][a] = H[a][a] * (1 + lambda) + 1e-30;
            Vec minusG(n);
            for(size_t a = 0; a < n; ++a) minusG[a] = -g[a];
            const std::optional<Vec> dx = SolveLinear(D, minusG);
            if(!dx) { lambda *= 10; continue; }
            Vec xn(n);
            for(size_t j = 0; j < n; ++j) xn[j] = x[j] + (*dx)[j];
            std::optional<Vec> rn = fn(xn);
            const double cn = rn ? SumSquares(rn->data(), rn->size()) : INF;
            if(cn < cost) {
                const bool done = (cost - cn) < 1e-12 * cost;
                x = xn; r = std::move(rn); cost = cn; lambda = std::max(lambda / 10, 1e-9); improved = true;
                if(done) return LmResult{x, cost};
            } else lambda *= 10;
        }
        if(!improved) break;
    }
    return LmResult{x, cost};
}

// ----- Initialisation -----------------------------------------------------------------

/// @brief  LED shape from the most central frame, taken as seen face on
std::array<double, 4> InitialShape(const Model &model, const std::vector<Frame> &frames, const double &k1, const double &k2)
{
    size_t best = 0;
    double bestD = INF;
    for(size_t i = 0; i < frames.size(); ++i) {
        const Frame &f = frames[i];
        const double mx = (f[0][0] + f[1][0] + f[2][0] + f[3][0]) / 4 - model.cx;
        const double my = (f[0][1] + f[1][1] + f[2][1] + f[3][1]) / 4 - model.cy;
        const double d = mx * mx + my * my;
        if(d < bestD) { bestD = d; best = i; }
    }
    const Mat3 kinv = model.Kinv(1);
    Frame pts;
    for(int j = 0; j < 4; ++j) {
        const Point c = model.Correct(frames[best][j], k1, k2);
        pts[j] = {kinv[0][0] * c[0] + kinv[0][2], kinv[1][1] * c[1] + kinv[1][2]};
    }
    // Similarity: vertex 0 at the origin, vertex 1 on +x at distance 1.
    const double ox = pts[0][0], oy = pts[0][1];
    const double ang = std::atan2(pts[1][1] - oy, pts[1][0] - ox);
    const double len = std::hypot(pts[1][0] - ox, pts[1][1] - oy);
    const double c = std::cos(-ang) / len, s = std::sin(-ang) / len;
    Frame w;
    for(int j = 0; j < 4; ++j) {
        const double x = pts[j][0], y = pts[j][1];
        w[j] = {(x - ox) * c - (y - oy) * s, (x - ox) * s + (y - oy) * c};
    }
    const std::optional<Mat3> A = SquareToQuad(w);
    if(!A) return {1, 0, 0, 0};
    const double sc = 1 / (*A)[0][0];
    return {(*A)[1][1] * sc, (*A)[0][1] * sc, (*A)[2][0], (*A)[2][1]};
}

/// @brief  Pose of each frame from its homography (rotation vector, translation)
std::vector<Pose> InitialPoses(const Model &model, const std::vector<Frame> &frames, const All &all)
{
    const Mat3 kinv = model.Kinv(all[G_F]);
    const std::optional<Mat3> ainv = Inv3(ShapeMatrix(all));
    // The WebApp stops the calculation with an error here (it then shows the failure).
    if(!ainv) throw std::runtime_error("shape");
    std::vector<Pose> out;
    out.reserve(frames.size());
    for(const Frame &f : frames) {
        const std::optional<Mat3> G = SquareToQuad(model.CorrectFrame(f, all[G_K1], all[G_K2]));
        if(!G) throw std::runtime_error("frame");
        Mat3 M = Mul3(Mul3(kinv, *G), *ainv);
        const double lam = std::hypot(M[0][0], M[1][0], M[2][0]);
        for(auto &row : M) for(double &v : row) v = v / lam;
        if(M[2][2] < 0) for(auto &row : M) for(double &v : row) v = -v;
        const double r1[3] = {M[0][0], M[1][0], M[2][0]}, r2[3] = {M[0][1], M[1][1], M[2][1]};
        const double r3[3] = {r1[1] * r2[2] - r1[2] * r2[1], r1[2] * r2[0] - r1[0] * r2[2], r1[0] * r2[1] - r1[1] * r2[0]};
        const std::optional<Mat3> R = Orthonormalise(Mat3{{{r1[0], r2[0], r3[0]}, {r1[1], r2[1], r3[1]}, {r1[2], r2[2], r3[2]}}});
        const std::array<double, 3> rv = RotationVector(R ? *R : Mat3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}});
        out.push_back({rv[0], rv[1], rv[2], M[0][2], M[1][2], M[2][2]});
    }
    return out;
}

// ----- Bundle adjustment ----------------------------------------------------------------

using Res8 = std::array<double, 8>;

// Residual of each vertex: corrected point minus projection, brought back to raw camera
// units through the inverse Jacobian of the correction (J = g I + d grad(g)^T), so that every
// point weighs as its own measurement noise, at the corners as at the centre.
Res8 FrameResiduals(const Model &model, const Frame &frame, const All &all, const std::array<Point, 4> &world, const Pose &pose)
{
    const Mat3 R = Rotation(pose);
    const double f = model.f0 * all[G_F];
    const double k1 = all[G_K1], k2 = all[G_K2];
    Res8 out{};
    for(int j = 0; j < 4; ++j) {
        const double X = world[j][0], Y = world[j][1];
        const double pc0 = R[0][0] * X + R[0][1] * Y + pose[3];
        const double pc1 = R[1][0] * X + R[1][1] * Y + pose[4];
        const double pc2 = R[2][0] * X + R[2][1] * Y + pose[5];
        const Point c = model.Correct(frame[j], k1, k2);
        const double rx = c[0] - (model.cx + f * pc0 / pc2);
        const double ry = c[1] - (model.cy + f * pc1 / pc2);
        const double dx = frame[j][0] - model.cx, dy = frame[j][1] - model.cy;
        const double nx = dx / model.cx, ny = dy / model.cy;
        const double q = nx * nx + ny * ny;
        const double g = 1 + k1 * q + k2 * q * q, dg = k1 + 2 * k2 * q;
        const double gx = dg * 2 * dx / (model.cx * model.cx), gy = dg * 2 * dy / (model.cy * model.cy);
        const double a = g + dx * gx, b = dx * gy, cc = dy * gx, d = g + dy * gy;
        const double det = a * d - b * cc;
        if(det > 1e-6) {
            out[2 * j] = (d * rx - b * ry) / det;
            out[2 * j + 1] = (a * ry - cc * rx) / det;
        } else {
            out[2 * j] = rx;
            out[2 * j + 1] = ry;
        }
    }
    return out;
}

struct Fit {
    All all{};
    std::vector<Pose> poses;
    double cost = 0, rms = 0, sigma2 = 0;
    std::optional<Mat> covariance;      // free globals
    std::vector<int> idx;
    Vec frameRms;
};

double Dot8(const Res8 &a, const Res8 &b)
{
    double s = 0;
    for(int k = 0; k < 8; ++k) s += a[k] * b[k];
    return s;
}

/// @brief  Levenberg-Marquardt on the globals (free mask) and the 6 pose values of each frame,
///         with the Schur complement: the normal equations reduce to the few global parameters.
std::optional<Fit> Bundle(const Model &model, const std::vector<Frame> &frames, const Free &free,
                          const All &start, const std::vector<Pose> &poses, const int &iterations, const Ctx &ctx)
{
    std::vector<int> idx;
    for(int i = 0; i < 7; ++i) if(free[i]) idx.push_back(i);
    const size_t P = idx.size(), N = frames.size();
    All all = start;
    std::vector<Pose> pose = poses;
    constexpr double gStep[7] = {1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-6, 1e-6};

    auto residuals = [&](const All &a, const std::vector<Pose> &ps) {
        const std::array<Point, 4> world = ShapePoints(a);
        std::vector<Res8> out;
        out.reserve(N);
        for(size_t i = 0; i < N; ++i) out.push_back(FrameResiduals(model, frames[i], a, world, ps[i]));
        return out;
    };
    auto costOf = [](const std::vector<Res8> &rs) {
        double s = 0;
        for(const Res8 &r : rs) s += SumSquares(r.data(), 8);
        return s;
    };
    std::vector<Res8> rs = residuals(all, pose);
    double cost = costOf(rs);
    if(!std::isfinite(cost)) return std::nullopt;
    double lambda = 1e-3;

    // Per-frame Jacobians: globals (8 x P) and pose (8 x 6).
    std::vector<std::vector<Res8>> Jg(N), Jp(N);
    auto linearise = [&]() {
        const std::array<Point, 4> world = ShapePoints(all);
        std::vector<All> shiftAll(P);
        std::vector<std::array<Point, 4>> shiftWorld(P);
        for(size_t n = 0; n < P; ++n) {
            const int g = idx[n];
            shiftAll[n] = all;
            shiftAll[n][g] += gStep[g];
            if(g >= 3) shiftWorld[n] = ShapePoints(shiftAll[n]);
        }
        for(size_t i = 0; i < N; ++i) {
            if((i & 63) == 63) ctx.Yield();
            const Res8 &r0 = rs[i];
            Jg[i].assign(P, Res8{});
            for(size_t n = 0; n < P; ++n) {
                const int g = idx[n];
                const Res8 rp = FrameResiduals(model, frames[i], shiftAll[n], g < 3 ? world : shiftWorld[n], pose[i]);
                for(int k = 0; k < 8; ++k) Jg[i][n][k] = (rp[k] - r0[k]) / gStep[g];
            }
            Jp[i].assign(6, Res8{});
            for(int k = 0; k < 6; ++k) {
                Pose p = pose[i];
                const double h = k < 3 ? 1e-6 : 1e-6 * std::max(1.0, std::fabs(p[5]));
                p[k] += h;
                const Res8 rp = FrameResiduals(model, frames[i], all, world, p);
                for(int m = 0; m < 8; ++m) Jp[i][k][m] = (rp[m] - r0[m]) / h;
            }
        }
    };

    using Mat6 = std::array<std::array<double, 6>, 6>;
    for(int it = 0; it < iterations; ++it) {
        ctx.Yield();
        linearise();
        // Blocks of the normal equations.
        Mat U(P, Vec(P, 0));
        Vec bg(P, 0);
        std::vector<Mat6> V(N);
        std::vector<Mat> W(N, Mat(P, Vec(6)));
        std::vector<std::array<double, 6>> bp(N);
        for(size_t i = 0; i < N; ++i) {
            const Res8 &r = rs[i];
            for(size_t a = 0; a < P; ++a) {
                bg[a] += Dot8(Jg[i][a], r);
                for(size_t b = 0; b <= a; ++b) U[a][b] += Dot8(Jg[i][a], Jg[i][b]);
            }
            for(int a = 0; a < 6; ++a) {
                bp[i][a] = Dot8(Jp[i][a], r);
                for(int b = 0; b <= a; ++b) V[i][a][b] = Dot8(Jp[i][a], Jp[i][b]);
            }
            for(int a = 0; a < 6; ++a) for(int b = a + 1; b < 6; ++b) V[i][a][b] = V[i][b][a];
            for(size_t a = 0; a < P; ++a) for(int b = 0; b < 6; ++b) W[i][a][b] = Dot8(Jg[i][a], Jp[i][b]);
        }
        for(size_t a = 0; a < P; ++a) for(size_t b = a + 1; b < P; ++b) U[a][b] = U[b][a];

        bool improved = false;
        for(int tries = 0; tries < 10 && !improved; ++tries) {
            if(tries) ctx.Yield();
            std::vector<Mat> Vinv(N);
            bool singular = false;
            for(size_t i = 0; i < N && !singular; ++i) {
                Mat d(6, Vec(6));
                for(int a = 0; a < 6; ++a) for(int b = 0; b < 6; ++b)
                    d[a][b] = a == b ? V[i][a][b] * (1 + lambda) + 1e-20 : V[i][a][b];
                std::optional<Mat> inv = Invert(d);
                if(!inv) singular = true;
                else Vinv[i] = std::move(*inv);
            }
            if(singular) { lambda *= 10; continue; }
            Mat Sd = U;
            for(size_t a = 0; a < P; ++a) Sd[a][a] = U[a][a] * (1 + lambda) + 1e-20;
            Vec rhs(P);
            for(size_t a = 0; a < P; ++a) rhs[a] = -bg[a];
            for(size_t i = 0; i < N; ++i) {
                // Y = W Vinv (P x 6)
                Mat Y(P, Vec(6));
                for(size_t a = 0; a < P; ++a) for(int c = 0; c < 6; ++c) {
                    double s = 0;
                    for(int k = 0; k < 6; ++k) s += W[i][a][k] * Vinv[i][k][c];
                    Y[a][c] = s;
                }
                for(size_t a = 0; a < P; ++a) {
                    for(size_t b = 0; b < P; ++b) {
                        double s = 0;
                        for(int k = 0; k < 6; ++k) s += Y[a][k] * W[i][b][k];
                        Sd[a][b] -= s;
                    }
                    double s = 0;
                    for(int k = 0; k < 6; ++k) s += Y[a][k] * bp[i][k];
                    rhs[a] += s;
                }
            }
            std::optional<Vec> dg = P ? SolveLinear(Sd, rhs) : std::optional<Vec>(Vec());
            if(!dg) { lambda *= 10; continue; }
            All allN = all;
            for(size_t n = 0; n < P; ++n) allN[idx[n]] += (*dg)[n];
            std::vector<Pose> poseN(N);
            for(size_t i = 0; i < N; ++i) {
                double t[6];
                for(int k = 0; k < 6; ++k) {
                    double s = 0;
                    for(size_t a = 0; a < P; ++a) s += W[i][a][k] * (*dg)[a];
                    t[k] = -bp[i][k] - s;
                }
                for(int r = 0; r < 6; ++r) {
                    double s = 0;
                    for(int k = 0; k < 6; ++k) s += Vinv[i][r][k] * t[k];
                    poseN[i][r] = pose[i][r] + s;
                }
            }
            std::vector<Res8> rsN = residuals(allN, poseN);
            const double costN = costOf(rsN);
            if(costN < cost) {
                const bool small = (cost - costN) < 1e-10 * cost;
                all = allN; pose = std::move(poseN); rs = std::move(rsN); cost = costN;
                lambda = std::max(lambda / 10, 1e-12); improved = true;
                if(small) it = iterations;
            } else lambda *= 10;
        }
        if(!improved) break;
    }

    // Marginal covariance of the free globals: sigma^2 * (U - W V^-1 W^T)^-1, undamped.
    const long long dof = (long long)N * 8 - ((long long)P + (long long)N * 6);
    const double sigma2 = dof > 0 ? cost / dof : NaN;
    Fit fit;
    if(P) {
        linearise();
        Mat S(P, Vec(P, 0));
        bool singular = false;
        for(size_t i = 0; i < N; ++i) {
            Mat v(6, Vec(6));
            for(int a = 0; a < 6; ++a) for(int b = 0; b < 6; ++b) v[a][b] = Dot8(Jp[i][a], Jp[i][b]);
            const std::optional<Mat> vinv = Invert(v);
            if(!vinv) singular = true;
            Mat w(P, Vec(6));
            for(size_t a = 0; a < P; ++a) for(int b = 0; b < 6; ++b) w[a][b] = Dot8(Jg[i][a], Jp[i][b]);
            for(size_t a = 0; a < P; ++a) for(size_t b = 0; b < P; ++b) {
                double s = Dot8(Jg[i][a], Jg[i][b]);
                if(vinv) for(int k = 0; k < 6; ++k) for(int m = 0; m < 6; ++m) s -= w[a][k] * (*vinv)[k][m] * w[b][m];
                S[a][b] += s;
            }
        }
        // A pose that cannot be separated from the lens, or a covariance that is not a valid
        // one, gives no uncertainty at all: the tier is then refused (prudent fallback).
        const std::optional<Mat> Sinv = singular ? std::nullopt : Invert(S);
        bool valid = !!Sinv;
        if(valid) for(size_t a = 0; a < P && valid; ++a) {
            if(!((*Sinv)[a][a] > 0)) valid = false;
            for(const double &x : (*Sinv)[a]) if(!std::isfinite(x)) valid = false;
        }
        if(valid) {
            Mat cov = *Sinv;
            for(Vec &row : cov) for(double &x : row) x = x * sigma2;
            fit.covariance = std::move(cov);
        }
    }
    fit.frameRms.resize(N);
    for(size_t i = 0; i < N; ++i) fit.frameRms[i] = std::sqrt(SumSquares(rs[i].data(), 8) / 8);
    fit.all = all;
    fit.poses = std::move(pose);
    fit.cost = cost;
    fit.rms = std::sqrt(cost / (double(N) * 8));
    fit.sigma2 = sigma2;
    fit.idx = idx;
    return fit;
}

// ----- Solve ----------------------------------------------------------------------------

constexpr std::array<std::array<bool, 3>, TierCount> TIER_FREE = {{
    {false, false, false},      // record
    {true, false, false},       // k1
    {true, true, false},        // k1k2
    {true, true, true},         // k1k2focal
}};

int TierParams(const int &t)
{
    return int(TIER_FREE[t][0]) + int(TIER_FREE[t][1]) + int(TIER_FREE[t][2]);
}

// A tier is kept over the simpler one only with a clear gain (in units of the noise) and
// values well determined over the part of the field the LEDs actually covered.
constexpr double SIGNIFICANCE = 25;         // cost decrease / sigma^2 per added parameter
constexpr double MAX_GAIN_STD = 0.004;      // 0.4% standard deviation of the correction factor
constexpr double MAX_FOCAL_STD = 0.01;      // 1% standard deviation of the focal ratio

struct Base {
    double k1, k2, f;
};

std::optional<Fit> FitTier(const Model &model, const std::vector<Frame> &frames, const int &tier,
                           const std::array<double, 4> &shapeGuess, const Base *base, const Ctx &ctx)
{
    // base: the lens values kept for what the tier does not refine (record by default).
    const Base b = base ? *base : Base{model.record.k1, model.record.k2, 1};
    const Free free7 = {TIER_FREE[tier][0], TIER_FREE[tier][1], TIER_FREE[tier][2], true, true, true, true};
    const All fixed = {b.k1, b.k2, b.f, shapeGuess[0], shapeGuess[1], shapeGuess[2], shapeGuess[3]};
    constexpr double stepsAll[7] = {1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-6, 1e-6};
    Vec x0, steps;
    for(int i = 0; i < 7; ++i) if(free7[i]) { x0.push_back(fixed[i]); steps.push_back(stepsAll[i]); }
    const std::optional<LmResult> geo = Levenberg(
        [&](const Vec &x) { return GeometricResiduals(model, frames, Expand(free7, x, fixed)); },
        x0, steps, 60, ctx);
    if(!geo) return std::nullopt;
    const All start = Expand(free7, geo->x, fixed);
    const std::vector<Pose> poses = InitialPoses(model, frames, start);
    return Bundle(model, frames, free7, start, poses, 40, ctx);
}

/// @brief  Cost of frames seen through fixed globals (lens and LED shape): only the poses are fitted
double HeldOutCost(const Model &model, const std::vector<Frame> &frames, const All &all, const Ctx &ctx)
{
    const std::optional<Fit> fit = Bundle(model, frames, Free{}, all, InitialPoses(model, frames, all), 20, ctx);
    return fit && std::isfinite(fit->cost) ? fit->cost : INF;
}

double GainStd(const Fit &fit, const double &qObs)
{
    // Largest standard deviation of 1 + K1 q + K2 q^2 over the covered radii (0..qObs):
    // K1 and K2 can compensate each other at one radius and not at another.
    if(!fit.covariance) return INF;
    const auto find = [&](const int &g) {
        const auto it = std::find(fit.idx.begin(), fit.idx.end(), g);
        return it == fit.idx.end() ? -1 : int(it - fit.idx.begin());
    };
    const int iK1 = find(G_K1), iK2 = find(G_K2);
    const size_t n = fit.idx.size();
    double worst = 0;
    for(int s = 1; s <= 20; ++s) {
        const double q = qObs * s / 20;
        Vec grad(n, 0);
        if(iK1 >= 0) grad[iK1] = q;
        if(iK2 >= 0) grad[iK2] = q * q;
        double v = 0;
        for(size_t a = 0; a < n; ++a) for(size_t b = 0; b < n; ++b) v += grad[a] * (*fit.covariance)[a][b] * grad[b];
        if(!(v >= 0) || !std::isfinite(v)) return INF;   // not a valid covariance
        worst = std::max(worst, v);
    }
    return std::sqrt(worst);
}

/// @brief  Lens values in use differ from the record (refined earlier)
bool IsCustom(const Info &info)
{
    const Lens &a = info.active, &r = info.record;
    const auto differs = [](const double &x, const double &y) {
        return std::isfinite(x) && std::isfinite(y) && std::fabs(x - y) > 1e-6 * std::max(1.0, std::fabs(y));
    };
    return differs(a.k1, r.k1) || differs(a.k2, r.k2) || differs(a.focal, r.focal);
}

Values TierValues(const int &tier, const All &all, const Lens &rec)
{
    Values v;
    if(TIER_FREE[tier][G_K1]) v.k1 = all[G_K1];
    if(TIER_FREE[tier][G_K2]) v.k2 = all[G_K2];
    if(TIER_FREE[tier][G_F]) v.focal = rec.focal * all[G_F];
    return v;
}

Result SolveSteps(const Info &info, const std::vector<Frame> &framesIn, Ctx &ctx)
{
    Result result;
    const Model model(info);
    const Lens &rec = info.record;
    if(!(rec.focal > 0)) { result.reason = "focal"; return result; }
    if(framesIn.size() < 40) { result.reason = "frames"; return result; }
    std::vector<Frame> frames = framesIn;
    const bool custom = IsCustom(info);
    ctx.stages = 7 + (custom ? 1 : 0);

    // Outliers (a reflection taken for an LED): drop frames far from the first fit.
    const std::array<double, 4> shape0 = InitialShape(model, frames, rec.k1, rec.k2);
    const std::optional<Fit> first = FitTier(model, frames, TierK1K2, shape0, nullptr, ctx);
    ctx.Done();
    if(first) {
        Vec sorted = first->frameRms;
        std::sort(sorted.begin(), sorted.end());
        const double median = sorted[sorted.size() >> 1];
        std::vector<Frame> keep;
        for(size_t i = 0; i < frames.size(); ++i)
            if(first->frameRms[i] <= std::max(4 * median, 1e-9)) keep.push_back(frames[i]);
        if(keep.size() >= 40) frames = std::move(keep);
    }

    double qObs = 0;
    {
        Vec qs;
        qs.reserve(frames.size() * 4);
        for(const Frame &f : frames) for(const Point &p : f) {
            const double nx = (p[0] - model.cx) / model.cx, ny = (p[1] - model.cy) / model.cy;
            qs.push_back(nx * nx + ny * ny);
        }
        std::sort(qs.begin(), qs.end());
        qObs = qs[size_t(std::floor(qs.size() * 0.95))];
    }

    const std::array<double, 4> shape = InitialShape(model, frames, rec.k1, rec.k2);
    std::array<std::optional<Fit>, TierCount> fits;
    for(int t = 0; t < TierCount; ++t) { fits[t] = FitTier(model, frames, t, shape, nullptr, ctx); ctx.Done(); }
    if(!fits[0]) { result.reason = "fit"; return result; }

    // Noise level from the richest model that converged; every tier is compared with the
    // record values: gain in units of the noise, minus a price for each refined value.
    double sigma2 = NaN;
    for(int t = TierCount - 1; t >= 0; --t)
        if(fits[t] && std::isfinite(fits[t]->sigma2) && fits[t]->sigma2 > 0) { sigma2 = fits[t]->sigma2; break; }
    struct Scored { int t; double score; };
    std::vector<Scored> candidates;
    std::vector<Scored> determined;     // valid and well determined, whether significant or not
    for(int t = 1; t < TierCount; ++t) {
        const std::optional<Fit> &fit = fits[t];
        if(!fit || !std::isfinite(fit->cost) || !(sigma2 > 0)) continue;
        const int params = TierParams(t);
        const All &values = fit->all;
        const bool valid = CurveValid(values[G_K1], values[G_K2]);
        const double gStd = GainStd(*fit, qObs);
        const auto itF = std::find(fit->idx.begin(), fit->idx.end(), G_F);
        const int iF = itF == fit->idx.end() ? -1 : int(itF - fit->idx.begin());
        const double fVar = iF >= 0 && fit->covariance ? (*fit->covariance)[iF][iF] : 0;
        const double fStd = fVar >= 0 && std::isfinite(fVar) ? std::sqrt(fVar) : INF;
        const bool focalOk = iF < 0 || (fStd < MAX_FOCAL_STD && values[G_F] > 0.5 && values[G_F] < 2);
        const double score = (fits[0]->cost - fit->cost) / sigma2 - SIGNIFICANCE * params;
        if(valid && gStd < MAX_GAIN_STD && focalOk) determined.push_back({t, score});
        if(valid && gStd < MAX_GAIN_STD && focalOk && score > 0) candidates.push_back({t, score});
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const Scored &a, const Scored &b) { return a.score > b.score; });

    // Check on views not used for the fit: the frames are split in two by where the LEDs
    // fall on the camera (fine checkerboard, so both halves cover the whole field and the
    // nearly equal consecutive views stay together). A refined tier fitted on one half must
    // also see the other half better than the record values fitted on the same half.
    int chosen = 0;
    int validated = -1;
    if(!candidates.empty()) {
        std::vector<Frame> half[2];
        for(const Frame &f : frames) {
            const double mx = (f[0][0] + f[1][0] + f[2][0] + f[3][0]) / 4, my = (f[0][1] + f[1][1] + f[2][1] + f[3][1]) / 4;
            const long long cell = (long long)std::floor(mx / (2 * model.cx) * 8) + (long long)std::floor(my / (2 * model.cy) * 8);
            half[cell & 1].push_back(f);
        }
        if(half[0].size() >= 40 && half[1].size() >= 40) {
            const bool firstBigger = half[0].size() >= half[1].size();
            const std::vector<Frame> &fitSet = firstBigger ? half[0] : half[1];
            const std::vector<Frame> &testSet = firstBigger ? half[1] : half[0];
            const std::array<double, 4> shapeA = InitialShape(model, fitSet, rec.k1, rec.k2);
            const std::optional<Fit> recordA = FitTier(model, fitSet, TierRecord, shapeA, nullptr, ctx);
            const double recordCost = recordA ? HeldOutCost(model, testSet, recordA->all, ctx) : INF;
            ctx.Done();
            validated = 0;
            for(const Scored &c : candidates) {
                const std::optional<Fit> fitA = FitTier(model, fitSet, c.t, shapeA, nullptr, ctx);
                const double cost = fitA ? HeldOutCost(model, testSet, fitA->all, ctx) : INF;
                if(cost < recordCost) { chosen = c.t; validated = 1; break; }
            }
            ctx.Done();
        } else chosen = candidates[0].t;   // too few views to split: statistical checks only
    }

    // Values already refined earlier: the new result must also beat them, otherwise they stay.
    const Fit &fit = *fits[chosen];
    std::optional<double> rmsActive;
    bool keepActive = false;
    if(custom) {
        const Lens &a = info.active;
        const Base base{a.k1, a.k2, a.focal > 0 ? a.focal / rec.focal : 1};
        const std::optional<Fit> fitActive = FitTier(model, frames, TierRecord, shape, &base, ctx);
        ctx.Done();
        if(fitActive && std::isfinite(fitActive->cost)) {
            rmsActive = fitActive->rms;
            // At least the gain asked of one parameter, also to go back to the record values:
            // views that cannot tell the two apart keep the values in use.
            const int params = std::max(1, TierParams(chosen));
            keepActive = !(sigma2 > 0) || (fitActive->cost - fit.cost) / sigma2 - SIGNIFICANCE * params <= 0;
        }
    }
    if(ctx.control != nullptr) ctx.control->progress.store(1);

    // The record values stay: the best tier that is valid and well determined but not
    // significant (or not confirmed on the other half) is still given, as values very close
    // to the record ones that the user may apply anyway.
    if(chosen == 0 && !determined.empty()) {
        Scored best = determined[0];
        for(const Scored &d : determined) if(d.score > best.score) best = d;
        result.hasNear = true;
        result.nearTier = best.t;
        result.near = TierValues(best.t, fits[best.t]->all, rec);
    }

    result.ok = true;
    result.tier = chosen;
    result.values = TierValues(chosen, fit.all, rec);
    result.rmsRecord = fits[0]->rms;
    result.rms = fit.rms;
    result.rmsActive = rmsActive;
    result.activeCustom = custom;
    result.keepActive = keepActive;
    result.validated = validated;
    result.qObs = qObs;
    result.frames = int(frames.size());
    result.dropped = int(framesIn.size() - frames.size());
    for(int t = 0; t < TierCount; ++t)
        if(fits[t]) result.fits[t] = {true, fits[t]->rms, fits[t]->all};
    return result;
}

} // namespace

// ----- Settings encoding and payloads ---------------------------------------------------

uint32_t ToBits(const std::optional<double> &value)
{
    if(!value || !std::isfinite(*value)) return LENS_UNSET;
    const float f = float(*value);
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    return bits;
}

std::optional<double> FromBits(const uint32_t &bits)
{
    if(bits == LENS_UNSET) return std::nullopt;
    float f;
    memcpy(&f, &bits, sizeof(f));
    if(!std::isfinite(f)) return std::nullopt;
    return double(f);
}

bool CurveValid(double k1, double k2)
{
    k1 = double(float(k1));
    k2 = double(float(k2));
    if(!(std::fabs(k1) <= 2) || !(std::fabs(k2) <= 2)) return false;
    double gainMin, gainMax, slopeMin, slopeMax;
    ParabolaRange(k1, k2, gainMin, gainMax);
    ParabolaRange(3 * k1, 5 * k2, slopeMin, slopeMax);
    return gainMin > CURVE_MARGIN && slopeMin > CURVE_MARGIN;
}

std::optional<Info> ParseInfo(const QByteArray &payload)
{
    // version, mouseResX/Y (int32), record then active K1, K2, max, min, focal (float32)
    if(payload.size() < 49 || (uint8_t)payload.at(0) != 1) return std::nullopt;
    const auto group = [&](const int &offset) {
        Lens l;
        l.k1 = ReadFloat32(payload, offset);
        l.k2 = ReadFloat32(payload, offset + 4);
        l.max = ReadFloat32(payload, offset + 8);
        l.min = ReadFloat32(payload, offset + 12);
        l.focal = ReadFloat32(payload, offset + 16);
        return l;
    };
    Info info;
    info.resX = ReadInt32(payload, 1);
    info.resY = ReadInt32(payload, 5);
    info.record = group(9);
    info.active = group(29);
    if(!(info.resX > 0) || !(info.resY > 0)) return std::nullopt;
    return info;
}

std::optional<Sample> ParseSample(const QByteArray &payload)
{
    // version, flags (bit 0 Diamond, bit 1 complete), 4 x (x, y) int32
    if(payload.size() < 34 || (uint8_t)payload.at(0) != 1) return std::nullopt;
    Sample s;
    const uint8_t flags = (uint8_t)payload.at(1);
    s.diamond = flags & 1;
    s.complete = flags & 2;
    for(int i = 0; i < 4; ++i)
        s.points[i] = {double(ReadInt32(payload, 2 + i * 8)), double(ReadInt32(payload, 6 + i * 8))};
    return s;
}

std::optional<Result> Solve(const Info &info, const std::vector<Frame> &frames, Control *control)
{
    Ctx ctx;
    ctx.control = control;
    try {
        return SolveSteps(info, frames, ctx);
    } catch(const Cancelled &) {
        return std::nullopt;
    } catch(const std::exception &error) {
        // As the WebApp: an error in the calculation is shown as a failure.
        Result result;
        result.reason = QString::fromUtf8(error.what());
        return result;
    }
}

// ----- Data collection ------------------------------------------------------------------

namespace {
int Sign(const double &v) { return v > 0 ? 1 : v < 0 ? -1 : 0; }
}

Collector::Collector(const Info &info)
    : info(info), cx(info.resX / 2.0), cy(info.resY / 2.0)
{
}

bool Collector::Add(const Sample &sample)
{
    const int *order = sample.diamond ? DIAMOND_PERIMETER : SQUARE_PERIMETER;
    Frame pts;
    for(int i = 0; i < 4; ++i) pts[i] = sample.points[order[i]];
    lastPoints = pts;
    hasLastPoints = true;
    lastComplete = sample.complete;
    // An incomplete view (strict checks not met: the tracker's own vertices) still moves
    // the aim and the tilt on screen, so that they follow the gun smoothly; it is never
    // kept for the fit.
    const double area = 0.5 * std::fabs((pts[2][0] - pts[0][0]) * (pts[3][1] - pts[1][1]) -
                                        (pts[3][0] - pts[1][0]) * (pts[2][1] - pts[0][1]));
    const double fieldArea = double(info.resX) * info.resY;
    if(!(area > fieldArea * 0.002)) return false;
    // Convex quad: the four turns have the same sign.
    int sign = 0;
    for(int i = 0; i < 4; ++i) {
        const Point &a = pts[i], &b = pts[(i + 1) & 3], &c = pts[(i + 2) & 3];
        const double z = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
        if(!z || (sign && Sign(z) != sign)) return false;
        sign = Sign(z);
    }
    // Aim: the image centre in the LED frame, as a screen fraction (uncalibrated, only to paint).
    const std::optional<Mat3> H = SquareToQuad(pts);
    const std::optional<Mat3> Hi = H ? Inv3(*H) : std::nullopt;
    if(!Hi) return false;
    const double w = (*Hi)[2][0] * cx + (*Hi)[2][1] * cy + (*Hi)[2][2];
    const double u = ((*Hi)[0][0] * cx + (*Hi)[0][1] * cy + (*Hi)[0][2]) / w;
    const double v = ((*Hi)[1][0] * cx + (*Hi)[1][1] * cy + (*Hi)[1][2]) / w;
    const double sx = sample.diamond ? 0.5 + 0.5 * (u - v) : u;
    const double sy = sample.diamond ? 0.5 * (u + v) : v;
    aim = {sx, sy};
    hasAim = true;
    {
        // Tilt of the gun (positive = clockwise seen from behind): the top edge of the LED
        // frame (Square), or left to right (Diamond). The camera image turns the other way,
        // unless the tracker mirrors it (the frame is then seen with the opposite winding).
        const Point &a = sample.diamond ? pts[3] : pts[0], &b = pts[1];
        const double imageRoll = std::atan2(b[1] - a[1], b[0] - a[0]);
        live = {pts, *Hi, sample.diamond, sign > 0 ? -imageRoll : imageRoll};
        hasLive = true;
    }
    if(!sample.complete) return false;

    // A gun held still adds the same view again: wait for a move of 1% of the field.
    const double scale = std::hypot(double(info.resX), double(info.resY)) * 0.01;
    if(hasLast) {
        bool still = true;
        for(int i = 0; i < 4 && still; ++i)
            if(!(std::hypot(pts[i][0] - last[i][0], pts[i][1] - last[i][1]) < scale)) still = false;
        if(still) return false;
    }

    const double size = std::sqrt(area / fieldArea);
    const int band = size < 0.25 ? 0 : size < 0.45 ? 1 : 2;
    const double colF = std::floor(sx * SCREEN_COLS), rowF = std::floor(sy * SCREEN_ROWS);
    const bool inside = colF >= 0 && colF < SCREEN_COLS && rowF >= 0 && rowF < SCREEN_ROWS;
    Meta m;
    m.cell = inside ? int(rowF) * SCREEN_COLS + int(colF) : -1;
    if(inside) m.key = (long long)m.cell * 3 + band;
    else {
        // Outside the screen: one bucket per side (as the WebApp's 'out' keys), NaN apart.
        const auto side = [](const double &s) {
            const double r = std::floor((s - 0.5) + 0.5);   // Math.round(s - 0.5)
            return std::isnan(r) ? 3 : Sign(r) + 1;
        };
        m.key = 1000 + (side(sx) * 4 + side(sy)) * 3 + band;
    }
    for(int j = 0; j < 4; ++j) m.cells[j] = Ring(pts[j]);
    const double roll = live.roll;
    m.roll = roll > ROLL_TARGET ? 1 : roll < -ROLL_TARGET ? -1 : 0;

    // Bounded collection that never gets stuck: when the screen cell (per size band) or
    // the total is full, a frame that adds something still missing replaces a redundant
    // one (whose removal leaves every count at its target), from the same cell when that
    // cell is full, otherwise from the most crowded one. A frame adding nothing is ignored.
    const auto bucket = buckets.find(m.key);
    const bool bucketFull = (bucket == buckets.end() ? 0 : bucket->second) >= CELL_CAP;
    const bool totalFull = int(frames.size()) >= MAX_FRAMES;
    if(bucketFull || totalFull) {
        if(!Adds(m)) return false;
        int victim = Redundant(bucketFull ? &m.key : nullptr);
        if(victim < 0 && bucketFull && totalFull) victim = Redundant(nullptr);
        if(victim >= 0) Remove(victim);
        else if(totalFull) return false;
    }
    last = pts;
    hasLast = true;
    Keep(pts, m);
    return true;
}

void Collector::Keep(const Frame &pts, const Meta &m)
{
    frames.push_back(pts);
    meta.push_back(m);
    Count(m, 1);
}

void Collector::Count(const Meta &m, const int &d)
{
    buckets[m.key] += d;
    if(m.cell >= 0) screen[m.cell] += d;
    for(const int &c : m.cells) field[c] += d;
    if(m.roll > 0) rollRight += d;
    if(m.roll < 0) rollLeft += d;
}

bool Collector::Adds(const Meta &m) const
{
    if(m.cell >= 0 && screen[m.cell] < SCREEN_TARGET) return true;
    for(const int &c : m.cells) if(field[c] < FIELD_TARGET) return true;
    return (m.roll > 0 && rollRight < ROLL_FRAMES) || (m.roll < 0 && rollLeft < ROLL_FRAMES);
}

int Collector::Redundant(const long long *onlyKey) const
{
    int best = -1, bestCount = -1;
    for(int i = 0; i < int(meta.size()); ++i) {
        const Meta &m = meta[i];
        if(onlyKey != nullptr && m.key != *onlyKey) continue;
        const auto bucket = buckets.find(m.key);
        const int count = bucket == buckets.end() ? 0 : bucket->second;
        if(count <= bestCount) continue;
        if(m.cell >= 0 && screen[m.cell] <= SCREEN_TARGET) continue;
        if(m.roll > 0 && rollRight <= ROLL_FRAMES) continue;
        if(m.roll < 0 && rollLeft <= ROLL_FRAMES) continue;
        bool needed = false;
        for(const int &c : m.cells) {
            const int same = int(std::count(m.cells.begin(), m.cells.end(), c));
            if(field[c] - same < FIELD_TARGET) { needed = true; break; }
        }
        if(needed) continue;
        best = i;
        bestCount = count;
    }
    return best;
}

void Collector::Remove(const int &i)
{
    Count(meta[i], -1);
    frames[i] = frames.back();
    meta[i] = meta.back();
    frames.pop_back();
    meta.pop_back();
}

Progress Collector::ProgressNow() const
{
    Progress p;
    double screenSum = 0, fieldSum = 0;
    for(const int &n : screen) screenSum += std::min(1.0, double(n) / SCREEN_TARGET);
    for(const int &n : field) fieldSum += std::min(1.0, double(n) / FIELD_TARGET);
    p.screen = screenSum / screen.size();
    p.field = fieldSum / field.size();
    p.rollLeft = std::min(1.0, double(rollLeft) / ROLL_FRAMES);
    p.rollRight = std::min(1.0, double(rollRight) / ROLL_FRAMES);
    p.roll = (p.rollLeft + p.rollRight) / 2;
    p.frames = std::min(1.0, double(frames.size()) / MIN_FRAMES);
    p.total = (p.screen + p.field + p.roll) / 3;
    p.done = p.screen >= 1 && p.field >= 1 && p.roll >= 1 && p.frames >= 1;
    p.count = int(frames.size());
    return p;
}

Guide Collector::GuideNow()
{
    Guide g;
    g.progress = ProgressNow();
    const Progress &p = g.progress;
    if(p.screen < 1 || p.frames < 1) { g.step = 1; return g; }
    if(p.field < 1) {
        // No reachable target: the outer rings need the gun closer, the inner ones farther.
        const std::optional<Target> target = FieldTarget();
        int missing = int(field.size()) - 1;
        while(missing >= 0 && field[missing] >= FIELD_TARGET) --missing;
        const bool outer = missing >= 2;
        g.step = 2;
        g.hasTarget = !!target;
        if(target) g.target = target->aim;
        g.closer = !target && outer;
        g.farther = !target && !outer;
        g.cell = target ? target->cell : -1;
        return g;
    }
    if(p.roll < 1) {
        g.step = 3;
        g.roll = hasLive ? live.roll : 0;
        g.left = p.rollLeft;
        g.right = p.rollRight;
        g.limit = ROLL_TARGET;
        return g;
    }
    g.step = 4;
    return g;
}

int Collector::Ring(const Point &p) const
{
    const double r = std::hypot((p[0] - cx) / cx, (p[1] - cy) / cy);
    int i = FIELD_RING_COUNT - 1;
    while(i > 0 && r < FIELD_RINGS[i]) --i;
    return i;
}

std::optional<Collector::Target> Collector::Reach(const int &ring, const int &dir) const
{
    // Moving the view by d moves every LED by d: the new aim is what now sits at centre - d.
    if(field[ring] >= FIELD_TARGET) return std::nullopt;
    const double margin = 0.02 * std::min(info.resX, info.resY);
    const Point aimNow = hasAim ? aim : Point{0.5, 0.5};
    const double r = ring + 1 < FIELD_RING_COUNT ? (FIELD_RINGS[ring] + FIELD_RINGS[ring + 1]) / 2 : FIELD_RINGS[ring] + 0.06;
    const double a = dir / 16.0 * 2 * PI;
    const double px = cx * (1 + r * std::cos(a)), py = cy * (1 + r * std::sin(a));
    std::optional<Target> best;
    for(const Point &led : live.pts) {
        const double dx = px - led[0], dy = py - led[1];
        bool fits = true;
        for(const Point &q : live.pts)
            if(!(q[0] + dx >= margin && q[0] + dx <= info.resX - margin &&
                 q[1] + dy >= margin && q[1] + dy <= info.resY - margin)) { fits = false; break; }
        if(!fits) continue;
        const Mat3 &Hi = live.Hi;
        const double x = cx - dx, y = cy - dy;
        const double w = Hi[2][0] * x + Hi[2][1] * y + Hi[2][2];
        const double u = (Hi[0][0] * x + Hi[0][1] * y + Hi[0][2]) / w, v = (Hi[1][0] * x + Hi[1][1] * y + Hi[1][2]) / w;
        const Point s = live.diamond ? Point{0.5 + 0.5 * (u - v), 0.5 * (u + v)} : Point{u, v};
        if(!(s[0] >= AIM_INSET && s[0] <= 1 - AIM_INSET && s[1] >= AIM_INSET && s[1] <= 1 - AIM_INSET)) continue;
        const double dist = std::hypot(s[0] - aimNow[0], s[1] - aimNow[1]);
        if(!best || dist < best->dist) best = Target{s, dist, ring * 16 + dir};
    }
    return best;
}

std::optional<Collector::Target> Collector::FieldTarget()
{
    // Screen point to aim at so that one LED lands in a camera field ring still missing.
    if(!hasLive) return std::nullopt;
    // Keep the same target while it is still missing and reachable, so it does not jump around.
    if(guideCell >= 0) {
        const std::optional<Target> kept = Reach(guideCell / 16, guideCell % 16);
        if(kept) return kept;
    }
    std::optional<Target> best;
    for(int ring = int(field.size()) - 1; ring >= 0 && !best; --ring)   // the outermost missing ring first
        for(int dir = 0; dir < 16; ++dir) {
            const std::optional<Target> c = Reach(ring, dir);
            if(c && (!best || c->dist < best->dist)) best = c;
        }
    guideCell = best ? best->cell : -1;
    return best;
}

} // namespace LensFit
