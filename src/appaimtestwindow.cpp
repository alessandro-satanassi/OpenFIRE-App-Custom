/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Fullscreen window of the aim test: port of the WebApp's js/app/aim-test-window.js.

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

#include "appaimtestwindow.h"

#include <QCloseEvent>
#include <QCoreApplication>
#include <QKeyEvent>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cstring>
#include <limits>
#include <utility>

namespace {

constexpr int TRIGGER = 0;                  // boardInputs_e btnTrigger
constexpr qreal TEST_W = 1920;              // sTestCoords space of the aim point
constexpr qreal TEST_H = 1080;
constexpr int COORDS_LENGTH = 12 * 4;
constexpr qint64 COORDS_MAX_AGE = 500;      // ms without sTestCoords: the camera does not see the emitters
constexpr qint64 BLOBS_MAX_AGE = 250;       // sTestBlobs come right before sTestCoords (appcali.cpp IRTEST_BLOBS_*)
constexpr uint8_t BLOBS_VERSION = 1;
constexpr int BLOBS_LENGTH = 21;
constexpr uint8_t BLOB_SEEN = 0x01;
constexpr uint8_t BLOB_TRACKED = 0x04;      // flag (all entries): the tracker updated the aim this frame
constexpr qint64 FLASH_TIME = 120;          // crosshair flash on a shot
constexpr qint64 SHOT_TIME = 2000;          // hit marks fade out in this time
constexpr qint64 SHOT_RING_TIME = 250;      // ring that opens on the shot
constexpr int SHOT_MARKS = 12;
constexpr qint64 BUTTON_TIME = 400;         // circle with the name of a button
constexpr int FADE_INTERVAL = 40;           // ms between frames while something fades
constexpr int GRID_LINES = 20;              // lines every twentieth of the screen, major every other one
constexpr qreal TARGET_INSET = 0.05;        // targets 5% in from the edges

const QColor COLOR_BACKGROUND(14, 14, 18);
const QColor COLOR_GRID_MINOR(38, 38, 48);  // 1 px
const QColor COLOR_GRID_MAJOR(70, 70, 86);  // 1.5 px, every tenth of the screen
const QColor COLOR_BORDER(110, 110, 130);   // 2 px
const QColor COLOR_TARGET(150, 150, 170);   // 2 px
const QColor COLOR_HINTS(200, 200, 200, 120);
const QColor COLOR_MESSAGE(220, 220, 220);
const QColor COLOR_AIM(255, 70, 70);
const QColor COLOR_AIM_PARTIAL(255, 165, 0); // 2-3 emitters seen: the aim is estimated
const QColor COLOR_EDGE(255, 165, 0);

// Names of the buttons (OpenFIREshared.h boardInputs_Strings), for the translations.
[[maybe_unused]] const char *const BUTTON_NAMES[] = {
    QT_TRANSLATE_NOOP("AppAimTestWindow", "Trigger"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "Button A"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "Button B"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "Button C"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "Start"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "Select"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "D-Pad Up"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "D-Pad Down"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "D-Pad Left"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "D-Pad Right"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "Pedal"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "Alt Pedal"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "Pump Action"),
    QT_TRANSLATE_NOOP("AppAimTestWindow", "Home Button"),
};

QColor Alpha(QColor color, const qreal &alpha)
{
    color.setAlphaF(std::clamp<qreal>(alpha, 0, 1));
    return color;
}

QFont Font(const qreal &pixels, const bool &bold = false)
{
    QFont font(QStringLiteral("Segoe UI"));
    font.setStyleHint(QFont::SansSerif);
    font.setPixelSize(std::max(1, qRound(pixels)));
    font.setBold(bold);
    return font;
}

/// @brief  Text centred on x with its top at y (canvas textAlign 'center', textBaseline 'top')
void TextTopCentre(QPainter &painter, const qreal &x, const qreal &y, const QString &text)
{
    const QFontMetricsF fm(painter.font());
    painter.drawText(QPointF(x - fm.horizontalAdvance(text) / 2, y + fm.ascent()), text);
}

void Line(QPainter &painter, const qreal &x0, const qreal &y0, const qreal &x1, const qreal &y1)
{
    painter.drawLine(QPointF(x0, y0), QPointF(x1, y1));
}

void Circle(QPainter &painter, const qreal &x, const qreal &y, const qreal &r)
{
    painter.drawEllipse(QPointF(x, y), r, r);
}

QPen Pen(const QColor &color, const qreal &width)
{
    QPen pen(color, width);
    pen.setCapStyle(Qt::FlatCap);
    return pen;
}

} // namespace

AppAimTestWindow::AppAimTestWindow(QWidget *parent)
    : QWidget(parent)
{
    setWindowTitle(tr("Aim Test"));
    setFocusPolicy(Qt::StrongFocus);
    setContextMenuPolicy(Qt::PreventContextMenu);
    setAutoFillBackground(false);
    setAttribute(Qt::WA_OpaquePaintEvent);
    setCursor(Qt::BlankCursor);   // the crosshair follows the gun, not the mouse (as the WebApp)
    clock.start();

    redraw.setSingleShot(true);
    connect(&redraw, &QTimer::timeout, this, QOverload<>::of(&QWidget::update));
}

void AppAimTestWindow::Exit()
{
    if(exiting)
        return;
    exiting = true;
    redraw.stop();
    emit WindowExiting();
}

void AppAimTestWindow::closeEvent(QCloseEvent *event)
{
    // As AppLensFitWindow: the window then closes itself, through WindowExiting (the main window
    // ends the IR test first). Any close that does not come from there leaves like ESC.
    if(!exiting) {
        event->ignore();
        QMetaObject::invokeMethod(this, [this]() { Exit(); }, Qt::QueuedConnection);
        return;
    }
    QWidget::closeEvent(event);
}

void AppAimTestWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier)) {
        QWidget::keyPressEvent(event);
        return;
    }
    switch(event->key()) {
    case Qt::Key_Escape:
        Exit();
        break;
    case Qt::Key_H:
        showHints = !showHints;
        update();
        break;
    case Qt::Key_F:
        feedback = !feedback;
        showHints = true;   // the hints come back to show the new state
        update();
        break;
    default:
        QWidget::keyPressEvent(event);
        return;
    }
    event->accept();
}

// ----- Board events --------------------------------------------------------------------

void AppAimTestWindow::TestModeDraw(const QByteArray &payload)
{
    if(exiting || payload.size() != COORDS_LENGTH)
        return;
    int32_t coords[12];
    memcpy(coords, payload.constData(), sizeof(coords));
    const qint64 now = clock.elapsed();
    const bool blobsFresh = hasBlobs && now - blobs.time <= BLOBS_MAX_AGE;
    aim.x = coords[8] / TEST_W;
    aim.y = coords[9] / TEST_H;
    aim.time = now;
    aim.seen = blobsFresh ? blobs.seen : -1;
    aim.tracked = blobsFresh ? int(blobs.tracked) : -1;
    hasAim = true;
    update();
}

void AppAimTestWindow::TestModeBlobs(const QByteArray &payload)
{
    if(exiting || payload.size() < BLOBS_LENGTH || (uint8_t)payload.at(0) != BLOBS_VERSION)
        return;
    int seen = 0;
    for(int i = 0; i < 4; ++i)
        seen += ((uint8_t)payload.at(1 + i * 5) & BLOB_SEEN) != 0;
    blobs.seen = seen;
    blobs.tracked = ((uint8_t)payload.at(1) & BLOB_TRACKED) != 0;
    blobs.time = clock.elapsed();
    hasBlobs = true;
}

void AppAimTestWindow::ButtonPressed(const int &index, const QString &name)
{
    if(exiting)
        return;
    const qint64 now = clock.elapsed();
    if(index == TRIGGER) {
        flash = now;
        if(AimFresh(now)) {
            shots.append({aim.x, aim.y, now});
            if(shots.size() > SHOT_MARKS)
                shots.removeFirst();
        }
        update();
        if(feedback)
            emit TriggerFired();
    } else {
        buttonName = QCoreApplication::translate("AppAimTestWindow", name.toUtf8().constData());
        buttonTime = now;
        update();
    }
}

bool AppAimTestWindow::AimFresh(const qint64 &now) const
{
    return hasAim && now - aim.time <= COORDS_MAX_AGE && aim.tracked != 0;
}

// ----- Drawing -------------------------------------------------------------------------

void AppAimTestWindow::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    const qreal w = width();
    const qreal h = height();
    const qreal unit = std::max<qreal>(8, h / 90); // the text and the targets follow the screen height
    const qint64 now = clock.elapsed();
    const bool fresh = AimFresh(now);

    // The grid never changes: drawn once for this size, then copied on every frame.
    const qreal dpr = devicePixelRatioF();
    const QSize gridSize(qRound(w * dpr), qRound(h * dpr));
    if(grid.size() != gridSize) {
        grid = QPixmap(gridSize);
        grid.setDevicePixelRatio(dpr);
        QPainter gridPainter(&grid);
        gridPainter.setRenderHint(QPainter::Antialiasing);
        DrawGrid(gridPainter, w, h, unit);
    }
    painter.drawPixmap(0, 0, grid);

    shots.erase(std::remove_if(shots.begin(), shots.end(),
                               [now](const Shot_s &shot) { return now - shot.time >= SHOT_TIME; }), shots.end());
    for(const Shot_s &shot : std::as_const(shots))
        DrawShot(painter, shot.x * w, shot.y * h, unit, now - shot.time);

    if(showHints) {
        painter.setFont(Font(unit * 1.15));
        painter.setPen(COLOR_HINTS);
        TextTopCentre(painter, w / 2, unit * 0.6, QStringLiteral("Esc %1    H %2    F %3")
                      .arg(tr("quit"), tr("hide this line"), feedback ? tr("feedback on") : tr("feedback off")));
    }
    if(!fresh) {
        // Emitters seen but the tracker not updated (after a dark moment): the aim is not usable yet.
        const bool seen = hasAim && now - aim.time <= COORDS_MAX_AGE && aim.seen > 0;
        painter.setFont(Font(unit * 1.5, true));
        painter.setPen(COLOR_MESSAGE);
        TextTopCentre(painter, w / 2, h * 0.40, seen ? tr("Aim unavailable") : tr("IR emitters not seen"));
    }
    if(hasAim)
        DrawAim(painter, w, h, unit, now, fresh);

    // Redraw every 40 ms only while something fades (hit marks, flash, button circle);
    // otherwise once, when the aim gets old. New coordinates redraw anyway (TestModeDraw).
    const bool fading = !shots.isEmpty() || now < flash + FLASH_TIME || now < buttonTime + BUTTON_TIME;
    qint64 delay = fading ? FADE_INTERVAL : std::numeric_limits<qint64>::max();
    if(fresh)
        delay = std::min(delay, aim.time + COORDS_MAX_AGE + 1 - now);
    if(delay != std::numeric_limits<qint64>::max() && !exiting)
        redraw.start(int(delay));
    else
        redraw.stop();
}

void AppAimTestWindow::DrawGrid(QPainter &painter, const qreal &w, const qreal &h, const qreal &unit)
{
    painter.fillRect(QRectF(0, 0, w, h), COLOR_BACKGROUND);
    for(int i = 1; i < GRID_LINES; ++i) {
        const bool major = i % 2 == 0;
        painter.setPen(Pen(major ? COLOR_GRID_MAJOR : COLOR_GRID_MINOR, major ? 1.5 : 1));
        Line(painter, w * i / GRID_LINES, 0, w * i / GRID_LINES, h);
        Line(painter, 0, h * i / GRID_LINES, w, h * i / GRID_LINES);
    }
    painter.setBrush(Qt::NoBrush);
    painter.setPen(Pen(COLOR_BORDER, 2));
    painter.drawRect(QRectF(1, 1, w - 3, h - 3));

    const qreal r = unit * 2.2;
    painter.setPen(Pen(COLOR_TARGET, 2));
    for(const qreal fx : {TARGET_INSET, 0.5, 1 - TARGET_INSET}) {
        for(const qreal fy : {TARGET_INSET, 0.5, 1 - TARGET_INSET}) {
            const qreal cx = w * fx;
            const qreal cy = h * fy;
            Circle(painter, cx, cy, r);
            Circle(painter, cx, cy, r / 3);
            Line(painter, cx - r * 1.5, cy, cx + r * 1.5, cy);
            Line(painter, cx, cy - r * 1.5, cx, cy + r * 1.5);
        }
    }
}

void AppAimTestWindow::DrawShot(QPainter &painter, const qreal &x, const qreal &y, const qreal &unit, const qint64 &age)
{
    // A ring that opens on the shot, then a dot that fades out.
    if(age < SHOT_RING_TIME) {
        const qreal k = qreal(age) / SHOT_RING_TIME;
        painter.setBrush(Qt::NoBrush);
        painter.setPen(Pen(Alpha(Qt::white, 0.8 * (1 - k)), 2));
        Circle(painter, x, y, unit * (1 + 4 * k));
    }
    painter.setPen(Qt::NoPen);
    painter.setBrush(Alpha(COLOR_AIM, 0.9 * (1 - qreal(age) / SHOT_TIME)));
    Circle(painter, x, y, unit * 0.45);
}

void AppAimTestWindow::DrawAim(QPainter &painter, const qreal &w, const qreal &h, const qreal &unit, const qint64 &now, const bool &fresh)
{
    // Crosshair (white flash on a shot), the name of the last button, OFF-SCREEN at the edge.
    const bool outside = aim.x < 0 || aim.x > 1 || aim.y < 0 || aim.y > 1;
    const qreal cx = std::clamp<qreal>(aim.x, 0, 1) * (w - 1);
    const qreal cy = std::clamp<qreal>(aim.y, 0, 1) * (h - 1);
    const qreal r = unit * 2.8;

    if(now - buttonTime < BUTTON_TIME) {
        const qreal fade = 1 - qreal(now - buttonTime) / BUTTON_TIME;
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(80, 150, 255, qRound(255 * 0.35 * fade)));
        Circle(painter, cx, cy, r * 2.6);
        painter.setFont(Font(unit * 1.4, true));
        painter.setPen(Alpha(Qt::white, fade));
        TextTopCentre(painter, cx, std::min(cy + r * 2.0, h - unit * 2), buttonName);
    }

    const bool flashing = now - flash < FLASH_TIME;
    painter.setOpacity(fresh ? 1 : 0.35); // aim lost: the last point stays, dimmed
    // Shape of the IR camera tester crosshair (ring 24.42, ticks 18.83-30, centre cross 3.8)
    // scaled to the same footprint as the WebApp (1.8 r).
    const qreal k = r * 1.8 / 30;
    const qreal ring = 24.42 * k;
    const qreal tick0 = 18.83 * k;
    const qreal tick1 = 30 * k;
    const qreal dot = 3.8 * k;
    const bool partial = aim.seen >= 0 && aim.seen < 4;
    const QColor color = flashing ? QColor(Qt::white) : partial ? COLOR_AIM_PARTIAL : COLOR_AIM;
    QPainterPath ticks;
    const qreal lines[6][4] = {{-tick1, 0, -tick0, 0}, {tick0, 0, tick1, 0}, {0, -tick1, 0, -tick0},
                               {0, tick0, 0, tick1}, {-dot, 0, dot, 0}, {0, -dot, 0, dot}};
    for(const auto &l : lines) {
        ticks.moveTo(cx + l[0], cy + l[1]);
        ticks.lineTo(cx + l[2], cy + l[3]);
    }
    painter.setBrush(Qt::NoBrush);
    const struct { QColor stroke; qreal ringWidth; qreal lineWidth; } passes[2] = {
        {QColor(0, 0, 0, 160), 6, 5},   // shadow
        {color, 3, 2},
    };
    for(const auto &pass : passes) {
        painter.setPen(Pen(pass.stroke, pass.ringWidth));
        Circle(painter, cx, cy, ring);
        painter.setPen(Pen(pass.stroke, pass.lineWidth));
        painter.drawPath(ticks);
    }
    if(flashing) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(Alpha(COLOR_AIM, 0.6));
        Circle(painter, cx, cy, r * 0.6);
    }
    if(outside && fresh) {
        painter.setFont(Font(unit * 1.3, true));
        const QString text = tr("OFF-SCREEN");
        const qreal half = QFontMetricsF(painter.font()).horizontalAdvance(text) / 2;
        const qreal tx = std::clamp(cx, half + 4, std::max(half + 4, w - half - 4));
        const qreal ty = std::clamp(cy - r * 2.2 - unit * 1.6, qreal(4), std::max<qreal>(4, h - unit * 1.6 - 4));
        painter.setPen(COLOR_EDGE);
        TextTopCentre(painter, tx, ty, text);
    }
    painter.setOpacity(1);
}
