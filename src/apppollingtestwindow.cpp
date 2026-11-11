/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Window of the polling rate test: port of the WebApp's js/app/polling-test.js.

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

#include "apppollingtestwindow.h"

#include <QCloseEvent>
#include <QCoreApplication>
#include <QCursor>
#include <QFrame>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWindow>

#include <algorithm>
#include <cmath>

namespace {

constexpr int FRAME_MS = 16;            // about one display frame: graph and state
constexpr double CURRENT_MS = 150;      // the big number: lively but readable
constexpr double NUMBERS_MS = 250;      // the boxes: 4 times a second, not at every frame

enum StateKind_e { stateIdle = 0, stateOk, stateWarn };

/// Box of a number: a thin rounded border (coloured for the average).
QString TileStyle(const QColor &border)
{
    return QStringLiteral("QFrame#pollingTile { border: 1px solid %1; border-radius: 6px; }").arg(border.name());
}

QString Fixed(const std::optional<double> &value)
{
    return value && std::isfinite(*value) ? QString::number(std::lround(*value)) : QStringLiteral("—");
}

/// Graph of the rate over the last 10 s of movement (the samples of AppPollingStats).
class PollingChart : public QWidget
{
public:
    explicit PollingChart(AppPollingTestWindow *owner) : QWidget(owner), owner(owner)
    {
        setMinimumHeight(160);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        setAttribute(Qt::WA_OpaquePaintEvent, false);
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        const AppPollingTestWindow::Colors_s &colors = owner->Colors();
        const auto &samples = owner->Stats().Samples();
        const int count = (int)samples.size();

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        double top = 300;
        for(const auto &sample : samples)
            top = std::max(top, sample.hz);
        top *= 1.1;

        QFont font = painter.font();
        font.setPointSizeF(std::max(7.0, font.pointSizeF() * 0.8));
        painter.setFont(font);
        const int labelWidth = painter.fontMetrics().horizontalAdvance(QStringLiteral("0000")) + 8;

        const double left = labelWidth, right = 6, topPad = 6, bottom = 14;
        const double w = width() - left - right;
        const double h = height() - topPad - bottom;
        if(w <= 0 || h <= 0)
            return;
        auto y = [&](const double &hz) { return topPad + h - hz / top * h; };
        // Newest point on the right edge, one step per SAMPLE_MS of movement.
        auto x = [&](const double &index) {
            return left + w - (count - 1 - index) / (AppPollingStats::HISTORY_SAMPLES - 1) * w;
        };

        // Grid and its numbers.
        const int step = top > 1200 ? 500 : top > 600 ? 200 : 100;
        painter.setPen(QPen(colors.grid, 1));
        for(int hz = 0; hz <= top; hz += step) {
            const double py = std::round(y(hz)) + 0.5;
            painter.setPen(QPen(colors.grid, 1));
            painter.drawLine(QPointF(left, py), QPointF(width() - right, py));
            painter.setPen(colors.faint);
            painter.drawText(QRectF(0, py - 8, left - 4, 16), Qt::AlignRight | Qt::AlignVCenter, QString::number(hz));
        }

        // Reference: frame rate of the camera.
        QPen dashed(colors.faint, 1, Qt::CustomDashLine);
        dashed.setDashPattern({ 4, 4 });
        painter.setPen(dashed);
        const double camera = std::round(y(AppPollingStats::CAMERA_FPS)) + 0.5;
        painter.drawLine(QPointF(left, camera), QPointF(width() - right, camera));

        // Where the gun was still: a faint mark, the line goes on without the pause.
        QPen pause(colors.grid, 1, Qt::CustomDashLine);
        pause.setDashPattern({ 2, 3 });
        painter.setPen(pause);
        for(int index = 0; index < count; ++index) {
            if(!samples[index].resumed) continue;
            const double px = std::round(x(index - 0.5)) + 0.5;
            painter.drawLine(QPointF(px, topPad), QPointF(px, topPad + h));
        }

        // The line takes the colours of the average (green, orange, red) by its height: a
        // stretch that crosses a threshold (180 or 60 Hz) changes colour right where it
        // crosses it, whatever the scale of the graph.
        auto color = [&](const AppPollingStats::Level_e &level) {
            return level == AppPollingStats::levelWarn ? colors.mid : level == AppPollingStats::levelBad ? colors.bad : colors.ok;
        };
        AppPollingStats::Level_e current = AppPollingStats::levelNone;
        QPainterPath path;
        auto stroke = [&]() {
            if(current == AppPollingStats::levelNone) return;
            QPen pen(color(current), 2);
            pen.setJoinStyle(Qt::RoundJoin);
            pen.setCapStyle(Qt::RoundCap);
            painter.strokePath(path, pen);
        };
        for(int index = 1; index < count; ++index) {
            for(const auto &piece : AppPollingStats::SplitAtThresholds(index - 1, samples[index-1].hz, index, samples[index].hz)) {
                AppPollingStats::Level_e level = AppPollingStats::AverageLevel((piece.fromHz + piece.toHz) / 2);
                if(level == AppPollingStats::levelNone) level = AppPollingStats::levelOk;
                if(level != current) {
                    stroke();
                    current = level;
                    path = QPainterPath();
                    path.moveTo(x(piece.fromIndex), y(piece.fromHz));
                }
                path.lineTo(x(piece.toIndex), y(piece.toHz));
            }
        }
        stroke();
    }

private:
    AppPollingTestWindow *owner;
};

} // namespace

AppPollingTestWindow::AppPollingTestWindow(QWidget *parent)
    : QWidget(parent, Qt::Window)
{
    setWindowTitle(tr("Polling Rate Test"));
    setFocusPolicy(Qt::StrongFocus);

    // Colours of the WebApp (style.css), light or dark like the main window.
    const bool light = palette().window().color().value() > palette().text().color().value();
    if(light) {
        colors.ok = QColor("#1d8a3e");
        colors.mid = QColor("#e06c00");
        colors.bad = QColor("#d0263a");
        colors.chipOkText = QColor("#0a7fc2");
        colors.chipWarnText = QColor("#b27a00");
    } else {
        colors.ok = QColor("#4cc96f");
        colors.mid = QColor("#fb923c");
        colors.bad = QColor("#ff5a6a");
        colors.chipOkText = QColor("#2fa6e4");
        colors.chipWarnText = QColor("#e5b53a");
    }
    colors.text = palette().text().color();
    colors.faint = palette().placeholderText().color();
    colors.grid = palette().mid().color();
    colors.chip = palette().alternateBase().color();
    colors.chipText = palette().placeholderText().color();
    colors.chipOk = colors.chipOkText;
    colors.chipOk.setAlpha(40);

    // Header: title, Reset and Close (no focus: ESC and R always reach the window).
    QLabel *title = new QLabel(tr("Polling Rate Test"));
    QFont titleFont = title->font();
    titleFont.setPointSizeF(titleFont.pointSizeF() * 1.4);
    titleFont.setBold(true);
    title->setFont(titleFont);
    QPushButton *resetButton = new QPushButton(tr("Reset"));
    resetButton->setFocusPolicy(Qt::NoFocus);
    resetButton->setToolTip(QStringLiteral("R"));
    connect(resetButton, &QPushButton::clicked, this, &AppPollingTestWindow::Reset);
    QPushButton *closeButton = new QPushButton(tr("Close"));
    closeButton->setFocusPolicy(Qt::NoFocus);
    closeButton->setToolTip(QStringLiteral("Esc"));
    connect(closeButton, &QPushButton::clicked, this, &QWidget::close);
    QHBoxLayout *header = new QHBoxLayout;
    header->addWidget(title);
    header->addStretch(1);
    header->addWidget(resetButton);
    header->addWidget(closeButton);

    QLabel *hint = new QLabel(QStringLiteral("%1 <b>%2</b>")
                              .arg(tr("Aim the lightgun inside this window and keep moving it. The lightgun must be in Run mode (not connected to the App) with mouse output.").toHtmlEscaped(),
                                   tr("Expected value with OpenFIRE: about %1 Hz, the frame rate of the camera (with the USB cable and with the dongle).").arg(AppPollingStats::CAMERA_FPS).toHtmlEscaped()));
    hint->setWordWrap(true);
    hint->setTextFormat(Qt::RichText);

    // Raw Input decided once, before the test starts, so the layout never moves during the test.
    rawInput = raw.Start();
    QLabel *note = new QLabel(tr("On this system the mouse moves can reach the App already joined: the rate measured can be lower than the real one. On Windows every report is measured."));
    note->setWordWrap(true);
    note->setStyleSheet(QStringLiteral("color: %1").arg(colors.chipWarnText.name()));
    note->setVisible(!rawInput);

    // Big number and state.
    currentValue = new QLabel(QStringLiteral("—"));
    QFont bigFont = currentValue->font();
    bigFont.setPointSizeF(bigFont.pointSizeF() * 4.2);
    bigFont.setBold(true);
    currentValue->setFont(bigFont);
    currentValue->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    currentValue->setMinimumWidth(QFontMetrics(bigFont).horizontalAdvance(QStringLiteral("0000")));
    QLabel *currentUnit = new QLabel(QStringLiteral("Hz"));
    QFont unitFont = currentUnit->font();
    unitFont.setPointSizeF(unitFont.pointSizeF() * 1.5);
    unitFont.setBold(true);
    currentUnit->setFont(unitFont);
    currentUnit->setStyleSheet(QStringLiteral("color: %1").arg(colors.faint.name()));
    QHBoxLayout *bigRow = new QHBoxLayout;
    bigRow->addStretch(1);
    bigRow->addWidget(currentValue, 0, Qt::AlignBaseline);
    bigRow->addWidget(currentUnit, 0, Qt::AlignBaseline);
    bigRow->addStretch(1);

    state = new QLabel;
    state->setAlignment(Qt::AlignCenter);
    state->setWordWrap(true);
    // Always two lines high: the longer messages wrap, and the layout must not move.
    state->setFixedHeight(state->fontMetrics().lineSpacing() * 2 + 12);
    state->setFixedWidth(state->fontMetrics().averageCharWidth() * 34);

    QVBoxLayout *hero = new QVBoxLayout;
    hero->addLayout(bigRow);
    hero->addWidget(state, 0, Qt::AlignHCenter);

    QHBoxLayout *tiles = new QHBoxLayout;
    averageTile = Tile(tr("Average (last 3 s)"), averageValue);
    averageTile->setToolTip(tr("Rate of the last 3 s of movement: green from 180 Hz, orange from 60 Hz, red below. Indicative thresholds."));
    tiles->addWidget(averageTile, 1);
    tiles->addWidget(Tile(tr("Maximum (last 10 s)"), maxValue), 1);
    tiles->addWidget(Tile(tr("Minimum (last 10 s)"), minValue), 1);

    QHBoxLayout *top = new QHBoxLayout;
    top->addLayout(hero, 0);
    top->addSpacing(12);
    top->addLayout(tiles, 1);

    QLabel *caption = new QLabel(tr("Rate over the last %1 s of movement (line: %2 fps of the camera)")
                                 .arg(AppPollingStats::HISTORY_S).arg(AppPollingStats::CAMERA_FPS));
    chart = new PollingChart(this);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addLayout(header);
    layout->addWidget(hint);
    layout->addWidget(note);
    layout->addSpacing(6);
    layout->addLayout(top);
    layout->addSpacing(6);
    layout->addWidget(caption);
    layout->addWidget(chart, 1);

    setMinimumSize(700, 500);
    resize(980, 640);

    frame.setTimerType(Qt::PreciseTimer);
    frame.setInterval(FRAME_MS);
    connect(&frame, &QTimer::timeout, this, &AppPollingTestWindow::Tick);
    frame.start();

    if(!rawInput) {
        // The moves of Qt, one by one: no compression of the mouse moves while the test is open.
        compressBefore = QCoreApplication::testAttribute(Qt::AA_CompressHighFrequencyEvents);
        QCoreApplication::setAttribute(Qt::AA_CompressHighFrequencyEvents, false);
    }

    Render(AppPollingRawInput::NowMs(), true);
}

AppPollingTestWindow::~AppPollingTestWindow()
{
    frame.stop();
    raw.Stop();
}

QFrame *AppPollingTestWindow::Tile(const QString &label, QLabel *&value)
{
    QFrame *tile = new QFrame;
    tile->setObjectName(QStringLiteral("pollingTile"));
    tile->setStyleSheet(TileStyle(colors.grid));
    QLabel *name = new QLabel(label);
    name->setStyleSheet(QStringLiteral("color: %1").arg(colors.faint.name()));
    value = new QLabel(QStringLiteral("—"));
    QFont valueFont = value->font();
    valueFont.setPointSizeF(valueFont.pointSizeF() * 1.6);
    valueFont.setBold(true);
    value->setFont(valueFont);
    QLabel *unit = new QLabel(QStringLiteral("Hz"));
    unit->setStyleSheet(QStringLiteral("color: %1").arg(colors.faint.name()));
    QHBoxLayout *row = new QHBoxLayout;
    row->setSpacing(4);
    row->addWidget(value, 0, Qt::AlignBaseline);
    row->addWidget(unit, 0, Qt::AlignBaseline);
    row->addStretch(1);
    QVBoxLayout *layout = new QVBoxLayout(tile);
    layout->setContentsMargins(10, 6, 10, 6);
    layout->setSpacing(2);
    layout->addWidget(name);
    layout->addLayout(row);
    return tile;
}

void AppPollingTestWindow::SetAverageLevel(const AppPollingStats::Level_e &level)
{
    if(level == averageLevel)
        return;
    averageLevel = level;
    if(level == AppPollingStats::levelNone) {
        averageTile->setStyleSheet(TileStyle(colors.grid));
        averageValue->setStyleSheet(QString());
        return;
    }
    const QColor color = level == AppPollingStats::levelOk ? colors.ok : level == AppPollingStats::levelWarn ? colors.mid : colors.bad;
    averageTile->setStyleSheet(TileStyle(color));
    averageValue->setStyleSheet(QStringLiteral("color: %1").arg(color.name()));
}

void AppPollingTestWindow::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    if(!rawInput && !filtering && windowHandle()) {
        // Without Raw Input: every mouse move that reaches this window, before Qt hands it to a widget.
        windowHandle()->installEventFilter(this);
        filtering = true;
    }
    activateWindow();
    setFocus();
}

bool AppPollingTestWindow::eventFilter(QObject *watched, QEvent *event)
{
    if(event->type() == QEvent::MouseMove && watched == windowHandle() && isActiveWindow())
        stats.AddReport(AppPollingRawInput::NowMs());
    return QWidget::eventFilter(watched, event);
}

bool AppPollingTestWindow::Listening() const
{
    return isActiveWindow() && rect().contains(mapFromGlobal(QCursor::pos()));
}

void AppPollingTestWindow::Tick()
{
    if(rawInput) {
        times.clear();
        raw.Take(times);
        if(Listening())
            for(const double &time : times)
                stats.AddReport(time);
    }
    const double now = AppPollingRawInput::NowMs();
    stats.Sample(now);
    Render(now, false);
}

void AppPollingTestWindow::Reset()
{
    stats.Reset();
    Render(AppPollingRawInput::NowMs(), true);
}

void AppPollingTestWindow::Render(const double &now, const bool &force)
{
    const AppPollingStats::Snapshot_s s = stats.Snap(now);
    QString text;
    int kind;
    if(s.moving) {
        text = tr("Measuring");
        kind = stateOk;
    } else if(!isActiveWindow()) {
        text = tr("Click on this window to measure");
        kind = stateWarn;
    } else if(!s.reports) {
        text = tr("Waiting for the lightgun: move it inside the window");
        kind = stateIdle;
    } else if(!rect().contains(mapFromGlobal(QCursor::pos()))) {
        text = tr("Aim outside the window");
        kind = stateWarn;
    } else {
        text = tr("Paused: keep moving the lightgun");
        kind = stateWarn;
    }
    if(state->text() != text)
        state->setText(text);
    if(kind != stateKind) {
        stateKind = kind;
        const QColor background = kind == stateOk ? colors.chipOk : colors.chip;
        const QColor color = kind == stateOk ? colors.chipOkText : kind == stateWarn ? colors.chipWarnText : colors.chipText;
        state->setStyleSheet(QStringLiteral("QLabel { background: rgba(%1, %2, %3, %4); color: %5; border-radius: 10px; padding: 4px 12px; }")
                             .arg(background.red()).arg(background.green()).arg(background.blue()).arg(background.alpha())
                             .arg(color.name()));
    }

    // Numbers: a few times a second, or they flicker too fast to read; at once when the
    // measure starts or stops (the graph is drawn at every frame).
    const bool measuring = s.current.has_value();
    const bool changed = force || measuring != shownMeasuring;
    shownMeasuring = measuring;
    if(changed || now - currentAt >= CURRENT_MS) {
        currentAt = now;
        currentValue->setText(Fixed(s.current));
    }
    if(changed || now - numbersAt >= NUMBERS_MS) {
        numbersAt = now;
        RenderNumbers(s);
    }
    chart->update();
}

void AppPollingTestWindow::RenderNumbers(const AppPollingStats::Snapshot_s &s)
{
    averageValue->setText(Fixed(s.average));
    SetAverageLevel(AppPollingStats::AverageLevel(s.average));
    maxValue->setText(Fixed(s.max));
    minValue->setText(Fixed(s.min));
}

void AppPollingTestWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Escape) {
        close();
        return;
    }
    if(event->key() == Qt::Key_R && !(event->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier))) {
        Reset();
        return;
    }
    QWidget::keyPressEvent(event);
}

void AppPollingTestWindow::closeEvent(QCloseEvent *event)
{
    if(!closing) {
        closing = true;
        frame.stop();
        raw.Stop();
        if(filtering && windowHandle())
            windowHandle()->removeEventFilter(this);
        filtering = false;
        if(!rawInput)
            QCoreApplication::setAttribute(Qt::AA_CompressHighFrequencyEvents, compressBefore);
        emit WindowExiting();
    }
    event->accept();
}
