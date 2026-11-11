/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Window of the polling rate test ("Measure Polling Rate", next to the COM port).

    Same test as the WebApp's js/app/polling-test.js: it measures how often the gun really sends
    its aim to the computer, before the App connects. The gun must be in Run mode (a docked gun
    stops reporting the aim) with mouse output; nothing is sent to it and the firmware is the same.

    It shows the current rate (big number), the average of the last 3 s of movement, maximum and
    minimum of the graph and the graph of the last 10 s of movement, which only runs while the
    gun moves. The average and the line of the graph are green from 180 Hz, orange from 60 Hz,
    red below (AppPollingStats). The main window opens it only while no gun is connected and
    closes it (Shutdown) when one connects.

    The reports are those of Raw Input on Windows (AppPollingRawInput: every report, with its own
    time); elsewhere the mouse moves of Qt, which the system can join (a note says so). They
    count while this window is the active one and the aim is inside it.

    Keys: ESC closes, R resets.

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

#ifndef APPPOLLINGTESTWINDOW_H
#define APPPOLLINGTESTWINDOW_H

#include "apppollingrawinput.h"
#include "apppollingstats.h"

#include <QColor>
#include <QTimer>
#include <QWidget>

#include <vector>

class QFrame;
class QLabel;
class QPushButton;

class AppPollingTestWindow : public QWidget
{
    Q_OBJECT

public:
    explicit AppPollingTestWindow(QWidget *parent = nullptr);
    ~AppPollingTestWindow();

    /// @brief      Closes the window (a gun connects, the App closes)
    void Shutdown() { close(); }

    /// @brief      The statistics, for the graph
    const AppPollingStats &Stats() const { return stats; }

    struct Colors_s {
        QColor ok, mid, bad, text, faint, grid, chip, chipText, chipOk, chipOkText, chipWarnText;
    };
    const Colors_s &Colors() const { return colors; }

signals:
    /// @brief      The window is closing
    void WindowExiting();

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void closeEvent(QCloseEvent *event) override;
    void showEvent(QShowEvent *event) override;
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    void Reset();
    void Tick();
    void Render(const double &now, const bool &force);
    void RenderNumbers(const AppPollingStats::Snapshot_s &s);
    QFrame *Tile(const QString &label, QLabel *&value);
    void SetAverageLevel(const AppPollingStats::Level_e &level);

    /// @brief      Whether a report counts now: window active and aim inside it
    bool Listening() const;

    AppPollingStats stats;
    AppPollingRawInput raw;
    bool rawInput = false;              // Raw Input running (Windows); otherwise the mouse moves of Qt
    bool compressBefore = true;         // Qt::AA_CompressHighFrequencyEvents, restored on close
    bool filtering = false;
    bool closing = false;
    std::vector<double> times;
    QTimer frame;

    Colors_s colors;
    QLabel *currentValue = nullptr;
    QLabel *state = nullptr;
    QLabel *averageValue = nullptr;
    QLabel *maxValue = nullptr;
    QLabel *minValue = nullptr;
    QFrame *averageTile = nullptr;
    QWidget *chart = nullptr;

    AppPollingStats::Level_e averageLevel = AppPollingStats::levelNone;
    int stateKind = -1;
    bool shownMeasuring = false;
    double currentAt = -1e12;
    double numbersAt = -1e12;
};

#endif // APPPOLLINGTESTWINDOW_H
