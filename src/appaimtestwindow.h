/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Fullscreen window of the aim test ("Open Aim Test...", Gun Tests).

    Same window as the WebApp's js/app/aim-test-window.js. It checks the aim against a grid with
    nine reference targets (centre, edge midpoints, corners), the buttons and the recoil, in one
    screen. The gun is in the same test mode as the IR camera tester (sIRTest): the main window
    hands over the aim point (sTestCoords, fields 8-9, in the 1920x1080 test space), the emitters
    seen (sTestBlobs) and the buttons pressed (sBtnPressed). The trigger flashes the crosshair,
    leaves a fading hit mark and asks the main window for the recoil (TriggerFired); any other
    button shows its name in a short semi-transparent circle around the crosshair.

    Keys: ESC closes, H hides the key hints, F turns the recoil (solenoid / rumble) off and on.
    The grid always stays.

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

#ifndef APPAIMTESTWINDOW_H
#define APPAIMTESTWINDOW_H

#include <QElapsedTimer>
#include <QPixmap>
#include <QTimer>
#include <QVector>
#include <QWidget>

class QPainter;

class AppAimTestWindow : public QWidget
{
    Q_OBJECT

public:
    explicit AppAimTestWindow(QWidget *parent = nullptr);

    /// @brief      sTestCoords: 12 int32, the aim point in fields 8 and 9 (1920x1080 test space).
    ///             The firmware sends them also without emitters (the old point stays).
    void TestModeDraw(const QByteArray &payload);

    /// @brief      sTestBlobs (sent just before each sTestCoords): version, then for each emitter
    ///             flags (bit 0 seen, bit 2 the tracker updated the aim this frame), average, max, area
    void TestModeBlobs(const QByteArray &payload);

    /// @brief      sBtnPressed: index of boardInputs_e and its English name (translated here)
    void ButtonPressed(const int &index, const QString &name);

    /// @brief      Leaves (board lost, error), as ESC
    void Shutdown() { Exit(); }

signals:
    /// @brief      The window is leaving: the main window ends the IR test and closes the window
    void WindowExiting();

    /// @brief      Trigger pressed with the feedback on (F): the main window fires the recoil
    void TriggerFired();

protected:
    void paintEvent(QPaintEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

    /// @brief      Closed by the system (Alt+F4, taskbar) or by anything else: leaves like ESC
    void closeEvent(QCloseEvent *event) override;

private:
    void Exit();

    /// @brief      Aim point recent and updated by the tracker (without sTestBlobs the coordinates
    ///             are trusted). A shot is marked only then.
    bool AimFresh(const qint64 &now) const;

    void DrawGrid(QPainter &painter, const qreal &w, const qreal &h, const qreal &unit);
    void DrawShot(QPainter &painter, const qreal &x, const qreal &y, const qreal &unit, const qint64 &age);
    void DrawAim(QPainter &painter, const qreal &w, const qreal &h, const qreal &unit, const qint64 &now, const bool &fresh);

    struct Aim_s {
        qreal x = 0;        // 0..1 of the screen
        qreal y = 0;
        qint64 time = 0;
        int seen = -1;      // emitters seen, -1 without recent sTestBlobs
        int tracked = -1;   // tracker updated (0/1), -1 without recent sTestBlobs
    };
    struct Blobs_s {
        int seen = 0;
        bool tracked = false;
        qint64 time = 0;
    };
    struct Shot_s {
        qreal x;
        qreal y;
        qint64 time;
    };

    bool exiting = false;
    bool showHints = true;
    bool feedback = true;               // F: recoil on the trigger
    bool hasAim = false;
    Aim_s aim;                          // last aim point
    bool hasBlobs = false;
    Blobs_s blobs;                      // last sTestBlobs
    QVector<Shot_s> shots;              // hit marks still fading
    qint64 flash = -1000000;            // time of the last shot
    QString buttonName;                 // last button other than the trigger
    qint64 buttonTime = -1000000;

    QElapsedTimer clock;                // time base of the window (ms)
    QTimer redraw;                      // only while something fades, or when the aim gets old
    QPixmap grid;                       // the grid drawn once per window size
};

#endif // APPAIMTESTWINDOW_H
