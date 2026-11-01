/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Fullscreen window of the lens refinement ("Refine lens", Gun Settings > camera).

    Same window as the WebApp's js/app/lensfit-window.js. The user sweeps the gun over the
    whole screen. The screen is a grid that gets "painted" where the gun points (grey = empty,
    yellow = some views, green = complete); an inset shows the camera view, painted where the
    LEDs landed: its edges matter most for the lens. When everything is green the lens values
    are fitted (applensfit.h, in a background thread) and shown in one result panel. Nothing is
    sent to the gun here: the main window puts the accepted values in the settings, saved with
    the normal Save.

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

#ifndef APPLENSFITWINDOW_H
#define APPLENSFITWINDOW_H

#include "applensfit.h"

#include <QElapsedTimer>
#include <QPointer>
#include <QTimer>
#include <QWidget>

#include <memory>

class QFrame;
class QPainter;
class QThread;

class AppLensFitWindow : public QWidget
{
    Q_OBJECT

public:
    explicit AppLensFitWindow(QWidget *parent = nullptr);
    ~AppLensFitWindow();

    /// @brief      sLensInfo: coordinate space and lens values of the gun (sent once)
    void SetLensInfo(const QByteArray &payload);

    /// @brief      sLensSample: the four tracker vertices of one frame
    void AddLensSample(const QByteArray &payload);

    /// @brief      Leaves without values (board lost, error), as ESC
    void Shutdown() { Exit(false); }

    /// @brief      Values chosen with Apply (empty value = camera record), valid when Applied()
    bool Applied() const { return applied; }
    const LensFit::Values &AppliedValues() const { return appliedValues; }

signals:
    /// @brief      The window is leaving: the main window reads Applied(), ends the IR test
    ///             and closes the window
    void WindowExiting();

protected:
    void paintEvent(QPaintEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

    /// @brief      Closed by the system (Alt+F4, taskbar) or by anything else: leaves like ESC
    void closeEvent(QCloseEvent *event) override;

private:
    enum {
        stateWaiting = 0,
        stateCollecting,
        stateSolving,
        stateResult,
        stateUnsupported
    } LensFitStates_e;

    void Exit(const bool &apply, const LensFit::Values &values = LensFit::Values());
    void StartSolve();
    void Restart();
    void ShowPanel();
    void PlacePanel();
    void DrawTextBox(QPainter &painter, const LensFit::Guide *g, const int &step, const qreal &font, const qreal &big);

    /// @brief      Words of text in lines no wider than maxWidth with the painter's font
    static QStringList Wrap(const QPainter &painter, const QString &text, const qreal &maxWidth);

    int state = stateWaiting;
    bool exiting = false;
    bool applied = false;
    LensFit::Values appliedValues;

    bool hasInfo = false;
    LensFit::Info info;
    std::unique_ptr<LensFit::Collector> collector;
    bool hasResult = false;
    LensFit::Result result;

    QElapsedTimer clock;                // time base of the window (ms)
    qint64 lastComplete = -1000000;     // last complete view

    QTimer tick;                        // redraws the pulse and the red warning
    QTimer infoTimer;                   // firmware without the lens refinement never sends sLensInfo

    std::shared_ptr<LensFit::Control> solveControl;
    QPointer<QThread> solveThread;

    QFrame *panel = nullptr;            // result panel (and unsupported firmware)
};

#endif // APPLENSFITWINDOW_H
