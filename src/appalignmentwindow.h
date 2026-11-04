/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    IR emitter placement planner window (Help > Open IR Emitter Alignment Assistant).

    Same window as the WebApp's js/app/emitter-alignment.js: screen size and format, camera,
    add-on lens and playing distance give the Square or Diamond LED positions in centimetres,
    with a green / orange / red movement margin, "Set ideal minimum distance" and
    "Force legacy placement". It runs offline and never changes the lightgun settings.

    Finestra dell'assistente posizionamento emettitori IR, uguale a quella della WebApp.
    Funziona offline e non modifica mai le impostazioni della pistola.

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

#ifndef APPALIGNMENTWINDOW_H
#define APPALIGNMENTWINDOW_H

#include "appemitterlayout.h"

#include <QLocale>
#include <QTimer>
#include <QWidget>

#include <optional>

class QComboBox;
class QDoubleSpinBox;
class QFrame;
class QGridLayout;
class QLabel;
class QPushButton;
class QSlider;
class QVBoxLayout;

class AppAlignmentWindow : public QWidget
{
    Q_OBJECT

public:
    /// @param  cameraModel  camera of the lightgun (current settings), empty when not connected
    /// @param  diamond      layout of the selected profile
    /// @param  lens         refined lens values of the lightgun camera, empty when not connected
    AppAlignmentWindow(std::optional<int> cameraModel, bool diamond,
                       std::optional<EmitterLayout::Custom> lens, QWidget *parent = nullptr);

    /// @brief  Closes the window (another fullscreen window is opened, board lost)
    void Shutdown() { close(); }

protected:
    void paintEvent(QPaintEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    /// @brief  A number with its spin box, slider and optional list of common values
    struct Field {
        QDoubleSpinBox *number = nullptr;
        QSlider *slider = nullptr;
        QComboBox *preset = nullptr;
        double min = 0, max = 0, step = 1;
        double *value = nullptr;
        bool rounded = false;   // distance: rounded to its step
    };

    QFrame *Group(QVBoxLayout *parent);
    QWidget *Range(QVBoxLayout *parent, Field &field, const QString &label, const QString &unit,
                   double min, double max, double step, int decimals, double *value, bool rounded,
                   const QList<double> &ticks, const QString &presetLabel = QString(), const QString &prompt = QString(),
                   const QList<QPair<double, QString>> &presets = {});
    QComboBox *Select(QVBoxLayout *parent, const QString &label);
    void FieldChanged(Field &field, double value, QObject *source);
    void SetFieldValue(Field &field, double value);
    void SetLens(double value);
    void ToggleLegacy();
    void Schedule(bool useIdealDistance = false);
    void Compute();
    double IdealDistance(const EmitterLayout::Custom *custom);
    void UpdateResults();
    void UpdateFooter();
    void PlaceBody();

    /// @brief  Number with at most `digits` decimals in the language of the App
    QString Number(double value, int digits) const;
    QString Cm(double metres) const { return Number(metres * 100, 1); }

    // input values (WebApp this.values)
    double inches = 55, distance = 2, aspect = 16.0 / 9.0, lens = 1;
    bool diamond = false;
    bool legacy = false;
    double factor = 1;

    std::optional<int> model;                       // camera of the lightgun
    std::optional<EmitterLayout::Custom> custom;    // its refined lens values
    const EmitterLayout::Camera *record = nullptr;  // camera chosen in the window

    bool hasResult = false;
    bool resultValid = false;
    EmitterLayout::Result result;
    double legacyRatio = 0;

    QLocale locale;
    QTimer timer;
    bool scheduledIdeal = false;

    QWidget *header = nullptr;
    QPushButton *hideButton = nullptr;
    QWidget *body = nullptr;
    QWidget *setup = nullptr;
    QPushButton *squareButton = nullptr, *diamondButton = nullptr;
    QComboBox *aspectBox = nullptr, *cameraBox = nullptr;
    Field inchesField, lensField, distanceField;
    QPushButton *idealButton = nullptr;
    QPushButton *legacyButton = nullptr;
    QFrame *resultsCard = nullptr;
    QWidget *resultsContent = nullptr;
    QLabel *footer = nullptr;
};

#endif // APPALIGNMENTWINDOW_H
