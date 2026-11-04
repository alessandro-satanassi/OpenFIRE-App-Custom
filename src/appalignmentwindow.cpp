/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    IR emitter placement planner window: port of the WebApp's js/app/emitter-alignment.js.

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

#include "appalignmentwindow.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGraphicsOpacityEffect>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <cmath>

using namespace EmitterLayout;

namespace {

// Same colours and sizes as the WebApp (style.css, .alignment-*)
const char *const STYLE = R"(
QWidget { color: #eaf2fc; font-size: 14px; }
QLabel { background: transparent; }
QLabel#kicker { color: #58d6e8; font-size: 10px; font-weight: 700; letter-spacing: 1.6px; }
QLabel#title { font-size: 24px; font-weight: 600; }
QLabel#cardTitle { font-size: 16px; font-weight: 600; }
QLabel#unit { color: #a9bad0; }
QLabel#hint { color: #a9bad0; font-size: 12px; }
QLabel#placement { color: #b7d0e5; }
QLabel#measureName { color: #a9bad0; font-size: 13px; }
QLabel#measureValue { color: #ffffff; font-size: 13px; font-weight: 600; }
QLabel#note { color: #b7c9dc; font-size: 12px; }
QLabel#noteEstimate { color: #ffd699; font-size: 12px; }
QLabel#noteSmall { color: #8fa7c0; font-size: 11px; }
QLabel#footer { color: #a9bad0; font-size: 12px; }
QLabel#status { border: 1px solid; border-radius: 8px; padding: 9px 12px; font-size: 15px; font-weight: 600; }
QLabel#status[status="good"] { color: #98ecc4; border-color: #286e55; background: #12382f; }
QLabel#status[status="limited"] { color: #ffd699; border-color: #886631; background: #382d1c; }
QLabel#status[status="close"] { color: #ffb2b6; border-color: #96464d; background: #3b202b; }
QLabel#status[status="legacy"] { color: #ffce9b; border-color: #b76d38; background: #3b251b; }
QFrame#notesLine { background: #2a405a; border: none; max-height: 1px; min-height: 1px; }
QPushButton { background: #19283b; border: 1px solid #3d526b; color: #eaf2fc; padding: 7px 12px; border-radius: 7px; }
QPushButton:hover { background: #253d55; }
QPushButton:focus { border-color: #58d6e8; }
QPushButton:disabled { background: #19283b; color: #6c7d92; }
QPushButton#layoutButton:checked { border-color: #58d6e8; color: #74e3f4; background: #163b4c; }
QPushButton#legacyButton { border-color: #b76d38; color: #ffce9b; background: #3b251b; }
QPushButton#legacyButton:hover { background: #563222; }
QPushButton#legacyButton:checked { border-color: #ffb464; color: #fff0df; background: #8a3e20; }
QFrame#card { background: #101e30; border: 1px solid #2a405a; border-radius: 14px; }
QFrame#group { background: #0b1626; border: 1px solid #22374f; border-radius: 10px; }
QDoubleSpinBox { background: #0b1421; color: #f1f7ff; border: 1px solid #3d526b; border-radius: 5px; padding: 4px 6px; }
QDoubleSpinBox:focus, QComboBox:focus { border-color: #58d6e8; }
QDoubleSpinBox:disabled, QComboBox:disabled { color: #8a99ab; }
QComboBox { background: #0b1421; color: #f1f7ff; border: 1px solid #3d526b; border-radius: 5px; padding: 5px; }
QComboBox#preset { color: #a9bad0; font-size: 12px; padding: 4px 6px; }
QComboBox QAbstractItemView { background: #0b1421; color: #f1f7ff; selection-background-color: #163b4c; border: 1px solid #3d526b; }
QSlider::groove:horizontal { height: 6px; border-radius: 3px; background: #3d526b; }
QSlider::sub-page:horizontal { height: 6px; border-radius: 3px; background: #58d6e8; }
QSlider::sub-page:horizontal:disabled { background: #5b6b7d; }
QSlider::handle:horizontal { width: 16px; height: 16px; margin: -5px 0; border-radius: 8px; background: #58d6e8; }
QSlider::handle:horizontal:disabled { background: #8a99ab; }
QScrollArea, QScrollArea > QWidget > QWidget#scrollContent { background: transparent; border: none; }
QScrollBar:vertical { background: transparent; width: 8px; }
QScrollBar::handle:vertical { background: #3d526b; border-radius: 4px; min-height: 24px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
)";

const QColor BACKGROUND(0x0b, 0x14, 0x21);

/// @brief  Number box written like the WebApp input: no trailing zeros (1, 0.67, 4.3)
class NumberBox : public QDoubleSpinBox
{
public:
    QString textFromValue(double value) const override
    {
        QString text = locale().toString(value, 'f', decimals());
        const QChar point = locale().decimalPoint().at(0);
        if(text.contains(point)) {
            while(text.endsWith('0')) text.chop(1);
            if(text.endsWith(point)) text.chop(1);
        }
        return text;
    }
};

/// @brief  Slider with the marks of the WebApp range (datalist ticks) below the groove
class TickSlider : public QSlider
{
public:
    TickSlider(const QList<double> &marks, double minimum, double maximum)
        : QSlider(Qt::Horizontal), ticks(marks), low(minimum), high(maximum)
    {
        if(!ticks.isEmpty()) setFixedHeight(26);
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QSlider::paintEvent(event);
        if(ticks.isEmpty() || high <= low) return;
        QPainter painter(this);
        painter.setPen(QPen(isEnabled() ? QColor(0xa9, 0xba, 0xd0) : QColor(0x5b, 0x6b, 0x7d), 1));
        const double half = HANDLE / 2., travel = width() - HANDLE;
        const int top = height() / 2 + 7;
        for(const double tick : ticks) {
            const double x = std::floor(half + (tick - low) / (high - low) * travel) + .5;
            painter.drawLine(QPointF(x, top), QPointF(x, top + 4));
        }
    }

private:
    static constexpr int HANDLE = 16;   // handle width of the style sheet
    QList<double> ticks;
    double low, high;
};

QLabel *Label(const QString &text, const char *name = nullptr, bool wrap = false)
{
    QLabel *label = new QLabel(text);
    if(name) label->setObjectName(name);
    label->setWordWrap(wrap);
    return label;
}

} // namespace


AppAlignmentWindow::AppAlignmentWindow(std::optional<int> cameraModel, bool diamondLayout,
                                       std::optional<Custom> lensValues, QWidget *parent)
    : QWidget(parent), diamond(diamondLayout), model(cameraModel), custom(lensValues),
      locale(tr("en_US"))   // translated: numbers in the language of the App (decimal comma in Italian)
{
    setWindowTitle(tr("Alignment Assistant"));
    setStyleSheet(STYLE);
    record = model ? CameraByModel(*model) : nullptr;
    if(!record && !Cameras().empty()) record = &Cameras().front();

    timer.setSingleShot(true);
    timer.setInterval(70);
    connect(&timer, &QTimer::timeout, this, &AppAlignmentWindow::Compute);

    // ----- Header ----------------------------------------------------------------------------
    header = new QWidget(this);
    QHBoxLayout *headerLayout = new QHBoxLayout(header);
    headerLayout->setContentsMargins(0, 0, 0, 0);
    QVBoxLayout *titles = new QVBoxLayout();
    titles->setSpacing(4);
    titles->addWidget(Label(QStringLiteral("OPENFIRE"), "kicker"));
    titles->addWidget(Label(tr("IR emitter placement"), "title"));
    headerLayout->addLayout(titles);
    headerLayout->addStretch(1);
    hideButton = new QPushButton(tr("Hide controls"));
    connect(hideButton, &QPushButton::clicked, this, [this]() {
        body->setVisible(!body->isVisible());
        hideButton->setText(body->isVisible() ? tr("Hide controls") : tr("Show controls"));
    });
    QPushButton *closeButton = new QPushButton(tr("Close"));
    connect(closeButton, &QPushButton::clicked, this, &QWidget::close);
    headerLayout->addWidget(hideButton);
    headerLayout->addWidget(closeButton);

    // ----- Body: setup and results cards --------------------------------------------------------
    QScrollArea *scroll = new QScrollArea(this);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->viewport()->setAutoFillBackground(false);
    body = scroll;
    QWidget *content = new QWidget();
    content->setObjectName("scrollContent");
    content->setAutoFillBackground(false);
    QVBoxLayout *centre = new QVBoxLayout(content);
    centre->setContentsMargins(4, 4, 4, 4);
    centre->addStretch(1);
    QGridLayout *cards = new QGridLayout();
    cards->setHorizontalSpacing(18);
    cards->setColumnStretch(0, 1);
    cards->setColumnStretch(1, 1);
    centre->addLayout(cards);
    centre->addStretch(1);
    scroll->setWidget(content);

    QFrame *controls = new QFrame();
    controls->setObjectName("card");
    QVBoxLayout *controlsLayout = new QVBoxLayout(controls);
    controlsLayout->setContentsMargins(20, 18, 20, 18);
    controlsLayout->setSpacing(0);
    controlsLayout->addWidget(Label(tr("Your setup"), "cardTitle"));
    controlsLayout->addSpacing(14);

    setup = new QWidget();
    QVBoxLayout *setupLayout = new QVBoxLayout(setup);
    setupLayout->setContentsMargins(0, 0, 0, 0);
    setupLayout->setSpacing(10);
    controlsLayout->addWidget(setup);
    QGraphicsOpacityEffect *dim = new QGraphicsOpacityEffect(setup);
    dim->setOpacity(.45);
    dim->setEnabled(false);
    setup->setGraphicsEffect(dim);

    QHBoxLayout *layouts = new QHBoxLayout();
    layouts->setSpacing(8);
    squareButton = new QPushButton(QStringLiteral("Square"));
    diamondButton = new QPushButton(QStringLiteral("Diamond"));
    for(QPushButton *button : {squareButton, diamondButton}) {
        button->setObjectName("layoutButton");
        button->setCheckable(true);
        button->setAutoExclusive(true);
        layouts->addWidget(button);
    }
    (diamond ? diamondButton : squareButton)->setChecked(true);
    connect(squareButton, &QPushButton::clicked, this, [this]() { diamond = false; Schedule(); });
    connect(diamondButton, &QPushButton::clicked, this, [this]() { diamond = true; Schedule(); });
    setupLayout->addLayout(layouts);
    setupLayout->addSpacing(6);

    // Three groups: screen, camera, play.
    QFrame *screen = Group(setupLayout), *camera = Group(setupLayout), *play = Group(setupLayout);
    QVBoxLayout *screenLayout = static_cast<QVBoxLayout *>(screen->layout());
    QVBoxLayout *cameraLayout = static_cast<QVBoxLayout *>(camera->layout());
    QVBoxLayout *playLayout = static_cast<QVBoxLayout *>(play->layout());

    // Every format of the profiles; the ultrawide ones are for the planner only and do not add
    // or change firmware profile values.
    aspectBox = Select(screenLayout, tr("Screen format"));
    for(const auto &format : {QPair<int, int>(16, 9), {16, 10}, {3, 2}, {5, 4}, {4, 3}, {21, 9}, {21, 10}, {32, 9}})
        aspectBox->addItem(QString("%1:%2").arg(format.first).arg(format.second), double(format.first) / double(format.second));
    aspectBox->setCurrentIndex(0);
    connect(aspectBox, &QComboBox::currentIndexChanged, this, [this](int index) {
        aspect = aspectBox->itemData(index).toDouble();
        Schedule();
    });
    QList<QPair<double, QString>> sizes;
    QList<double> sizeTicks;
    for(const int size : {20, 24, 27, 32, 42, 43, 48, 50, 55, 65, 75, 77, 83, 85, 98, 100, 120}) {
        sizes.append({double(size), QString::number(size) + QStringLiteral("″")});
        sizeTicks.append(size);
    }
    Range(screenLayout, inchesField, tr("Screen diagonal"), QStringLiteral("″"), 20, 120, 1, 0, &inches, false, sizeTicks,
          tr("Common screen sizes"), tr("Choose a common screen size…"), sizes);

    cameraBox = Select(cameraLayout, tr("Camera"));
    for(const Camera &c : Cameras())
        cameraBox->addItem(QString::fromUtf8(c.label), c.model);
    if(record) cameraBox->setCurrentIndex(cameraBox->findData(record->model));
    connect(cameraBox, &QComboBox::currentIndexChanged, this, [this](int index) {
        if(const Camera *c = CameraByModel(cameraBox->itemData(index).toInt())) record = c;
        SetLens(1);
        Schedule();
    });
    // Add-on wide converter as a factor: 1 = none, any value down to 0.30x.
    Range(cameraLayout, lensField, tr("Additional lens"), QStringLiteral("×"), .3, 1, .01, 2, &lens, false,
          {.3, .45, .5, .6, .67, .75, 1},
          tr("Common lenses"), tr("Choose a common lens…"),
          {{1, tr("None")}, {.67, tr("0.67x converter")}, {.45, tr("0.45x converter")}});

    QWidget *distanceBox = Range(playLayout, distanceField, tr("Playing distance"), QStringLiteral("m"), .5, MAX_DISTANCE,
                                 DISTANCE_STEP, 1, &distance, true, {.5, 1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10});
    idealButton = new QPushButton(tr("Set ideal minimum distance"));
    idealButton->setEnabled(false);
    connect(idealButton, &QPushButton::clicked, this, [this]() { Schedule(true); });
    QVBoxLayout *distanceLayout = static_cast<QVBoxLayout *>(distanceBox->layout());
    distanceLayout->addSpacing(8);
    distanceLayout->addWidget(idealButton);
    QLabel *distanceHint = Label(tr("Measured from the gun to the screen."), "hint", true);
    distanceHint->setAlignment(Qt::AlignCenter);
    distanceLayout->addWidget(distanceHint);

    legacyButton = new QPushButton(tr("Force legacy placement"));
    legacyButton->setObjectName("legacyButton");
    legacyButton->setCheckable(true);
    connect(legacyButton, &QPushButton::clicked, this, &AppAlignmentWindow::ToggleLegacy);
    controlsLayout->addSpacing(16);
    controlsLayout->addWidget(legacyButton);
    controlsLayout->addStretch(1);

    resultsCard = new QFrame();
    resultsCard->setObjectName("card");
    QVBoxLayout *resultsLayout = new QVBoxLayout(resultsCard);
    resultsLayout->setContentsMargins(20, 18, 8, 18);
    QScrollArea *resultsScroll = new QScrollArea();
    resultsScroll->setFrameShape(QFrame::NoFrame);
    resultsScroll->setWidgetResizable(true);
    resultsScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    resultsScroll->viewport()->setAutoFillBackground(false);
    // As tall as the setup card: long advice scrolls inside, the setup card never moves.
    resultsScroll->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    resultsLayout->addWidget(resultsScroll);
    resultsContent = new QWidget();
    resultsContent->setObjectName("scrollContent");
    resultsScroll->setWidget(resultsContent);
    QVBoxLayout *contentLayout = new QVBoxLayout(resultsContent);
    contentLayout->setContentsMargins(0, 0, 12, 0);

    cards->addWidget(controls, 0, 0);
    cards->addWidget(resultsCard, 0, 1);

    footer = Label(tr("Measure between LED centres. After moving the emitters, calibrate again."), "footer", true);
    footer->setParent(this);
    footer->setAlignment(Qt::AlignCenter);
    footer->setAttribute(Qt::WA_TransparentForMouseEvents);

    if(!record) {
        contentLayout->addWidget(Label(tr("Camera optics are unavailable."), nullptr, true));
        return;
    }
    Schedule(true);
}


QFrame *AppAlignmentWindow::Group(QVBoxLayout *parent)
{
    QFrame *group = new QFrame();
    group->setObjectName("group");
    QVBoxLayout *layout = new QVBoxLayout(group);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(12);
    parent->addWidget(group);
    return group;
}


QComboBox *AppAlignmentWindow::Select(QVBoxLayout *parent, const QString &label)
{
    QHBoxLayout *line = new QHBoxLayout();
    line->setSpacing(10);
    QLabel *title = Label(label);
    QComboBox *box = new QComboBox();
    title->setBuddy(box);
    line->addWidget(title, 4);
    line->addWidget(box, 5);
    parent->addLayout(line);
    return box;
}


QWidget *AppAlignmentWindow::Range(QVBoxLayout *parent, Field &field, const QString &label, const QString &unit,
                                   double min, double max, double step, int decimals, double *value, bool rounded,
                                   const QList<double> &ticks, const QString &presetLabel, const QString &prompt,
                                   const QList<QPair<double, QString>> &presets)
{
    QWidget *box = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(box);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(4);

    QHBoxLayout *line = new QHBoxLayout();
    line->setSpacing(6);
    QLabel *title = Label(label);
    field.number = new NumberBox();
    field.number->setLocale(locale);
    field.number->setRange(min, max);
    field.number->setDecimals(decimals);
    field.number->setSingleStep(step);
    field.number->setButtonSymbols(QAbstractSpinBox::NoButtons);
    field.number->setFixedWidth(70);
    field.number->setValue(*value);
    field.number->setAccessibleName(label);
    title->setBuddy(field.number);
    QLabel *unitLabel = Label(unit, "unit");
    unitLabel->setFixedWidth(18);
    line->addWidget(title, 1);
    line->addWidget(field.number);
    line->addWidget(unitLabel);
    layout->addLayout(line);

    field.slider = new TickSlider(ticks, min, max);
    field.slider->setRange((int)std::lround(min / step), (int)std::lround(max / step));
    field.slider->setValue((int)std::lround(*value / step));
    field.slider->setAccessibleName(label + QStringLiteral(" — ") + tr("Slider"));
    layout->addWidget(field.slider);

    field.min = min;
    field.max = max;
    field.step = step;
    field.value = value;
    field.rounded = rounded;

    Field *f = &field;
    connect(field.number, &QDoubleSpinBox::valueChanged, this, [this, f](double v) { FieldChanged(*f, v, f->number); });
    connect(field.slider, &QSlider::valueChanged, this, [this, f](int v) { FieldChanged(*f, v * f->step, f->slider); });

    if(!presets.isEmpty()) {
        field.preset = new QComboBox();
        field.preset->setObjectName("preset");
        field.preset->setAccessibleName(presetLabel);
        field.preset->addItem(prompt, QVariant());
        for(const auto &entry : presets)
            field.preset->addItem(entry.second, entry.first);
        connect(field.preset, &QComboBox::activated, this, [this, f](int index) {
            if(index <= 0) return;
            const double v = f->preset->itemData(index).toDouble();
            {
                const QSignalBlocker blocker(f->number);
                f->number->setValue(v);
            }
            FieldChanged(*f, v, f->number);
            f->preset->setCurrentIndex(0);
        });
        layout->addWidget(field.preset);
    }
    parent->addWidget(box);
    return box;
}


void AppAlignmentWindow::FieldChanged(Field &field, double value, QObject *source)
{
    if(!std::isfinite(value)) return;
    const double rounded = field.rounded ? std::floor(value * (1 / field.step) + .5) / (1 / field.step) : value;
    *field.value = std::max(field.min, std::min(field.max, rounded));
    if(source != field.number || value > field.max) {
        const QSignalBlocker blocker(field.number);
        field.number->setValue(*field.value);
    }
    if(source != field.slider) {
        const QSignalBlocker blocker(field.slider);
        field.slider->setValue((int)std::lround(*field.value / field.step));
    }
    Schedule();
}


void AppAlignmentWindow::SetFieldValue(Field &field, double value)
{
    *field.value = value;
    const QSignalBlocker numberBlocker(field.number);
    const QSignalBlocker sliderBlocker(field.slider);
    field.number->setValue(value);
    field.slider->setValue((int)std::lround(value / field.step));
}


void AppAlignmentWindow::SetLens(double value)
{
    SetFieldValue(lensField, value);
}


void AppAlignmentWindow::ToggleLegacy()
{
    legacy = legacyButton->isChecked();
    // The whole setup is locked without changing the individual disabled states.
    setup->setEnabled(!legacy);
    if(setup->graphicsEffect()) setup->graphicsEffect()->setEnabled(legacy);
    Schedule();
}


void AppAlignmentWindow::Schedule(bool useIdealDistance)
{
    timer.stop();
    if(!legacy && !record) return;
    // The R3 already has a very wide native field. This is metadata driven, not a camera ID
    // check; future wide records get the same behaviour.
    if(!legacy) {
        const bool wide = record->addOnLens == false;
        lensField.number->setEnabled(!wide);
        lensField.slider->setEnabled(!wide);
        if(lensField.preset) lensField.preset->setEnabled(!wide);
        if(wide) SetLens(1);
        factor = lens;
    }
    scheduledIdeal = useIdealDistance;
    timer.start();
}


double AppAlignmentWindow::IdealDistance(const Custom *lensValues)
{
    // Reads the cached geometry to find the first green step (or the range limit). Also used to
    // disable the button when applying it would change nothing.
    const double min = distanceField.min, max = distanceField.max, scale = 1 / distanceField.step;
    const auto good = [&](double d) {
        Settings s;
        s.inches = inches; s.aspect = aspect; s.distance = d; s.diamond = diamond;
        Result r;
        return Calculate(s, *record, factor, lensValues, false, r) && r.status == StatusGood;
    };
    int lo = (int)std::ceil(min * scale), hi = (int)std::floor(max * scale);
    while(lo < hi) {
        const int mid = (lo + hi) / 2;
        if(good(mid / scale)) hi = mid; else lo = mid + 1;
    }
    // If no green distance fits the range, keep its upper limit and the honest warning.
    return hi / scale;
}


void AppAlignmentWindow::Compute()
{
    Settings input;
    input.inches = inches; input.aspect = aspect; input.distance = distance; input.diamond = diamond;
    if(legacy) {
        // Fixed geometry only; camera and distance do not move the legacy LEDs.
        const Settings s = Clamp(input);
        const Dimensions dim = ScreenSize(s.inches, s.aspect);
        result = Result();
        result.settings = s;
        result.width = dim.width;
        result.height = dim.height;
        result.ratio = s.diamond ? s.aspect : DEFAULT_RATIO;
        result.base = result.ratio * dim.height;
        resultValid = true;
    } else {
        const Custom *lensValues = model && record->model == *model && custom ? &*custom : nullptr;
        const double ideal = IdealDistance(lensValues);
        if(scheduledIdeal) {
            SetFieldValue(distanceField, ideal);
            input.distance = distance;
        }
        idealButton->setEnabled(distance != ideal);
        resultValid = Calculate(input, *record, factor, lensValues, true, result);
    }
    scheduledIdeal = false;
    hasResult = true;
    UpdateResults();
    UpdateFooter();
    update();
}


QString AppAlignmentWindow::Number(double value, int digits) const
{
    // Like toLocaleString(lang, {maximumFractionDigits}) of the WebApp: halves rounded away from
    // zero on the decimal value, no trailing zeros.
    if(!std::isfinite(value)) return QString::number(value);
    const bool negative = value < 0;
    QString text = QString::asprintf("%.30f", std::fabs(value));
    const int dot = text.indexOf('.');
    QString whole = text.left(dot), fraction = text.mid(dot + 1);
    const bool up = fraction.size() > digits && fraction.at(digits) >= QLatin1Char('5');
    fraction.truncate(digits);
    if(up) {
        QString all = whole + fraction;
        int i = all.size() - 1;
        for(; i >= 0; --i) {
            if(all.at(i) == QLatin1Char('9')) all[i] = QLatin1Char('0');
            else { all[i] = QChar(all.at(i).unicode() + 1); break; }
        }
        if(i < 0) all.prepend(QLatin1Char('1'));
        whole = all.left(all.size() - digits);
        fraction = all.right(digits);
    }
    while(fraction.endsWith(QLatin1Char('0'))) fraction.chop(1);
    QString result = whole;
    if(!fraction.isEmpty()) result += locale.decimalPoint() + fraction;
    if(negative && result != QLatin1String("0")) result.prepend(locale.negativeSign());
    return result;
}


void AppAlignmentWindow::UpdateResults()
{
    QVBoxLayout *layout = static_cast<QVBoxLayout *>(resultsContent->layout());
    while(QLayoutItem *item = layout->takeAt(0)) {
        if(item->widget()) delete item->widget();
        else if(item->layout()) {
            while(QLayoutItem *sub = item->layout()->takeAt(0)) { delete sub->widget(); delete sub; }
        }
        delete item;
    }
    layout->setSpacing(0);
    if(!resultValid) {
        layout->addWidget(Label(tr("Camera optics are unavailable."), nullptr, true));
        layout->addStretch(1);
        return;
    }
    const Result &r = result;
    const Settings &s = r.settings;
    layout->addWidget(Label(legacy ? tr("Legacy placement") : tr("Suggested placement"), "cardTitle"));
    layout->addSpacing(14);
    QLabel *badge = Label(legacy ? tr("Legacy placement active") :
                          r.status == StatusGood ? tr("Good movement margin") :
                          r.status == StatusLimited ? tr("Limited movement margin") : tr("Reduced LED coverage"), "status", true);
    badge->setProperty("status", legacy ? "legacy" : r.status == StatusGood ? "good" : r.status == StatusLimited ? "limited" : "close");
    layout->addWidget(badge);
    layout->addSpacing(12);
    layout->addWidget(Label(s.diamond ? tr("Diamond: centre of each screen edge") : tr("Square: two emitters above and two below"),
                            "placement", true));
    layout->addSpacing(15);

    QGridLayout *grid = new QGridLayout();
    grid->setHorizontalSpacing(14);
    grid->setVerticalSpacing(8);
    grid->setColumnStretch(0, 1);
    const auto entry = [&](const QString &name, const QString &value) {
        const int row = grid->rowCount();
        QLabel *n = Label(name, "measureName", true), *v = Label(value, "measureValue");
        v->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        grid->addWidget(n, row, 0);
        grid->addWidget(v, row, 1);
    };
    const auto metres = [&](double m) { return Number(m, 1) + QStringLiteral(" m"); };
    entry(tr("Screen size"), Cm(r.width) + QStringLiteral(" × ") + Cm(r.height) + QStringLiteral(" cm"));
    if(!s.diamond) {
        entry(tr("Horizontal LED spacing"), Cm(r.base) + QStringLiteral(" cm"));
        entry(tr("From the screen centre"), QStringLiteral("± ") + Cm(r.base / 2) + QStringLiteral(" cm"));
        entry(tr("From each side edge"), Cm((r.width - r.base) / 2) + QStringLiteral(" cm"));
        entry(tr("Base / height"), Number(r.ratio, 3));
    }
    entry(tr("Vertical LED spacing"), Cm(r.height) + QStringLiteral(" cm"));
    if(!legacy) {
        if(r.moveTo) entry(tr("For fuller LED coverage"), metres(*r.moveTo));
        if(!r.moveTo || *r.moveTo != r.minDistance)
            entry(r.status != StatusGood ? tr("For more freedom of movement") : tr("Suggested minimum distance"),
                  r.minDistance ? metres(*r.minDistance) : tr("More than 10 m"));
    }
    layout->addLayout(grid);

    layout->addSpacing(16);
    QFrame *line = new QFrame();
    line->setObjectName("notesLine");
    layout->addWidget(line);
    layout->addSpacing(8);
    const auto note = [&](const QString &text, const char *name = "note") {
        layout->addSpacing(8);
        layout->addWidget(Label(text, name, true));
    };
    if(legacy) {
        if(!s.diamond) note(tr("Square uses the fixed legacy base / height ratio of 0.711."));
        note(tr("These LED positions work well in most situations, but they are not optimised for your setup."));
        note(tr("This placement does not adapt to the camera or playing distance."));
        note(tr("Your choices are preserved. Press the legacy button again to return to automatic placement."));
        note(tr("Check all four screen corners from your playing positions in the IR camera test."));
        layout->addStretch(1);
        return;
    }
    if(r.status == StatusLimited)
        note(tr("Usable at the selected distance, with less room to move. Move farther back for more freedom."));
    if(r.status == StatusClose) {
        note(r.moveTo ? tr("Showing the first usable placement farther back. The colour refers to your selected distance.")
                      : tr("No tested placement meets the visibility checks within 10 m."));
        note(tr("Some LEDs may leave the view even with the gun upright. Aim may be less precise or tracking may be lost."));
    }
    note(tr("The placement prioritises aiming precision, then LED visibility. You can calibrate farther back without moving the LEDs."));
    note(tr("Checks four LEDs with the gun upright and at least three when rolled up to 15° to either side, including side movements."));
    note(tr("When you tilt the gun, one LED may leave the view: the gun fills it in, with slightly less precise aim."));
    if(r.optics.estimated)
        note(tr("Uses the entered lens factor combined with the camera’s native field of view."), "noteEstimate");
    if(factor >= .5 && factor < 1)
        note(tr("After fitting the lens, use Refine lens in the App, so the gun knows the new lens."), "noteEstimate");
    else if(factor < .5)
        note(tr("This lens may be too strong for Refine lens in the App: aiming may be less precise."), "noteEstimate");
    if(r.optics.refined)
        note(tr("Uses the lens refinement currently selected in the App."));
    note(tr("Check all four screen corners from your playing positions in the IR camera test."));
    note(tr("Coverage is a geometric estimate, not a guarantee of detection range. LED power, mounting and reflections also matter."),
         "noteSmall");
    layout->addStretch(1);
}


void AppAlignmentWindow::UpdateFooter()
{
    if(!hasResult || !resultValid || height() <= 0) return;
    const double screenAspect = double(width()) / double(height());
    footer->setText(std::fabs(screenAspect / result.settings.aspect - 1) < .025 ?
                    tr("Measure between LED centres. After moving the emitters, calibrate again.") :
                    tr("Scaled preview: use the centimetre measurements to position the LEDs."));
}


void AppAlignmentWindow::PlaceBody()
{
    const int w = width(), h = height();
    // .alignment-header: top 16, left/right 76; .alignment-body: top 100, bottom 72, width min(850, 100% - 112)
    header->setGeometry(76, 21, std::max(0, w - 152), std::max(header->sizeHint().height(), 44));
    const int bodyWidth = std::max(0, std::min(850, w - 112));
    body->setGeometry((w - bodyWidth) / 2, 100, bodyWidth, std::max(0, h - 172));
    const int footerWidth = int(w * .76);
    footer->setGeometry((w - footerWidth) / 2, h - 17 - 34, footerWidth, 34);
    footer->setAlignment(Qt::AlignHCenter | Qt::AlignBottom);
}


void AppAlignmentWindow::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    PlaceBody();
    UpdateFooter();
}


void AppAlignmentWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Escape) {
        event->accept();
        close();
        return;
    }
    QWidget::keyPressEvent(event);
}


void AppAlignmentWindow::paintEvent(QPaintEvent *)
{
    QPainter ctx(this);
    ctx.setRenderHint(QPainter::Antialiasing);
    const double w = width(), h = height();
    ctx.fillRect(rect(), BACKGROUND);
    if(!hasResult || !resultValid) return;
    const Result &r = result;
    const double aspectRatio = r.settings.aspect, screenAspect = w / h;
    const double rw = screenAspect > aspectRatio ? h * aspectRatio : w, rh = rw / aspectRatio;
    const double x = (w - rw) / 2, y = (h - rh) / 2;

    ctx.setPen(QPen(QColor(0x32, 0x42, 0x59), 1));
    ctx.setBrush(Qt::NoBrush);
    ctx.drawRect(QRectF(x + .5, y + .5, rw - 1, rh - 1));

    const double fraction = r.base / r.width;
    const bool diamondLayout = r.settings.diamond;
    std::array<QPointF, 4> p;
    if(diamondLayout) p = {QPointF(.5, 0), QPointF(1, .5), QPointF(.5, 1), QPointF(0, .5)};
    else p = {QPointF(.5 - fraction / 2, 0), QPointF(.5 + fraction / 2, 0), QPointF(.5 - fraction / 2, 1), QPointF(.5 + fraction / 2, 1)};
    const QColor color = diamondLayout ? QColor(0xff, 0xbd, 0x66) : QColor(0x58, 0xd6, 0xe8);
    const auto alpha = [&](int a) { QColor c = color; c.setAlpha(a); return c; };
    const auto at = [&](const QPointF &u) { return QPointF(x + u.x() * rw, y + u.y() * rh); };

    // dashed outline joining the LEDs
    QPen dashed(alpha(0x50), 1);
    dashed.setDashPattern({6, 8});
    ctx.setPen(dashed);
    QPainterPath outline;
    const int order[2][5] = {{0, 1, 3, 2, 0}, {0, 1, 2, 3, 0}};
    for(int i = 0; i < 5; ++i) {
        const QPointF q = at(p[order[diamondLayout][i]]);
        if(i) outline.lineTo(q); else outline.moveTo(q);
    }
    ctx.drawPath(outline);

    // Keep labels inside the preview, reducing margins in compact views.
    const double mx = std::min(35.0, rw / 4), mt = std::min(38.0, rh / 3), mb = std::min(46.0, rh / 3);
    QFont font = this->font();
    font.setPixelSize(12);
    font.setWeight(QFont::DemiBold);
    for(int i = 0; i < 4; ++i) {
        const QPointF c = at(p[i]);
        ctx.setPen(Qt::NoPen);
        ctx.setBrush(alpha(0x25));
        ctx.drawEllipse(c, 24, 24);
        ctx.setPen(QPen(color, 2));
        ctx.setBrush(Qt::NoBrush);
        ctx.drawEllipse(c, 13, 13);
        ctx.setPen(Qt::NoPen);
        ctx.setBrush(color);
        ctx.drawEllipse(c, 5, 5);
        const double lx = std::max(x + mx, std::min(x + rw - mx, c.x())), ly = std::max(y + mt, std::min(y + rh - mb, c.y()));
        ctx.setFont(font);
        ctx.setPen(color);
        const QString text = QStringLiteral("LED %1").arg(i + 1);
        const QFontMetricsF metrics(font);
        ctx.drawText(QPointF(lx - metrics.horizontalAdvance(text) / 2, ly), text);
    }
    if(!diamondLayout) {
        const double left = at(p[0]).x(), right = at(p[1]).x(), dy = std::max(y + 55, 64.0);
        ctx.setPen(QPen(alpha(0x80), 1));
        ctx.drawLine(QPointF(left, dy - 5), QPointF(left, dy + 5));
        ctx.drawLine(QPointF(left, dy), QPointF(right, dy));
        ctx.drawLine(QPointF(right, dy - 5), QPointF(right, dy + 5));
        const QString text = Cm(r.base) + QStringLiteral(" cm");
        QFont big = font;
        big.setPixelSize(13);
        const QFontMetricsF metrics(big);
        const double tw = metrics.horizontalAdvance(text);
        ctx.fillRect(QRectF((left + right - tw) / 2 - 7, dy - 10, tw + 14, 20), BACKGROUND);
        ctx.setFont(big);
        ctx.setPen(color);
        ctx.drawText(QPointF((left + right - tw) / 2, dy + 5), text);
    }
}
