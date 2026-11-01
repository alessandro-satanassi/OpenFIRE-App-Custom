/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Fullscreen window of the lens refinement: port of the WebApp's js/app/lensfit-window.js.

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

#include "applensfitwindow.h"

#include <QCloseEvent>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QThread>
#include <QVBoxLayout>

#include <cmath>
#include <functional>

namespace {

constexpr int INFO_TIMEOUT = 3000;      // ms: firmware without the lens refinement never sends sLensInfo
constexpr int LOST_TIME = 400;          // ms without a complete view before the red warning
constexpr qreal PI = 3.14159265358979323846;

const QColor COLOR_EMPTY(58, 62, 70);
const QColor COLOR_SOME(230, 190, 40);
const QColor COLOR_DONE(40, 170, 80);
const QColor BACKGROUND(0x15, 0x18, 0x1d);
const QColor YELLOW(0xff, 0xd7, 0x5a);

QColor CellColor(const qreal &fraction)
{
    if(fraction <= 0) return COLOR_EMPTY;
    const qreal t = std::min<qreal>(1, fraction);
    const QColor &from = t < 1 ? COLOR_SOME : COLOR_DONE;
    const qreal k = t < 1 ? 0.35 + 0.65 * t : 1;
    const auto mix = [&](const int &e, const int &v) { return int(std::lround(e + (v - e) * k)); };
    return QColor(mix(COLOR_EMPTY.red(), from.red()), mix(COLOR_EMPTY.green(), from.green()), mix(COLOR_EMPTY.blue(), from.blue()));
}

QString Format(const std::optional<double> &value, const int &digits)
{
    return !value || !std::isfinite(*value) ? QStringLiteral("-") : QString::number(*value, 'f', digits);
}

QFont Font(const qreal &pixels, const bool &bold = false)
{
    QFont font(QStringLiteral("Segoe UI"));
    font.setStyleHint(QFont::SansSerif);
    font.setPixelSize(std::max(1, int(std::lround(pixels))));
    font.setBold(bold);
    return font;
}

QPen Pen(const QColor &color, const qreal &width)
{
    QPen pen(color, width);
    pen.setCapStyle(Qt::FlatCap);
    pen.setJoinStyle(Qt::MiterJoin);
    return pen;
}

/// @brief  Text with its top at y (canvas textBaseline 'top')
void TextTop(QPainter &painter, const qreal &x, const qreal &y, const QString &text)
{
    painter.drawText(QPointF(x, y + painter.fontMetrics().ascent()), text);
}

/// @brief  Text centred on (x, y) (canvas textAlign 'center', textBaseline 'middle')
void TextCentre(QPainter &painter, const qreal &x, const qreal &y, const QString &text)
{
    const QFontMetricsF fm(painter.font());
    painter.drawText(QPointF(x - fm.horizontalAdvance(text) / 2, y + (fm.ascent() - fm.descent()) / 2), text);
}

/// @brief  Canvas arc (angles in radians, clockwise from 3 o'clock, as ctx.arc)
void Arc(QPainter &painter, const qreal &cx, const qreal &cy, const qreal &r, const qreal &from, const qreal &to)
{
    QPainterPath path;
    const QRectF rect(cx - r, cy - r, 2 * r, 2 * r);
    path.arcMoveTo(rect, -from * 180 / PI);
    path.arcTo(rect, -from * 180 / PI, -(to - from) * 180 / PI);
    painter.drawPath(path);
}

} // namespace

AppLensFitWindow::AppLensFitWindow(QWidget *parent)
    : QWidget(parent)
{
    setWindowTitle(tr("Refine camera lens"));
    setFocusPolicy(Qt::StrongFocus);
    setContextMenuPolicy(Qt::PreventContextMenu);
    setAutoFillBackground(false);
    setAttribute(Qt::WA_OpaquePaintEvent);
    clock.start();

    connect(&tick, &QTimer::timeout, this, QOverload<>::of(&QWidget::update));
    tick.start(250);

    infoTimer.setSingleShot(true);
    connect(&infoTimer, &QTimer::timeout, this, [this]() {
        if(!hasInfo && !exiting) {
            state = stateUnsupported;
            ShowPanel();
            update();
        }
    });
    infoTimer.start(INFO_TIMEOUT);
}

AppLensFitWindow::~AppLensFitWindow()
{
    // A fit still running stops at its next check and is waited for here.
    if(solveControl)
        solveControl->cancel.store(true);
    if(solveThread != nullptr)
        solveThread->wait();
}

void AppLensFitWindow::Exit(const bool &apply, const LensFit::Values &values)
{
    if(exiting)
        return;
    exiting = true;
    applied = apply;
    appliedValues = values;
    if(solveControl)
        solveControl->cancel.store(true);
    tick.stop();
    infoTimer.stop();
    emit WindowExiting();
}

void AppLensFitWindow::closeEvent(QCloseEvent *event)
{
    // As AppCaliWindow: the window then closes itself, through WindowExiting (the main window
    // ends the IR test first). Any close that does not come from there leaves like ESC.
    if(!exiting) {
        event->ignore();
        QMetaObject::invokeMethod(this, [this]() { Exit(false); }, Qt::QueuedConnection);
        return;
    }
    QWidget::closeEvent(event);
}

void AppLensFitWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Escape) {
        event->accept();
        Exit(false);
        return;
    }
    QWidget::keyPressEvent(event);
}

void AppLensFitWindow::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    PlacePanel();
}

void AppLensFitWindow::SetLensInfo(const QByteArray &payload)
{
    if(exiting || hasInfo)
        return;
    const std::optional<LensFit::Info> parsed = LensFit::ParseInfo(payload);
    if(!parsed)
        return;
    info = *parsed;
    hasInfo = true;
    infoTimer.stop();
    collector = std::make_unique<LensFit::Collector>(info);
    state = info.record.focal > 0 ? stateCollecting : stateUnsupported;
    if(state == stateUnsupported)
        ShowPanel();
    update();
}

void AppLensFitWindow::AddLensSample(const QByteArray &payload)
{
    if(exiting || state != stateCollecting)
        return;
    const std::optional<LensFit::Sample> sample = LensFit::ParseSample(payload);
    if(!sample)
        return;
    if(sample->complete)
        lastComplete = clock.elapsed();
    collector->Add(*sample);
    if(collector->ProgressNow().done)
        StartSolve();
    update();
}

void AppLensFitWindow::StartSolve()
{
    state = stateSolving;
    // The fit runs in its own thread: the window keeps drawing the progress and ESC closes it
    // (the fit then stops at its next check).
    solveControl = std::make_shared<LensFit::Control>();
    auto control = solveControl;
    auto out = std::make_shared<std::optional<LensFit::Result>>();
    const LensFit::Info solveInfo = info;
    const std::vector<LensFit::Frame> frames = collector->Frames();
    QThread *thread = QThread::create([control, out, solveInfo, frames]() {
        *out = LensFit::Solve(solveInfo, frames, control.get());
    });
    connect(thread, &QThread::finished, this, [this, thread, out]() {
        if(thread != solveThread || exiting || state != stateSolving)
            return;
        solveThread = nullptr;
        if(!*out)
            return;     // cancelled
        result = **out;
        hasResult = true;
        state = stateResult;
        ShowPanel();
        update();
    });
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    solveThread = thread;
    thread->start();
}

void AppLensFitWindow::Restart()
{
    collector = std::make_unique<LensFit::Collector>(info);
    hasResult = false;
    result = LensFit::Result();
    state = stateCollecting;
    if(panel != nullptr) {
        panel->deleteLater();
        panel = nullptr;
    }
    setFocus();
    update();
}

// ----- Result panel ---------------------------------------------------------------------

void AppLensFitWindow::ShowPanel()
{
    if(panel != nullptr)
        panel->deleteLater();
    panel = new QFrame(this);
    panel->setObjectName(QStringLiteral("lensFitPanel"));
    panel->setStyleSheet(QStringLiteral(
        "QFrame#lensFitPanel { background: #2b2f36; border: 2px solid rgb(160, 160, 164); border-radius: 14px; }"
        "QFrame#lensFitPanel QLabel { color: #f2f2f2; background: transparent; font-size: 16px; }"
        "QFrame#lensFitPanel QLabel#title { font-size: 20px; font-weight: 600; }"
        "QFrame#lensFitPanel QLabel[header=\"true\"] { font-weight: 600; }"
        "QFrame#lensFitPanel QLabel#note { color: #ffd75a; }"
        "QFrame#lensFitPanel QLabel[cell=\"true\"] { padding: 4px 8px; border-bottom: 1px solid #444a52; }"
        "QFrame#lensFitPanel QLabel[changed=\"true\"] { color: #6fe08f; font-weight: 600; }"
        "QFrame#lensFitPanel QLabel[error=\"true\"] { color: #ff6b6b; font-weight: 600; }"
        "QPushButton { min-width: 110px; padding: 6px 14px; font-size: 16px; color: #f2f2f2; background: #3d434c;"
        "  border: 1px solid #6b727c; border-radius: 4px; }"
        "QPushButton:hover { background: #4a515b; }"
        "QPushButton:focus { border: 1px solid #f2f2f2; }"
        "QPushButton[primary=\"true\"] { font-weight: 600; color: #ffffff; background: #0a7cc2; border-color: #0a7cc2; }"
        "QPushButton[primary=\"true\"]:hover { background: #108fdd; }"
        "QPushButton[primary=\"true\"]:focus { border: 1px solid #ffffff; }"));
    panel->setFont(Font(16));

    QVBoxLayout *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(24, 20, 24, 20);
    layout->setSpacing(10);

    QLabel *title = new QLabel(tr("Refine camera lens"), panel);
    title->setObjectName(QStringLiteral("title"));
    title->setFont(Font(20, true));
    title->setAlignment(Qt::AlignCenter);
    layout->addWidget(title);

    const auto paragraph = [&](const QString &text, const QString &name = QString()) {
        QLabel *label = new QLabel(text, panel);
        label->setWordWrap(true);
        if(!name.isEmpty()) label->setObjectName(name);
        layout->addWidget(label);
    };

    QHBoxLayout *buttons = new QHBoxLayout();
    buttons->setSpacing(10);
    buttons->addStretch(1);
    QPushButton *first = nullptr;
    const auto button = [&](const QString &text, const std::function<void()> &onClick, const bool &primary = false) {
        QPushButton *b = new QPushButton(text, panel);
        b->setProperty("primary", primary);
        b->setAutoDefault(false);
        b->setCursor(Qt::PointingHandCursor);
        connect(b, &QPushButton::clicked, this, onClick);
        buttons->addWidget(b);
        if(first == nullptr) first = b;
    };

    if(state == stateUnsupported) {
        paragraph(hasInfo
            ? tr("The camera record has no focal ratio: the lens cannot be refined.")
            : tr("The gun did not answer: its firmware does not support the lens refinement. Update the firmware."));
        button(tr("Close"), [this]() { Exit(false); }, true);
    } else {
        // One screen for every outcome: the values in use next to the calculated ones
        // (ERROR in their place when the calculation failed).
        const bool ok = hasResult && result.ok;
        // Confirmed: a refined tier, or the default values when they beat the values in use.
        const bool better = ok && !result.keepActive && (result.tier != LensFit::TierRecord || result.activeCustom);
        // What can be applied: the confirmed result, a refined tier that does not beat the
        // values in use, or else the closest calculated values not confirmed (near). None
        // when nothing could be determined: the default values are never shown as calculated.
        std::optional<LensFit::Values> proposal;
        if(ok) {
            if(better || result.tier != LensFit::TierRecord) proposal = result.values;
            else if(result.hasNear) proposal = result.near;
        }
        const LensFit::Lens &rec = info.record, &active = info.active;
        std::optional<double> next[3], current[3] = {active.k1, active.k2, active.focal};
        if(proposal && hasInfo) {
            next[0] = proposal->k1 ? *proposal->k1 : rec.k1;
            next[1] = proposal->k2 ? *proposal->k2 : rec.k2;
            next[2] = proposal->focal ? *proposal->focal : rec.focal;
        }
        const QString rowNames[3] = {tr("Lens radial correction K1"), tr("Lens radial correction K2"), tr("Lens focal ratio")};
        constexpr int digits = 4;
        const auto changed = [&](const int &i) { return proposal && Format(next[i], digits) != Format(current[i], digits); };
        const bool anyChange = changed(0) || changed(1) || changed(2);
        paragraph(!ok ? tr("The lens values could not be computed. Try again, moving the gun more slowly over the whole screen.")
                  : !proposal ? tr("The analysis found no reliable change: keep the current values.")
                  : !anyChange ? tr("The current values are already right for your camera: nothing needs to change.")
                  : better ? tr("Values better suited to your camera were found: the aim should be more precise, above all towards the edges of the screen.")
                  : tr("The calculated values do not clearly improve on the current ones: better to keep the current ones, so the profiles need no new calibration."));
        if(hasInfo) {
            QGridLayout *table = new QGridLayout();
            table->setSpacing(0);
            table->setColumnStretch(0, 1);
            const auto cell = [&](const QString &text, const int &row, const int &col, const char *flag = nullptr) {
                QLabel *label = new QLabel(text, panel);
                label->setProperty("cell", true);
                if(flag != nullptr) label->setProperty(flag, true);
                if(row == 0) label->setProperty("header", true);
                label->setAlignment((col == 0 ? Qt::AlignLeft : Qt::AlignRight) | Qt::AlignVCenter);
                table->addWidget(label, row, col);
            };
            cell(QString(), 0, 0);
            cell(tr("Current"), 0, 1);
            cell(tr("Calculated"), 0, 2);
            for(int i = 0; i < 3; ++i) {
                cell(rowNames[i], i + 1, 0);
                cell(Format(current[i], digits), i + 1, 1);
                if(!ok) cell(tr("ERROR"), i + 1, 2, "error");
                else if(proposal) cell(Format(next[i], digits), i + 1, 2, changed(i) ? "changed" : nullptr);
                else cell(QStringLiteral("—"), i + 1, 2);
            }
            layout->addLayout(table);
        }
        const LensFit::Values applyValues = proposal ? *proposal : LensFit::Values();
        const auto apply = [this, applyValues]() { Exit(true, applyValues); };
        const auto retry = [this]() { Restart(); };
        const auto cancel = [this]() { Exit(false); };
        if(anyChange && better) {
            paragraph(tr("After saving, calibrate again every profile you use."), QStringLiteral("note"));
            button(tr("Apply"), apply, true);
            button(tr("Retry"), retry);
            button(tr("Cancel"), cancel);
        } else if(anyChange) {
            // Not confirmed: keeping the current ones is the suggested choice (the sentence
            // above says that applying them means calibrating again, and so does the status
            // line after Apply).
            button(tr("Keep current"), cancel, true);
            button(tr("Apply"), apply);
            button(tr("Retry"), retry);
        } else if(ok) {
            button(tr("Close"), cancel, true);
            button(tr("Retry"), retry);
        } else {
            button(tr("Retry"), retry, true);
            button(tr("Cancel"), cancel);
        }
    }
    buttons->addStretch(1);
    layout->addSpacing(4);
    layout->addLayout(buttons);
    PlacePanel();
    panel->show();
    if(first != nullptr)
        first->setFocus();
}

void AppLensFitWindow::PlacePanel()
{
    if(panel == nullptr)
        return;
    // As the WebApp: at least min(420px, 94%) and at most min(720px, 94%) wide, centred.
    const int maxW = std::min(720, int(width() * 0.94));
    const int minW = std::min(420, maxW);
    panel->setMinimumWidth(minW);
    panel->setMaximumWidth(maxW);
    panel->setMaximumHeight(int(height() * 0.94));
    panel->adjustSize();
    // Shrink to fit as the WebApp's panel: wide enough for its sentences on one line when they fit.
    int natural = panel->sizeHint().width();
    const QMargins margins = panel->layout()->contentsMargins();
    for(const QLabel *label : panel->findChildren<QLabel*>())
        if(label->wordWrap())
            natural = std::max(natural, int(std::ceil(QFontMetricsF(label->font()).horizontalAdvance(label->text()))) +
                                        margins.left() + margins.right() + 2 * panel->frameWidth() + 4);
    const int w = std::max(minW, std::min(maxW, natural));
    const int h = std::min(int(height() * 0.94), panel->heightForWidth(w) > 0 ? panel->heightForWidth(w) : panel->sizeHint().height());
    panel->setGeometry((width() - w) / 2, (height() - h) / 2, w, h);
}

// ----- Drawing ------------------------------------------------------------------------

QStringList AppLensFitWindow::Wrap(const QPainter &painter, const QString &text, const qreal &maxWidth)
{
    const QFontMetricsF fm(painter.font());
    QStringList lines;
    QString line;
    for(const QString &word : text.split(QLatin1Char(' '))) {
        const QString test = line.isEmpty() ? word : line + QLatin1Char(' ') + word;
        if(fm.horizontalAdvance(test) > maxWidth && !line.isEmpty()) {
            lines << line;
            line = word;
        } else line = test;
    }
    if(!line.isEmpty())
        lines << line;
    return lines;
}

void AppLensFitWindow::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    const qreal width = this->width(), height = this->height();
    painter.fillRect(rect(), BACKGROUND);
    LensFit::Collector *c = collector.get();

    // Screen grid: painted in step 1; afterwards it stays green, dimmed, behind the guide.
    LensFit::Guide guide;
    const LensFit::Guide *g = nullptr;
    if(c != nullptr && state == stateCollecting) {
        guide = c->GuideNow();
        g = &guide;
    }
    const int step = g ? g->step : 0;
    const qreal gap = std::max<qreal>(2, std::round(std::min(width, height) * 0.004));
    const qreal cw = width / LensFit::SCREEN_COLS, ch = height / LensFit::SCREEN_ROWS;
    painter.setOpacity(step >= 2 ? 0.35 : 1);
    for(int row = 0; row < LensFit::SCREEN_ROWS; ++row) for(int col = 0; col < LensFit::SCREEN_COLS; ++col) {
        const int n = c ? c->screen[row * LensFit::SCREEN_COLS + col] : 0;
        painter.fillRect(QRectF(col * cw + gap, row * ch + gap, cw - 2 * gap, ch - 2 * gap), CellColor(qreal(n) / LensFit::SCREEN_TARGET));
    }
    painter.setOpacity(1);

    const qint64 now = clock.elapsed();
    const bool lost = state == stateCollecting && now - lastComplete > LOST_TIME;
    const qreal font = std::max<qreal>(14, std::round(std::min(width, height) * 0.024));
    const qreal big = std::round(font * 1.6);
    const qreal pulse = 0.5 + 0.5 * std::sin(now / 180.0);
    const bool hasAim = c && c->hasAim && !lost;
    const QPointF aim = hasAim ? QPointF(c->aim[0] * width, c->aim[1] * height) : QPointF();

    // Step 2: the yellow circle to aim at (always inside the screen), joined to the gun's aim by an arrow.
    if(step == 2 && g->hasTarget && !lost) {
        const qreal r = std::min(width, height) * 0.06;
        const qreal tx = std::min(width - r * 1.2, std::max(r * 1.2, g->target[0] * width));
        const qreal ty = std::min(height - r * 1.2, std::max(r * 1.2, g->target[1] * height));
        if(hasAim && std::hypot(tx - aim.x(), ty - aim.y()) > r * 1.5) {
            const qreal ang = std::atan2(ty - aim.y(), tx - aim.x());
            const qreal ex = tx - std::cos(ang) * r * 1.2, ey = ty - std::sin(ang) * r * 1.2;
            const QColor arrow(255, 215, 90, int(std::lround(0.9 * 255)));
            painter.setPen(Pen(arrow, std::max<qreal>(4, font * 0.3)));
            painter.drawLine(aim, QPointF(ex, ey));
            const qreal h = font * 1.2;
            QPolygonF head;
            head << QPointF(ex + std::cos(ang) * h * 0.4, ey + std::sin(ang) * h * 0.4)
                 << QPointF(ex - std::cos(ang - 0.5) * h, ey - std::sin(ang - 0.5) * h)
                 << QPointF(ex - std::cos(ang + 0.5) * h, ey - std::sin(ang + 0.5) * h);
            painter.setPen(Qt::NoPen);
            painter.setBrush(arrow);
            painter.drawPolygon(head);
        }
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(255, 215, 90, int(std::lround((0.25 + 0.25 * pulse) * 255))));
        const qreal halo = r * (1 + 0.15 * pulse);
        painter.drawEllipse(QPointF(tx, ty), halo, halo);
        painter.setBrush(Qt::NoBrush);
        painter.setPen(Pen(YELLOW, std::max<qreal>(4, font * 0.3)));
        painter.drawEllipse(QPointF(tx, ty), r, r);
        painter.setPen(Qt::NoPen);
        painter.setBrush(YELLOW);
        painter.drawEllipse(QPointF(tx, ty), r * 0.15, r * 0.15);
        painter.setBrush(Qt::NoBrush);
    }

    // Step 2 without a reachable circle: closer or farther, with a big animated frame.
    if(step == 2 && !g->hasTarget && !lost) {
        const qreal cx = width / 2, cy = height / 2, base = std::min(width, height) * 0.16;
        const qreal grow = g->closer ? pulse : 1 - pulse;
        painter.setPen(Pen(YELLOW, std::max<qreal>(4, font * 0.3)));
        painter.setBrush(Qt::NoBrush);
        for(int k = 0; k < 3; ++k) {
            const qreal s = base * (0.6 + 0.5 * std::fmod(k / 3.0 + grow / 3, 1.0) * 2);
            painter.setOpacity(0.35 + 0.65 * (k + 1) / 3);
            painter.drawRect(QRectF(cx - s * 16 / 9, cy - s, 2 * s * 16 / 9, 2 * s));
        }
        painter.setOpacity(1);
    }

    // Step 3: a dial; the needle follows the tilt of the gun, each side turns green.
    if(step == 3 && !lost) {
        const qreal cx = width / 2, cy = height * 0.62, R = std::min(width, height) * 0.26;
        const qreal span = 40 * PI / 180, lim = g->limit;
        painter.setBrush(Qt::NoBrush);
        const auto arc = [&](const qreal &from, const qreal &to, const QColor &color) {
            painter.setPen(Pen(color, R * 0.22));
            Arc(painter, cx, cy, R, -PI / 2 + from, -PI / 2 + to);
        };
        arc(-span, span, COLOR_EMPTY);
        arc(-span, -lim, CellColor(g->left));
        arc(lim, span, CellColor(g->right));
        const qreal a = std::max(-span, std::min(span, g->roll));
        painter.setPen(Pen(Qt::white, std::max<qreal>(5, font * 0.35)));
        painter.drawLine(QPointF(cx, cy), QPointF(cx + std::sin(a) * R * 1.12, cy - std::cos(a) * R * 1.12));
        painter.setPen(Qt::NoPen);
        painter.setBrush(Qt::white);
        painter.drawEllipse(QPointF(cx, cy), font * 0.6, font * 0.6);
        painter.setBrush(Qt::NoBrush);
        painter.setFont(Font(big, true));
        painter.setPen(Qt::white);
        TextCentre(painter, cx - std::sin(span) * R * 1.35, cy - std::cos(span) * R * 1.35,
                   g->left >= 1 ? QStringLiteral("✔") : QStringLiteral("↶"));
        TextCentre(painter, cx + std::sin(span) * R * 1.35, cy - std::cos(span) * R * 1.35,
                   g->right >= 1 ? QStringLiteral("✔") : QStringLiteral("↷"));
    }

    // Aim of the gun (uncalibrated, only to show where it points; in step 3 too, the aim has to
    // keep moving while the gun is tilted).
    if(hasAim && state == stateCollecting) {
        const qreal x = aim.x(), y = aim.y();
        painter.setPen(Pen(Qt::white, 3));
        painter.setBrush(Qt::NoBrush);
        painter.drawEllipse(QPointF(x, y), 18, 18);
        painter.drawLine(QPointF(x - 30, y), QPointF(x + 30, y));
        painter.drawLine(QPointF(x, y - 30), QPointF(x, y + 30));
    }

    DrawTextBox(painter, g, step, font, big);

    // Red warning: no valid view for a while. The gun cannot say why (an LED out of view, too
    // much tilt, a view not measured exactly): the text names the usual causes.
    if(lost) {
        const QString text = step <= 1 ? tr("Invalid views: step back a little and keep all four LEDs in view.")
                                       : tr("Invalid views: keep all four LEDs in view and rotate the gun less.");
        painter.setFont(Font(font, true));
        const QFontMetricsF fm(painter.font());
        const qreal w = std::min(width * 0.95, fm.horizontalAdvance(text) + font * 2);
        // Wrapped on narrow screens, not squeezed.
        const QStringList rows = Wrap(painter, text, w - 2 * font);
        const qreal h = font * (0.9 + 1.3 * rows.size());
        painter.fillRect(QRectF((width - w) / 2, height / 2 - font, w, h), QColor(200, 30, 30, int(std::lround(0.9 * 255))));
        painter.setPen(Qt::white);
        for(int i = 0; i < rows.size(); ++i)
            TextTop(painter, (width - w) / 2 + font, height / 2 - font * 0.45 + i * font * 1.3, rows.at(i));
    }

    // Inset: camera view, its rings painted where the LEDs landed (bigger in step 2, in the
    // bottom corner away from the yellow circle).
    if(c && hasInfo && state == stateCollecting) {
        const qreal size = std::min(width, height) * (step == 2 ? 0.3 : 0.2);
        const bool left = step == 2 && g->hasTarget && g->target[0] > 0.5;
        const qreal ix = left ? font : width - size - font, iy = height - size - font * 1.2;
        painter.fillRect(QRectF(ix - 6, iy - font * 1.6, size + 12, size + font * 1.6 + 6), QColor(0, 0, 0, int(std::lround(0.75 * 255))));
        painter.setFont(Font(font * 0.8));
        painter.setPen(Qt::white);
        TextTop(painter, ix, iy - font * 1.35, tr("Camera view"));
        const qreal ox = ix + size / 2, oy = iy + size / 2, rr = size / 2 / 1.06;
        painter.save();
        painter.setClipRect(QRectF(ix, iy, size, size));
        for(int k = LensFit::FIELD_RING_COUNT - 1; k >= 0; --k) {
            const qreal outer = (k + 1 < LensFit::FIELD_RING_COUNT ? LensFit::FIELD_RINGS[k + 1] : 1.5) * rr;
            painter.setBrush(CellColor(qreal(c->field[k]) / LensFit::FIELD_TARGET));
            painter.setPen(Pen(BACKGROUND, 2));
            painter.drawEllipse(QPointF(ox, oy), outer, outer);
        }
        painter.restore();
        if(c->hasLastPoints) {
            painter.setPen(Qt::NoPen);
            painter.setBrush(c->lastComplete ? QColor(Qt::white) : QColor(0xff, 0x40, 0x40));
            for(const LensFit::Point &pt : c->lastPoints) {
                const qreal px = ox + (pt[0] / info.resX - 0.5) * 2 * rr, py = oy + (pt[1] / info.resY - 0.5) * 2 * rr;
                if(px < ix || py < iy || px > ix + size || py > iy + size) continue;
                painter.drawEllipse(QPointF(px, py), 5, 5);
            }
            painter.setBrush(Qt::NoBrush);
        }
    }
}

void AppLensFitWindow::DrawTextBox(QPainter &painter, const LensFit::Guide *g, const int &step, const qreal &font, const qreal &big)
{
    // Text panel: step dots, one short instruction, the overall progress.
    if(state == stateResult || state == stateUnsupported)
        return;
    const qreal width = this->width();
    struct Line { QString text; QColor color; qreal size; };
    QList<Line> lines;
    const QColor white(Qt::white), grey(0xdd, 0xdd, 0xdd);
    if(state == stateWaiting)
        lines << Line{tr("Waiting for the gun..."), white, font};
    if(step == 1) {
        lines << Line{tr("Step 1 of 3: paint the screen"), white, big};
        lines << Line{tr("Move the gun slowly over the whole screen until every square is green."), grey, font};
    } else if(step == 2) {
        lines << Line{tr("Step 2 of 3: edges of the camera view"), white, big};
        lines << Line{g->hasTarget ? tr("Bring the white crosshair into the yellow circle.")
                      : g->closer ? tr("Come closer to the screen until the yellow circle appears.")
                      : tr("Step back from the screen until the yellow circle appears."), YELLOW, font};
    } else if(step == 3) {
        lines << Line{tr("Step 3 of 3: rotate the gun"), white, big};
        lines << Line{tr("Rotate the gun on its own axis, like turning a key, to one side; holding it there, slowly move the white crosshair over the screen until that side turns green, then do the same on the other side."), YELLOW, font};
    } else if(step == 4 || state == stateSolving) {
        const double progress = solveControl ? solveControl->progress.load() : 0;
        lines << Line{tr("Done!"), white, big};
        lines << Line{tr("Computing the lens values... %1% (Esc cancels)").arg(int(std::floor(progress * 100))), grey, font};
    }
    if(lines.isEmpty())
        return;

    const qreal boxW = std::min(width * 0.7, font * 36), pad = font * 0.7;
    QList<Line> wrapped;
    for(const Line &line : lines) {
        painter.setFont(Font(line.size, line.size == big));
        const qreal maxW = boxW - 2 * pad - (line.size == big && step ? big * 3.4 : 0);   // the title leaves room for the step dots
        for(const QString &text : Wrap(painter, line.text, maxW))
            wrapped << Line{text, line.color, line.size};
    }
    const bool bar = state == stateCollecting || state == stateSolving;
    qreal textH = 0;
    for(const Line &line : wrapped) textH += line.size * 1.3;
    const qreal boxH = textH + 2 * pad + (bar ? font * 1.2 : 0);
    const qreal bx = (width - boxW) / 2, by = font;
    painter.fillRect(QRectF(bx, by, boxW, boxH), QColor(0, 0, 0, int(std::lround(0.78 * 255))));
    qreal ty = by + pad;
    for(const Line &line : wrapped) {
        painter.setFont(Font(line.size, line.size == big));
        painter.setPen(line.color);
        TextTop(painter, bx + pad, ty, line.text);
        ty += line.size * 1.3;
    }
    if(bar) {
        const qreal barY = ty + font * 0.2, barW = boxW - 2 * pad;
        const qreal share = state == stateSolving ? (solveControl ? solveControl->progress.load() : 0)
                                                  : (collector ? collector->ProgressNow().total : 0);
        painter.fillRect(QRectF(bx + pad, barY, barW, font * 0.5), COLOR_EMPTY);
        painter.fillRect(QRectF(bx + pad, barY, barW * share, font * 0.5), QColor(0x28, 0xaa, 0x50));
    }
    // Step dots on the right of the title: done = green, current = yellow.
    if(step) for(int k = 1; k <= 3; ++k) {
        const qreal dx = bx + boxW - pad - (4 - k) * big * 1.1 + big * 0.4, dy = by + pad + big * 0.55;
        painter.setPen(Qt::NoPen);
        painter.setBrush(k < step ? QColor(0x28, 0xaa, 0x50) : k == step ? YELLOW : COLOR_EMPTY);
        painter.drawEllipse(QPointF(dx, dy), big * 0.4, big * 0.4);
        painter.setBrush(Qt::NoBrush);
        painter.setPen(k <= step ? BACKGROUND : QColor(0xaa, 0xaa, 0xaa));
        painter.setFont(Font(std::round(big * 0.5), true));
        TextCentre(painter, dx, dy + 1, k < step ? QStringLiteral("✔") : QString::number(k));
    }
    if(state == stateCollecting) {
        painter.setFont(Font(std::round(font * 0.8)));
        painter.setPen(QColor(0xaa, 0xaa, 0xaa));
        TextTop(painter, bx + pad, by + boxH + font * 0.3, tr("Esc cancels."));
    }
}
