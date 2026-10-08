/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Fullscreen Windows interface for Calibration et al.

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

#include "appcali.h"
#include "ui_appcali.h"

#include <QScreen>
#include <QBitmap>
#include <QPainter>
#include <QPainterPath>
#include <QFile>
#include <QHash>
#include <QImage>
#include <QtMath>
#include <QGraphicsEllipseItem>
#include <QRadialGradient>
#include <QFontMetricsF>
#include <QLinearGradient>

namespace {

// ----- IR test blobs (sTestBlobs) ---------------------------------------------------
// Same values as the web app (webapp/js/app/fullscreen.js). The camera drivers of the
// firmware already normalise the data, so every camera uses the same scale here.
// Radius (1920x1080 test space) grows with sqrt(area): IRTEST_BLOB_RADIUS_MAX at
// IRTEST_BLOB_AREA_MAX, the largest blob the PAJ7025 DSP reports. The firmware DFRobot
// driver relies on these two values for its equivalent area when it reads the camera's
// Extended format (ReadDFRobotExtended); with the Full format it reports the box area.
constexpr int IRTEST_BLOB_AREA_MAX = 300;
constexpr qreal IRTEST_BLOB_RADIUS_MAX = 60;
constexpr qreal IRTEST_BLOB_RADIUS_MIN = 6;
// Size of the circles of the emitters seen: multiplies the radius above (min and max
// included). Change only this to draw them bigger or smaller.
constexpr qreal IRTEST_BLOB_RADIUS_SCALE = 3;
constexpr qreal IRTEST_EMITTER_RADIUS = 25; // circle not seen / older firmware, not scaled
// Detected blobs are always above the camera brightness threshold (130-150): this
// range becomes 0..1 of the fill (radial gradient: centre = max, edge = average).
constexpr int IRTEST_BRIGHTNESS_MIN = 128;
constexpr int IRTEST_BRIGHTNESS_MAX = 255;
// Exponential average of radius and brightness (not of the position) against flicker:
// weight of the new 20 Hz sample.
constexpr qreal IRTEST_SMOOTHING = 0.5;
// Blob data older than this (ms) is not used: classic view as with older firmware.
constexpr qint64 IRTEST_BLOBS_MAX_AGE = 250;
constexpr uint8_t IRTEST_BLOBS_VERSION = 1;
constexpr int IRTEST_BLOBS_LENGTH = 21;
constexpr uint8_t IRTEST_BLOB_WEAK = 0x02; // flag: emitter seen but weak (firmware IR_WEAK_MAX_BRIGHTNESS)
constexpr qreal IRTEST_LEGEND_WIDTH = 1010;
constexpr qreal IRTEST_LEGEND_HEIGHT = 184; // horizontal panel, centred below the ESC instruction
constexpr qreal IRTEST_AIM_SCALE = 25 / 24.42; // same geometry as crosshair.svg, with a 50 px circle

QPainterPath IrTestAimTicks()
{
    QPainterPath path;
    const qreal lines[6][4] = {{-30, 0, -18.84, 0}, {0, -29.99, 0, -18.83},
        {29.99, 0, 18.83, 0}, {0, 29.99, 0, 18.83}, {-3.78, 0, 3.8, 0}, {0, -3.78, 0, 3.8}};
    for(uint8_t i = 0; i < 6; ++i) {
        path.moveTo(lines[i][0] * IRTEST_AIM_SCALE, lines[i][1] * IRTEST_AIM_SCALE);
        path.lineTo(lines[i][2] * IRTEST_AIM_SCALE, lines[i][3] * IRTEST_AIM_SCALE);
    }
    return path;
}

// ----- IR view in the calibration window -----------------------------------------------
// Same values as the web app. Firmware that knows caliFlagIrView sends sTestBlobs + sTestCoords
// during calibration too: the crosshair changes colour with the weakest emitter (red when one
// is missing), a small panel in the bottom right corner shows each emitter, and the board refuses
// every target shot (sCaliIrWarning) while one is missing. A weak emitter (orange crosshair)
// is accepted.
constexpr qreal CALI_IR_PANEL_WIDTH = 0.14;     // of the window width (the panel is square)
constexpr qreal CALI_IR_PANEL_MAX_WIDTH = 300;  // px
constexpr qreal CALI_IR_PANEL_MIN_WIDTH = 140;  // px, also on small screens
constexpr qreal CALI_LEGEND_SIZE = 268.8;       // same square as the IR panel at 1920x1080
constexpr qint64 CALI_IR_MAX_AGE = 500;         // ms without coordinates: older firmware, no panel
constexpr int CALI_IR_WARNING_TIME = 4000;      // ms the refusal message stays on screen
constexpr uint8_t CALI_IR_MISSING = 0x01;       // sCaliIrWarning bit
// Colour of each emitter seen in the panel, and of the crosshair from the emitter with the
// lowest brightness; missing = intense red (the crosshair only; the panel keeps the dashed
// circle). Continuous scale of the brightness, without jumps: red at the camera threshold
// (130), light orange at CALI_IR_BRIGHTNESS_WEAK - CALI_IR_BLEND, then through yellow-green
// to light green at CALI_IR_BRIGHTNESS_WEAK + CALI_IR_BLEND, full green at CALI_IR_FULL_GREEN.
// CALI_IR_BRIGHTNESS_WEAK is the firmware limit (IR_WEAK_MAX_BRIGHTNESS). The colours
// follow brightness continuously, independently of the firmware weak flag/hysteresis.
constexpr int CALI_IR_BRIGHTNESS_MIN = 130;
constexpr int CALI_IR_BRIGHTNESS_WEAK = 170;
constexpr int CALI_IR_BLEND = 20;               // half width of the orange to green blend around the limit
constexpr int CALI_IR_FULL_GREEN = 230;
struct CaliIrRgb_s { qreal r, g, b; };
constexpr CaliIrRgb_s CALI_IR_MISSING_RGB   = {255, 20, 20};
constexpr CaliIrRgb_s CALI_IR_WEAK_LOW_RGB  = {255, 40, 40};    // 130
constexpr CaliIrRgb_s CALI_IR_WEAK_HIGH_RGB = {255, 190, 90};   // limit - blend
constexpr CaliIrRgb_s CALI_IR_GOOD_LOW_RGB  = {170, 255, 140};  // limit + blend
constexpr CaliIrRgb_s CALI_IR_GOOD_HIGH_RGB = {0, 220, 0};      // full green

// ----- Rotating target ring in calibration (same values as the web app) ------------------
// Only the outer dashed ring rotates. The original crosshair stays still and unchanged;
// both use the same colour. Acquisition holds the crosshair during a solid white flash.
constexpr qreal CROSSHAIR_RING = 24.42 / 61.44;  // ring radius / crosshair size (crosshair.svg)
constexpr qreal CALI_TARGET_RING_RADIUS = 48.0 / 39; // outer / original ring radius, as in the preview
constexpr qint64 CALI_TARGET_RING_PERIOD = 8000; // ms of one full clockwise turn
constexpr int CALI_TARGET_RING_DASH = 21;       // degrees; ten identical segments, 15-degree gaps
constexpr int CALI_TARGET_RING_STEP = 36;
constexpr qreal CALI_TARGET_RING_WIDTH = 2.1;  // slightly thinner than the original crosshair's 2.4 stroke
constexpr int CALI_ANIMATION_FRAME_TIME = 33;   // ms, about 30 animation frames per second
constexpr int CALI_SHOT_TIME = 120;             // ms: confirms a target acquired, not a save to flash
constexpr int CALI_SHOT_HOLD = 40;              // ms fully white, followed by a quick fade

/// Colour between a and b (t 0..1, clamped).
QColor CaliIrMix(const CaliIrRgb_s &a, const CaliIrRgb_s &b, qreal t)
{
    t = qBound((qreal)0, t, (qreal)1);
    return QColor(qRound(a.r + (b.r - a.r) * t), qRound(a.g + (b.g - a.g) * t), qRound(a.b + (b.b - a.b) * t));
}

/// Calibration IR view: colour of an emitter seen, from its brightness (continuous scale).
QColor CaliIrEmitterColor(const int &brightness)
{
    constexpr int orange = CALI_IR_BRIGHTNESS_WEAK - CALI_IR_BLEND;
    constexpr int green = CALI_IR_BRIGHTNESS_WEAK + CALI_IR_BLEND;
    if(brightness < orange)
        return CaliIrMix(CALI_IR_WEAK_LOW_RGB, CALI_IR_WEAK_HIGH_RGB,
                         (qreal)(brightness - CALI_IR_BRIGHTNESS_MIN) / (orange - CALI_IR_BRIGHTNESS_MIN));
    if(brightness < green)
        return CaliIrMix(CALI_IR_WEAK_HIGH_RGB, CALI_IR_GOOD_LOW_RGB, (qreal)(brightness - orange) / (green - orange));
    return CaliIrMix(CALI_IR_GOOD_LOW_RGB, CALI_IR_GOOD_HIGH_RGB,
                     (qreal)(brightness - green) / (CALI_IR_FULL_GREEN - green));
}

/// Splits each line at the spaces so that no line is longer than maxChars (longer words stay whole).
QStringList CaliIrWrapLines(const QStringList &lines, const int &maxChars)
{
    QStringList wrapped;
    for(const QString &line : lines) {
        QString current;
        for(const QString &word : line.split(' ')) {
            if(!current.isEmpty() && current.length() + 1 + word.length() > maxChars) {
                wrapped << current;
                current = word;
            } else current = current.isEmpty() ? word : current + ' ' + word;
        }
        wrapped << current;
    }
    return wrapped;
}

QColor IrTestNotSeenColor() { return QColor(0xff, 0x30, 0x30); }

/// Glyph (8x8) of the "test" typeface for a character, as in the web app: the 94 glyphs '!'..'~';
/// typographic punctuation mapped to them; accented Latin letters composed from their letter and
/// an accent drawn in the typeface style (rows 0-1, only where the letter leaves them free).
/// Null image when the typeface has no glyph (other scripts).
QImage TestFontGlyph(const QChar &ch)
{
    static QHash<ushort, QImage> cache;
    const auto cached = cache.constFind(ch.unicode());
    if(cached != cache.constEnd())
        return cached.value();

    // Accents over lowercase letters without ascenders (rows 0-1): '#' main, 's' shadow.
    struct Accent_s { ushort mark; const char *rows[2]; };
    static const Accent_s accents[] = {
        {0x0300, {".##s", "..##s"}},    // grave
        {0x0301, {"...##s", "..##s"}},  // acute
        {0x0302, {"..##s", ".#s.#s"}},  // circumflex
        {0x0308, {"", "##s##s"}},       // diaeresis
        {0x0303, {".##s#s", "#s##s"}},  // tilde
    };

    QChar mapped = ch;
    switch(ch.unicode()) {
    case 0x2018: case 0x2019:                         mapped = '\''; break;
    case 0x201C: case 0x201D: case 0x00AB: case 0x00BB: mapped = '"';  break;
    case 0x2013: case 0x2014:                         mapped = '-';  break;
    case 0x00B0:                                      mapped = 'o';  break;
    default: break;
    }

    ushort code = mapped.unicode();
    const Accent_s *accent = nullptr;
    QImage glyph;
    if(code > 126) {
        const QString parts = QString(mapped).normalized(QString::NormalizationForm_D);
        code = parts.at(0).unicode();
        const ushort mark = parts.length() > 1 ? parts.at(1).unicode() : 0;
        for(const Accent_s &known : accents)
            if(known.mark == mark)
                accent = &known;
        // Cedilla/ogonek have no room below the letter: plain letter.
        if(code < 33 || code > 126 || (mark && !accent && mark != 0x0327 && mark != 0x0328))
            code = 0;
    }
    if(code >= 33 && code <= 126) {
        glyph = QImage(QString(":/testFont/testFont/%1").arg(code), "PNG").convertToFormat(QImage::Format_ARGB32);
        if(accent && !glyph.isNull()) {
            const bool dotted = code == 'i' || code == 'j'; // the dot makes room for the accent
            bool topUsed = false;
            for(int y = 0; y < 2; ++y)
                for(int x = 0; x < 8; ++x)
                    topUsed = topUsed || qAlpha(glyph.pixel(x, y)) != 0;
            // An ascender or a capital leaves no room: plain letter.
            if(!topUsed || dotted) {
                const int shift = dotted ? 1 : 0;
                for(int y = 0; y < 2; ++y) {
                    for(int x = 0; x < 8; ++x)
                        glyph.setPixel(x, y, qRgba(0, 0, 0, 0));
                    for(int x = 0; accent->rows[y][x] && x + shift < 8; ++x)
                        if(accent->rows[y][x] != '.')
                            glyph.setPixel(x + shift, y, accent->rows[y][x] == '#' ? qRgba(255, 255, 255, 255)
                                                                                     : qRgba(115, 115, 115, 255));
                }
            }
        }
    }
    cache.insert(ch.unicode(), glyph);
    return glyph;
}

/// Brightness 0..255 -> 0..1 fill over the range of detected blobs.
qreal IrTestBrightnessLevel(const int &value)
{
    const qreal t = (qreal)(value - IRTEST_BRIGHTNESS_MIN) / (IRTEST_BRIGHTNESS_MAX - IRTEST_BRIGHTNESS_MIN);
    return qBound((qreal)0, t, (qreal)1);
}

/// Lines padded to the same length, centred like the web app bitmap text.
QStringList IrTestCentredLines(const QStringList &lines)
{
    int width = 0;
    for(const QString &line : lines)
        width = qMax(width, (int)line.length());
    QStringList centred;
    for(const QString &line : lines) {
        const int left = (width - (int)line.length()) / 2;
        centred << QString(left, QChar(' ')) + line + QString(width - (int)line.length() - left, QChar(' '));
    }
    return centred;
}

} // namespace

AppCaliWindow::AppCaliWindow(QWidget *parent, const int &windowMode)
    : QWidget(parent)
    , ui(new Ui::AppCaliWindow)
{
    ui->setupUi(this);

    mode = windowMode;

    connect(&scene, &AppCaliGraphicsScene::sceneMouseEvent, this, &AppCaliWindow::sceneEventReceiver);
    connect(&scene, &AppCaliGraphicsScene::sceneKeyCloseSignal, this, &AppCaliWindow::sceneKeyCloseReceiver);

    ui->graphicsView->setScene(&scene);

    // limit scale to screen size
    // (lightgun only functions properly on a single screen anyways, so no need to worry I think)
    scene.setSceneRect(QApplication::primaryScreen()->geometry());

    // every mode at least has a header line
    // set a global scale/point size for headers
    if(bitmapText) {
        headerBitmap = new QGraphicsPixmapItem();
        headerBitmap->setScale(GetTextScale(TextHeading));
    } else {
        headerText = new QGraphicsTextItem();
        headerText->setFont(QFont("Monospace", GetTextScale(TextHeading)));
    }

    switch(windowMode) {
    case modeCalibrate:
        this->setWindowTitle("Calibration Window");
        ui->graphicsView->setBackgroundBrush(QBrush(QColor("dimgray")));

        // Right-align translated labels, so the numeric values keep a common column.
        {
            int width = 0;
            for(const QString &prefix : caliTypesPrefixes)
                width = qMax(width, (int)prefix.length());
            for(QString &prefix : caliTypesPrefixes)
                prefix = prefix.rightJustified(width);
        }

        crosshairItem = new QGraphicsSvgItem(":/cali/crosshair.svg");
        // this is supposed to work, but doesn't, so we just offset every setPos command by the center point of the SVG, mult'd by scale
        //crosshairItem->setTransformOriginPoint(crosshairItem->boundingRect().center());

        // add items to scene
        // alignment lines, used during all but the initial Cali stage.
        alignmentLines[0] = new QGraphicsPolygonItem(QPolygonF() << QPointF(scene.sceneRect().center().x(), scene.sceneRect().top()-10)
                                                                 << QPointF(scene.sceneRect().center().x(), scene.sceneRect().bottom()+10)
                                                                 << QPointF(scene.sceneRect().left()-10,    scene.sceneRect().bottom()+10)
                                                                 << QPointF(scene.sceneRect().left()-10,    scene.sceneRect().center().y())
                                                                 << QPointF(scene.sceneRect().right()+10,   scene.sceneRect().center().y())
                                                                 << QPointF(scene.sceneRect().right()+10,  scene.sceneRect().top()-10));
        alignmentLines[0]->setPen(QPen(QColor("white"), 2));
        scene.addItem(alignmentLines[0]);

        if(bitmapText) {
            caliStageBitmap = new QGraphicsPixmapItem();
            caliStageBitmap->setScale(GetTextScale(TextHeading));
            tutorialBitmap = new QGraphicsPixmapItem();
            tutorialBitmap->setScale(GetTextScale(TextSub));
            for(int i = 0; i < 6; i++) {
                profileBitmaps[i] = new QGraphicsPixmapItem();
                profileBitmaps[i]->setScale(GetTextScale(TextSub));
                scene.addItem(profileBitmaps[i]);
                profileBitmaps[i]->setPixmap(GenerateText({caliTypesPrefixes.at(i)}));
                if(i == 0)
                    profileBitmaps[i]->setPos(scene.sceneRect().left()       + (80 * GetTextScale(TextSub)),
                                              scene.sceneRect().center().y() - (24 * GetTextScale(TextSub)));
                else profileBitmaps[i]->setPos(profileBitmaps[i-1]->pos().x(),
                                              profileBitmaps[i-1]->pos().y() + (profileBitmaps[i-1]->boundingRect().height()*GetTextScale(TextSub)));
                profileBitmaps[i]->setVisible(false);
            }

            scene.addItem(headerBitmap);
            scene.addItem(caliStageBitmap);
            scene.addItem(tutorialBitmap);

            // A static reminder for every calibration stage, including final verification.
            caliPositionHintBitmap = new QGraphicsPixmapItem(GenerateText(IrTestCentredLines(
                {tr("Stand in front of the centre of the screen,"), tr("without rotating the lightgun.")})));
            caliPositionHintBitmap->setScale(GetTextScale(TextSub));
            scene.addItem(caliPositionHintBitmap);

            // IR view (caliFlagIrView): hint under the header, refusal message in place of
            // the tutorial. Hidden until the firmware sends the emitters.
            // The colour word (%1) is drawn in the full green of the crosshair.
            {
                const QStringList hint = tr("Preferably shoot when the crosshair is %1.").split("%1");
                const CaliIrRgb_s &green = CALI_IR_GOOD_HIGH_RGB;
                caliIrHintBitmap = new QGraphicsPixmapItem(GenerateTextSegments(
                    {{hint.value(0), QColor()}, {tr("green"), QColor(green.r, green.g, green.b)}, {hint.value(1), QColor()}},
                    GetTextScale(TextSub)));
            }
            caliIrHintBitmap->setScale(GetTextScale(TextSub));
            caliIrHintBitmap->setVisible(false);
            scene.addItem(caliIrHintBitmap);
            caliIrWarningBox = new QGraphicsRectItem();
            caliIrWarningBox->setBrush(QBrush(QColor(0, 0, 0, 153)));
            caliIrWarningBox->setPen(Qt::NoPen);
            caliIrWarningBox->setVisible(false);
            scene.addItem(caliIrWarningBox);
            caliIrWarningBitmap = new QGraphicsPixmapItem();
            caliIrWarningBitmap->setScale(GetTextScale(TextSub));
            caliIrWarningBitmap->setVisible(false);
            scene.addItem(caliIrWarningBitmap);
        } else {
            caliStageText = new QGraphicsTextItem();
            caliStageText->setFont(QFont("Monospace", GetTextScale(TextHeading)));
            tutorialText = new QGraphicsTextItem();
            tutorialText->setFont(QFont("Monospace", GetTextScale(TextSub)));
            for(int i = 0; i < 6; i++) {
                profileText[i] = new QGraphicsTextItem();
                profileText[i]->setFont(QFont("Monospace", GetTextScale(TextSub)));
                scene.addItem(profileText[i]);
            }

            scene.addItem(headerText);
            scene.addItem(caliStageText);
            scene.addItem(tutorialText);

            caliPositionHintText = new QGraphicsTextItem();
            caliPositionHintText->setFont(QFont("Monospace", GetTextScale(TextSub)));
            caliPositionHintText->setDefaultTextColor(Qt::white);
            caliPositionHintText->setHtml("<p align=\"center\">" +
                tr("Stand in front of the centre of the screen,").toHtmlEscaped() + "<br>" +
                tr("without rotating the lightgun.").toHtmlEscaped() + "</p>");
            scene.addItem(caliPositionHintText);
        }

        crosshairItem->setVisible(false);
        crosshairItem->setScale(GetTextScale(Crosshair));
        scene.addItem(crosshairItem);

        // Build the ten arcs once; animation only changes the item's rotation/colour.
        {
            const qreal radius = crosshairItem->boundingRect().width() * CROSSHAIR_RING * CALI_TARGET_RING_RADIUS;
            const QRectF ringRect(-radius, -radius, 2 * radius, 2 * radius);
            QPainterPath ringPath;
            for(int angle = 0; angle < 360; angle += CALI_TARGET_RING_STEP) {
                ringPath.arcMoveTo(ringRect, -angle);
                ringPath.arcTo(ringRect, -angle, -CALI_TARGET_RING_DASH);
            }
            caliTargetRing = new QGraphicsPathItem(ringPath);
            caliTargetRing->setBrush(Qt::NoBrush);
            caliTargetRing->setScale(crosshairItem->scale());
            caliTargetRing->setZValue(-1);
            caliTargetRing->setVisible(false);
            scene.addItem(caliTargetRing);
            connect(&caliTargetRingTimer, &QTimer::timeout, this, &AppCaliWindow::CaliTargetRingUpdate);
        }

        for(QGraphicsEllipseItem *&dot : caliProgressDots) {
            dot = new QGraphicsEllipseItem();
            dot->setZValue(-2); // progress stays below the crosshair and its rings
            scene.addItem(dot);
        }
        {
            QPainterPath shotPath;
            shotPath.addEllipse(caliTargetRing->path().boundingRect());
            caliShotRing = new QGraphicsPathItem(shotPath);
        }
        caliShotRing->setBrush(Qt::NoBrush);
        caliShotRing->setScale(crosshairItem->scale());
        caliShotRing->setPen(QPen(Qt::white, CALI_TARGET_RING_WIDTH, Qt::SolidLine, Qt::RoundCap));
        caliShotRing->setZValue(-1);
        caliShotRing->setVisible(false);
        scene.addItem(caliShotRing);
        caliShotRingTimer.setSingleShot(true);
        connect(&caliShotRingTimer, &QTimer::timeout, this, &AppCaliWindow::CaliShotRingUpdate);

        if(bitmapText) {
            // IR panels below the crosshair/rings, including free aiming during verification.
            caliIrPanel = new QGraphicsPixmapItem();
            caliIrPanel->setZValue(-2);
            caliIrPanel->setVisible(false);
            scene.addItem(caliIrPanel);
            // Static help only; it does not depend on a live sample or consume mouse input.
            caliIrLegend = new QGraphicsPixmapItem();
            caliIrLegend->setZValue(-2);
            caliIrLegend->setAcceptedMouseButtons(Qt::NoButton);
            scene.addItem(caliIrLegend);
            QFile crosshairFile(":/cali/crosshair.svg");
            if(crosshairFile.open(QIODevice::ReadOnly))
                caliCrosshairSvg = crosshairFile.readAll();
            caliIrWarningTimer.setSingleShot(true);
            connect(&caliIrWarningTimer, &QTimer::timeout, this, [this]() {
                caliIrWarningBits = 0;
                CaliIrRefresh();
            });
            caliIrExpiryTimer.setSingleShot(true);
            connect(&caliIrExpiryTimer, &QTimer::timeout, this, [this]() {
                CaliIrRefresh();
                CaliIrArmExpiry();
            });
        }

        ui->graphicsView->setMouseTracking(false);

        CaliModeSet(Cali_Init);
        break;
    case modeAlignment:
        this->setWindowTitle(tr("Alignment Assistant"));
        ui->graphicsView->setBackgroundBrush(QBrush(QColor("darkslategray")));

        if(bitmapText) {
            headerBitmap->setPixmap(GenerateText({tr("       Depending on your desired layout,       "),
                                                  tr("      your IR emitters should be aligned       "),
                                                  tr("         to either one of the two sets         "),
                                                  tr("               of colored boxes:               ")}));
            headerBitmap->setPos(scene.sceneRect().center().x()     - (headerBitmap->boundingRect().center().x() * headerBitmap->scale()),
                                 scene.sceneRect().height() * 0.15  - (headerBitmap->boundingRect().center().y() * headerBitmap->scale()));

            alignmentBitmapLeft = new QGraphicsPixmapItem(GenerateText({tr("      For Square Layout,     "),
                                                                        tr("    the emitters should be   "),
                                                                        tr("    placed at the top and    "),
                                                                        tr("    bottom of the display;   "),
                                                                        tr("each one being aligned to the")}));
            alignmentBitmapLeft->setScale(GetTextScale(TextSmall));
            alignmentBitmapLeft->setPos(scene.sceneRect().width() * 0.02,
                                        (scene.sceneRect().height() * 0.3) + (alignmentBitmapLeft->boundingRect().center().y() * alignmentBitmapLeft->scale()));
            alignmentBitmapColoredLeft = new QGraphicsPixmapItem(GenerateText({tr("      Red-colored boxes.     ")}, QColor(255, 100, 100)));
            alignmentBitmapColoredLeft->setScale(GetTextScale(TextSmall));
            alignmentBitmapColoredLeft->setPos(alignmentBitmapLeft->pos().x(),
                                               alignmentBitmapLeft->pos().y() + (alignmentBitmapLeft->boundingRect().height() * alignmentBitmapLeft->scale()));

            alignmentBitmapRight = new QGraphicsPixmapItem(GenerateText({tr("     For Diamond Layout,     "),
                                                                         tr("the emitters should be placed"),
                                                                         tr("  at the center of the four  "),
                                                                         tr("    edges of the display;    "),
                                                                         tr("each one being aligned to the")}));
            alignmentBitmapRight->setScale(GetTextScale(TextSmall));
            alignmentBitmapRight->setPos(scene.sceneRect().width()   * 0.98   - (alignmentBitmapRight->boundingRect().width()      * alignmentBitmapRight->scale()),
                                         (scene.sceneRect().height() * 0.3) + (alignmentBitmapRight->boundingRect().center().y() * alignmentBitmapRight->scale()));
            alignmentBitmapColoredRight = new QGraphicsPixmapItem(GenerateText({tr("     Green-colored boxes.    ")}, QColor(100, 255, 100)));
            alignmentBitmapColoredRight->setScale(GetTextScale(TextSmall));
            alignmentBitmapColoredRight->setPos(alignmentBitmapRight->pos().x(),
                                                alignmentBitmapRight->pos().y() + (alignmentBitmapRight->boundingRect().height() * alignmentBitmapRight->scale()));

            tutorialBitmap = new QGraphicsPixmapItem(GenerateText({tr("Press ESC to exit alignment tool.")}));
            tutorialBitmap->setScale(GetTextScale(TextSub));
            tutorialBitmap->setPos(scene.sceneRect().width() * 0.05,
                                   (scene.sceneRect().bottom() * 0.9) - (tutorialBitmap->boundingRect().center().y() * tutorialBitmap->scale()));
        } else {
            headerText->setHtml(tr("<p align=\"justify\">Depending on your desired layout,<br>"
                                   "your IR Emitters should be aligned<br>"
                                   "to either one of the two sets<br>"
                                   "of colored boxes:</p>"));
            headerText->setPos(scene.sceneRect().center().x()      - headerText->boundingRect().center().x(),
                               (scene.sceneRect().height() * 0.25) - headerText->boundingRect().center().y());

            alignmentTextLeft = new QGraphicsTextItem();
            alignmentTextLeft->setHtml(tr("<p align=\"justify\">For <b>Square Layout,</b><br>"
                                          "the emitters should be<br>"
                                          "placed at the top and<br>"
                                          "bottom of the display;<br>"
                                          "each one being aligned to the<br>"
                                          "<p style=\"color: tomato\">Red-colored boxes.</p></p>"));
            alignmentTextLeft->setFont(QFont("Monospace", GetTextScale(TextSub)));
            alignmentTextLeft->setPos(scene.sceneRect().width() * 0.075,
                                      (scene.sceneRect().height() * 0.3) + alignmentTextLeft->boundingRect().center().y());

            alignmentTextRight = new QGraphicsTextItem();
            alignmentTextRight->setHtml(tr("<p align=\"justify\">For <b>Diamond Layout,</b><br>"
                                           "the emitters should be placed<br>"
                                           "at the center of the four<br>"
                                           "edges of the display;<br>"
                                           "each one being aligned to the</p>"
                                           "<p style=\"color: palegreen\">Green-colored boxes.</p>"));
            alignmentTextRight->setFont(QFont("Monospace", TextSub));
            alignmentTextRight->setPos(scene.sceneRect().width() * 0.70,
                                       (scene.sceneRect().height() * 0.3) + alignmentTextRight->boundingRect().center().y());

            tutorialText = new QGraphicsTextItem();
            tutorialText->setPlainText(tr("Press ESC to exit alignment tool."));
            tutorialText->setFont(QFont("Monospace", TextSub));
            tutorialText->setPos(scene.sceneRect().width() * 0.05,
                                 (scene.sceneRect().bottom() * 0.9) - tutorialText->boundingRect().center().y());
        }

        // Construct scene with shapes + lines
        for(int i = 0; i < 4; i++) {
            alignmentBoxesSquare[i] = new QGraphicsRectItem();
            alignmentBoxesSquare[i]->setBrush(QBrush(QColor("firebrick")));
            alignmentBoxesDiamond[i] = new QGraphicsRectItem();
            alignmentBoxesDiamond[i]->setBrush(QBrush(QColor("olivedrab")));
            scene.addItem(alignmentBoxesSquare[i]);
            scene.addItem(alignmentBoxesDiamond[i]);
        }

        // square aligners
        /*
        // widescreen
        if(scene.sceneRect().width() / scene.sceneRect().height() > 1.4) {
            alignmentBoxesSquare[0]->setRect(QRectF(QPointF((scene.sceneRect().width()*0.3)-(15*GetTextScale(TextSub)), scene.sceneRect().top()-10),
                                                    QPointF((scene.sceneRect().width()*0.3)+(15*GetTextScale(TextSub)), scene.sceneRect().top()+(10*GetTextScale(TextSub)))));
            alignmentBoxesSquare[1]->setRect(QRectF(QPointF((scene.sceneRect().width()*0.7)-(15*GetTextScale(TextSub)), scene.sceneRect().top()-10),
                                                    QPointF((scene.sceneRect().width()*0.7)+(15*GetTextScale(TextSub)), scene.sceneRect().top()+(10*GetTextScale(TextSub)))));
            alignmentBoxesSquare[2]->setRect(QRectF(QPointF((scene.sceneRect().width()*0.3)-(15*GetTextScale(TextSub)), scene.sceneRect().bottom()+10),
                                                    QPointF((scene.sceneRect().width()*0.3)+(15*GetTextScale(TextSub)), scene.sceneRect().bottom()-(10*GetTextScale(TextSub)))));
            alignmentBoxesSquare[3]->setRect(QRectF(QPointF((scene.sceneRect().width()*0.7)-(15*GetTextScale(TextSub)), scene.sceneRect().bottom()+10),
                                                    QPointF((scene.sceneRect().width()*0.7)+(15*GetTextScale(TextSub)), scene.sceneRect().bottom()-(10*GetTextScale(TextSub)))));
            alignmentLines[0] = new QGraphicsPolygonItem(QPolygonF() << QPointF(scene.sceneRect().width()*0.3, scene.sceneRect().top()-10)
                                                                     << QPointF(scene.sceneRect().width()*0.3, scene.sceneRect().bottom()+10)
                                                                     << QPointF(scene.sceneRect().width()*0.7, scene.sceneRect().bottom()+10)
                                                                     << QPointF(scene.sceneRect().width()*0.7, scene.sceneRect().top()-10));
        // 4:3
        } else {
            alignmentBoxesSquare[0]->setRect(QRectF(QPointF((scene.sceneRect().width()*0.2)-(15*GetTextScale(TextSub)), scene.sceneRect().top()-10),
                                                    QPointF((scene.sceneRect().width()*0.2)+(15*GetTextScale(TextSub)), scene.sceneRect().top()+(10*GetTextScale(TextSub)))));
            alignmentBoxesSquare[1]->setRect(QRectF(QPointF((scene.sceneRect().width()*0.8)-(15*GetTextScale(TextSub)), scene.sceneRect().top()-10),
                                                    QPointF((scene.sceneRect().width()*0.8)+(15*GetTextScale(TextSub)), scene.sceneRect().top()+(10*GetTextScale(TextSub)))));
            alignmentBoxesSquare[2]->setRect(QRectF(QPointF((scene.sceneRect().width()*0.2)-(15*GetTextScale(TextSub)), scene.sceneRect().bottom()+10),
                                                    QPointF((scene.sceneRect().width()*0.2)+(15*GetTextScale(TextSub)), scene.sceneRect().bottom()-(10*GetTextScale(TextSub)))));
            alignmentBoxesSquare[3]->setRect(QRectF(QPointF((scene.sceneRect().width()*0.8)-(15*GetTextScale(TextSub)), scene.sceneRect().bottom()+10),
                                                    QPointF((scene.sceneRect().width()*0.8)+(15*GetTextScale(TextSub)), scene.sceneRect().bottom()-(10*GetTextScale(TextSub)))));
            alignmentLines[0] = new QGraphicsPolygonItem(QPolygonF() << QPointF(scene.sceneRect().width()*0.2, scene.sceneRect().top()-10)
                                                                     << QPointF(scene.sceneRect().width()*0.2, scene.sceneRect().bottom()+10)
                                                                     << QPointF(scene.sceneRect().width()*0.8, scene.sceneRect().bottom()+10)
                                                                     << QPointF(scene.sceneRect().width()*0.8, scene.sceneRect().top()-10));
        }
        */
        {
            // Decouple the geometry from the monitor's aspect ratio.
            // This same unified code block works perfectly for both
            // Widescreen (16:9, 16:10, Ultrawide) and classic 4:3 monitors.
            // Calculate dimensions and center based solely on the scene's height.
            qreal w = scene.sceneRect().width();
            qreal h = scene.sceneRect().height();
            qreal centerX = w / 2.0;

            // BASE / HEIGHT RATIO
            // The 0.711 value replicates the exact proportions of the old code on 16:9 monitors.
            // NOTE FOR FUTURE MODIFICATIONS (e.g., rounding down to 0.70 or lower):
            // - LOWER values: The IR points get closer to each other. Horizontal precision drops and JITTER increases,
            //   but camera rotations are handled better (you can go down to a value of 0.55 at the cost of slight precision loss/more jitter).
            // - HIGHER values: The IR points move further apart. Risk of clipping (points leaving the camera's FOV) during extreme sensor rotations.

            qreal larghezzaBase = h * 0.711; // 0.711
            qreal offset = larghezzaBase / 2.0;

            qreal leftX = centerX - offset;
            qreal rightX = centerX + offset;

            // Corner aligners
            alignmentBoxesSquare[0]->setRect(QRectF(QPointF(leftX-(15*GetTextScale(TextSub)), scene.sceneRect().top()-10),
                                                    QPointF(leftX+(15*GetTextScale(TextSub)), scene.sceneRect().top()+(10*GetTextScale(TextSub)))));

            alignmentBoxesSquare[1]->setRect(QRectF(QPointF(rightX-(15*GetTextScale(TextSub)), scene.sceneRect().top()-10),
                                                    QPointF(rightX+(15*GetTextScale(TextSub)), scene.sceneRect().top()+(10*GetTextScale(TextSub)))));

            alignmentBoxesSquare[2]->setRect(QRectF(QPointF(leftX-(15*GetTextScale(TextSub)), scene.sceneRect().bottom()+10),
                                                    QPointF(leftX+(15*GetTextScale(TextSub)), scene.sceneRect().bottom()-(10*GetTextScale(TextSub)))));

            alignmentBoxesSquare[3]->setRect(QRectF(QPointF(rightX-(15*GetTextScale(TextSub)), scene.sceneRect().bottom()+10),
                                                    QPointF(rightX+(15*GetTextScale(TextSub)), scene.sceneRect().bottom()-(10*GetTextScale(TextSub)))));

            // Connecting polygon
            alignmentLines[0] = new QGraphicsPolygonItem(QPolygonF() << QPointF(leftX, scene.sceneRect().top()-10)
                                                                     << QPointF(leftX, scene.sceneRect().bottom()+10)
                                                                     << QPointF(rightX, scene.sceneRect().bottom()+10)
                                                                     << QPointF(rightX, scene.sceneRect().top()-10));
        }

        // diamond aligners
        alignmentBoxesDiamond[0]->setRect(QRectF(QPointF(scene.sceneRect().center().x()-(15*GetTextScale(TextSub)), scene.sceneRect().top()-10),
                                                 QPointF(scene.sceneRect().center().x()+(15*GetTextScale(TextSub)), scene.sceneRect().top()+(10*GetTextScale(TextSub)))));
        alignmentBoxesDiamond[1]->setRect(QRectF(QPointF(scene.sceneRect().center().x()-(15*GetTextScale(TextSub)), scene.sceneRect().bottom()+10),
                                                 QPointF(scene.sceneRect().center().x()+(15*GetTextScale(TextSub)), scene.sceneRect().bottom()-(10*GetTextScale(TextSub)))));
        alignmentBoxesDiamond[2]->setRect(QRectF(QPointF(scene.sceneRect().left()-10,                               scene.sceneRect().center().y()-(15*GetTextScale(TextSub))),
                                                 QPointF(scene.sceneRect().left()+(10*GetTextScale(TextSub)),       scene.sceneRect().center().y()+(15*GetTextScale(TextSub)))));
        alignmentBoxesDiamond[3]->setRect(QRectF(QPointF(scene.sceneRect().right()+10,                              scene.sceneRect().center().y()-(15*GetTextScale(TextSub))),
                                                 QPointF(scene.sceneRect().right()-(10*GetTextScale(TextSub)),      scene.sceneRect().center().y()+(15*GetTextScale(TextSub)))));
        alignmentLines[1] = new QGraphicsPolygonItem(QPolygonF() << QPointF(scene.sceneRect().center().x(), scene.sceneRect().top()-10)
                                                                 << QPointF(scene.sceneRect().center().x(), scene.sceneRect().bottom()+10)
                                                                 << QPointF(scene.sceneRect().left()-10,    scene.sceneRect().bottom()+10)
                                                                 << QPointF(scene.sceneRect().left()-10,    scene.sceneRect().center().y())
                                                                 << QPointF(scene.sceneRect().right()+10,   scene.sceneRect().center().y())
                                                                 << QPointF(scene.sceneRect().right()+10,  scene.sceneRect().top()-10));

        alignmentLines[0]->setPen(QPen(QColor("firebrick"), 2));
        alignmentLines[1]->setPen(QPen(QColor("olivedrab"), 2));

        for(int i = 0; i < 2; ++i)
            scene.addItem(alignmentLines[i]);

        if(bitmapText) {
            scene.addItem(tutorialBitmap);
            scene.addItem(alignmentBitmapLeft);
            scene.addItem(alignmentBitmapColoredLeft);
            scene.addItem(alignmentBitmapRight);
            scene.addItem(alignmentBitmapColoredRight);
            scene.addItem(headerBitmap);
        } else {
            scene.addItem(tutorialText);
            scene.addItem(alignmentTextLeft);
            scene.addItem(alignmentTextRight);
            scene.addItem(headerText);
        }

        break;
    case modeIRTest:
        this->setWindowTitle(tr("IR Emitters Test"));
        ui->graphicsView->setBackgroundBrush(QBrush(QColor("midnightblue")));

        qreal scaleX = scene.sceneRect().width() / 1920.0;
        qreal scaleY = scene.sceneRect().height() / 1080.0;
        qreal scale = qMin(scaleX, scaleY);
        qreal offsetX = scene.sceneRect().left() + (scene.sceneRect().width() - (1920.0 * scale)) / 2.0;
        qreal offsetY = scene.sceneRect().top() + (scene.sceneRect().height() - (1080.0 * scale)) / 2.0;

        if(bitmapText) {
            headerBitmap->setPixmap(GenerateText({tr("     The array of shapes displayed onscreen     "),
                                                  tr("represents the emitters that the camera can see."),
                                                  tr("   The colored points should move opposite to   "),
                                                  tr("    your aim, and the gray crosshair should be  "),
                                                  tr("          lining up with your gun sight.        ")}));
            headerBitmap->setPos(scene.sceneRect().center().x()     - (headerBitmap->boundingRect().center().x() * headerBitmap->scale()),
                                 (scene.sceneRect().bottom() * 0.1) - (headerBitmap->boundingRect().center().y() * headerBitmap->scale()));
            scene.addItem(headerBitmap);

            tutorialBitmap = new QGraphicsPixmapItem(GenerateText({tr("Press ESC to exit test mode.")}));
            tutorialBitmap->setScale(GetTextScale(TextSub));
            tutorialBitmap->setPos(scene.sceneRect().center().x() - (tutorialBitmap->boundingRect().center().x() * tutorialBitmap->scale()),
                                   scene.sceneRect().bottom() * 0.85);
            scene.addItem(tutorialBitmap);
        } else {
            headerText->setHtml(tr("<p align=justify>The array of shapes onscreen<br>"
                                   "represents the emitters that the camera can see.<br>"
                                   "The points should move opposite to your aim,<br>"
                                   "and the gray crosshair should be lining up with your gun sight.</p>"));
            headerText->setPos(scene.sceneRect().center().x() - headerText->boundingRect().center().x(),
                               (scene.sceneRect().bottom() * 0.1) - headerText->boundingRect().center().y());
            scene.addItem(headerText);

            tutorialText = new QGraphicsTextItem();
            tutorialText->setPlainText(tr("Press ESC to exit test mode."));
            tutorialText->setFont(QFont("Monospace", GetTextScale(TextSub)));
            tutorialText->setPos(scene.sceneRect().center().x() - tutorialText->boundingRect().center().x(), scene.sceneRect().bottom() * 0.85);
            scene.addItem(tutorialText);
        }

        for(int i = 0; i < testPointsCount; i++) {
            testPoints[i] = new QGraphicsEllipseItem();
            testPoints[i]->setTransform(QTransform::fromScale(scale, scale));
            testPoints[i]->setPos(offsetX, offsetY);
            scene.addItem(testPoints[i]);
        }

        // Mouse position must still cover the entire monitor area.
        testPoints[testPointMed]->setTransform(QTransform::fromScale(scaleX, scaleY));
        testPoints[testPointMed]->setPos(scene.sceneRect().left(), scene.sceneRect().top());
        // A child inherits the circle's existing per-axis screen transform and stacking.
        // Its path is built once; only its centre moves when new coordinates arrive.
        testAimTicks = new QGraphicsPathItem(testPoints[testPointMed]);
        testAimTicks->setPath(IrTestAimTicks());
        QPen aimPen(Qt::gray, 1.44 * IRTEST_AIM_SCALE);
        aimPen.setCapStyle(Qt::SquareCap);
        testAimTicks->setPen(aimPen);
        testAimTicks->setVisible(false);

        testScale = scale;
        testOffsetX = offsetX;
        testOffsetY = offsetY;
        testBlobDebug = QCoreApplication::arguments().contains("--irdebug");

        // Red X of the emitters not seen (sTestBlobs): drawn with its emitter,
        // below the mouse circle, the D point and the box like in the web app.
        for(int i = 0; i < 4; ++i) {
            testNotSeenMarks[i] = new QGraphicsPathItem();
            testNotSeenMarks[i]->setTransform(QTransform::fromScale(scale, scale));
            testNotSeenMarks[i]->setPos(offsetX, offsetY);
            QPen markPen(IrTestNotSeenColor(), 3);
            markPen.setCapStyle(Qt::RoundCap);
            testNotSeenMarks[i]->setPen(markPen);
            testNotSeenMarks[i]->setVisible(false);
            scene.addItem(testNotSeenMarks[i]);
            testNotSeenMarks[i]->stackBefore(testPoints[testPointMed]);
        }

        testBox = new QGraphicsPolygonItem();
        testBox->setTransform(QTransform::fromScale(scale, scale));
        testBox->setPos(offsetX, offsetY);
        testBox->setPen(QPen(Qt::gray, 2));
        scene.addItem(testBox);

        // --irdebug: area and brightness under each emitter, over everything else.
        for(int i = 0; i < 4; ++i) {
            testBlobInfo[i] = new QGraphicsPixmapItem();
            testBlobInfo[i]->setScale(GetTextScale(TextSmall));
            testBlobInfo[i]->setVisible(false);
            scene.addItem(testBlobInfo[i]);
        }

        testPoints[testPointTL] ->setPen(QPen(Qt::green, 3));
        testPoints[testPointTR] ->setPen(QPen(Qt::green, 3));
        testPoints[testPointBL] ->setPen(QPen(Qt::cyan,  3));
        testPoints[testPointBR] ->setPen(QPen(Qt::cyan,  3));
        testPoints[testPointMed]->setPen(QPen(Qt::gray, 2.4 * IRTEST_AIM_SCALE));
        testPoints[testPointD]  ->setPen(QPen(Qt::red,   3));

        // IR vertices inside the camera FOV are normally shown as hollow circles.
        testPoints[testPointTL] ->setBrush(QBrush(Qt::NoBrush));
        testPoints[testPointTR] ->setBrush(QBrush(Qt::NoBrush));
        testPoints[testPointBL] ->setBrush(QBrush(Qt::NoBrush));
        testPoints[testPointBR] ->setBrush(QBrush(Qt::NoBrush));
        testLegend = new QGraphicsPixmapItem();
        testLegend->setAcceptedMouseButtons(Qt::NoButton);
        testLegend->setZValue(-1); // below live IR points, aim and reconstruction lines
        scene.addItem(testLegend);
        IrTestDrawLegend();
        break;
    }
}

AppCaliWindow::~AppCaliWindow()
{
    delete ui;
}

int AppCaliWindow::GetTextScale(const int &type)
{
    if(scale == -1) {
        if(scene.sceneRect().height() >= 1440)
            scale = ScaleHiDPI;
        else if(scene.sceneRect().height() >= 1080)
            scale = ScaleBig;
        else if(scene.sceneRect().height() >= 720)
            scale = ScaleSmall;
        else scale = ScaleTiny;
    }

    switch(scale) {
    case ScaleHiDPI:
        switch(type) {
        case TextHeading:
            if(bitmapText) return 5;
            else return 20;
        case TextSub:
            if(bitmapText) return 4;
            else return 18;
        case TextSmall:
            if(bitmapText) return 3;
            else return 16;
        case Crosshair:
            return 4;
        }
    case ScaleBig:
        switch(type) {
        case TextHeading:
            if(bitmapText) return 4;
            else return 18;
        case TextSub:
            if(bitmapText) return 3;
            else return 16;
        case TextSmall:
            if(bitmapText) return 2;
            else return 14;
        case Crosshair:
            return 3;
        }
    case ScaleSmall:
        switch(type) {
        case TextHeading:
            if(bitmapText) return 3;
            else return 16;
        case TextSub:
            if(bitmapText) return 2;
            else return 14;
        case TextSmall:
            if(bitmapText) return 2;
            else return 12;
        case Crosshair:
            return 3;
        }
    case ScaleTiny:
        switch(type) {
        case TextHeading:
            if(bitmapText) return 2;
            else return 14;
        case TextSub:
            if(bitmapText) return 2;
            else return 12;
        case TextSmall:
            if(bitmapText) return 1;
            else return 10;
        case Crosshair:
            return 2;
        }
    }

    // Shouldn't be possible to reach this, otherwise something horrible's gone wrong.
    return 0;
}

QPixmap AppCaliWindow::GenerateTextSegments(const QList<QPair<QString, QColor>> &segments, const int &renderScale)
{
    // One line made of GenerateText pieces side by side: 8 logical pixels per character.
    const int s = qMax(1, renderScale);
    int chars = 0;
    for(const auto &segment : segments)
        chars += segment.first.length();

    QPixmap line(8 * s * chars, 8 * s);
    line.fill(QColor(0,0,0,0));
    line.setDevicePixelRatio(s);
    QPainter painter(&line);
    int x = 0;
    for(const auto &segment : segments) {
        if(segment.first.isEmpty())
            continue;
        painter.drawPixmap(QPoint(x, 0), GenerateText({segment.first}, segment.second, s));
        x += 8 * segment.first.length();
    }
    painter.end();
    return line;
}

QPixmap AppCaliWindow::GenerateText(const QStringList &strings, const QColor &tintColor, const int &renderScale)
{
    // With renderScale > 1 the pixmap is drawn at the size it is shown (its item scale is
    // renderScale): the typeface glyphs are enlarged pixel by pixel, so they look the same as
    // before, and characters not in the typeface are drawn smooth with the system font, like
    // the web app.
    const int s = qMax(1, renderScale);
    const int cell = 8 * s;
    // Colours of the typeface (white with a grey shadow), or of the tint (shadow 140 units darker).
    const QColor mainColor = tintColor.isValid() ? tintColor : QColor(255, 255, 255);
    const QColor shadowColor = tintColor.isValid() ? QColor(tintColor.red()   > 140 ? tintColor.red()   - 140 : 0,
                                                            tintColor.green() > 140 ? tintColor.green() - 140 : 0,
                                                            tintColor.blue()  > 140 ? tintColor.blue()  - 140 : 0)
                                                   : QColor(115, 115, 115);

    QVector<QPixmap> imageBuffer;
    int maxWidth = 0;

    // used to copy data from resources bitmaps to individual images.
    QPainter painter;

    for(auto const &line : strings) {
        imageBuffer << QPixmap(cell*line.length(), cell);
        // init with transparent color
        imageBuffer.last().fill(QColor(0,0,0,0));
        painter.begin(&imageBuffer.last());

        if(maxWidth < imageBuffer.last().width()) maxWidth = imageBuffer.last().width();

        for(int i = 0; i < line.length(); i++)
            if(line.at(i) >= '!' && line.at(i) <= '~')
                painter.drawPixmap(QRect(cell*i, 0, cell, cell), QPixmap(QString(":/testFont/testFont/%1").arg((int)line.at(i).unicode()), "PNG"));
            else if(line.at(i).isSpace())
                continue;
            else if(const QImage glyph = TestFontGlyph(line.at(i)); !glyph.isNull())
                painter.drawImage(QRect(cell*i, 0, cell, cell), glyph); // accented letters, punctuation
            else {
                // Not in the typeface (other scripts): system font in the same cell,
                // in the text colours with the shadow one pixel (of the typeface) lower right.
                // The fonts the browser uses for "monospace" (Windows, Linux, macOS), as in the web app.
                QFont font;
                font.setFamilies({"Consolas", "DejaVu Sans Mono", "Menlo", "Monospace"});
                font.setStyleHint(QFont::Monospace);
                font.setBold(true);
                if(s > 1)
                    font.setPixelSize(qRound(cell * 0.95));
                else {
                    font.setPixelSize(8);
                    font.setStyleStrategy(QFont::NoAntialias);
                }
                painter.setFont(font);
                painter.setPen(shadowColor);
                painter.drawText(QRect(cell*i + s, s, cell, cell), Qt::AlignCenter, QString(line.at(i)));
                painter.setPen(mainColor);
                painter.drawText(QRect(cell*i, 0, cell, cell), Qt::AlignCenter, QString(line.at(i)));
            }

        painter.end();
    }

    QPixmap combinedBuffer(maxWidth, cell*strings.count());
    combinedBuffer.fill(QColor(0,0,0,0));

    painter.begin(&combinedBuffer);
    for(int i = 0; i < imageBuffer.count(); i++)
        painter.drawPixmap(QPoint(0,cell*i), imageBuffer.at(i));

    if(tintColor.isValid()) {
        QBitmap mask1 = combinedBuffer.createMaskFromColor(QColor(255, 255, 255), Qt::MaskOutColor);
        QBitmap mask2 = combinedBuffer.createMaskFromColor(QColor(115, 115, 115), Qt::MaskOutColor);
        painter.setPen(QPen(tintColor));
        painter.drawPixmap(combinedBuffer.rect(), mask1);
        // shadowed area gets tinted with a 140 units darker shade
        // need to be careful not to dip into negatives, as that invalidates the QColor
        painter.setPen(QPen(shadowColor));
        painter.drawPixmap(combinedBuffer.rect(), mask2);
    }

    painter.end();

    combinedBuffer.setDevicePixelRatio(s);
    return combinedBuffer;
}

void AppCaliWindow::CaliModeSet(const int &caliStage)
{
    if(caliStage < Cali_Init || caliStage > Cali_End)
        return;
    if(caliShotStage >= 0 && caliShotTimer.elapsed() >= CALI_SHOT_TIME)
        CaliShotRingUpdate();
    // Keep data/stage handling immediate; only defer the crosshair's new position.
    const QPointF previousPosition = caliTargetRing != nullptr ? crosshairItem->pos() : QPointF();
    if(caliShotStage >= 0)
        crosshairItem->setPos(caliNextTargetPos); // the switch must work from the latest logical position
    switch(caliStage) {
    case Cali_Init:
        ui->graphicsView->setMouseTracking(false);
        mouseCanBeTracked = false;

        alignmentLines[0]->setVisible(false);

        crosshairItem->setVisible(true);
        crosshairItem->setPos(scene.sceneRect().center().x() - (crosshairItem->boundingRect().center().x()*crosshairItem->scale()),
                              scene.sceneRect().center().y() - (crosshairItem->boundingRect().center().y()*crosshairItem->scale()));

        // reset calibration info, in case we're restarted to here
        topOffset = -1, bottomOffset = -1, leftOffset = -1, rightOffset = -1, topLeftLed = -1, topRightLed = -1;

        if(bitmapText) {
            caliStageBitmap->setPixmap( GenerateText({tr("Calibration: step 1 of 6")}));
            caliStageBitmap->setPos(scene.sceneRect().center().x()      - (caliStageBitmap->boundingRect().center().x() * caliStageBitmap->scale()),
                                    (scene.sceneRect().height() * 0.25) - (caliStageBitmap->boundingRect().center().y() * caliStageBitmap->scale()));

            headerBitmap->setPixmap(    GenerateText({tr("Shoot at the target to start calibration.")}));
            headerBitmap->setPos(scene.sceneRect().center().x() - (headerBitmap->boundingRect().center().x() * headerBitmap->scale()),
                                 caliStageBitmap->pos().y()     + (caliStageBitmap->boundingRect().height()  * caliStageBitmap->scale()));

            tutorialBitmap->setPixmap(  GenerateText({tr("Calibration can be exited without changes"),
                                                    tr("  by pressing either Button A, Button B, "),
                                                    tr("       or Button C (if available).       ")}));
            tutorialBitmap->setPos(scene.sceneRect().center().x()     - (tutorialBitmap->boundingRect().center().x() * tutorialBitmap->scale()),
                                   (scene.sceneRect().bottom() * 0.8) - (tutorialBitmap->boundingRect().center().y() * tutorialBitmap->scale()));

            for(int i = 0; i < 6; i++) {
                profileBitmaps[i]->setVisible(false);
                profileBitmaps[i]->setPixmap(GenerateText({caliTypesPrefixes.at(i)}));
            }
        } else {
            caliStageText->setPlainText(tr("Calibration: step 1 of 6"));
            caliStageText->setPos(scene.sceneRect().center().x()      - caliStageText->boundingRect().center().x(),
                                  (scene.sceneRect().bottom() * 0.25) - caliStageText->boundingRect().center().y());

            headerText->setPlainText(tr("Shoot at the target to start calibration."));
            headerText->setPos(scene.sceneRect().center().x() - headerText->boundingRect().center().x(),
                               caliStageText->pos().y()       + caliStageText->boundingRect().height());

            tutorialText->setHtml(tr("<p align=\"center\">Calibration can be exited without changes<br>"
                                     "by pressing either <i>Button A,</i> <i>Button B,</i><br>"
                                     "or <i>Button C (if available).</i></p>"));
            tutorialText->setPos(scene.sceneRect().center().x()     - tutorialText->boundingRect().center().x(),
                                 (scene.sceneRect().bottom() * 0.8) - tutorialText->boundingRect().center().y());
        }

        break;
    case Cali_Top:
        alignmentLines[0]->setVisible(true);

        crosshairItem->setPos(scene.sceneRect().center().x() - (crosshairItem->boundingRect().center().x()*crosshairItem->scale()),
                              scene.sceneRect().top() - (crosshairItem->boundingRect().center().y()*crosshairItem->scale()));

        if(bitmapText) {
            caliStageBitmap->setPixmap(GenerateText({tr("Calibration: step 2 of 6")}));
            caliStageBitmap->setPos(scene.sceneRect().center().x()      - (caliStageBitmap->boundingRect().center().x() * caliStageBitmap->scale()),
                                    (scene.sceneRect().height() * 0.25) - (caliStageBitmap->boundingRect().center().y() * caliStageBitmap->scale()));

            headerBitmap->setPixmap(GenerateText({tr("Shoot at the top edge of the screen.")}));
            headerBitmap->setPos(scene.sceneRect().center().x() - (headerBitmap->boundingRect().center().x() * headerBitmap->scale()),
                                 caliStageBitmap->pos().y()     + (caliStageBitmap->boundingRect().height()  * caliStageBitmap->scale()));

            tutorialBitmap->setPixmap(GenerateText({tr("Calibration can be exited without changes"),
                                                    tr("  by pressing either Button A, Button B, "),
                                                    tr("       or Button C (if available).       ")}));
            tutorialBitmap->setPos(scene.sceneRect().center().x()     - (tutorialBitmap->boundingRect().center().x() * tutorialBitmap->scale()),
                                   (scene.sceneRect().bottom() * 0.8) - (tutorialBitmap->boundingRect().center().y() * tutorialBitmap->scale()));

            for(int i = 0; i < 6; i++)
                profileBitmaps[i]->setVisible(true);
        } else {
            caliStageText->setPlainText(tr("Calibration: step 2 of 6"));
            caliStageText->setPos(scene.sceneRect().center().x()      - caliStageText->boundingRect().center().x(),
                                  (scene.sceneRect().bottom() * 0.25) - caliStageText->boundingRect().center().y());

            headerText->setPlainText(tr("Shoot at the top edge of the screen."));
            headerText->setPos(scene.sceneRect().center().x() - headerText->boundingRect().center().x(),
                               caliStageText->pos().y()       + caliStageText->boundingRect().height());

            tutorialText->setHtml(tr("<p align=\"center\">Calibration can be exited without changes<br>"
                                     "by pressing either <i>Button A,</i> <i>Button B,</i><br>"
                                     "or <i>Button C (if available).</i></p>"));
            tutorialText->setPos(scene.sceneRect().center().x()     - tutorialText->boundingRect().center().x(),
                                 (scene.sceneRect().bottom() * 0.8) - tutorialText->boundingRect().center().y());
        }

        break;
    case Cali_Bottom:
        crosshairItem->setPos(scene.sceneRect().center().x() - (crosshairItem->boundingRect().center().x()*crosshairItem->scale()),
                              scene.sceneRect().bottom()     - (crosshairItem->boundingRect().center().y()*crosshairItem->scale()));

        if(bitmapText) {
            caliStageBitmap->setPixmap(GenerateText({tr("Calibration: step 3 of 6")}));

            headerBitmap->setPixmap(GenerateText({tr("Shoot at the bottom edge of the screen.")}));
            headerBitmap->setPos(scene.sceneRect().center().x() - (headerBitmap->boundingRect().center().x() * headerBitmap->scale()),
                                 caliStageBitmap->pos().y()     + (caliStageBitmap->boundingRect().height()  * caliStageBitmap->scale()));
        } else {
            caliStageText->setPlainText(tr("Calibration: step 3 of 6"));

            headerText->setPlainText(tr("Shoot at the bottom edge of the screen."));
            headerText->setPos(scene.sceneRect().center().x() - headerText->boundingRect().center().x(),
                               caliStageText->pos().y()       + caliStageText->boundingRect().height());
        }

        break;
    case Cali_Left:
        crosshairItem->setPos(scene.sceneRect().left()       - (crosshairItem->boundingRect().center().x() * crosshairItem->scale()),
                              scene.sceneRect().center().y() - (crosshairItem->boundingRect().center().y() * crosshairItem->scale()));

        if(bitmapText) {
            caliStageBitmap->setPixmap(GenerateText({tr("Calibration: step 4 of 6")}));

            headerBitmap->setPixmap(GenerateText({tr("Shoot at the left edge of the screen.")}));
            headerBitmap->setPos(scene.sceneRect().center().x() - (headerBitmap->boundingRect().center().x() * headerBitmap->scale()),
                                 caliStageBitmap->pos().y()     + (caliStageBitmap->boundingRect().height()  * caliStageBitmap->scale()));
        } else {
            caliStageText->setPlainText(tr("Calibration: step 4 of 6"));

            headerText->setPlainText(tr("Shoot at the left edge of the screen."));
            headerText->setPos(scene.sceneRect().center().x() - headerText->boundingRect().center().x(),
                               caliStageText->pos().y()       + caliStageText->boundingRect().height());
        }

        break;
    case Cali_Right:
        crosshairItem->setPos(scene.sceneRect().right() - (crosshairItem->boundingRect().center().x()*crosshairItem->scale()),
                              scene.sceneRect().center().y() - (crosshairItem->boundingRect().center().y()*crosshairItem->scale()));

        if(bitmapText) {
            caliStageBitmap->setPixmap(GenerateText({tr("Calibration: step 5 of 6")}));

            headerBitmap->setPixmap(GenerateText({tr("Shoot at the right edge of the screen.")}));
            headerBitmap->setPos(scene.sceneRect().center().x() - (headerBitmap->boundingRect().center().x() * headerBitmap->scale()),
                                 caliStageBitmap->pos().y()     + (caliStageBitmap->boundingRect().height()  * caliStageBitmap->scale()));
        } else {
            caliStageText->setPlainText(tr("Calibration: step 5 of 6"));

            headerText->setPlainText(tr("Shoot at the right edge of the screen."));
            headerText->setPos(scene.sceneRect().center().x() - headerText->boundingRect().center().x(),
                               caliStageText->pos().y()       + caliStageText->boundingRect().height());
        }

        break;
    case Cali_Center:
        crosshairItem->setPos(scene.sceneRect().center().x() - (crosshairItem->boundingRect().center().x()*crosshairItem->scale()),
                              scene.sceneRect().center().y() - (crosshairItem->boundingRect().center().y()*crosshairItem->scale()));

        if(bitmapText) {
            caliStageBitmap->setPixmap(GenerateText({tr("Calibration: step 6 of 6")}));

            headerBitmap->setPixmap(GenerateText({tr("Shoot at the final target in the center.")}));
            headerBitmap->setPos(scene.sceneRect().center().x() - (headerBitmap->boundingRect().center().x() * headerBitmap->scale()),
                                 caliStageBitmap->pos().y()     + (caliStageBitmap->boundingRect().height()  * caliStageBitmap->scale()));
        } else {
            caliStageText->setPlainText(tr("Calibration: step 6 of 6"));

            headerText->setPlainText(tr("Shoot at the final target in the center."));
            headerText->setPos(scene.sceneRect().center().x() - headerText->boundingRect().center().x(),
                               caliStageText->pos().y()       + caliStageText->boundingRect().height());
        }

        break;
    case Cali_Verify:
        ui->graphicsView->setMouseTracking(true);
        mouseCanBeTracked = true;

        if(bitmapText) {
            if(topOffset    >= -32768  &&  topOffset    <= 32768    &&
                bottomOffset >= -32768  &&  bottomOffset <= 32768    &&
                leftOffset   >= -32768  &&  leftOffset   <= 32768    &&
                rightOffset  >= -32768  &&  rightOffset  <= 32768    &&
                topLeftLed   >= -32768  &&  topLeftLed   <= 32768    &&
                topRightLed  >= -32768  &&  topRightLed  <= 32768) {

                caliStageBitmap->setPixmap( GenerateText({tr("Verify aiming:")}));
                caliStageBitmap->setPos(scene.sceneRect().center().x()      - (caliStageBitmap->boundingRect().center().x() * caliStageBitmap->scale()),
                                        scene.sceneRect().height() * 0.15   - (caliStageBitmap->boundingRect().center().y() * caliStageBitmap->scale()));

                headerBitmap->setPixmap(GenerateText({tr("    Confirm that the bullseye     "),
                                                      tr("   lines up with the gun sight.   "),
                                                      tr("If this calibration is acceptable,"),
                                                      tr("  confirm by pulling the trigger. ")}));
                headerBitmap->setPos(scene.sceneRect().center().x() - (headerBitmap->boundingRect().center().x() * headerBitmap->scale()),
                                     caliStageBitmap->pos().y()     + (caliStageBitmap->boundingRect().height()  * caliStageBitmap->scale()));

                tutorialBitmap->setPixmap(GenerateText({tr("       If this target accuracy isn't desirable,      "),
                                                        tr("          press either Button A or Button B          "),
                                                        tr("         to restart the calibration process.         "),
                                                        tr(""),
                                                        tr("[You can also exit calibration without saving changes"),
                                                        tr("        by pressing Button C (if available).]        ")}));
                tutorialBitmap->setPos(scene.sceneRect().center().x()       - (tutorialBitmap->boundingRect().center().x() * tutorialBitmap->scale()),
                                       (scene.sceneRect().bottom() * 0.8)   - (tutorialBitmap->boundingRect().center().y() * tutorialBitmap->scale()));
            } else if(topLeftLed == -1 || topRightLed == -1) {
                caliStageBitmap->setPixmap( GenerateText({tr("Verify aiming:")}));
                caliStageBitmap->setPos(scene.sceneRect().center().x()      - (caliStageBitmap->boundingRect().center().x() * caliStageBitmap->scale()),
                                        scene.sceneRect().height() * 0.15   - (caliStageBitmap->boundingRect().center().y() * caliStageBitmap->scale()));

                tutorialBitmap->setPixmap(GenerateText({tr("")}));
            } else {
                caliStageBitmap->setPixmap(GenerateText({tr("WARNING: Possibly Malformed Calibration!!")}, QColor(225,25,25)));
                caliStageBitmap->setPos(scene.sceneRect().center().x()      - (caliStageBitmap->boundingRect().center().x() * caliStageBitmap->scale()),
                                        scene.sceneRect().height() * 0.15   - (caliStageBitmap->boundingRect().center().y() * caliStageBitmap->scale()));

                headerBitmap->setPixmap(GenerateText({tr("  The current pending values for this profile  "),
                                                      tr("will likely cause incorrect or broken tracking.")},
                                                     QColor(225,25,25)));
                headerBitmap->setPos(scene.sceneRect().center().x() - (headerBitmap->boundingRect().center().x() * headerBitmap->scale()),
                                     caliStageBitmap->pos().y()     + (caliStageBitmap->boundingRect().height()  * caliStageBitmap->scale()));

                tutorialBitmap->setPixmap(GenerateText({tr("  Press Button A or Button B to restart calibration, "),
                                                        tr("   or pull trigger to continue with these settings.  "),
                                                        tr(""),
                                                        tr("[You can also exit calibration without saving changes"),
                                                        tr("        by pressing Button C (if available).]        ")},
                                                       QColor(225,25,25)));
                tutorialBitmap->setPos(scene.sceneRect().center().x()       - (tutorialBitmap->boundingRect().center().x() * tutorialBitmap->scale()),
                                       (scene.sceneRect().bottom() * 0.8)   - (tutorialBitmap->boundingRect().center().y() * tutorialBitmap->scale()));
            }
        } else {
            caliStageText->setPlainText(tr("Verify aiming:"));
            caliStageText->setPos(scene.sceneRect().center().x()        - caliStageText->boundingRect().center().x(),
                                  (scene.sceneRect().bottom() * 0.25)   - caliStageText->boundingRect().center().y());

            headerText->setHtml(tr("<p align=\"center\">Confirm that the bullseye lines up with gun sight.<br>"
                                   "If this calibration is acceptable, confirm by pulling the trigger.</p>"));
            headerText->setPos(scene.sceneRect().center().x()   - headerText->boundingRect().center().x(),
                               caliStageText->pos().y()         + caliStageText->boundingRect().height());

            tutorialText->setHtml(tr("<p align=\"center\">If the target accuracy isn't desirable,<br>"
                                     "press either <i>Button A</i> or <i>Button B</i><br>"
                                     "to restart the calibration process.<br>"
                                     "<br>"
                                     "[You can also exit calibration without saving changes<br>"
                                     "by pressing <i>Button C (if available).</i>]</p>"));
            tutorialText->setPos(scene.sceneRect().center().x() - tutorialText->boundingRect().center().x(),
                                 (scene.sceneRect().bottom() * 0.8) - tutorialText->boundingRect().center().y());
        }

        break;
    case Cali_End:
        caliTargetStage = Cali_End;
        caliTargetRingTimer.stop();
        if(caliTargetRing != nullptr)
            caliTargetRing->setVisible(false);
        caliShotRingTimer.stop();
        caliShotStage = -1;
        if(caliShotRing != nullptr)
            caliShotRing->setVisible(false);
        emit WindowExiting(mode, topOffset, bottomOffset, leftOffset, rightOffset, topLeftLed, topRightLed);
        return;
    }

    if(caliPositionHintText != nullptr)
        caliPositionHintText->setPos(scene.sceneRect().center().x() - caliPositionHintText->boundingRect().center().x(),
            tutorialText->pos().y() - caliPositionHintText->boundingRect().height() - GetTextScale(TextSub));

    // Only consecutive forward updates acknowledge a shot. Duplicates do not restart the ring.
    if(caliStage != caliIrStage) {
        caliShotRingTimer.stop();
        caliShotStage = caliStage == caliIrStage + 1 && caliStage <= Cali_Verify ? caliIrStage : -1;
        if(caliShotStage >= 0) {
            caliNextTargetPos = crosshairItem->pos();
            caliShotRing->setPos(previousPosition + crosshairItem->boundingRect().center() * crosshairItem->scale());
            caliShotTimer.start();
        }
    }
    if(caliShotStage >= 0)
        crosshairItem->setPos(caliShotRing->pos() - crosshairItem->boundingRect().center() * crosshairItem->scale());

    // IR view: a new stage, the refused shot is no longer current.
    caliIrStage = caliStage;
    caliIrWarningBits = 0;
    caliIrWarningTimer.stop();
    CaliIrRefresh();

    // A new target restarts only the outer ring's rotation; none while verifying.
    if(caliStage != caliTargetStage) {
        caliTargetStage = caliStage;
        caliTargetTimer.start();
    }
    CaliTargetRingUpdate();
    CaliProgressUpdate();
    CaliShotRingUpdate();
}

void AppCaliWindow::CaliProgressUpdate()
{
    if(caliProgressDots[0] == nullptr)
        return; // alignment and IR-test windows have no calibration progress

    const qreal scale = bitmapText ? (qreal)GetTextScale(TextSub) / 2 :
                        scene.sceneRect().height() >= 1440 ? 2 : scene.sceneRect().height() >= 1080 ? 1.5 : 1;
    const qreal top = bitmapText ? caliStageBitmap->pos().y() : caliStageText->pos().y();
    const qreal y = top - 52 * scale; // 45 px higher at 1080p; clear of the upper target
    const qreal x = scene.sceneRect().center().x() - 2.5 * 22 * scale;
    const QColor grey(160, 160, 164);
    for(int i = 0; i < 6; ++i) {
        const bool current = i == caliIrStage;
        const bool done = i < caliIrStage;
        const qreal radius = (current ? 8 : 6) * scale;
        caliProgressDots[i]->setRect(x + i * 22 * scale - radius, y - radius, 2 * radius, 2 * radius);
        caliProgressDots[i]->setPen(current || done ? QPen(Qt::NoPen) : QPen(grey, 2 * scale));
        caliProgressDots[i]->setBrush(current ? QBrush(QColor(0, 220, 0)) : done ? QBrush(grey) : QBrush(Qt::NoBrush));
    }
}

void AppCaliWindow::CaliShotRingUpdate()
{
    if(caliShotRing == nullptr)
        return;
    const qint64 elapsed = caliShotStage >= 0 ? caliShotTimer.elapsed() : CALI_SHOT_TIME;
    if(caliShotStage < 0 || elapsed >= CALI_SHOT_TIME) {
        if(caliShotStage >= 0) {
            crosshairItem->setPos(caliNextTargetPos);
            caliTargetTimer.start();
        }
        caliShotStage = -1;
        caliShotRing->setVisible(false);
        caliShotRingTimer.stop();
        CaliTargetRingUpdate();
        return;
    }
    caliShotRing->setOpacity(qMin((qreal)1, (qreal)(CALI_SHOT_TIME - elapsed) / (CALI_SHOT_TIME - CALI_SHOT_HOLD)));
    caliShotRing->setVisible(true);
    if(!caliShotRingTimer.isActive())
        caliShotRingTimer.start(qMin(CALI_ANIMATION_FRAME_TIME, (int)(CALI_SHOT_TIME - elapsed)));
}

void AppCaliWindow::CaliTargetRingUpdate()
{
    if(caliTargetRing == nullptr)
        return;
    if(caliShotStage >= 0 || caliTargetStage > Cali_Center || !crosshairItem->isVisible()) {
        caliTargetRing->setVisible(false);
        caliTargetRingTimer.stop();
        return;
    }

    const qreal scale = crosshairItem->scale();
    const QPointF center = crosshairItem->pos() + crosshairItem->boundingRect().center() * scale;
    const QColor color = caliCrosshairColor.isValid() ? caliCrosshairColor : QColor(0xff, 0x87, 0x58);
    caliTargetRing->setPen(QPen(color, CALI_TARGET_RING_WIDTH, Qt::SolidLine, Qt::RoundCap));
    caliTargetRing->setPos(center);
    caliTargetRing->setRotation((qreal)(caliTargetTimer.elapsed() % CALI_TARGET_RING_PERIOD) * 360 / CALI_TARGET_RING_PERIOD);
    caliTargetRing->setVisible(true);
    if(!caliTargetRingTimer.isActive())
        caliTargetRingTimer.start(CALI_ANIMATION_FRAME_TIME);
}

void AppCaliWindow::CaliModeTextUpdate(const uint8_t &type, const char* data)
{
    if(bitmapText) {
        switch(type) {
        case 1: // top offset
            memcpy(&topOffset, data, 4);
            profileBitmaps[0]->setPixmap(GenerateText({caliTypesPrefixes.at(0) + QString::number(topOffset)}));
            break;
        case 2: // bottom offset
            memcpy(&bottomOffset, data, 4);
            profileBitmaps[1]->setPixmap(GenerateText({caliTypesPrefixes.at(1) + QString::number(bottomOffset)}));
            break;
        case 3: // left offset
            memcpy(&leftOffset, data, 4);
            profileBitmaps[2]->setPixmap(GenerateText({caliTypesPrefixes.at(2) + QString::number(leftOffset)}));
            break;
        case 4: // right offset
            memcpy(&rightOffset, data, 4);
            profileBitmaps[3]->setPixmap(GenerateText({caliTypesPrefixes.at(3) + QString::number(rightOffset)}));
            break;
        case 5: // topleft
            memcpy(&topLeftLed, data, 4);
            profileBitmaps[4]->setPixmap(GenerateText({caliTypesPrefixes.at(4) + QString::number(topLeftLed, 'g', 6)}));
            break;
        case 6: // topright
            memcpy(&topRightLed, data, 4);
            profileBitmaps[5]->setPixmap(GenerateText({caliTypesPrefixes.at(5) + QString::number(topRightLed, 'g', 6)}));
            // HACK: re-update final scene since this may happen AFTER the verification check stage
            CaliModeSet(Cali_Verify);
            break;
        }
    } else {
        switch(type) {
        case 1: // top offset
            break;
        case 2: // bottom offset
            break;
        case 3: // left offset
            break;
        case 4: // right offset
            break;
        case 5: // topleft
            break;
        case 6: // topright
            break;
        }
    }
}

void AppCaliWindow::SetIrTestDiamond(const bool &diamond)
{
    if(mode != modeIRTest || testLegend == nullptr || testLegendDiamond == diamond)
        return;
    testLegendDiamond = diamond;
    testLegend->setPixmap(QPixmap()); // rebuild only when the applied layout changes
    IrTestDrawLegend();
}

void AppCaliWindow::IrTestDrawLegend()
{
    if(testLegend == nullptr)
        return;
    if(testLegend->pixmap().isNull()) {
        const qreal ratio = devicePixelRatioF();
        QPixmap pixmap(qCeil(IRTEST_LEGEND_WIDTH * ratio), qCeil(IRTEST_LEGEND_HEIGHT * ratio));
        pixmap.setDevicePixelRatio(ratio);
        pixmap.fill(Qt::transparent);
        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setRenderHint(QPainter::TextAntialiasing);
        painter.setBrush(QColor(72, 72, 72));
        painter.setPen(QPen(Qt::gray, 2));
        painter.drawRoundedRect(QRectF(1, 1, IRTEST_LEGEND_WIDTH - 2, IRTEST_LEGEND_HEIGHT - 2), 17, 17);
        QFont font("Segoe UI");
        auto text = [&](const QString &value, const QRectF &rect, const int32_t pixels,
                        const int32_t alignment, const bool medium = false) {
            font.setPixelSize(pixels);
            font.setWeight(medium ? QFont::Medium : QFont::Normal);
            font.setUnderline(false);
            painter.setFont(font);
            painter.setPen(QColor(247, 247, 247));
            painter.drawText(rect, alignment, value);
        };
        // Reuse the existing translations, with the note on the title's baseline.
        const QString title = tr("IR Camera Test Legend") + QStringLiteral(" - ");
        const QString note = tr("Green and cyan identify emitter positions, not signal quality.");
        font.setPixelSize(16); font.setWeight(QFont::Medium);
        const QFont titleFont = font;
        const QFontMetricsF titleMetrics(titleFont);
        const qreal titleWidth = titleMetrics.horizontalAdvance(title);
        font.setPixelSize(13); font.setWeight(QFont::Normal);
        const qreal headerWidth = titleWidth + QFontMetricsF(font).horizontalAdvance(note);
        const QPointF header((IRTEST_LEGEND_WIDTH - headerWidth) / 2,
                             14 + (21 - titleMetrics.height()) / 2 + titleMetrics.ascent());
        painter.setPen(QColor(247, 247, 247));
        painter.setFont(titleFont); painter.drawText(header, title);
        painter.setFont(font); painter.drawText(header + QPointF(titleWidth, 0), note);
        painter.setPen(QPen(QColor(119, 119, 119), 1));
        painter.drawLine(QPointF(18, 47), QPointF(IRTEST_LEGEND_WIDTH - 18, 47));
        painter.drawLine(QPointF(260.5, 68), QPointF(260.5, 168));
        painter.drawLine(QPointF(725.5, 68), QPointF(725.5, 168));
        const QColor colours[4] = {Qt::green, Qt::green, Qt::cyan, Qt::cyan};
        for(uint8_t layout = 0; layout < 2; ++layout) {
            const qreal x = layout ? 139 : 19;
            font.setPixelSize(13);
            font.setWeight(QFont::Normal);
            font.setUnderline(testLegendDiamond == bool(layout));
            painter.setFont(font);
            painter.setPen(QColor(247, 247, 247));
            painter.drawText(QRectF(x, 80, 108, 17), Qt::AlignCenter, layout ? "Diamond" : "Square");
            const QPointF square[4] = {{27, 11.856}, {81, 11.856}, {27, 37.544}, {81, 37.544}};
            const QPointF diamond[4] = {{54, 6.916}, {21.6, 24.7}, {54, 42.484}, {86.4, 24.7}};
            for(uint8_t i = 0; i < 4; ++i) {
                painter.setPen(Qt::NoPen);
                painter.setBrush(colours[i]);
                painter.drawEllipse(QPointF(x, 107) + (layout ? diamond[i] : square[i]), 8, 8);
            }
        }
        const qreal signalLeft = 273 + (214 - 164.5) / 2;
        const qreal sizeLeft = 499 + (214 - 164.5) / 2;
        text(tr("IR signal intensity"), QRectF(273, 64, 214, 17), 13, Qt::AlignCenter, true);
        text(tr("IR point size"), QRectF(499, 64, 214, 17), 13, Qt::AlignCenter, true);
        const qreal light[5] = {.15, .35, .55, .8, 1};
        const qreal edge[5] = {.07, .15, .25, .4, .7};
        for(uint8_t row = 0; row < 2; ++row) {
            const QColor colour = row ? QColor(Qt::cyan) : QColor(Qt::green);
            for(uint8_t i = 0; i < 5; ++i) {
                const QPointF point(signalLeft + 13 + 34.625 * i, 106 + 30 * row);
                QColor centerColor = colour, edgeColor = colour, transparent = colour;
                centerColor.setAlphaF(light[i]); edgeColor.setAlphaF(edge[i]); transparent.setAlpha(0);
                QRadialGradient gradient(point, 11);
                gradient.setColorAt(0, centerColor); gradient.setColorAt(.75, edgeColor); gradient.setColorAt(1, transparent);
                painter.setBrush(gradient); painter.setPen(QPen(colour, 1));
                painter.drawEllipse(point, 11, 11);
            }
        }
        const qreal diameters[5] = {3, 7, 11, 16, 22};
        qreal blobX = sizeLeft + 4;
        for(uint8_t i = 0; i < 5; ++i) {
            painter.setBrush(Qt::NoBrush); painter.setPen(QPen(QColor(238, 238, 238), 1));
            painter.drawEllipse(QPointF(blobX + diameters[i] / 2, 99), diameters[i] / 2, diameters[i] / 2);
            blobX += diameters[i] + 24.375;
        }
        text(tr("Weak"), QRectF(signalLeft, 156, 164.5, 16), 12, Qt::AlignLeft | Qt::AlignTop);
        text(tr("Strong"), QRectF(signalLeft, 156, 164.5, 16), 12, Qt::AlignRight | Qt::AlignTop);
        text(tr("Small"), QRectF(sizeLeft, 112, 164.5, 16), 12, Qt::AlignLeft | Qt::AlignTop);
        text(tr("Large"), QRectF(sizeLeft, 112, 164.5, 16), 12, Qt::AlignRight | Qt::AlignTop);
        font.setPixelSize(13); font.setUnderline(false); font.setWeight(QFont::Medium);
        const qreal missingWidth = QFontMetricsF(font).horizontalAdvance(tr("Emitter not detected"));
        font.setPixelSize(12); font.setWeight(QFont::Normal);
        const qreal labelWidth = qMax(missingWidth, QFontMetricsF(font).horizontalAdvance(tr("Reconstructed position")));
        const qreal start = 499 + (214 - 62 - labelWidth) / 2;
        for(uint8_t i = 0; i < 2; ++i) {
            const QPointF center(start + 11.5 + 29 * i, 155.75);
            QPen outline(i ? QColor(Qt::cyan) : QColor(Qt::green), 1.5);
            outline.setDashPattern({2, 2});
            painter.setPen(outline); painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(center, 11.5, 11.5);
            painter.setPen(QPen(IrTestNotSeenColor(), 2));
            painter.drawLine(center + QPointF(-8, -8), center + QPointF(8, 8));
            painter.drawLine(center + QPointF(8, -8), center + QPointF(-8, 8));
        }
        text(tr("Emitter not detected"), QRectF(start + 62, 139.5, labelWidth, 17), 13, Qt::AlignLeft | Qt::AlignTop, true);
        text(tr("Reconstructed position"), QRectF(start + 62, 156.5, labelWidth, 16), 12, Qt::AlignLeft | Qt::AlignTop);
        painter.save();
        painter.translate(770, 84);
        painter.scale(24 / (61.44 * IRTEST_AIM_SCALE), 24 / (61.44 * IRTEST_AIM_SCALE));
        painter.setBrush(Qt::NoBrush); painter.setPen(QPen(Qt::gray, 2.4 * IRTEST_AIM_SCALE));
        painter.drawEllipse(QPointF(0, 0), 25, 25);
        QPen aimPen(Qt::gray, 1.44 * IRTEST_AIM_SCALE); aimPen.setCapStyle(Qt::SquareCap);
        painter.setPen(aimPen); painter.drawPath(IrTestAimTicks());
        painter.restore();
        painter.setBrush(Qt::NoBrush); painter.setPen(QPen(Qt::red, 2));
        painter.drawEllipse(QPointF(770, 118), 9, 9);
        painter.setPen(QPen(Qt::gray, 1.5));
        painter.drawPolygon(QPolygonF() << QPointF(761, 145) << QPointF(783, 145) << QPointF(779, 159) << QPointF(757, 159));
        text(tr("Lightgun aim"), QRectF(802, 72, 190, 24), 13, Qt::AlignLeft | Qt::AlignVCenter | Qt::TextSingleLine);
        text(tr("Emitter centre"), QRectF(802, 106, 190, 24), 13, Qt::AlignLeft | Qt::AlignVCenter | Qt::TextSingleLine);
        text(tr("Lines joining IR points"), QRectF(802, 140, 190, 24), 13, Qt::AlignLeft | Qt::AlignVCenter | Qt::TextSingleLine);
        painter.end();
        testLegend->setPixmap(pixmap);
    }
    const QRectF rect = scene.sceneRect();
    const qreal scale = qMin(qreal(1), qMin(rect.width() * .8 / IRTEST_LEGEND_WIDTH,
                                         rect.height() * .22 / IRTEST_LEGEND_HEIGHT));
    const qreal width = IRTEST_LEGEND_WIDTH * scale;
    const qreal height = IRTEST_LEGEND_HEIGHT * scale;
    const qreal margin = qMin(qreal(40), rect.height() / 27);
    testLegend->setScale(scale);
    const qreal top = rect.bottom() - margin - height;
    testLegend->setPos(rect.center().x() - width / 2, top);
    // Keep the existing ESC instruction above the panel in both text modes.
    QGraphicsItem *instructions = bitmapText ? static_cast<QGraphicsItem *>(tutorialBitmap) : tutorialText;
    if(instructions) {
        const QRectF bounds = instructions->boundingRect();
        const qreal textScale = instructions->scale();
        instructions->setPos(rect.center().x() - bounds.center().x() * textScale,
                             top - qMax(qreal(8), 24 * scale) - bounds.bottom() * textScale);
    }
}

void AppCaliWindow::TestModeDraw(const int coordsList[12])
{
    // Calibration IR view: the emitters are drawn in the panel, at their place in the layout.
    if(mode == modeCalibrate) {
        for(int i = 0; i < 4; ++i)
            caliIrSeenByCoords[i] = (coordsList[i * 2] % 2) == 0; // bit 0 of X: outside the FOV
        caliIrCoordsTimer.start();
        CaliIrRefresh();
        CaliIrArmExpiry();
        return;
    }
    if(mode != modeIRTest)
        return;

    const int pointItems[4] = {
        testPointTL,
        testPointTR,
        testPointBL,
        testPointBR
    };

    int pointX[4];
    int pointY[4];
    bool outsideFov[4];

    // With recent sTestBlobs data the emitters show the camera blobs,
    // otherwise (older firmware) the classic circles are drawn.
    const bool blobView = TestBlobsFresh();

    // The firmware encodes the outside-FOV state in bit 0 of each IR point X:
    // even = inside FOV, odd = outside FOV. Decode X without losing its sign.
    for(int i = 0; i < 4; ++i) {
        int encodedX = coordsList[i * 2];

        outsideFov[i] = (encodedX % 2) != 0;
        pointX[i] = (encodedX - (outsideFov[i] ? 1 : 0)) / 2;
        pointY[i] = coordsList[(i * 2) + 1];

        if(blobView) {
            TestModeDrawEmitter(i, pointX[i], pointY[i]);
            continue;
        }

        // Classic view: plain 3 px outline, no X.
        testPoints[pointItems[i]]->setPen(QPen(testPoints[pointItems[i]]->pen().color(), 3));
        testNotSeenMarks[i]->setVisible(false);

        if(outsideFov[i]) {
            // Filled circle = estimated vertex outside the camera FOV.
            testPoints[pointItems[i]]->setBrush(QBrush(testPoints[pointItems[i]]->pen().color()));
        } else {
            testPoints[pointItems[i]]->setBrush(QBrush(Qt::NoBrush));
        }

        // Always draw the marker at its real decoded position.
        // Qt naturally clips it at the canvas edge: it remains partially visible
        // while it intersects the canvas and disappears once fully outside.
        testPoints[pointItems[i]]->setRect(pointX[i] - 25, pointY[i] - 25, 50, 50);
    }

    testPoints[testPointMed]->setRect(coordsList[8]  - 25, coordsList[9]  - 25, 50, 50);
    testAimTicks->setPos(coordsList[8], coordsList[9]);
    testAimTicks->setVisible(true);
    testPoints[testPointD]  ->setRect(coordsList[10] - 25, coordsList[11] - 25, 50, 50);

    // Reconstruct the complete IR rectangle using the real decoded coordinates,
    // even when one or more estimated vertices are outside the camera FOV.
    testBox->setPolygon(QPolygonF() << QPointF(pointX[0], pointY[0])
                                    << QPointF(pointX[1], pointY[1])
                                    << QPointF(pointX[3], pointY[3])
                                    << QPointF(pointX[2], pointY[2])
                                    << QPointF(pointX[0], pointY[0]));

    // --irdebug: area and max/average brightness under each emitter circle.
    for(int i = 0; i < 4; ++i) {
        if(!testBlobDebug || !blobView) {
            testBlobInfo[i]->setVisible(false);
            continue;
        }
        const TestBlob_s &blob = testBlobs[i];
        const QStringList text = blob.seen
            ? QStringList{QString("A %1").arg((int)blob.area), QString("%1/%2").arg((int)blob.max).arg((int)blob.avg)}
            : QStringList{QString("--")};
        testBlobInfo[i]->setPixmap(GenerateText(IrTestCentredLines(text)));
        const qreal radius = testBlobLevels[i].valid ? testBlobLevels[i].radius : IRTEST_EMITTER_RADIUS;
        const qreal x = testOffsetX + pointX[i] * testScale;
        const qreal y = testOffsetY + (pointY[i] + radius + 4) * testScale;
        testBlobInfo[i]->setPos(x - testBlobInfo[i]->boundingRect().width() * testBlobInfo[i]->scale() / 2, y);
        testBlobInfo[i]->setVisible(true);
    }
}

void AppCaliWindow::TestModeBlobs(const uint8_t *payload, const int &length)
{
    if((mode != modeIRTest && mode != modeCalibrate) || payload == nullptr ||
       length < IRTEST_BLOBS_LENGTH || payload[0] != IRTEST_BLOBS_VERSION)
        return;

    // After a gap the average starts again from the new values.
    if(!TestBlobsFresh())
        for(TestBlobLevel_s &level : testBlobLevels)
            level.valid = false;

    for(int i = 0; i < 4; ++i) {
        const uint8_t *entry = payload + 1 + i * 5;
        TestBlob_s &blob = testBlobs[i];
        blob.seen = (entry[0] & 0x01) != 0;
        blob.weak = (entry[0] & IRTEST_BLOB_WEAK) != 0;
        blob.avg = entry[1];
        blob.max = entry[2];
        blob.area = (uint16_t)(entry[3] | (entry[4] << 8));

        TestBlobLevel_s &level = testBlobLevels[i];
        if(!blob.seen) {
            level.valid = false; // a new sighting starts from its own values
            continue;
        }

        const qreal k = qSqrt((qreal)qMin((int)blob.area, IRTEST_BLOB_AREA_MAX) / IRTEST_BLOB_AREA_MAX);
        const qreal radius = qMax(IRTEST_BLOB_RADIUS_MIN, IRTEST_BLOB_RADIUS_MAX * k) * IRTEST_BLOB_RADIUS_SCALE;
        const qreal center = IrTestBrightnessLevel(blob.max);
        const qreal edge = IrTestBrightnessLevel(blob.avg);

        if(!level.valid) {
            level.valid = true;
            level.radius = radius;
            level.center = center;
            level.edge = edge;
        } else {
            level.radius += IRTEST_SMOOTHING * (radius - level.radius);
            level.center += IRTEST_SMOOTHING * (center - level.center);
            level.edge += IRTEST_SMOOTHING * (edge - level.edge);
        }
    }

    testBlobsTimer.start();
    if(mode == modeCalibrate)
        CaliIrArmExpiry();
}

bool AppCaliWindow::TestBlobsFresh() const
{
    return testBlobsTimer.isValid() && testBlobsTimer.elapsed() <= IRTEST_BLOBS_MAX_AGE;
}

void AppCaliWindow::TestModeDrawEmitter(const int &index, const qreal &x, const qreal &y)
{
    QGraphicsEllipseItem *item = testPoints[testPointTL + index];
    const QColor color = item->pen().color();
    const TestBlobLevel_s &level = testBlobLevels[index];

    if(testBlobs[index].seen && level.valid) {
        // Seen: radius from the blob area, radial gradient from the max brightness
        // (centre) to the average brightness (edge), outline in the emitter colour.
        const qreal r = level.radius;
        QColor centerColor = color;
        QColor edgeColor = color;
        centerColor.setAlphaF((float)level.center);
        edgeColor.setAlphaF((float)level.edge);
        QRadialGradient gradient(QPointF(x, y), r);
        gradient.setColorAt(0, centerColor);
        gradient.setColorAt(1, edgeColor);
        item->setBrush(QBrush(gradient));
        item->setPen(QPen(color, qMax((qreal)2, 1 / testScale))); // at least 1 screen pixel
        item->setRect(x - r, y - r, 2 * r, 2 * r);
        testNotSeenMarks[index]->setVisible(false);
    } else {
        // Not seen: dashed circle in the emitter colour with a red X.
        const qreal r = IRTEST_EMITTER_RADIUS;
        QPen pen(color, 3);
        pen.setCapStyle(Qt::FlatCap); // dashes of exactly 6 px like the web app
        pen.setDashPattern({6.0 / 3, 5.0 / 3}); // 6 px dash, 5 px gap (units of the 3 px width)
        item->setPen(pen);
        item->setBrush(QBrush(Qt::NoBrush));
        item->setRect(x - r, y - r, 2 * r, 2 * r);

        const qreal d = r * 0.62;
        QPainterPath cross;
        cross.moveTo(x - d, y - d);
        cross.lineTo(x + d, y + d);
        cross.moveTo(x + d, y - d);
        cross.lineTo(x - d, y + d);
        testNotSeenMarks[index]->setPath(cross);
        testNotSeenMarks[index]->setVisible(true);
    }
}

void AppCaliWindow::CaliIrWarning(const uint8_t &bits)
{
    if(mode != modeCalibrate || !bits)
        return;
    caliIrWarningBits = bits;
    caliIrWarningTimer.start(CALI_IR_WARNING_TIME);
    CaliIrRefresh();
}

QColor AppCaliWindow::CaliIrColor() const
{
    // Red when the board refuses the shot (an emitter missing), otherwise the continuous
    // scale of the weakest emitter.
    // Without recent IR data (older firmware) the crosshair keeps its colour.
    if(mode != modeCalibrate || !caliIrCoordsTimer.isValid() ||
       caliIrCoordsTimer.elapsed() > CALI_IR_MAX_AGE || !TestBlobsFresh())
        return QColor();

    // The emitter with the lowest brightness gives the colour, so the crosshair always has
    // the colour of one of the four circles.
    int worst = 255;
    for(const TestBlob_s &blob : testBlobs) {
        if(!blob.seen)
            return CaliIrMix(CALI_IR_MISSING_RGB, CALI_IR_MISSING_RGB, 0); // intense red
        worst = qMin(worst, (int)blob.max);
    }
    return CaliIrEmitterColor(worst);
}

QStringList AppCaliWindow::CaliIrWarningLines(const uint8_t &bits)
{
    return {(bits & CALI_IR_MISSING) ? tr("Shot refused: the camera does not see all four IR emitters.")
                                     : QString(),
            tr("Check the emitters, your distance and the IR sensitivity, then shoot again.")};
}

void AppCaliWindow::CaliIrArmExpiry()
{
    caliIrExpiryTimer.stop();
    if(mode != modeCalibrate)
        return;
    qint64 remaining = -1;
    if(caliIrCoordsTimer.isValid() && caliIrCoordsTimer.elapsed() <= CALI_IR_MAX_AGE)
        remaining = CALI_IR_MAX_AGE - caliIrCoordsTimer.elapsed();
    if(TestBlobsFresh()) {
        const qint64 blobsRemaining = IRTEST_BLOBS_MAX_AGE - testBlobsTimer.elapsed();
        remaining = remaining < 0 ? blobsRemaining : qMin(remaining, blobsRemaining);
    }
    if(remaining >= 0)
        caliIrExpiryTimer.start((int)remaining + 20);
}

void AppCaliWindow::CaliIrRefresh()
{
    if(mode != modeCalibrate || !bitmapText || caliIrPanel == nullptr)
        return;

    const QRectF rect = scene.sceneRect();
    const int sub = GetTextScale(TextSub);
    const QColor color = CaliIrColor();

    // Crosshair in the colour of the worst emitter, in steps of 8 so that the SVG is not
    // reloaded at every update; the original colour without IR data.
    QColor crosshairColor;
    if(color.isValid())
        crosshairColor = QColor(qMin(255, qRound(color.red()   / 8.0) * 8),
                                qMin(255, qRound(color.green() / 8.0) * 8),
                                qMin(255, qRound(color.blue()  / 8.0) * 8));
    if(crosshairColor != caliCrosshairColor && !caliCrosshairSvg.isEmpty()) {
        QByteArray svg = caliCrosshairSvg;
        if(crosshairColor.isValid())
            svg.replace("#ff8758", crosshairColor.name().toLatin1());
        if(caliCrosshairRenderer.load(svg)) {
            crosshairItem->setSharedRenderer(&caliCrosshairRenderer);
            crosshairItem->update();
            caliCrosshairColor = crosshairColor;
        }
    }
    // Recolour the outer ring in the same update, not at the next animation tick.
    CaliTargetRingUpdate();

    // IR view: a target shot is refused only with the crosshair red (an emitter missing).
    const bool hint = color.isValid() && caliIrStage <= Cali_Center;
    if(hint)
        caliIrHintBitmap->setPos(rect.center().x() - (caliIrHintBitmap->boundingRect().width() * sub) / 2,
                                 headerBitmap->pos().y() + headerBitmap->boundingRect().height() * headerBitmap->scale() + 8 * sub);
    caliIrHintBitmap->setVisible(hint);

    // A refused shot: the reason takes the place of the tutorial for a few seconds,
    // at most 60% of the width, so that the panel at the bottom right stays clear.
    const int warningChars = qMax(16, (int)((rect.width() * 0.6) / (8 * sub)));
    auto warningWidth = [&](const uint8_t &bits) {
        int chars = 0;
        for(const QString &line : CaliIrWrapLines(CaliIrWarningLines(bits), warningChars))
            chars = qMax(chars, (int)line.length());
        return (qreal)chars * 8 * sub;
    };
    // The panel is sized for this text too, so it keeps its size when a warning appears.
    qreal textRight = rect.center().x() + warningWidth(CALI_IR_MISSING) / 2 + 8 * sub;
    if(tutorialBitmap->boundingRect().width() > 0)
        textRight = qMax(textRight, tutorialBitmap->pos().x() + tutorialBitmap->boundingRect().width() * tutorialBitmap->scale());

    if(caliIrWarningBits) {
        const QStringList lines = CaliIrWrapLines(CaliIrWarningLines(caliIrWarningBits), warningChars);
        caliIrWarningBitmap->setPixmap(GenerateText(IrTestCentredLines(lines), QColor(255, 90, 90), sub));
        const qreal width = caliIrWarningBitmap->boundingRect().width() * sub;
        const qreal height = caliIrWarningBitmap->boundingRect().height() * sub;
        const QPointF pos(rect.center().x() - width / 2, rect.top() + rect.height() * 0.8 - height / 2);
        caliIrWarningBitmap->setPos(pos);
        caliIrWarningBox->setRect(pos.x() - 8 * sub, pos.y() - 6 * sub, width + 16 * sub, height + 12 * sub);
    }
    caliIrWarningBitmap->setVisible(caliIrWarningBits != 0);
    caliIrWarningBox->setVisible(caliIrWarningBits != 0);
    tutorialBitmap->setVisible(caliIrWarningBits == 0);

    // Leave a bitmap row between this reminder and the tutorial (or refusal box).
    // Do not move the existing texts; a longer verification block only moves the reminder up.
    qreal instructionsTop = rect.bottom() * 0.8 - qMax(qreal(24), tutorialBitmap->boundingRect().height()) * sub / 2;
    if(caliIrWarningBits)
        instructionsTop = qMin(instructionsTop, caliIrWarningBox->rect().top());
    caliPositionHintBitmap->setPos(rect.center().x() - caliPositionHintBitmap->boundingRect().width() * sub / 2,
        instructionsTop - caliPositionHintBitmap->boundingRect().height() * sub - 8 * sub);

    CaliIrDrawLegend(CaliIrPanelSide(textRight), 12 * GetTextScale(TextSmall));
    CaliIrDrawPanel(textRight);
}

qreal AppCaliWindow::CaliIrPanelSide(const qreal &textRight)
{
    const QRectF rect = scene.sceneRect();
    const qreal margin = 12 * GetTextScale(TextSmall);
    const qreal free = rect.right() - textRight - 2 * margin;
    return qMax(CALI_IR_PANEL_MIN_WIDTH, qMin(qMin(rect.width() * CALI_IR_PANEL_WIDTH, CALI_IR_PANEL_MAX_WIDTH), free));
}

void AppCaliWindow::CaliIrDrawLegend(const qreal &side, const qreal &margin)
{
    if(caliIrLegend == nullptr)
        return;
    // Build once, not on the 20 Hz IR updates or on the target animation timer.
    if(caliIrLegend->pixmap().isNull()) {
        const qreal ratio = devicePixelRatioF();
        QPixmap pixmap(qCeil(CALI_LEGEND_SIZE * ratio), qCeil(CALI_LEGEND_SIZE * ratio));
        pixmap.setDevicePixelRatio(ratio);
        pixmap.fill(Qt::transparent);
        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setRenderHint(QPainter::TextAntialiasing);
        QPainterPath panel;
        panel.addRoundedRect(QRectF(1, 1, CALI_LEGEND_SIZE - 2, CALI_LEGEND_SIZE - 2),
                             CALI_LEGEND_SIZE * 0.075, CALI_LEGEND_SIZE * 0.075);
        painter.setPen(QPen(QColor(160, 160, 164), 2));
        painter.setBrush(QColor(74, 74, 74));
        painter.drawPath(panel);
        QFont font("Segoe UI");
        font.setPixelSize(12);
        painter.setFont(font);
        const qreal padding = 12;
        const qreal width = CALI_LEGEND_SIZE - 2 * padding;
        auto text = [&](const QString &value, const qreal y, const qreal height,
                        const int32_t pixels, const int32_t alignment) {
            font.setPixelSize(pixels);
            font.setWeight(pixels == 15 ? QFont::Medium : QFont::Normal);
            painter.setFont(font);
            painter.setPen(QColor(247, 247, 247));
            painter.drawText(QRectF(padding, y, width, height), alignment, value);
        };
        text(tr("Legend"), 12, 18, 15, Qt::AlignCenter);
        text(tr("IR signal intensity"), 34, 14, 12, Qt::AlignCenter);
        QLinearGradient gradient(padding, 0, padding + width, 0);
        gradient.setColorAt(0, CaliIrEmitterColor(CALI_IR_BRIGHTNESS_MIN));
        gradient.setColorAt(0.2, CaliIrEmitterColor(CALI_IR_BRIGHTNESS_WEAK - CALI_IR_BLEND));
        gradient.setColorAt(0.6, CaliIrEmitterColor(CALI_IR_BRIGHTNESS_WEAK + CALI_IR_BLEND));
        gradient.setColorAt(1, CaliIrEmitterColor(CALI_IR_FULL_GREEN));
        painter.setPen(Qt::NoPen);
        painter.setBrush(gradient);
        painter.drawRoundedRect(QRectF(padding, 51, width, 5), 2.5, 2.5);
        const qreal diameters[7] = {3, 5, 8, 11, 15, 19, 23};
        for(uint8_t i = 0; i < 7; ++i) {
            const qreal x = padding + 12.5 + (width - 25) * i / 6;
            const QColor color = CaliIrEmitterColor(CALI_IR_BRIGHTNESS_MIN +
                qRound((CALI_IR_FULL_GREEN - CALI_IR_BRIGHTNESS_MIN) * i / 6.0));
            // The same SVG as the main crosshair, without the separately drawn rotating ring.
            QByteArray svg = caliCrosshairSvg;
            svg.replace("#ff8758", color.name().toLatin1());
            QSvgRenderer renderer(svg);
            renderer.render(&painter, QRectF(x - 12.5, 59, 25, 25));
            painter.setPen(QPen(QColor(247, 247, 247), 1));
            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(QPointF(x, 133), diameters[i] / 2, diameters[i] / 2);
        }
        text(tr("Weak"), 85, 13, 11, Qt::AlignLeft | Qt::AlignTop);
        text(tr("Strong"), 85, 13, 11, Qt::AlignRight | Qt::AlignTop);
        text(tr("IR point size"), 104, 14, 12, Qt::AlignCenter);
        text(tr("Small"), 146, 13, 11, Qt::AlignLeft | Qt::AlignTop);
        text(tr("Large"), 146, 13, 11, Qt::AlignRight | Qt::AlignTop);
        font.setPixelSize(12);
        painter.setFont(font);
        const QString missing = tr("IR not detected");
        const qreal textWidth = QFontMetricsF(font).horizontalAdvance(missing);
        const qreal x = (CALI_LEGEND_SIZE - 20 - 8 - textWidth) / 2;
        QPen missingPen(IrTestNotSeenColor(), 1.5, Qt::DashLine);
        missingPen.setDashPattern({2, 2});
        painter.setPen(missingPen);
        painter.drawEllipse(QRectF(x, 166, 20, 20));
        painter.setPen(QPen(IrTestNotSeenColor(), 1.5));
        painter.drawLine(QPointF(x + 4, 170), QPointF(x + 16, 182));
        painter.drawLine(QPointF(x + 16, 170), QPointF(x + 4, 182));
        painter.setPen(QColor(247, 247, 247));
        painter.drawText(QRectF(x + 28, 166, textWidth + 1, 20), Qt::AlignLeft | Qt::AlignVCenter, missing);
        painter.setPen(QPen(QColor(119, 119, 119), 1));
        painter.drawLine(QPointF(padding, 195), QPointF(padding + width, 195));
        // Explicit 13 px line spacing, like the WebApp: Qt's default paragraph
        // spacing would clip the longer Italian second line in this compact square.
        font.setPixelSize(11);
        font.setWeight(QFont::Normal);
        painter.setFont(font);
        painter.setPen(QColor(222, 222, 222));
        const QFontMetricsF metrics(font);
        auto note = [&](const QString &value, qreal y) {
            QString line;
            for(const QString &word : value.split(' ')) {
                const QString next = line.isEmpty() ? word : line + ' ' + word;
                if(!line.isEmpty() && metrics.horizontalAdvance(next) > width) {
                    painter.drawText(QPointF(padding, y + metrics.ascent()), line);
                    y += 13;
                    line = word;
                } else line = next;
            }
            painter.drawText(QPointF(padding, y + metrics.ascent()), line);
        };
        note(tr("The crosshair colour indicates the emitter with the weakest signal."), 201);
        note(tr("All 4 emitters must be detected to proceed."), 234);
        painter.end();
        caliIrLegend->setPixmap(pixmap);
    }
    const QRectF rect = scene.sceneRect();
    caliIrLegend->setScale(side / CALI_LEGEND_SIZE);
    caliIrLegend->setPos(rect.left() + margin, rect.bottom() - margin - side);
    caliIrLegend->setVisible(true);
}

void AppCaliWindow::CaliIrDrawPanel(const qreal &textRight)
{
    // Drawn only while the firmware sends the coordinates (caliFlagIrView).
    if(!caliIrCoordsTimer.isValid() || caliIrCoordsTimer.elapsed() > CALI_IR_MAX_AGE) {
        caliIrPanel->setVisible(false);
        return;
    }

    const QRectF rect = scene.sceneRect();
    const int small = GetTextScale(TextSmall);
    const qreal margin = 12 * small;
    // Right of the tutorial text (bottom centre), which it must not cover on small screens.
    // The emitters are drawn in their layout on a square of this side, not where the camera sees them.
    const qreal side = CaliIrPanelSide(textRight);

    const bool blobs = TestBlobsFresh();
    QPixmap pixmap(qCeil(side), qCeil(side));
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    QPainterPath panelPath;
    panelPath.addRoundedRect(QRectF(1, 1, side - 2, side - 2), side * 0.075, side * 0.075);
    painter.setPen(QPen(QColor(160, 160, 164), 2));
    painter.setBrush(QColor(0, 0, 0, 77));
    painter.drawPath(panelPath);

    // Each emitter at its place in the layout, on a virtual square: Square at the corners
    // (sTestCoords order TL, TR, BL, BR), Diamond at the middle of the sides (Diamond slots:
    // 0 top, 1 left, 2 bottom, 3 right on the screen). No lines between them.
    painter.save();
    painter.setClipPath(panelPath);
    // Match the WebApp: two short lines fit between even the largest Diamond blobs.
    // Draw before the emitters, keeping their shapes, colours and sizes unchanged.
    painter.save();
    const qreal labelScale = side / CALI_LEGEND_SIZE;
    QFont labelFont("Segoe UI");
    labelFont.setPixelSize(qRound(14 * labelScale));
    labelFont.setWeight(QFont::Medium);
    painter.setFont(labelFont);
    painter.setPen(QColor(222, 222, 222));
    painter.setRenderHint(QPainter::TextAntialiasing);
    const QFontMetricsF labelMetrics(labelFont);
    const qreal baselineOffset = (labelMetrics.ascent() - labelMetrics.descent()) / 2;
    const QStringList labelRows = tr("IR LEDs").split(' ');
    for(uint8_t i = 0; i < 2; ++i) {
        const QString line = labelRows.value(i);
        painter.drawText(QPointF((side - labelMetrics.horizontalAdvance(line)) / 2,
            side / 2 + (i * 16 - 8) * labelScale + baselineOffset), line);
    }
    painter.restore();
    const qreal d = side * 0.25;
    QPointF places[4];
    if(caliIrDiamond) {
        places[0] = QPointF(side / 2, d);
        places[1] = QPointF(d, side / 2);
        places[2] = QPointF(side / 2, side - d);
        places[3] = QPointF(side - d, side / 2);
    } else {
        places[0] = QPointF(d, d);
        places[1] = QPointF(side - d, d);
        places[2] = QPointF(d, side - d);
        places[3] = QPointF(side - d, side - d);
    }
    // Same radii as before: the largest seen blob gets 18% of the side; missing gets 9%.
    const qreal seenScale = side * 0.18 / (IRTEST_BLOB_RADIUS_MAX * IRTEST_BLOB_RADIUS_SCALE);
    const qreal notSeenScale = side * 0.09 / IRTEST_EMITTER_RADIUS;
    const QColor colors[4] = {Qt::green, Qt::green, Qt::cyan, Qt::cyan};
    for(int i = 0; i < 4; ++i) {
        const QPointF center = places[i];
        const bool emitterSeen = blobs ? testBlobs[i].seen : caliIrSeenByCoords[i];
        // Seen: the crosshair scale with its own brightness (older firmware: layout colour);
        // not seen: red dashed circle with the red X (only here, the IR test view keeps its colours).
        const QColor emitterColor = !emitterSeen ? IrTestNotSeenColor() :
                                    blobs ? CaliIrEmitterColor(testBlobs[i].max) : colors[i];
        // Without blob data (older firmware) the classic circle size is used.
        TestBlobLevel_s level = testBlobLevels[i];
        if(!blobs) {
            level.valid = true;
            level.radius = IRTEST_EMITTER_RADIUS;
            level.center = 1;
            level.edge = 0.4;
        }

        if(emitterSeen && level.valid) {
            // Seen: radius from the blob area, radial gradient from the max brightness
            // (centre) to the average brightness (edge), outline in the emitter colour.
            const qreal r = level.radius * seenScale;
            QColor centerColor = emitterColor;
            QColor edgeColor = emitterColor;
            centerColor.setAlphaF((float)level.center);
            edgeColor.setAlphaF((float)level.edge);
            QRadialGradient gradient(center, r);
            gradient.setColorAt(0, centerColor);
            gradient.setColorAt(1, edgeColor);
            painter.setBrush(QBrush(gradient));
            painter.setPen(QPen(emitterColor, qMax((qreal)1, 2 * seenScale)));
            painter.drawEllipse(center, r, r);
        } else {
            // Not seen: red dashed circle with a red X.
            const qreal r = IRTEST_EMITTER_RADIUS * notSeenScale;
            QPen pen(emitterColor, 3 * notSeenScale);
            pen.setCapStyle(Qt::FlatCap);
            pen.setDashPattern({6.0 / 3, 5.0 / 3}); // 6 and 5 units of the circle scale (pattern in pen widths)
            painter.setPen(pen);
            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(center, r, r);

            const qreal x = r * 0.62;
            QPen markPen(IrTestNotSeenColor(), 3 * notSeenScale);
            markPen.setCapStyle(Qt::RoundCap);
            painter.setPen(markPen);
            painter.drawLine(center + QPointF(-x, -x), center + QPointF(x, x));
            painter.drawLine(center + QPointF(x, -x), center + QPointF(-x, x));
        }
    }
    painter.restore();

    painter.end();

    caliIrPanel->setPixmap(pixmap);
    caliIrPanel->setPos(rect.right() - margin - side, rect.bottom() - margin - side);
    caliIrPanel->setVisible(true);
}
