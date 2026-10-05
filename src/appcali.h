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

#ifndef APPCALI_H
#define APPCALI_H

#include <QWidget>
#include <QGraphicsScene>
#include <QKeyEvent>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSvgItem>
#include <QElapsedTimer>
#include <QSvgRenderer>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui {
class AppCaliWindow;
}
QT_END_NAMESPACE

class AppCaliGraphicsScene : public QGraphicsScene
{
    Q_OBJECT

signals:
    void sceneMouseEvent(const QPointF &);
    void sceneKeyCloseSignal();

protected:
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override {
        emit sceneMouseEvent(mouseEvent->scenePos());
    }

    void keyPressEvent(QKeyEvent *keyEvent) override {
        if(keyEvent->key() == Qt::Key_Escape)
            emit sceneKeyCloseSignal();
    }

};

class AppCaliWindow : public QWidget
{
    Q_OBJECT

public:
    enum {
        modeCalibrate = 0,
        modeAlignment,
        modeIRTest
    } AppCaliStates_e;

    enum {
        testPointTL = 0,
        testPointTR,
        testPointBL,
        testPointBR,
        testPointMed,
        testPointD,
        testPointsCount
    } TestPointsPos_e;

    enum {
        Cali_Init = 0,
        Cali_Top,
        Cali_Bottom,
        Cali_Left,
        Cali_Right,
        Cali_Center,
        Cali_Verify,
        Cali_End
    } CaliSteps_e;

    explicit AppCaliWindow(QWidget *parent = nullptr, const int & = modeCalibrate);
    ~AppCaliWindow();

    /// @brief      Update calibration window with new status
    /// @param      int
    ///             One of CaliSteps_e
    void CaliModeSet(const int &);

    /// @brief      Update calibration window's profile text with new data
    /// @param      uint8_t
    ///             Type of text to update
    /// @param      Char string
    ///             Buffer data
    void CaliModeTextUpdate(const uint8_t &, const char*);

    /// @brief      Update test mode window with list of new coords
    /// @param      Array of ints
    ///             Coordinates of the twelve test points used to draw
    void TestModeDraw(const int[12]);

    /// @brief      Store the camera blob data of the test points (sTestBlobs)
    /// @details    Sent by the firmware just before each sTestCoords: version byte, then for each
    ///             emitter (TL, TR, BL, BR) flags (bit 0 seen), average brightness, max brightness,
    ///             area (uint16 LE). Unknown versions or short payloads are ignored.
    /// @param      Payload bytes and their count
    void TestModeBlobs(const uint8_t *, const int &);

    /// @brief      Calibration IR view (caliFlagIrView): emitters of a Diamond layout are drawn
    ///             at the middle of the sides of the panel, the others at its corners
    void SetCaliIrDiamond(const bool &diamond) { caliIrDiamond = diamond; }

    /// @brief      Calibration IR view: the board refused a target shot (sCaliIrWarning)
    /// @param      uint8_t
    ///             bit 0 an emitter is missing
    void CaliIrWarning(const uint8_t &);

    int GetWindowMode() { return mode; }

    /// @brief      Objects to hold new cali profile values
    int topOffset = -1;
    int bottomOffset = -1;
    int leftOffset = -1;
    int rightOffset = -1;
    float topLeftLed = -1;
    float topRightLed = -1;

    void Shutdown() { emit WindowExiting(mode); }

signals:
    /// @brief      Signals back to the main app that the window is exiting
    /// @details    This should be emitted in any situation that the window may exit
    /// @param      mode
    ///             Window mode this object was launched in; must be one of ApCaliStates_e
    /// @param      ints & floats
    ///             Profile offsets for Calibration Mode
    void WindowExiting(const int &mode, const int & = -1, const int & = -1, const int & = -1, const int & = -1, const float & = -1, const float & = -1);

    /// @brief      Signals to the main app that a calibration window has received an exit keystroke
    ///             and to request the board to follow.
    /// @note       Main app is the only part that has access to the current active serialPort object.
    void CaliRequestToExit();

private:
    enum {
        ScaleTiny = 0,
        ScaleSmall,
        ScaleBig,
        ScaleHiDPI
    } ScaleTypes_e;

    enum {
        TextHeading = 0,
        TextSub,
        TextSmall,
        Crosshair
    } TextScaleTypes_e;

    QStringList caliTypesPrefixes = {
        tr("Top Offset: "),
        tr("Bottom Offset: "),
        tr("Left Offset: "),
        tr("Right Offset: "),
        tr("Top Left LED: "),
        tr("Top Right LED: ")
    };

    Ui::AppCaliWindow *ui;

    /// @brief      Very basic approximated HiDPI-aware values for both bitmap and font texts
    /// @returns    An appropriate scaling value, depending on value passed through of TextScaleTypes_e
    ///             and depending on whether bitmapText is enabled or not.
    int GetTextScale(const int &);

    /// @brief      Generates input from text into a bitmap representation using the app's builtin "test" typeface
    /// @details    Optional color bit tints the pixmap into the provided color.
    ///             Optional scale: the pixmap is drawn at the size it is shown with that item scale
    ///             (device pixel ratio), so that characters not in the typeface are drawn smooth.
    /// @returns    Pixmap of the string list provided.
    QPixmap GenerateText(const QStringList &, const QColor & = QColor(), const int & = 1);

    /// @brief      One line of text in pieces of their own colour (invalid QColor: white)
    /// @details    Each piece is drawn with GenerateText at the given scale and placed after the previous one.
    /// @returns    Pixmap of the line.
    QPixmap GenerateTextSegments(const QList<QPair<QString, QColor>> &, const int & = 1);

    /// @brief      Type of text scale appropriate for this window
    /// @details    Value is one of ScaleTypes_e
    int scale = -1;

    /// @brief      What mode this window was opened as
    /// @returns    A value of AppCaliStates_e
    int mode = -1;

    /// @brief      Whether to use bitmaps or font-based text
    /// @details    In case normal fonts are ever desirable.
    bool bitmapText = true;

    /// @brief      Indicator that mouse tracking is allowed
    bool mouseCanBeTracked = false;

    /// @brief      The cali screen renderer itself
    /// @details    Subclass was needed to implement specific mouse capture and key event listening
    AppCaliGraphicsScene scene;

    /// @brief      The crosshair used in calibration UX
    QGraphicsSvgItem *crosshairItem;

    /// @brief      Objects used to construct the look of the alignment view
    /// @details    Two sets of differently colored boxes for Square and Diamond IR layouts,
    ///             And two polygons that connects these together.
    QGraphicsRectItem *alignmentBoxesSquare[4];
    QGraphicsRectItem *alignmentBoxesDiamond[4];
    QGraphicsPolygonItem *alignmentLines[2];

    /// @brief      Test Mode screen points
    QGraphicsEllipseItem *testPoints[testPointsCount];

    /// @brief      Test Mode polygon that connects across the test points
    QGraphicsPolygonItem *testBox;

    /// @brief      Test Mode blob data (sTestBlobs) of each emitter, as received
    struct TestBlob_s {
        bool seen = false;
        bool weak = false;  // seen, but below the firmware brightness limit (flag bit 1)
        uint8_t avg = 0;
        uint8_t max = 0;
        uint16_t area = 0;
    };
    TestBlob_s testBlobs[4];

    /// @brief      Test Mode circle of each emitter, smoothed over the 20 Hz updates
    struct TestBlobLevel_s {
        bool valid = false;
        qreal radius = 0;   // 1920x1080 test space
        qreal center = 0;   // fill 0..1 at the centre (max brightness)
        qreal edge = 0;     // fill 0..1 at the edge (average brightness)
    };
    TestBlobLevel_s testBlobLevels[4];

    /// @brief      Time of the last sTestBlobs; blob data older than IRTEST_BLOBS_MAX_AGE is not used
    QElapsedTimer testBlobsTimer;

    /// @brief      True when recent sTestBlobs data is available (otherwise the classic view is drawn)
    bool TestBlobsFresh() const;

    /// @brief      Draws one emitter from its blob data (seen: sized gradient circle; not seen: dashed circle with a red X)
    void TestModeDrawEmitter(const int &, const qreal &, const qreal &);

    /// @brief      Red X of the emitters not seen by the camera
    QGraphicsPathItem *testNotSeenMarks[4] = {nullptr, nullptr, nullptr, nullptr};

    /// @brief      Area and brightness under each emitter, shown with the --irdebug argument
    QGraphicsPixmapItem *testBlobInfo[4] = {nullptr, nullptr, nullptr, nullptr};
    bool testBlobDebug = false;

    /// @brief      Scale and offset of the 1920x1080 test space in the scene
    qreal testScale = 1;
    qreal testOffsetX = 0;
    qreal testOffsetY = 0;

    /// @brief      Calibration IR view (caliFlagIrView): the firmware also sends sTestBlobs and
    ///             sTestCoords during calibration. The crosshair turns green when all four emitters
    ///             are seen and none is weak, a panel in the bottom right corner shows each emitter,
    ///             and the board refuses every target shot (sCaliIrWarning) while one is missing.
    ///             Nothing changes with firmware that does not send them.
    bool caliIrDiamond = false;
    int caliIrStage = Cali_Init;                // stage shown (the hint line is only for the target shots)
    QElapsedTimer caliIrCoordsTimer;            // last sTestCoords
    bool caliIrSeenByCoords[4] = {false, false, false, false}; // without blob data: outside-FOV bit
    uint8_t caliIrWarningBits = 0;              // last refused shot, shown for CALI_IR_WARNING_TIME
    QTimer caliIrWarningTimer;
    QTimer caliIrExpiryTimer;                   // redraws when the IR data gets old
    QSvgRenderer caliCrosshairRenderer;         // crosshair, recoloured by CaliIrColor()
    QByteArray caliCrosshairSvg;                // original SVG
    QColor caliCrosshairColor;                  // colour loaded in caliCrosshairRenderer (invalid = original)
    QGraphicsPixmapItem *caliIrHintBitmap = nullptr;
    QGraphicsRectItem *caliIrWarningBox = nullptr;
    QGraphicsPixmapItem *caliIrWarningBitmap = nullptr;
    QGraphicsPixmapItem *caliIrPanel = nullptr;

    /// @brief      Calibration IR view: crosshair colour from the worst emitter, or invalid without recent data
    QColor CaliIrColor() const;

    /// @brief      Calibration IR view: redraws crosshair colour, hint line, refusal message and panel
    void CaliIrRefresh();

    /// @brief      Calibration IR view: arms caliIrExpiryTimer for the moment the IR data gets old
    ///             (blobs first, then coordinates), so that a stopped link does not leave a green crosshair
    void CaliIrArmExpiry();

    /// @brief      Calibration: only an outer dashed ring rotates, in the crosshair colour.
    ///             Original crosshair size/position are unchanged; no coloured ring while verifying.
    void CaliTargetRingUpdate();
    QGraphicsPathItem *caliTargetRing = nullptr;
    QTimer caliTargetRingTimer;
    QElapsedTimer caliTargetTimer;              // since the crosshair reached its target
    int caliTargetStage = -1;                   // stage of caliTargetTimer

    /// @brief Six target dots, without shot animation.
    void CaliProgressUpdate();
    QGraphicsEllipseItem *caliProgressDots[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

    /// @brief Solid white flash; keep the crosshair still until it ends, without delaying data handling.
    void CaliShotRingUpdate();
    QGraphicsPathItem *caliShotRing = nullptr;
    QTimer caliShotRingTimer;
    QElapsedTimer caliShotTimer;
    QPointF caliNextTargetPos;                // latest position to display when the flash ends
    int caliShotStage = -1;                    // latest target acquired, not a save to flash

    /// @brief      Calibration IR view: lines of the refusal message for the given sCaliIrWarning bits
    QStringList CaliIrWarningLines(const uint8_t &);

    /// @brief      Calibration IR view: the emitters panel, at most textRight wide on the left (scene x)
    void CaliIrDrawPanel(const qreal &textRight);

    /// @brief      Text bitmaps generated from test font files
    /// @details    Contents are generated at runtime from GenerateText

    /// @brief      Main top text bitmap, used for primary info strings
    /// @details    For cali, this displays tutorial text to the user
    QGraphicsPixmapItem *headerBitmap;

    /// @brief      Indicator of cali stage that goes above headerText in Calibration Mode
    QGraphicsPixmapItem *caliStageBitmap;

    /// @brief      Text bitmaps used for showing current profile data during calibration
    /// @details    In order: Top Offset, Bottom Offset, Left Offset, Right Offset, Top Left LED, Top Right LED
    QGraphicsPixmapItem *profileBitmaps[6];

    /// @brief      Bottom text bitmap used for showing how to exit calibration (or other button related instructions) to the user
    QGraphicsPixmapItem *tutorialBitmap;

    /// @brief      Posture reminder above the bottom instructions throughout calibration, including verification.
    QGraphicsPixmapItem *caliPositionHintBitmap = nullptr;
    QGraphicsTextItem *caliPositionHintText = nullptr;

    /// @brief      Side text used to present Alignment Mode information about the different IR emitter layouts
    QGraphicsPixmapItem *alignmentBitmapLeft;
    QGraphicsPixmapItem *alignmentBitmapRight;

    /// @brief      Side colored text used to emphasize different layout types in Alignment Mode
    QGraphicsPixmapItem *alignmentBitmapColoredLeft;
    QGraphicsPixmapItem *alignmentBitmapColoredRight;

    // Below is older native font-based implementations of text objects
    // Could be useful in future for non-English languages?
    /// @brief      Main top text item, used for primary info strings
    /// @details    For cali, this displays tutorial text to the user
    QGraphicsTextItem *headerText;

    /// @brief      Indicator of cali stage that goes above headerText in Calibration Mode
    QGraphicsTextItem *caliStageText;

    /// @brief      Text used for showing current profile data during calibration
    /// @details    In order: Top Offset, Bottom Offset, Left Offset, Right Offset, Top Left LED, Top Right LED
    QGraphicsTextItem *profileText[6];

    /// @brief      Bottom text used for showing how to exit calibration (or other button related instructions) to the user
    QGraphicsTextItem *tutorialText;

    /// @brief      Side text used to present Alignment Mode information about the different IR emitter layouts
    QGraphicsTextItem *alignmentTextLeft;
    QGraphicsTextItem *alignmentTextRight;

private slots:

    /// @brief      Takes mouse coords from QGraphicsScene mouse move event and update crosshair position
    /// @details    Crosshair image is offset (adjusted by crosshair scale) to center it to the mouse's hotspot,
    ///             as otherwise it wouldn't be accurate at all.
    void sceneEventReceiver(const QPointF &currentPos) {
        if(mouseCanBeTracked) {
            const QPointF position = currentPos - crosshairItem->boundingRect().center() * crosshairItem->scale();
            if(caliShotStage >= 0)
                caliNextTargetPos = position; // preserve the latest verification cursor, but keep the flash still
            else
                crosshairItem->setPos(position);
        }
    }

    /// @brief      Key listener, which only listens for ESC keypresses and exits when pressed in not Cali mode
    void sceneKeyCloseReceiver() {
        if(mode != modeCalibrate)
             emit WindowExiting(mode);
        else emit CaliRequestToExit();
    }

};

#endif // APPCALI_H
