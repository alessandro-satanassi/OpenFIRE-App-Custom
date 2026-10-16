/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Main interface.

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

#include "appmainwindow.h"
#include "appabout.h"
#include "appcommon.h"
#include "ui_appmainwindow.h"
#include "../boards/OpenFIREshared.h"
#include <QThread>

#include <QGraphicsScene>
#include <QMessageBox>
#include <QSvgRenderer>
#include <QPainter>
#include <QSerialPortInfo>
#include <QProgressBar>
#include <QProcess>
//#include <QStorageInfo>
#include <QtConcurrentRun>
#include <QFileDialog>
#include <QColorDialog>
#include <QInputDialog>
#include <QDesktopServices>
#include <QFontDatabase>
#include <QUrl>
#include <QScopedValueRollback>
#include <algorithm>
#include <iterator>

guiWindow::guiWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::guiWindow)
{
    ui->setupUi(this);

#if !defined(Q_OS_MAC) && !defined(Q_OS_WIN)
    if(qEnvironmentVariable("USER") != "root") {
        QProcess *externalProg = new QProcess;
        QStringList args;
        externalProg->start("/usr/bin/groups", args);
        externalProg->waitForFinished();
        if(!externalProg->readAllStandardOutput().contains("dialout")) {
            QMessageBox::critical(this, tr("ERROR: User doesn't have serial permissions!"),
                                        tr("Currently, your user is not allowed to have access to serial devices.\n\n"
                                        "To add yourself to the right group, run this command in a terminal and then re-login to your session:\n\n"
                                        "sudo usermod -aG dialout ") + qEnvironmentVariable("USER"));
            exit(0);
        }
    } else {
        QMessageBox::critical(this, tr("ERROR: Running as root is not allowed!"), tr("Please run the OpenFIRE app as a normal user."));
        exit(2);
    }
#endif

#ifndef OFAPP_DEBUG
    ui->actionDebug_Window->setVisible(false);
#endif

    // Connect together Serial stuff
    connect(&serialSearchWatcher, &QFutureWatcher<bool>::finished, this, &guiWindow::serialPort_SearchFinished);
    connect(&serial, &AppSerial::Serial_EventReceived, this, &guiWindow::serialPort_readyRead, Qt::QueuedConnection);
    connect(&serial, &AppSerial::Serial_SetProgressRange, this, &guiWindow::serialPort_progressSet);
    connect(&serial, &AppSerial::Serial_ProgressUpdate, this, &guiWindow::serialPort_progressUpdate);

    connect(&aliveTimer, &QTimer::timeout, this, &guiWindow::aliveTimer_timeout);

    // Start initial serial search
    aliveTimer.start(ALIVE_TIMER);
    aliveTimer_timeout();

#ifdef OFAPP_GITHASH
    this->setWindowTitle("OpenFIRE App - " + QString(OFAPP_CODENAME) + " [v" + QString(OFAPP_VERSION) + '-' + QString(OFAPP_GITHASH) + ']');
#else
    this->setWindowTitle("OpenFIRE App - " + QString(OFAPP_CODENAME) + " [v" + QString(OFAPP_VERSION) + ']');
#endif // OFAPP_GITHASH

    // get all fixed interactable elements marked to use event filter for hover stuff:
    for(const auto &child : this->findChildren<QPushButton*>())
        if(!child->property("trackable").isNull()) child->installEventFilter(this);

    for(const auto &child : this->findChildren<QCheckBox*>())
        if(!child->property("trackable").isNull()) child->installEventFilter(this);

    for(const auto &child : this->findChildren<QLineEdit*>())
        if(!child->property("trackable").isNull()) child->installEventFilter(this);

    for(const auto &child : this->findChildren<QSpinBox*>())
        if(!child->property("trackable").isNull()) child->installEventFilter(this);

    for(const auto &child : this->findChildren<QComboBox*>())
        if(!child->property("trackable").isNull()) child->installEventFilter(this);

    for(const auto &child : this->findChildren<QRadioButton*>())
        if(!child->property("trackable").isNull()) child->installEventFilter(this);

    // Connect boards view "custom layouts" actions to the button
    ui->customLayoutToolBtn->addActions({ui->actionImport_Custom_Layout, ui->actionExport_Custom_Layout});

    // NC inputs: the box unlocking the pins' NC checkboxes sits next to "Use Custom Pins", shown with custom pins only
    invertEditToggle = new QCheckBox(tr("Edit NC Inputs"));
    invertEditToggle->setToolTip(tr("Enable to change the NC (normally closed) checkboxes of the pins. "
                                    "This box is not saved: it only protects the inversions from changes made by mistake."));
    invertEditToggle->setSizePolicy(ui->customPinsEnabled->sizePolicy());
    invertEditToggle->setMaximumHeight(ui->customPinsEnabled->maximumHeight());
    invertEditToggle->setHidden(true);
    ui->PinsBottomHalf->insertWidget(ui->PinsBottomHalf->indexOf(ui->customPinsEnabled)+1, invertEditToggle, 0, Qt::AlignBottom);
    connect(invertEditToggle, &QCheckBox::toggled, this, [this](bool) { InvertsUpdate(); });

    // Add center board pic above the Sub Pins layout
    ui->PinsCenter->insertWidget(0, &boardPic, 1);
    boardPic.setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // Setup inputs map screen stuff
    for(auto i = App_Common::keyboardInputsMap.cbegin(), end = App_Common::keyboardInputsMap.cend(); i != end; ++i)
        App_Common::kbOrderedStrings[i.value().at(1)] = (char*)i.key().c_str();
    for(auto i = App_Common::mouseMap.cbegin(), end = App_Common::mouseMap.cend(); i != end; ++i)
        App_Common::mouseOrderedStrings[i.value().at(1)] = (char*)i.key().c_str();
    for(auto i = App_Common::gamepadMap.cbegin(), end = App_Common::gamepadMap.cend(); i != end; ++i)
        App_Common::gpadOrderedStrings[i.value().at(1)] = (char*)i.key().c_str();

    for(int i = 0; i < BUTTON_COUNT-1; ++i) {
        for(const auto &item : App_Common::inputFuncTypesStrings) {
            btnFuncBox[0][0][i].addItem(item);
            btnFuncBox[1][0][i].addItem(item);
            btnFuncBox[2][0][i].addItem(item);
        }

        btnFuncLayout << new QHBoxLayout();
        btnFuncLayout.at(i)->addWidget(new QLabel(App_Common::OFPresets.boardInputs_sortedStr[i+1]), 1, Qt::AlignRight);
        btnFuncLayout.at(i)->addSpacing(4);

        for(int slot = 0; slot < 3; ++slot) {
            if(slot > 0) btnFuncLayout.at(i)->addSpacing(6);

            btnFuncLayout.at(i)->addWidget(&btnFuncBox[slot][0][i]);

            btnFuncBox[slot][0][i].setCurrentIndex(-1);
            btnFuncBox[slot][0][i].setProperty("curType", -1);

            // only onscreen/offscreen have editable func type selectors
            if(slot >= 2) {
                btnFuncBox[slot][0][i].setEnabled(false);
                btnFuncBox[slot][0][i].setFrame(false);
            }
            connect(&btnFuncBox[slot][0][i], SIGNAL(currentIndexChanged(int)), this, SLOT(btnFuncTypeBox_currentIndexChanged(int)));

            btnFuncBox[slot][0][i].setProperty("slot", slot);
            btnFuncBox[slot][0][i].setProperty("btn", i);
            btnFuncBox[slot][0][i].setProperty("trackable", App_Common::trackButtonMapItem);
            btnFuncBox[slot][0][i].installEventFilter(this);

            btnFuncLayout.at(i)->addWidget(&btnFuncBox[slot][1][i], 1);
            btnFuncBox[slot][1][i].setMaxVisibleItems(10);
            btnFuncBox[slot][1][i].setProperty("slot", slot);
            btnFuncBox[slot][1][i].setProperty("btn", i);
            btnFuncBox[slot][1][i].setProperty("trackable", App_Common::trackButtonMapItem);
            btnFuncBox[slot][1][i].installEventFilter(this);
            switch(slot) {
            case 0:
                btnFuncBox[slot][0][i].setAccessibleName(tr("Onscreen Button Output Type for %1").arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                btnFuncBox[slot][0][i].setWhatsThis(tr("<p>Select the type of button that <i>%1</i> will function as <b>when the gun is pointing at the screen.</b></p>"
                                                       "<p>Each available input defined in the current <i>Board Layout</i> can be defined as a button press for one of the "
                                                       "three available device outputs that the gun presents to the connected device.</p>")
                                                    .arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                btnFuncBox[slot][1][i].setAccessibleName(tr("Onscreen Button Mapping for %1").arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                btnFuncBox[slot][1][i].setWhatsThis(tr("<p>Select the output that <i>%1</i> will send to the connected device <b>when the gun is pointing at the screen.</b></p>"
                                                       "<p>Each available input defined in the current <i>Board Layout</i> can be defined as a button press for one of the "
                                                       "three available device outputs that the gun presents to the connected device.</p>")
                                                    .arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                break;
            case 1:
                btnFuncBox[slot][0][i].setAccessibleName(tr("Offscreen Button Output Type for %1").arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                btnFuncBox[slot][0][i].setWhatsThis(tr("<p>Select the type of button that <i>%1</i> will function as <b>when the gun is pointing outside of the screen.</b></p>"
                                                       "<p>Each available input defined in the current <i>Board Layout</i> can be defined as a button press for one of the "
                                                       "three available device outputs that the gun presents to the connected device.</p>")
                                                    .arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                btnFuncBox[slot][1][i].setAccessibleName(tr("Offscreen Button Mapping for %1").arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                btnFuncBox[slot][1][i].setWhatsThis(tr("<p>Select the output that <i>%1</i> will send to the connected device <b>when the gun is pointing outside of the screen.</b></p>"
                                                       "<p>Each available input defined in the current <i>Board Layout</i> can be defined as a button press for one of the "
                                                       "three available device outputs that the gun presents to the connected device.</p>")
                                                    .arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                break;
            case 2:
                btnFuncBox[slot][0][i].setAccessibleName(tr("Gamepad Mode Output Type for %1").arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                btnFuncBox[slot][0][i].setWhatsThis(tr("<p>Select the type of button that <i>%1</i> will function as <b>when the gun set to Gamepad Output Mode.</b></p>"
                                                       "<p>Only Gamepad-type outputs are available for Gamepad Output Mode, which can be set via "
                                                       "<a href='https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32/wiki/Serial_Commands_OpenFIRE_EN'><span style=' text-decoration: underline; color:#8ab4f8;'>Serial command</span></a> "
                                                       "<tt>M0x1</tt>.</p>")
                                                    .arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                btnFuncBox[slot][1][i].setAccessibleName(tr("Gamepad Mode Button Mapping for %1").arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                btnFuncBox[slot][1][i].setWhatsThis(tr("<p>Select the output that <i>%1</i> will send to the connected device <b>when the gun is set to Gamepad Output Mode.</b></p>"
                                                       "<p>Only Gamepad buttons are available to be mapped for Gamepad Output Mode, which can be set via "
                                                       "<a href='https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32/wiki/Serial_Commands_OpenFIRE_EN'><span style=' text-decoration: underline; color:#8ab4f8;'>Serial command</span></a> "
                                                       "<tt>M0x1</tt>.</p>"
                                                       "<p><b>NOTE:</b> When connected to the MiSTer FPGA device, these mappings will NOT be reflected, "
                                                       "as OpenFIRE has a hard-coded button layout specifically optimized for the MiSTer ecosystem.</p>")
                                                    .arg(App_Common::OFPresets.boardInputs_sortedStr[i+1]));
                break;
            }

            connect(&btnFuncBox[slot][1][i], &QComboBox::currentTextChanged, this, &guiWindow::btnFuncBox_currentTextChanged);
        }

        btnFuncGBoxes << new QGroupBox();
        btnFuncGBoxes.at(i)->setFlat(true);
        btnFuncGBoxes.at(i)->setLayout(btnFuncLayout.at(i));

        ui->btnFuncLayout->addWidget(btnFuncGBoxes.at(i));
    }

    // Setup test screen buttons
    for(int i = 0; i < BUTTON_COUNT; ++i) {
        testLabel << new QLabel(App_Common::OFPresets.boardInputs_sortedStr[i+1]);

        testLabel.at(i)->setEnabled(false);
        testLabel.at(i)->setAlignment(Qt::AlignCenter);
        testLabel.at(i)->setFrameStyle(QFrame::Box | QFrame::Raised);

        // A long name (the wireless pedal ones) wraps inside its own box
        // instead of stretching its column.
        testLabel.at(i)->setWordWrap(true);

        // third/second/first row of buttons
        if(i > 9)        ui->btnsLayout->addWidget(testLabel.at(i), 2, i-10);
        else if(i > 4)   ui->btnsLayout->addWidget(testLabel.at(i), 1, i-5);
        else             ui->btnsLayout->addWidget(testLabel.at(i), 0, i);
    }

    // The five columns share the width evenly, so no single label can widen one
    // of them and leave the others cramped; the three rows share the height the
    // same way, so a box that wraps onto three lines does not shrink the others.
    for(int column = 0; column < 5; ++column)
        ui->btnsLayout->setColumnStretch(column, 1);
    for(int row = 0; row < 3; ++row)
        ui->btnsLayout->setRowStretch(row, 1);

    // Setup analog stick viewer
    ui->analogGfxView->setScene(&analogGfxScene);
    analogGfxScene.setSceneRect(ui->analogGfxView->rect());
    analogPos = analogGfxScene.addEllipse(analogGfxScene.sceneRect().center().x()-8,
                                          analogGfxScene.sceneRect().center().y()-8,
                                          16, 16,
                                          QPen(QColor(255,100,0), 0), QBrush(QColor(255,125,0)));
    analogPos->setTransform(QTransform::fromScale(analogGfxScene.sceneRect().width() / 256, analogGfxScene.sceneRect().height() / 256));

    // hiding tUSB elements by default since this can't be done from the off
    ui->tUSBLayoutAdvanced->setVisible(false);

    // set hidden by default until a board with presets is loaded
    ui->presetsBox->setVisible(false);
    ui->solenoidTempBox->setVisible(false);

    statusBar()->showMessage(tr("Welcome to the OpenFIRE app!"), 3000);

    statusProgressBar = new QProgressBar();
    statusProgressBar->setVisible(false);
    ui->statusBar->addPermanentWidget(statusProgressBar);

    // Disable ONLY the tab widget (doing this from the form also disables children, including the scroll area)
    ui->tabWidget->setEnabled(false);

    // light mode styling adjustments:
    if(this->palette().window().color().value() > this->palette().text().color().value()) {
        ui->settingsDescBox->setStyleSheet("QGroupBox::title { color: #909000 }");
        ui->settingsDescText->setStyleSheet("color: doubledarkgray");
        ui->profilesDescBox->setStyleSheet("QGroupBox::title { color: #909000 }");
        ui->profilesDescText->setStyleSheet("color: doubledarkgray");
        ui->btnFuncDescBox->setStyleSheet("QGroupBox::title { color: #909000 }");
        ui->btnFuncDescText->setStyleSheet("color: doubledarkgray");
    }
}

guiWindow::~guiWindow()
{

    aliveTimer.stop();

    if(serialSearchWatcher.isRunning())
        serialSearchFuture.waitForFinished();

    serialSearchPending = false;

    if(ui->comPortSelector->currentIndex() > 0)
        serial.Disconnect();

#ifdef OFAPP_DEBUG
    debugWindow.close();
#endif

    delete ui;
}


bool guiWindow::eventFilter(QObject* object, QEvent* event)
{
    if(event->type() == QEvent::Enter) {
        switch(object->property("trackable").toInt()) {
        case App_Common::trackPinbox:
        {
            // Copy and modify board pic array to change opacity of selected pin element, if existing.
            highlightBoardPic = origBoardPicFile;
            int i = highlightBoardPic.indexOf(QString("id=\"OF_pin%1\"").arg(object->property("slot").toInt()));
            if(i > -1) {
                i = highlightBoardPic.indexOf("opacity:0", i);
                if(i > -1) {
                    highlightBoardPic.replace(i+8, 1, '1');
                    boardPic.load(highlightBoardPic.toLocal8Bit());
                    boardPic.renderer()->setAspectRatioMode(Qt::KeepAspectRatio);
                }
            }
            break;
        }
        case App_Common::trackSettingsItem:
            ui->settingsDescBox->setTitle(object->property("accessibleName").toString());
            ui->settingsDescText->setText(object->property("whatsThis").toString());
            break;
        case App_Common::trackProfileItem:
            ui->profilesDescBox->setTitle(object->property("accessibleName").toString());
            ui->profilesDescText->setText(object->property("whatsThis").toString());
            break;
        case App_Common::trackButtonMapItem:
            ui->btnFuncDescBox->setTitle(object->property("accessibleName").toString());
            ui->btnFuncDescText->setText(object->property("whatsThis").toString());
            break;
        case App_Common::trackTestItem:
            break;
        }
    } else if(event->type() == QEvent::Leave) {
        if(object->property("trackable").toInt() == App_Common::trackPinbox) {
            boardPic.load(origBoardPicFile);
            boardPic.renderer()->setAspectRatioMode(Qt::KeepAspectRatio);
        }
    }

    // disable QComboBox scroll events (to prevent accidental pinbox index changing)
    if(!(event->type() == QEvent::Wheel && object->inherits("QComboBox")))
        return QWidget::eventFilter(object, event);
    else return true;
}


void guiWindow::aliveTimer_timeout()
{
    if(serialSearchPending || serialSearchWatcher.isRunning())
        return;

    serialSearchPending = true;


#if QT_VERSION_MAJOR > 5
    serialSearchFuture = QtConcurrent::run(&AppSerial::SearchPorts, &serial);
#else
    serialSearchFuture = QtConcurrent::run(&serial, &AppSerial::SearchPorts);
#endif

    serialSearchWatcher.setFuture(serialSearchFuture);
}

void guiWindow::BoxesUpdate()
{
    const QScopedValueRollback<int> bulk(pinBoxesDepth, pinBoxesDepth+1);

    // enabling custom pins
    if(App_Common::boolSettings[App_Common::dataCurrent][OF_Const::customPins]) {
        // enable pinboxes
        for(const auto &box : std::as_const(pinBoxes))
            box->setEnabled(true);

        // if the custom pins setting *grabbed from the gun* has been set
        if(App_Common::boolSettings[App_Common::dataOrig][OF_Const::customPins]) {
            // reset pinboxes
            for(const auto &box : std::as_const(pinBoxes))
                box->setCurrentIndex(OF_Const::btnUnmapped+1);

            // set pinboxes to copied values (pinbox index is off by 1)
            for(int i = 0; i < App_Common::inputsMap_orig.count(); ++i)
                if(App_Common::inputsMap_orig.value(i) > OF_Const::btnUnmapped &&
                   App_Common::inputsMap_orig.value(i) < pinBoxes.count() &&
                   i < OF_Const::boardInputsCount)
                    pinBoxes.at(App_Common::inputsMap_orig.value(i))->setCurrentIndex(i+1);

        // else, if the board *was using default maps* before switching to custom (no need to re-set pinboxes)
        } else {
            // copy original map, which clears this map (as boards using defaults comes with no actual map instated)
            App_Common::inputsMap = App_Common::inputsMap_orig;

            // copy presets to inputs map
            if(App_Common::OFPresets.boardsPresetsMap.count(App_Common::board.type.constData()))
                for(size_t i = 0; i < pinBoxes.count(); ++i)
                    if(presetMap->second.at(i) > OF_Const::btnUnmapped)
                        App_Common::inputsMap[presetMap->second.at(i)] = i;
        }

        // not when called in the middle of a preset or an import, which check at their end
        if(pinBoxesDepth == 1)
            FeedbackOffWithoutPins();
        return;

    // disabling custom pins, reset to presets
    } else {
        // reset inputs map, as it's not even referenced when custom pins are disabled
        for(const auto &box : std::as_const(pinBoxes))
            box->setEnabled(false), box->setCurrentIndex(OF_Const::btnUnmapped+1);

        // if available, copy preset layout to pinboxes
        if(App_Common::OFPresets.boardsPresetsMap.count(App_Common::board.type.constData()))
            for(size_t i = 0; i < pinBoxes.count(); ++i)
                pinBoxes.at(i)->setCurrentIndex(presetMap->second.at(i)+1);

        // not when called in the middle of a preset or an import, which check at their end
        if(pinBoxesDepth == 1)
            FeedbackOffWithoutPins();
        return;
    }
}


void guiWindow::DiffUpdate()
{
    CommitRecoveryUiUpdate();
    InvertsUpdate();
    size_t settingsDiff = 0;

    if(memcmp(App_Common::boolSettings[App_Common::dataCurrent], App_Common::boolSettings[App_Common::dataOrig], sizeof(App_Common::boolSettings[App_Common::dataCurrent])))
        ++settingsDiff;

    if(App_Common::boolSettings[App_Common::dataCurrent][OF_Const::customPins])
        if(App_Common::inputsMap_orig != App_Common::inputsMap)
            ++settingsDiff;

    if(memcmp(App_Common::settingsTable[App_Common::dataCurrent], App_Common::settingsTable[App_Common::dataOrig], sizeof(App_Common::settingsTable[App_Common::dataCurrent])))
        ++settingsDiff;

    if(memcmp(App_Common::inputFuncTable[App_Common::dataCurrent], App_Common::inputFuncTable[App_Common::dataOrig], sizeof(App_Common::inputFuncTable[App_Common::dataCurrent])))
        ++settingsDiff;

    if(memcmp(&App_Common::tinyUSBtable_orig, &App_Common::tinyUSBtable, sizeof(App_Common::tinyUSBtable_s)))
        ++settingsDiff;

    if(App_Common::board.selectedProfile != App_Common::board.previousProfile)
        ++settingsDiff;

    for(size_t i = 0; i < App_Common::profilesTable.count(); ++i)
        if(memcmp(&App_Common::profilesTable_orig.at(i), &App_Common::profilesTable.at(i), sizeof(App_Common::profilesTable_s)))
            ++settingsDiff;

    if(settingsDiff || serial.CommitNeedsRetry()) {
        ui->confirmButton->setText(tr("Save and Send Settings"));
        ui->confirmButton->setEnabled(true);
        ui->confirmButton->setIcon(QIcon::fromTheme("document-save"));
        SavePulseSet(true);
    } else {
        ui->confirmButton->setText(tr("[Nothing To Save]"));
        ui->confirmButton->setEnabled(false);
        ui->confirmButton->setIcon(QIcon());
        SavePulseSet(false);
    }
}

void guiWindow::SavePulseSet(const bool &on)
{
    // Same pace as the web app: one breath every 1.6 s.
    constexpr int SAVE_PULSE_PERIOD = 1600;  // ms
    constexpr qreal SAVE_PULSE_TINT = 0.4;   // button colour at most 40% towards the highlight colour
    constexpr qreal SAVE_PULSE_GLOW = 16;    // px, largest blur of the glow

    if(!on) {
        if(savePulse != nullptr)
            savePulse->stop();
        if(savePulseGlow != nullptr) {
            ui->confirmButton->setGraphicsEffect(nullptr); // deletes the effect
            savePulseGlow = nullptr;
            ui->confirmButton->setPalette(QPalette());     // back to the application palette
        }
        return;
    }

    if(savePulse == nullptr) {
        savePulse = new QVariantAnimation(this);
        savePulse->setDuration(SAVE_PULSE_PERIOD);
        savePulse->setStartValue(0.0);
        savePulse->setKeyValueAt(0.5, 1.0);
        savePulse->setEndValue(0.0);
        savePulse->setEasingCurve(QEasingCurve::InOutSine);
        savePulse->setLoopCount(-1);
        connect(savePulse, &QVariantAnimation::valueChanged, this, [this](const QVariant &value) {
            // Disabled elsewhere (test mode, saving, not connected): stop and restore the button.
            if(!ui->confirmButton->isEnabled()) {
                SavePulseSet(false);
                return;
            }
            const QPalette appPalette = QApplication::palette(ui->confirmButton);
            const QColor base = appPalette.color(QPalette::Button);
            const QColor accent = appPalette.color(QPalette::Highlight);

            // Breath: tint towards the highlight colour and back.
            const qreal t = value.toReal() * SAVE_PULSE_TINT;
            QPalette palette = appPalette;
            palette.setColor(QPalette::Button, QColor::fromRgbF(base.redF()   + (accent.redF()   - base.redF())   * t,
                                                                base.greenF() + (accent.greenF() - base.greenF()) * t,
                                                                base.blueF()  + (accent.blueF()  - base.blueF())  * t));
            ui->confirmButton->setPalette(palette);

            // Glow: spreads out and fades over each breath, like the ring of the web app.
            if(savePulseGlow == nullptr) {
                savePulseGlow = new QGraphicsDropShadowEffect();
                savePulseGlow->setOffset(0, 0);
                ui->confirmButton->setGraphicsEffect(savePulseGlow);
            }
            const qreal ring = (qreal)savePulse->currentLoopTime() / SAVE_PULSE_PERIOD;
            QColor glow = accent;
            glow.setAlphaF((float)(0.8 * (1 - ring)));
            savePulseGlow->setColor(glow);
            savePulseGlow->setBlurRadius(2 + SAVE_PULSE_GLOW * ring);
        });
    }
    if(savePulse->state() != QAbstractAnimation::Running)
        savePulse->start();
}


// In recovery, keep editing and Save available but avoid using partial HW state.
void guiWindow::CommitRecoveryUiUpdate()
{
    const bool pending = serial.CommitNeedsRetry();
    if(!pending && !commitRecoveryUiActive)
        return;
    commitRecoveryUiActive = pending;
    ui->buttonsTestArea->setEnabled(!pending);
    ui->feedbackTestsBox->setEnabled(!pending);
    ui->testBtn->setEnabled(!pending);
    for(int i = 0; i < caliBtn.size(); ++i)
        caliBtn.at(i)->setEnabled(!pending);
    for(int i = 0; i < selectedProfile.size(); ++i)
        selectedProfile.at(i)->setEnabled(!pending);
}

// Only runs either on initial load or save
void guiWindow::LabelsUpdate()
{
    // because App_Common::inputsMap uses pin no. starting from 0
    for(uint8_t i = 0; i < testLabel.count(); ++i) {
        const QString name = QString::fromUtf8(App_Common::OFPresets.boardInputs_sortedStr[i+1]);

        testLabel.at(i)->setStyleSheet("");
        if(App_Common::inputsMap.value(i) >= 0) {
            testLabel.at(i)->setText(name);
            testLabel.at(i)->setEnabled(true);

        // No pin for either pedal and the board was saved with the wireless pedal
        // box ticked: say so, and whether one actually paired at startup.
        } else if(IsPedal(i) && PedalWirelessMode()) {
            if(App_Common::board.pedalWireless) {
                testLabel.at(i)->setText(tr("%1\n(wireless)").arg(name));
                testLabel.at(i)->setEnabled(true);
            } else {
                testLabel.at(i)->setText(tr("%1\n(wireless, disconnected)").arg(name));
                testLabel.at(i)->setEnabled(false);
            }

        } else {
            testLabel.at(i)->setText(tr("%1 (N/C)").arg(name));
            testLabel.at(i)->setEnabled(false);
        }
    }

    ui->tmp36Label->setStyleSheet("");
    if(App_Common::inputsMap.value(OF_Const::tempPin) >= 0) {
        ui->tmp36Label->setText(tr("Temperature Read..."));
        ui->tmp36Label->setEnabled(true);
    } else {
        ui->tmp36Label->setText(tr("Temperature Sensor (N/C)"));
        ui->tmp36Label->setEnabled(false);
    }

    if(App_Common::inputsMap.value(OF_Const::ledR) >= 0) ui->redLedTestBtn->setEnabled(true);   else ui->redLedTestBtn->setEnabled(false);
    if(App_Common::inputsMap.value(OF_Const::ledG) >= 0) ui->greenLedTestBtn->setEnabled(true); else ui->greenLedTestBtn->setEnabled(false);
    if(App_Common::inputsMap.value(OF_Const::ledB) >= 0) ui->blueLedTestBtn->setEnabled(true);  else ui->blueLedTestBtn->setEnabled(false);
    if(App_Common::inputsMap.value(OF_Const::analogX) >= 0 && App_Common::inputsMap.value(OF_Const::analogY) >= 0)
        ui->analogGroup->setEnabled(true),  ui->aPosLabel->clear();
    else ui->analogGroup->setEnabled(false), ui->aPosLabel->setText(tr("Not Connected"));

    ui->boardLabel->setText(PrettifyName(App_Common::tinyUSBtable.tinyUSBname));
}


QString guiWindow::PrettifyName(QString name)
{
    if(name.isEmpty())
        name = "Unnamed Device";

    // append name of board to gun name string.
    if(App_Common::OFPresets.boardNames.count(App_Common::board.type.toStdString()))
         return name + " | " + App_Common::OFPresets.boardNames.at(App_Common::board.type.toStdString());
    else return name + " | " + App_Common::OFPresets.boardNames.at("generic");
}


void guiWindow::PixelsDiff()
{
    if( App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcount ] == App_Common::settingsTable[App_Common::dataOrig][OF_Const::customLEDcount ] &&
        App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDstatic] == App_Common::settingsTable[App_Common::dataOrig][OF_Const::customLEDstatic] &&
        App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcolor1] == App_Common::settingsTable[App_Common::dataOrig][OF_Const::customLEDcolor1] &&
        App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcolor2] == App_Common::settingsTable[App_Common::dataOrig][OF_Const::customLEDcolor2] &&
        App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcolor3] == App_Common::settingsTable[App_Common::dataOrig][OF_Const::customLEDcolor3]) {
        ui->pixelChangeNotice->setVisible(false);
    } else {
        ui->pixelChangeNotice->setVisible(true);
    }
}


void guiWindow::NewCaliWindow(const int &type) {
    if(caliWindow != nullptr)
        caliWindow->Shutdown();

    caliWindow = new AppCaliWindow(nullptr, type);
    connect(caliWindow, &AppCaliWindow::WindowExiting, this, &guiWindow::CaliWindowExiting);

    switch(type) {
    case AppCaliWindow::modeCalibrate:
    {
        const int profile = sender()->property("slot").toInt();
        caliWindow->setProperty("profile", profile);
        // IR view: the emitters panel shows the layout of the profile being calibrated.
        caliWindow->SetCaliIrDiamond((int)App_Common::profilesTable.at(profile).layoutType == OF_Const::layoutDiamond);
        connect(caliWindow, &AppCaliWindow::CaliRequestToExit,  this, &guiWindow::CaliWindowRequestedExit);
        break;
    }
    case AppCaliWindow::modeIRTest:
        caliWindow->SetIrTestDiamond(App_Common::profilesTable_orig.at(App_Common::board.selectedProfile).layoutType == OF_Const::layoutDiamond);
        ui->buttonsTestArea->setEnabled(false);
        ui->confirmButton->setEnabled(false);
        ui->confirmButton->setText(tr("[Disabled while in Test Mode]"));
        ui->confirmButton->setIcon(QIcon());
        ui->pinsTab->setEnabled(false);
        ui->settingsTab->setEnabled(false);
        ui->profilesTab->setEnabled(false);
        ui->feedbackTestsBox->setEnabled(false);
        ui->dangerZoneBox->setEnabled(false);
        break;
    case AppCaliWindow::modeAlignment:
    default:
        break;
    }

    caliWindow->showFullScreen();
}


void guiWindow::on_confirmButton_clicked()
{
    bool cameraChanged = (App_Common::settingsTable[App_Common::dataOrig][OF_Const::cameraModel] != App_Common::settingsTable[App_Common::dataCurrent][OF_Const::cameraModel]);

    QMessageBox messageBox(QMessageBox::Information, tr("Commit Confirmation"), tr("Are these settings okay?"), QMessageBox::Yes | QMessageBox::No);
    messageBox.setInformativeText(tr("These settings will be committed to your lightgun. Is that okay?"));

    if(messageBox.exec() == QMessageBox::Yes) {
        serialActive = true;

        ui->tabWidget->setEnabled(false);
        ui->comPortSelector->setEnabled(false);

        if(serial.CommitSettings()) {
            statusBar()->showMessage(tr("Sent settings successfully!"), 5000);
            ui->confirmButton->setEnabled(false);
            ui->confirmButton->setIcon(QIcon());

            if (cameraChanged) {
                QMessageBox advBox(QMessageBox::Information, tr("Camera Changed"), tr("Camera Model Changed Successfully"));
                advBox.setInformativeText(tr("The lightgun has automatically swapped the active sensors.\n\nIf you experience any unexpected issues or the new camera fails to respond, we recommend physically restarting the microcontroller."));
                advBox.exec();
            }

            // sync settings
            memcpy(App_Common::boolSettings[App_Common::dataOrig],
                   App_Common::boolSettings[App_Common::dataCurrent],
                   sizeof(App_Common::boolSettings[App_Common::dataOrig]));

            if(App_Common::boolSettings[App_Common::dataOrig][OF_Const::customPins])
                App_Common::inputsMap_orig = App_Common::inputsMap;
            else for(int i = 0; i < App_Common::inputsMap.size(); ++i)
                App_Common::inputsMap_orig[i] = -1;

            memcpy(App_Common::settingsTable[App_Common::dataOrig],
                   App_Common::settingsTable[App_Common::dataCurrent],
                   sizeof(App_Common::settingsTable[App_Common::dataCurrent]));

            // Copia “originale” delle associazioni pulsanti nell’App
            memcpy(App_Common::inputFuncTable[App_Common::dataOrig],
                   App_Common::inputFuncTable[App_Common::dataCurrent],
                   sizeof(App_Common::inputFuncTable[App_Common::dataOrig]));

            memcpy(&App_Common::tinyUSBtable_orig,
                   &App_Common::tinyUSBtable,
                   sizeof(App_Common::tinyUSBtable_s));

            App_Common::board.previousProfile = App_Common::board.selectedProfile;

            for(size_t i = 0; i < App_Common::profilesTable.count(); ++i)
                memcpy(&App_Common::profilesTable_orig[i],
                       &App_Common::profilesTable[i],
                       sizeof(App_Common::profilesTable_s));

            // Reflect new names in UI
            LabelsUpdate();

            // update (clear) diffs
            PixelsDiff();
            DiffUpdate();
            
            serialPort_progressSet(0);
            ui->tabWidget->setEnabled(true);
            ui->comPortSelector->setEnabled(true);
            serialActive = false;

        } else { 
            printf("Settings syncing failed!?\n"); 
            DiffUpdate(); // Save stays available even when normal diffs are zero.
            serialPort_progressSet(0);
            ui->tabWidget->setEnabled(true);
            ui->comPortSelector->setEnabled(true);
            serialActive = false;
        }

    } else { statusBar()->showMessage(tr("Save operation canceled."), 3000); }
}


void guiWindow::on_comPortSelector_currentTextChanged(const QString &text)
{
    // Clear stale states if any
    if(caliWindow != nullptr)
        if(caliWindow->GetWindowMode() != AppCaliWindow::modeAlignment)
            caliWindow->Shutdown();

    if(ui->comPortSelector->currentIndex() > 0) {
        printf("COM port set to %d\n", ui->comPortSelector->currentIndex());

        // try to init serial port
        // if returns false, it failed, so just turn the index back to initial.
        serialActive = true;
        if(serial.GetSettings(text)) {
            for(size_t i = 0; i < topOffset.count(); ++i) {
                delete topOffset.at(i);
                delete bottomOffset.at(i);
                delete leftOffset.at(i);
                delete rightOffset.at(i);
                delete TLled.at(i);
                delete TRled.at(i);
                delete renameBtn.at(i);
                delete selectedProfile.at(i);
                delete irSens.at(i);
                delete runMode.at(i);
                delete layoutMode.at(i);
                delete aspectRatio.at(i);
                delete color.at(i);
                delete caliBtn.at(i);
            }

            topOffset.clear();
            bottomOffset.clear();
            leftOffset.clear();
            rightOffset.clear();
            TLled.clear();
            TRled.clear();
            renameBtn.clear();
            selectedProfile.clear();
            irSens.clear();
            runMode.clear();
            layoutMode.clear();
            aspectRatio.clear();
            color.clear();
            caliBtn.clear();

            int caliBtnRow;
            for(int i = 0; i < App_Common::profilesTable.size(); ++i) {
                caliBtnRow = i/4;

                // create new assets for this profile
                renameBtn << new QPushButton();
                renameBtn.at(i)->setFlat(true);
                renameBtn.at(i)->setFixedWidth(20);
                renameBtn.at(i)->setIcon(QIcon(":/icon/edit.png"));
                renameBtn.at(i)->installEventFilter(this);
                renameBtn.at(i)->setProperty("slot", i);
                renameBtn.at(i)->setProperty("trackable", App_Common::trackProfileItem);
                renameBtn.at(i)->setAccessibleName(tr("Rename Profile %1").arg(i+1));
                renameBtn.at(i)->setWhatsThis(tr("<p>Click to rename this Calibration Profile.</p>"
                                                 "<p>Aside from differentiating between different profiles for different displays, "
                                                 "Cali Profile names are displayed in Pause Mode when using a compatible <i>I2C Display.</i></p>"));
                connect(renameBtn.at(i), &QPushButton::clicked, this, &guiWindow::renameBoxes_clicked);

                selectedProfile << new QRadioButton(QString("%1. %2").arg(i+1).arg(App_Common::profilesTable.at(i).profName));
                selectedProfile.at(i)->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
                if(i == App_Common::board.selectedProfile) selectedProfile.at(i)->setChecked(true);
                selectedProfile.at(i)->setProperty("slot", i);
                connect(selectedProfile.at(i), &QRadioButton::toggled, this, &guiWindow::selectedProfile_isChecked);

                topOffset       << new QLabel(QString("<tt>%1</tt>").arg(App_Common::profilesTable.at(i).topOffset      ));
                bottomOffset    << new QLabel(QString("<tt>%1</tt>").arg(App_Common::profilesTable.at(i).bottomOffset   ));
                leftOffset      << new QLabel(QString("<tt>%1</tt>").arg(App_Common::profilesTable.at(i).leftOffset     ));
                rightOffset     << new QLabel(QString("<tt>%1</tt>").arg(App_Common::profilesTable.at(i).rightOffset    ));
                TLled           << new QLabel(QString("<tt>%1</tt>").arg(App_Common::profilesTable.at(i).TLled          ));
                TRled           << new QLabel(QString("<tt>%1</tt>").arg(App_Common::profilesTable.at(i).TRled          ));

                irSens << new QComboBox();
                irSens.at(i)->addItems({"Default", "Higher", "Highest"});
                irSens.at(i)->setCurrentIndex(App_Common::profilesTable.at(i).irSensitivity);
                irSens.at(i)->installEventFilter(this);
                irSens.at(i)->setProperty("slot", i);
                irSens.at(i)->setProperty("type", App_Common::pBoxIRsens);
                irSens.at(i)->setProperty("trackable", App_Common::trackProfileItem);
                irSens.at(i)->setAccessibleName(tr("Camera Sensitivity for Profile %1").arg(i+1));
                irSens.at(i)->setWhatsThis(tr("<p>This setting determines the sensitivity of the IR Camera for this Calibration Profile.</p>"
                                              "<p>If the camera seems to have trouble picking up IR emitters (and is causing coarse cursor movement), "
                                              "adjusting this setting higher might fix issues with tracking.<br>"
                                              "Conversely, setting sensitivity too high may cause indirect IR sources "
                                              "(such as sunlight or IR bouncing off of reflective surfaces) "
                                              "to be picked up instead, causing the cursor to jitter or erratically jump across the screen.</p>"));
                connect(irSens.at(i), SIGNAL(activated(int)), this, SLOT(profileBoxes_activated(int)));

                runMode << new QComboBox();
                runMode.at(i)->addItems({"Normal", "1-Frame Avg", "2-Frame Avg"});
                runMode.at(i)->setCurrentIndex(App_Common::profilesTable.at(i).runMode);
                runMode.at(i)->installEventFilter(this);
                runMode.at(i)->setProperty("slot", i);
                runMode.at(i)->setProperty("type", App_Common::pBoxRunMode);
                runMode.at(i)->setProperty("trackable", App_Common::trackProfileItem);
                runMode.at(i)->setAccessibleName(tr("Camera Position Averaging Mode for Profile %1").arg(i+1));
                runMode.at(i)->setWhatsThis(tr("<p>This setting determines the cursor Averaging Mode for this Calibration Profile.</p>"
                                               "<p>The movement of the aiming cursor can be smoothed out by averaging a select number of frames, "
                                               "at the cost of a small increase in latency; conversely, disabling this position averaging can "
                                               "reduce latency, at the cost of some added jitter in mouse movement.</p>"
                                               "<p>The default is <b>1-Frame Avg</b>, which should be the preferred balance for most people.</p>"));
                connect(runMode.at(i), SIGNAL(activated(int)), this, SLOT(profileBoxes_activated(int)));

                layoutMode << new QComboBox();
                layoutMode.at(i)->addItems({"Square", "Diamond"});
                layoutMode.at(i)->setCurrentIndex(App_Common::profilesTable.at(i).layoutType);
                layoutMode.at(i)->installEventFilter(this);
                layoutMode.at(i)->setProperty("slot", i);
                layoutMode.at(i)->setProperty("type", App_Common::pBoxLayout);
                layoutMode.at(i)->setProperty("trackable", App_Common::trackProfileItem);
                layoutMode.at(i)->setAccessibleName(tr("IR Emitter Layout for Profile %1").arg(i+1));
                layoutMode.at(i)->setWhatsThis(tr("<p>This setting determines the IR Layout to be used with this Calibration Profile.</p>"
                                                  "<p>Each Cali Profile can be set to use either the <i>Square Layout,</i> "
                                                  "which uses two pairs of emitters on the top and bottom, and <i>Diamond Layout,</i> "
                                                  "which uses one emitter at the center of each side of the display.</p>"
                                                  "<p><i>Square Layout</i> generally has much higher accuracy at any angle and allows for "
                                                  "playing closer to the screen or using external Fish Eye lenses without viewport distortion, "
                                                  "while <i>Diamond Layout</i> is for screen compatibility with certain legacy lightgun systems "
                                                  "(allowing OpenFIRE guns to play with such other lightgun systems on the same display).</p>"
                                                  "<p>If unsure, use <b>Square Layout</b> "
                                                  "(unless you also use a different brand of lightgun that needs a diamond IR layout to function).</p>"));
                connect(layoutMode.at(i), SIGNAL(activated(int)), this, SLOT(profileBoxes_activated(int)));

                aspectRatio << new QComboBox();
                aspectRatio.at(i)->addItems({"16:9", "16:10", "3:2", "5:4", "4:3"});
                aspectRatio.at(i)->setCurrentIndex(App_Common::profilesTable.at(i).aspectRatio);
                aspectRatio.at(i)->installEventFilter(this);
                aspectRatio.at(i)->setProperty("slot", i);
                aspectRatio.at(i)->setProperty("type", App_Common::pBoxAR);
                aspectRatio.at(i)->setProperty("trackable", App_Common::trackProfileItem);
                aspectRatio.at(i)->setAccessibleName(tr("Aspect Ratio Correction for Profile %1").arg(i+1));
                aspectRatio.at(i)->setWhatsThis(tr("<p>This setting determines the type of Aspect Ratio Correction used for this Profile when <i>4:3 Mode</i> is set.</p>"
                                                   "<p>When <a href='https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32/wiki/Serial_Commands_OpenFIRE_EN'><span style=' text-decoration: underline; color:#8ab4f8;'>Serial command</span></a> "
                                                   "<tt>M3x1</tt> is received, the firmware stretches the effective range for fullscreen applications in Windows "
                                                   "that runs in resolutions <b>narrower</b> than the full display width/height; "
                                                   "this setting determines the stretch factor that will be used for these 4:3 applications.</p>"
                                                   "<p>Do note that this restriction exclusively applies to the <b>Windows Operating System ONLY "
                                                   "for legacy applications that DON'T support the monitor's full resolution;</b> <i>Linux-based systems</i> and games run via <i>Wine/Proton</i> "
                                                   "<b>do not need this workaround,</b> except for certain applications like <i>CXBX-Reloaded</i> that don't scale down the effective range correctly for 4:3 content.</p>"
                                                   "<p>If unsure, set to <b>the aspect ratio of your display.</b> Setting to <i>4:3</i> effectively disables range stretching when toggled.</p>"));
                connect(aspectRatio.at(i), SIGNAL(activated(int)), this, SLOT(profileBoxes_activated(int)));

                color << new QPushButton();
                color.at(i)->setFixedWidth(32);
                color.at(i)->setStyleSheet(QString("background-color: #%1").arg(App_Common::profilesTable.at(i).color, 6, 16, QLatin1Char('0')));
                color.at(i)->installEventFilter(this);
                color.at(i)->setProperty("slot", i);
                color.at(i)->setProperty("trackable", App_Common::trackProfileItem);
                color.at(i)->setAccessibleName(tr("Profile Menu Color for Cali Profile %1").arg(i+1));
                color.at(i)->setWhatsThis(tr("<p>Open a window to select the color used to represent this profile in <i>Pause Mode.</i></p>"
                                             "<p>Each profile can be assigned a color used to identify them when switching profiles on the lightgun itself, "
                                             "which is emitted by a 4-pin RGB LED and/or an active NeoPixel strand.</p>"));
                connect(color.at(i), &QPushButton::clicked, this, &guiWindow::colorBoxes_clicked);

                caliBtn << new QPushButton(tr("Calibrate Profile %1").arg(i+1));
                caliBtn.at(i)->installEventFilter(this);
                caliBtn.at(i)->setProperty("slot", i);
                caliBtn.at(i)->setProperty("trackable", App_Common::trackProfileItem);
                caliBtn.at(i)->setAccessibleName(tr("Open Calibration Window for Cali Profile %1").arg(i+1));
                caliBtn.at(i)->setWhatsThis(tr("Click to start the calibration process for this profile."));
                connect(caliBtn.at(i), &QPushButton::clicked, this, &guiWindow::caliBtns_clicked);

                topOffset.at(i)     ->setAlignment(Qt::AlignCenter);
                bottomOffset.at(i)  ->setAlignment(Qt::AlignCenter);
                leftOffset.at(i)    ->setAlignment(Qt::AlignCenter);
                rightOffset.at(i)   ->setAlignment(Qt::AlignCenter);
                TLled.at(i)         ->setAlignment(Qt::AlignCenter);
                TRled.at(i)         ->setAlignment(Qt::AlignCenter);

                ui->profilesArea->addWidget(renameBtn.at(i),       i+1, 0);
                ui->profilesArea->addWidget(selectedProfile.at(i), i+1, 1);
                ui->profilesArea->addWidget(topOffset.at(i),       i+1, 3);
                ui->profilesArea->addWidget(bottomOffset.at(i),    i+1, 5);
                ui->profilesArea->addWidget(leftOffset.at(i),      i+1, 7);
                ui->profilesArea->addWidget(rightOffset.at(i),     i+1, 9);
                ui->profilesArea->addWidget(TLled.at(i),           i+1, 11);
                ui->profilesArea->addWidget(TRled.at(i),           i+1, 13);
                ui->profilesArea->addWidget(irSens.at(i),          i+1, 15);
                ui->profilesArea->addWidget(runMode.at(i),         i+1, 17);
                ui->profilesArea->addWidget(layoutMode.at(i),      i+1, 19);
                ui->profilesArea->addWidget(aspectRatio.at(i),     i+1, 21);
                ui->profilesArea->addWidget(color.at(i),           i+1, 23);

                //ui->caliBtnsLayout->addWidget(caliBtn.at(i), caliBtnRow, i);
                ui->caliBtnsLayout->addWidget(caliBtn.at(i), caliBtnRow, i % 4);
            }

            // update button mapping tab
            for(int i = 0; i < BUTTON_COUNT-1; ++i) {
                // update the type boxes; the signal for these will handle the other box for each type
                for(int slot = 0; slot < 3; ++slot) {
                    btnFuncBox[slot][0][i].setCurrentIndex(-1);
                    btnFuncBox[slot][0][i].setCurrentIndex(App_Common::inputFuncTable[App_Common::dataOrig][i][2*slot]);
                }
            }

            // Clears old board layout items
            if(pinBoxes.count()) {
                for(int i = 0; i < pinBoxes.count(); ++i)
                    delete pinBoxes.at(i);
                for(int i = 0; i < pinLabel.count(); ++i)
                    delete pinLabel.at(i);
                for(int i = 0; i < pinInverts.count(); ++i)
                    delete pinInverts.at(i);

                pinBoxes.clear();
                pinLabel.clear();
                pinInverts.clear();
            }

            // check for recognized board; else, refer to generic maps
            QFile resource;
            presetMap = App_Common::OFPresets.boardsPresetsMap.find(App_Common::board.type.constData());
            layoutMap = App_Common::OFPresets.boardsBoxPositions.find(App_Common::board.type.constData());
            if(presetMap == App_Common::OFPresets.boardsPresetsMap.cend()) {
                presetMap = App_Common::OFPresets.boardsPresetsMap.find("generic");
                layoutMap = App_Common::OFPresets.boardsBoxPositions.find("generic");
                resource.setFileName(":/boardPics/generic");
            } else resource.setFileName(":/boardPics/" + App_Common::board.type);

            // check if board has pin capability overrides, else fallback to architecture capabilities
            pinCapabilityMap = App_Common::OFPresets.mcuCapableMaps.find(App_Common::board.type.constData());
            if(pinCapabilityMap == App_Common::OFPresets.mcuCapableMaps.cend())
                pinCapabilityMap = App_Common::OFPresets.mcuCapableMaps.find(App_Common::board.arch.constData());

            for(int i = 0; i < layoutMap->second.size(); ++i) {
                pinBoxes << new QComboBox();
                pinBoxes.at(i)->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
                pinBoxes.at(i)->setProperty("slot", i);
                pinBoxes.at(i)->setProperty("prevMapping", OF_Const::btnUnmapped+1);
                pinBoxes.at(i)->setProperty("trackable", App_Common::trackPinbox);
                pinBoxes.at(i)->installEventFilter(this);
                // install items
                for(auto item : App_Common::OFPresets.boardInputs_sortedStr)
                    pinBoxes.at(i)->addItem(item);

                // clear out analog options for digital pins
                // (entrylist is offset by one, as "Unmapped" == -1 in our enum)
                if(!(pinCapabilityMap->second.at(i) & OF_Const::pinHasADC)) {
                    SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::analogX+1, false);
                    SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::analogY+1, false);
                    SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::tempPin+1, false);
                }

                // filter out SCL/SDA if possible.
                if(!(pinCapabilityMap->second.at(i) & OF_Const::pinAnyI2C)) {
                    if(pinCapabilityMap->second.at(i) & OF_Const::pinCanI2C) {
                        if(pinCapabilityMap->second.at(i) & OF_Const::pinIsI2CSCL) {
                            SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::camSDA+1,     false);
                            SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::periphSDA+1,  false);
                        } else {
                            SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::camSCL+1,     false);
                            SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::periphSCL+1,  false);
                        }

                        if(pinCapabilityMap->second.at(i) & OF_Const::pinIsI2C1)
                            pinLabel << new QLabel(QString("<font color=#FF8800>«GPIO%1»</font>").arg(i));
                        else pinLabel << new QLabel(QString("<font color=#0099FF>«GPIO%1»</font>").arg(i));
                    } else {
                        SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::camSDA+1,     false);
                        SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::camSCL+1,     false);
                        SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::periphSDA+1,  false);
                        SetComboBoxItemEnabled(pinBoxes.at(i), OF_Const::periphSCL+1,  false);
                        pinLabel << new QLabel(QString("«GPIO%1»").arg(i));
                    }
                } else pinLabel << new QLabel(QString("<font color=#BE00B0>«GPIO%1»</font>").arg(i));

                // connect up combobox signal
                connect(pinBoxes.at(i), SIGNAL(currentIndexChanged(int)), this, SLOT(pinBoxes_currentIndexChanged(int)));

                pinLabel.at(i)->setEnabled(false);
                pinLabel.at(i)->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
                pinLabel.at(i)->setToolTip(tr("GPIO Pin No. %1.\n\n"
                                              "Blue pin numbers are members of I2C0.\n"
                                              "Orange are members of I2C1.\n"
                                              "Purple pin numbers can automatically select any I2C channel in software.\n"
                                              "Gray cannot use I2C devices.").arg(i));
            }

            if(App_Common::board.version.indexOf('-') > -1) {
                // every board runs this project's firmware: the commit is always in its repository, as in the WebApp
                ui->versionLabel->setText(QString("FW <tt>v%1<a href='https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32/commit/%2'><span style=' text-decoration: underline; color:#8ab4f8;'>%2</span></a></tt>")
                                                 .arg(App_Common::board.version.left(App_Common::board.version.indexOf('-')+1), App_Common::board.version.mid(App_Common::board.version.indexOf('-')+1)));
            } else ui->versionLabel->setText("FW <tt>v" + App_Common::board.version + "</tt>");

            // update presets box if this board has any
            ui->presetsBox->clear();

            if(App_Common::OFPresets.boardsAltPresets.count(App_Common::board.type.constData())) {
                ui->presetsBox->setHidden(false);
                ui->presetsBox->setEnabled(true);

                auto iter = App_Common::OFPresets.boardsAltPresets.equal_range(App_Common::board.type.constData());
                for(auto i = iter.first; i != iter.second; ++i)
                    ui->presetsBox->addItem(i->second.name);
            } else {
                ui->presetsBox->setEnabled(false);
                ui->presetsBox->setHidden(true);
            }

            // set boxes to reflect indexes of inputsMap
            BoxesUpdate();

            LabelsUpdate();

            // Drawing the actual board view page by referencing the board maps data from OpenFIREshared.h
            resource.open(QIODevice::ReadOnly);
            origBoardPicFile = resource.readAll();

            // NC checkbox of each shown pin, on the outer side of its box (below it for the middle pins);
            // hidden, keeping its space, for the functions that can't be inverted (see InvertsUpdate)
            pinInverts.fill(nullptr, pinBoxes.count());
            // margin: some space from the border on its side (below for the middle pins)
            auto newInvert = [this](const int &slot, const char *margin) {
                QCheckBox *invert = new QCheckBox(tr("NC"));
                invert->setStyleSheet(QString("QCheckBox { %1: 8px; }").arg(margin));
                invert->setProperty("margin", margin);
                invert->setToolTip(tr("Inverted logic: tick it for a normally closed (NC) contact, read as released while closed."));
                invert->setProperty("slot", slot);
                QSizePolicy policy = invert->sizePolicy();
                policy.setRetainSizeWhenHidden(true);
                invert->setSizePolicy(policy);
                invert->setHidden(true);
                connect(invert, &QCheckBox::clicked, this, &guiWindow::pinInverts_clicked);
                pinInverts[slot] = invert;
                return invert;
            };

            for(int i = 0; i < pinBoxes.count(); ++i) {
                switch(layoutMap->second.at(i) & OF_Const::posCheck) {
                case OF_Const::posLeft:
                    ui->PinsLeft->addWidget(newInvert(i, "margin-left"),
                                            layoutMap->second.at(i) ^ OF_Const::posLeft,
                                            0);
                    ui->PinsLeft->addWidget(pinBoxes.at(i),
                                            layoutMap->second.at(i) ^ OF_Const::posLeft,
                                            1);
                    ui->PinsLeft->addWidget(pinLabel.at(i),
                                            layoutMap->second.at(i) ^ OF_Const::posLeft,
                                            2);
                    break;
                case OF_Const::posRight:
                    ui->PinsRight->addWidget(pinBoxes.at(i),
                                             layoutMap->second.at(i) ^ OF_Const::posRight,
                                             1);
                    ui->PinsRight->addWidget(pinLabel.at(i),
                                             layoutMap->second.at(i) ^ OF_Const::posRight,
                                             0);
                    ui->PinsRight->addWidget(newInvert(i, "margin-right"),
                                             layoutMap->second.at(i) ^ OF_Const::posRight,
                                             2);
                    break;
                case OF_Const::posMiddle:
                    ui->PinsCenterSub->addWidget(pinBoxes.at(i),
                                                 1,
                                                 layoutMap->second.at(i) ^ OF_Const::posMiddle);
                    ui->PinsCenterSub->addWidget(pinLabel.at(i),
                                                 0,
                                                 layoutMap->second.at(i) ^ OF_Const::posMiddle);
                    ui->PinsCenterSub->addWidget(newInvert(i, "margin-bottom"),
                                                 2,
                                                 layoutMap->second.at(i) ^ OF_Const::posMiddle,
                                                 Qt::AlignHCenter);
                    break;
                case OF_Const::posNothing:
                    break;
                }
            }

            // aspect ratio hint needs to be set every time a new asset is loaded
            boardPic.load(origBoardPicFile);
            boardPic.renderer()->setAspectRatioMode(Qt::KeepAspectRatio);

            for(int i = 1; i < ui->PinsLeft->rowCount(); ++i)
                if(ui->PinsLeft->itemAtPosition(i, 0) == nullptr)
                    ui->PinsLeft->setRowMinimumHeight(i, 25);

            for(int i = 1; i < ui->PinsRight->rowCount(); ++i)
                if(ui->PinsRight->itemAtPosition(i, 0) == nullptr)
                    ui->PinsRight->setRowMinimumHeight(i, 25);

            ui->tabWidget->setEnabled(true);
            ui->customPinsEnabled->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::customPins]);
            // the NC checkboxes are locked again at every load
            invertEditToggle->setChecked(false);
            InvertsUpdate();

            ui->aStickModeBox->setCurrentIndex(App_Common::settingsTable[App_Common::dataOrig][OF_Const::analogMode]);
            ui->aStickInvertXToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::analogInvertX]);
            ui->aStickInvertYToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::analogInvertY]);

            ui->rumbleToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::rumble]);
            ui->rumbleSettingsBox->setEnabled(App_Common::boolSettings[App_Common::dataOrig][OF_Const::rumble]);
            ui->solenoidToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::solenoid]);
            ui->solenoidSettingsBox->setEnabled(App_Common::boolSettings[App_Common::dataOrig][OF_Const::solenoid]);
            ui->autofireToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::autofire]);
            ui->autofireToggle->setEnabled((App_Common::boolSettings[App_Common::dataOrig][OF_Const::solenoid] || App_Common::boolSettings[App_Common::dataOrig][OF_Const::rumbleFF]));
            ui->simplePauseToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::simplePause]);
            ui->holdToPauseToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::holdToPause]);
            ui->commonAnodeToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::commonAnode]);
            ui->lowButtonsToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::lowButtonsMode]);
            ui->pedalWirelessToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::pedalWireless]);
            ui->bootOutputModeBox->setCurrentIndex(App_Common::settingsTable[App_Common::dataOrig][OF_Const::bootOutputMode]);
            ui->rumbleFFToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::rumbleFF]);
            ui->rumbleIntensityBox->setValue(App_Common::settingsTable[App_Common::dataOrig][OF_Const::rumbleStrength]);
            ui->rumbleLengthBox->setValue(App_Common::settingsTable[App_Common::dataOrig][OF_Const::rumbleInterval]);
            ui->holdToPauseLengthBox->setValue(App_Common::settingsTable[App_Common::dataOrig][OF_Const::holdToPauseLength]);
            ui->solenoidOnLengthBox->setValue(App_Common::settingsTable[App_Common::dataOrig][OF_Const::solenoidOnLength]);
            ui->solenoidOffLengthBox->setValue(App_Common::settingsTable[App_Common::dataOrig][OF_Const::solenoidOffLength]);
            ui->solenoidHoldLengthBox->setValue(App_Common::settingsTable[App_Common::dataOrig][OF_Const::solenoidHoldLength]);
            ui->invertStaticPixelsBox->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::invertStaticPixels]);
            ui->tempWarningBox->setValue(App_Common::settingsTable[App_Common::dataOrig][OF_Const::tempWarning]);
            ui->tempShutoffBox->setValue(App_Common::settingsTable[App_Common::dataOrig][OF_Const::tempShutdown]);

            ui->i2cOLEDtoggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::i2cOLED]);
            ui->oledAltAddrsToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::i2cOLEDaltAddr]);

            ui->neopixelStrandLengthBox->setValue(App_Common::settingsTable[App_Common::dataOrig][OF_Const::customLEDcount]);
            ui->customLEDstaticSpinbox->setValue(App_Common::settingsTable[App_Common::dataOrig][OF_Const::customLEDstatic]);
            ui->customLEDstaticBtn1->setStyleSheet(QString("background-color: #%1").arg(App_Common::settingsTable[App_Common::dataOrig][OF_Const::customLEDcolor1], 6, 16, QLatin1Char('0')));
            ui->customLEDstaticBtn2->setStyleSheet(QString("background-color: #%1").arg(App_Common::settingsTable[App_Common::dataOrig][OF_Const::customLEDcolor2], 6, 16, QLatin1Char('0')));
            ui->customLEDstaticBtn3->setStyleSheet(QString("background-color: #%1").arg(App_Common::settingsTable[App_Common::dataOrig][OF_Const::customLEDcolor3], 6, 16, QLatin1Char('0')));

            switch(App_Common::settingsTable[App_Common::dataOrig][OF_Const::cameraModel]) {
            case OF_Const::DFRobot_SEN0158: ui->camModel_DFRobot->setChecked(true); break;
            case OF_Const::PixArt_PAJ7025R2: ui->camModel_PAJ7025R2->setChecked(true); break;
            case OF_Const::PixArt_PAJ7025R3: ui->camModel_PAJ7025R3->setChecked(true); break;
            }
            ui->cameraModelUnlockToggle->setChecked(false);

            switch(App_Common::tinyUSBtable.tinyUSBid) {
            case 1:
                ui->tUSB_p1->setChecked(true);
                ui->tUSBLayoutAdvanced->setVisible(false);
                ui->tUSBLayoutSimple->setVisible(true);
                ui->tinyUSBLayoutToggle->setChecked(false);
                break;
            case 2:
                ui->tUSB_p2->setChecked(true);
                ui->tUSBLayoutAdvanced->setVisible(false);
                ui->tUSBLayoutSimple->setVisible(true);
                ui->tinyUSBLayoutToggle->setChecked(false);
                break;
            case 3:
                ui->tUSB_p3->setChecked(true);
                ui->tUSBLayoutAdvanced->setVisible(false);
                ui->tUSBLayoutSimple->setVisible(true);
                ui->tinyUSBLayoutToggle->setChecked(false);
                break;
            case 4:
                ui->tUSB_p4->setChecked(true);
                ui->tUSBLayoutAdvanced->setVisible(false);
                ui->tUSBLayoutSimple->setVisible(true);
                ui->tinyUSBLayoutToggle->setChecked(false);
                break;
            default:
                ui->tUSB_p1->setChecked(false);
                ui->tUSB_p2->setChecked(false);
                ui->tUSB_p3->setChecked(false);
                ui->tUSB_p4->setChecked(false);
                ui->tUSBLayoutSimple->setVisible(false);
                ui->tUSBLayoutAdvanced->setVisible(true);
                ui->tinyUSBLayoutToggle->setChecked(true);
                break;
            }

            ui->productIdInput->setValue(App_Common::tinyUSBtable.tinyUSBid);
            ui->productNameInput->setText(App_Common::tinyUSBtable.tinyUSBname);

            ui->tabWidget->setCurrentIndex(0);
            ui->comPortSelector->setItemText(0, tr("[Disconnect Current Device]"));

            if(App_Common::board.arch == App_Common::OFPresets.boardArchs[OF_Const::boardRP]) {
                ui->baudResetBtn->setText(tr("Reboot to Bootloader"));
            } else {
                ui->baudResetBtn->setText(tr("Restart Microcontroller in Firmware Update Mode"));
            }

            DiffUpdate(); // Also reflects a pending save reported on reconnect.
        } else ui->comPortSelector->setCurrentIndex(0);

        serialPort_progressSet(0);
    } else {
        ui->boardLabel->clear();
        ui->versionLabel->clear();
        App_Common::board.type.clear();
        App_Common::board.pedalWireless = false;

        // just in case
        ui->buttonsTestArea->setEnabled(true);
        ui->pinsTab->setEnabled(true);
        ui->settingsTab->setEnabled(true);
        ui->profilesTab->setEnabled(true);
        ui->feedbackTestsBox->setEnabled(true);
        ui->dangerZoneBox->setEnabled(true);

        ui->tabWidget->setEnabled(false);

        if(ui->comPortSelector->count() > 0)
            ui->comPortSelector->setItemText(0, tr("[Select a Device to Configure]"));

        if(serial.port.isOpen())
            serial.Disconnect();

        CommitRecoveryUiUpdate();

        // reset temp label stylesheet to neutral
        ui->tmp36Label->setStyleSheet("");
        ui->confirmButton->setEnabled(false);
        ui->confirmButton->setText(tr("[Currently Not Connected]"));
        ui->confirmButton->setIcon(QIcon());
    }
    serialActive = false;
}


void guiWindow::pinBoxes_currentIndexChanged(int index)
{
    // using comboboxes' "slot" property to figure caller,
    // and "prevMapping" to get previous index, as this method immediately overwrites what it was mapped to.
    // always remember to sync the change to "prevMapping" property at the end of its logic path!
    const QScopedValueRollback<int> depth(pinBoxesDepth, pinBoxesDepth+1);

    /* For debugging pinBoxes (too lazy to ifdef guard)
    if(index >= 0 && index <= App_Common::inputsMap.size()) {
        //printf("Requesting pinbox %d to set to %s\n", sender()->property("slot").toInt(), App_Common::OFPresets.boardInputs_sortedStr.at(index).toLocal8Bit().constData());
    } else printf("Oops! Seems like pinbox %d is trying to set itself to index %d, which is out of range!\n", sender()->property("slot").toInt(), index);
    //*/

    // reset presets box, as it's no longer accurate for this layout
    if(ui->presetsBox->currentIndex() > -1)
        ui->presetsBox->setCurrentIndex(-1);

    // if it's being set to 0 (unmapped), unmap this pin without question.
    if(index <= 0) {
        if(sender()->property("prevMapping").toInt() > OF_Const::btnUnmapped+1) {
            App_Common::inputsMap[sender()->property("prevMapping").toInt()-1] = OF_Const::btnUnmapped;
            sender()->setProperty("prevMapping", OF_Const::btnUnmapped+1);
        }

    // else, it's a function, so check for duplicates
    } else if(sender()->property("prevMapping").toInt() != index) {
        int8_t btnRequest = index - 1;

        // unmap pinbox's previous function, if mapped to any
        if(sender()->property("prevMapping").toInt() > OF_Const::btnUnmapped+1)
            App_Common::inputsMap[sender()->property("prevMapping").toInt()-1] = OF_Const::btnUnmapped;

        // Remove whatever pin mapping that this function belonged to, if it was mapped
        // (making sure we don't disable the pin trying to be set)
        if(App_Common::inputsMap.value(btnRequest)  > OF_Const::btnUnmapped &&
           App_Common::inputsMap.value(btnRequest) != sender()->property("slot").toInt())
            pinBoxes.at(App_Common::inputsMap.value(btnRequest))->setCurrentIndex(OF_Const::btnUnmapped+1);

        // if function is I2C, check for other things
        if(!(pinCapabilityMap->second.at(sender()->property("slot").toInt()) & OF_Const::pinAnyI2C)) {
            // I2C Data Members
            if(btnRequest == OF_Const::camSDA || btnRequest == OF_Const::periphSDA) {
                // if it's mapped, check that this I2C type's pair pin isn't mapped to the opposite I2C channel
                if(App_Common::inputsMap.value(btnRequest+1) > OF_Const::btnUnmapped &&
                   (pinCapabilityMap->second.at(sender()->property("slot").toInt()) & OF_Const::pinIsI2C1) != (pinCapabilityMap->second.at(App_Common::inputsMap.value(btnRequest+1)) & OF_Const::pinIsI2C1)) {
                    // channels mismatched, unmap the other pin
                    if(btnRequest == OF_Const::camSDA) {
                        pinBoxes.at(App_Common::inputsMap.value(OF_Const::camSCL))->setCurrentIndex(OF_Const::btnUnmapped+1);
                        ui->statusBar->showMessage(tr("Camera pins are not on the same I2C channel! Please check camera pins' mappings."), 10000);
                    } else {
                        pinBoxes.at(App_Common::inputsMap.value(OF_Const::periphSCL))->setCurrentIndex(OF_Const::btnUnmapped+1);
                        ui->statusBar->showMessage(tr("Peripheral pins are not on the same I2C channel! Please check peripheral pins' mappings."), 10000);
                    }
                }
                // check that opposite I2C type SDA isn't mapped to this I2C channel
                switch(btnRequest) {
                case OF_Const::camSDA:
                    if(App_Common::inputsMap.value(OF_Const::periphSDA) > OF_Const::btnUnmapped &&
                       (pinCapabilityMap->second.at(sender()->property("slot").toInt()) & OF_Const::pinIsI2C1) == (pinCapabilityMap->second.at(App_Common::inputsMap.value(OF_Const::periphSDA)) & OF_Const::pinIsI2C1)) {
                        // channels matched, unmap peripheral data
                        pinBoxes.at(App_Common::inputsMap.value(OF_Const::periphSDA))->setCurrentIndex(OF_Const::btnUnmapped+1);
                        ui->statusBar->showMessage(tr("Camera and Peripheral Data pins clashed! Please remap Peripheral SDA."), 10000);
                    }
                    break;
                case OF_Const::periphSDA:
                    if(App_Common::inputsMap.value(OF_Const::camSDA) > OF_Const::btnUnmapped &&
                       (pinCapabilityMap->second.at(sender()->property("slot").toInt()) & OF_Const::pinIsI2C1) == (pinCapabilityMap->second.at(App_Common::inputsMap.value(OF_Const::camSDA)) & OF_Const::pinIsI2C1)) {
                        // channels matched, unmap peripheral data
                        pinBoxes.at(App_Common::inputsMap.value(OF_Const::camSDA))->setCurrentIndex(OF_Const::btnUnmapped+1);
                        ui->statusBar->showMessage(tr("Camera and Peripheral Data pins clashed! Please remap Camera SDA."), 10000);
                    }
                    break;
                }
            } else if(btnRequest == OF_Const::camSCL || btnRequest == OF_Const::periphSCL) {
                // if it's mapped, check that this I2C type's pair pin isn't mapped to the opposite I2C channel
                if(App_Common::inputsMap.value(btnRequest-1) > OF_Const::btnUnmapped &&
                   (pinCapabilityMap->second.at(sender()->property("slot").toInt()) & OF_Const::pinIsI2C1) != (pinCapabilityMap->second.at(App_Common::inputsMap.value(btnRequest-1)) & OF_Const::pinIsI2C1)) {
                    // channels mismatched, unmap the other pin
                    if(btnRequest == OF_Const::camSCL) {
                        pinBoxes.at(App_Common::inputsMap.value(OF_Const::camSDA))->setCurrentIndex(OF_Const::btnUnmapped+1);
                        ui->statusBar->showMessage(tr("Camera pins are not on the same I2C channel! Please check camera pins' mappings."), 10000);
                    } else {
                        pinBoxes.at(App_Common::inputsMap.value(OF_Const::periphSDA))->setCurrentIndex(OF_Const::btnUnmapped+1);
                        ui->statusBar->showMessage(tr("Peripheral pins are not on the same I2C channel! Please check peripheral pins' mappings."), 10000);
                    }
                }
                // check that opposite I2C type SCL isn't mapped to this I2C channel
                switch(btnRequest) {
                case OF_Const::camSCL:
                    if(App_Common::inputsMap.value(OF_Const::periphSCL) > OF_Const::btnUnmapped &&
                       (pinCapabilityMap->second.at(sender()->property("slot").toInt()) & OF_Const::pinIsI2C1) == (pinCapabilityMap->second.at(App_Common::inputsMap.value(OF_Const::periphSCL)) & OF_Const::pinIsI2C1)) {
                        // channels matched, unmap peripheral data
                        pinBoxes.at(App_Common::inputsMap.value(OF_Const::periphSCL))->setCurrentIndex(OF_Const::btnUnmapped+1);
                        ui->statusBar->showMessage(tr("Camera and Peripheral Data pins clashed! Please remap Peripheral SCL."), 10000);
                    }
                    break;
                case OF_Const::periphSCL:
                    if(App_Common::inputsMap.value(OF_Const::camSCL) > OF_Const::btnUnmapped &&
                       (pinCapabilityMap->second.at(sender()->property("slot").toInt()) & OF_Const::pinIsI2C1) == (pinCapabilityMap->second.at(App_Common::inputsMap.value(OF_Const::camSCL)) & OF_Const::pinIsI2C1)) {
                        // channels matched, unmap peripheral data
                        pinBoxes.at(App_Common::inputsMap.value(OF_Const::camSCL))->setCurrentIndex(OF_Const::btnUnmapped+1);
                        ui->statusBar->showMessage(tr("Camera and Peripheral Data pins clashed! Please remap Camera SCL."), 10000);
                    }
                    break;
                }
            // if we ever get SPI accessories, we'll need to add them here too.
            }
        }

        // Then map the thing, and sync this change to the property value
        App_Common::inputsMap[btnRequest] = sender()->property("slot").toInt();
        sender()->setProperty("prevMapping", index);
    }

    // update settings panel to reflect pins map changes and prevent illegal values/combinations
    ui->rumbleFFBox->setEnabled(App_Common::inputsMap.value(OF_Const::rumblePin) >= 0);
    ui->solenoidFFBox->setEnabled(App_Common::inputsMap.value(OF_Const::solenoidPin) >= 0);

    ui->solenoidTempBox->setEnabled(App_Common::inputsMap.value(OF_Const::tempPin) > -1);
    ui->forceFeedbackBox->setEnabled(App_Common::inputsMap.value(OF_Const::rumblePin) > -1 || App_Common::inputsMap.value(OF_Const::solenoidPin) > -1);
    ui->neopixelGroupBox->setEnabled(App_Common::inputsMap.value(OF_Const::neoPixel) > -1);
    ui->commonAnodeToggle->setEnabled(App_Common::inputsMap.value(OF_Const::ledR) > -1 && App_Common::inputsMap.value(OF_Const::ledG) > -1 && App_Common::inputsMap.value(OF_Const::ledB) > -1);
    ui->i2cGroup->setEnabled(App_Common::inputsMap.value(OF_Const::periphSDA) > -1 && App_Common::inputsMap.value(OF_Const::periphSCL) > -1);

    ui->aStickFuncBox->setEnabled(App_Common::inputsMap.value(OF_Const::analogX) >= 0 && App_Common::inputsMap.value(OF_Const::analogY) >= 0);
    for(int i = 0; i < BUTTON_COUNT-1; ++i)
        btnFuncGBoxes.at(i)->setEnabled(InputAvailable(i));

    // only once the change is complete, so a function moved to another pin keeps its inversion,
    // and rumble/solenoid stay on when the layout still has their pins
    if(pinBoxesDepth == 1) {
        FeedbackOffWithoutPins();
        PruneInverts();
    }

    DiffUpdate();
}


/// button mapping
void guiWindow::btnFuncTypeBox_currentIndexChanged(int index)
{
    if(sender()->property("curType").toInt() != index) {
        sender()->setProperty("curType", index);
        if(index > -1) {
            QComboBox *btnFuncBoxPtr = &btnFuncBox[sender()->property("slot").toInt()][1][sender()->property("btn").toInt()];

            App_Common::inputFuncTable[App_Common::dataCurrent][sender()->property("btn").toInt()][sender()->property("slot").toInt() << 1] = index;
            btnFuncBoxPtr->blockSignals(true);
            btnFuncBoxPtr->clear();

            switch(index) {
            case App_Common::inputMouse:
                for(const auto &item : App_Common::mouseOrderedStrings)
                    btnFuncBoxPtr->addItem(item);
                break;
            case App_Common::inputKB:
                for(const auto &item : App_Common::kbOrderedStrings)
                    btnFuncBoxPtr->addItem(item);
                break;
            case App_Common::inputGamepad:
                for(const auto &item : App_Common::gpadOrderedStrings)
                    btnFuncBoxPtr->addItem(item);
                break;
            }

            btnFuncBoxPtr->blockSignals(false);

            // HACK SHACK: very brute-force-y method because I didn't wanna make another redundant af map for this lookup sorz :(
            if(App_Common::inputFuncTable[App_Common::dataOrig][sender()->property("btn").toInt()][sender()->property("slot").toInt() << 1] == index) {
                for(auto i = App_Common::inputFuncMaps[index]->cbegin(), end = App_Common::inputFuncMaps[index]->cend(); i != end; ++i)
                    if(i.value().at(0) == App_Common::inputFuncTable[App_Common::dataCurrent][sender()->property("btn").toInt()][1 | sender()->property("slot").toInt() << 1]) {
                        btnFuncBoxPtr->setCurrentIndex(i.value().at(1));
                        break;
                    }
            }

            DiffUpdate();
        }
    }
}


void guiWindow::btnFuncBox_currentTextChanged(const QString &str)
{
    if(!str.isEmpty()) {
        uint8_t *dataPtr = &App_Common::inputFuncTable[App_Common::dataCurrent][sender()->property("btn").toInt()][(2*sender()->property("slot").toInt())+1];
        switch(*(dataPtr-1)) {
        case App_Common::inputMouse:
            *dataPtr = App_Common::mouseMap.value(str.toStdString()).at(0);
            break;
        case App_Common::inputKB:
            *dataPtr = App_Common::keyboardInputsMap.value(str.toStdString()).at(0);
            break;
        case App_Common::inputGamepad:
            *dataPtr = App_Common::gamepadMap.value(str.toStdString()).at(0);
            break;
        }

        DiffUpdate();
    }
}


void guiWindow::on_aStickModeBox_currentIndexChanged(int index)
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::analogMode] = index;

    DiffUpdate();
}


// For analog sticks wired or mounted the other way round: the firmware reverses the axis
// before every stick output (gamepad stick, D-pad, arrow keys) and in the test view.
void guiWindow::on_aStickInvertXToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::analogInvertX] = arg1;
    DiffUpdate();
}


void guiWindow::on_aStickInvertYToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::analogInvertY] = arg1;
    DiffUpdate();
}


/// settings
void guiWindow::pinInverts_clicked(bool checked)
{
    const int slot = sender()->property("slot").toInt();
    const int input = pinBoxes.at(slot)->currentIndex()-1;
    if(Invertible(input))
        SetInverted(input, checked);

    DiffUpdate();
}


bool guiWindow::Invertible(const int &input)
{
    return App_Common::boolSettings[App_Common::dataCurrent][OF_Const::customPins] &&
           std::find(std::begin(OF_Const::invertibleInputs), std::end(OF_Const::invertibleInputs), input) != std::end(OF_Const::invertibleInputs);
}


bool guiWindow::Inverted(const int &input)
{
    return input >= 0 && input < 32 &&
           ((App_Common::settingsTable[App_Common::dataCurrent][OF_Const::inputInvertMask] >> input) & 1);
}


void guiWindow::SetInverted(const int &input, const bool &on)
{
    if(input < 0 || input >= 32) return;
    uint32_t &mask = App_Common::settingsTable[App_Common::dataCurrent][OF_Const::inputInvertMask];
    if(on) mask |= (1u << input);
    else mask &= ~(1u << input);
}


void guiWindow::FeedbackOffWithoutPins()
{
    if(App_Common::inputsMap.value(OF_Const::rumblePin) < 0) {
        ui->rumbleToggle->setChecked(false);
        ui->rumbleFFToggle->setChecked(false);
    }

    if(App_Common::inputsMap.value(OF_Const::solenoidPin) < 0)
        ui->solenoidToggle->setChecked(false);
}


void guiWindow::PruneInverts()
{
    for(const auto &input : OF_Const::invertibleInputs)
        if(App_Common::inputsMap.value(input) <= OF_Const::btnUnmapped)
            SetInverted(input, false);
}


void guiWindow::InvertsUpdate()
{
    const bool customOn = App_Common::boolSettings[App_Common::dataCurrent][OF_Const::customPins];
    if(!customOn && invertEditToggle->isChecked())
        invertEditToggle->setChecked(false);   // calls InvertsUpdate() again
    invertEditToggle->setHidden(!customOn);

    for(int i = 0; i < pinInverts.count() && i < pinBoxes.count(); ++i) {
        QCheckBox *invert = pinInverts.at(i);
        if(invert == nullptr) continue;

        const int input = pinBoxes.at(i)->currentIndex()-1;
        const bool allowed = Invertible(input);
        const bool inverted = allowed && Inverted(input);
        // locked: only the inverted inputs show it, as an "NC" tag without the box
        const bool badge = inverted && !invertEditToggle->isChecked();
        invert->setHidden(!allowed || (!invertEditToggle->isChecked() && !inverted));
        invert->setEnabled(allowed && invertEditToggle->isChecked());
        invert->setChecked(inverted);
        if(invert->property("badge").toBool() != badge) {
            invert->setProperty("badge", badge);
            const QString margin = invert->property("margin").toString();
            if(badge) {
                // drawn in the box's own space: an orange tag with a switch symbol and "NC", as in the WebApp
                const QSize box = invert->sizeHint();
                const QSizeF tag(box.width() - (margin == "margin-bottom" ? 0 : 8),
                                 qMin(18, box.height() - (margin == "margin-bottom" ? 8 : 0)));   // never taller than the box
                const qreal dpr = invert->devicePixelRatioF();
                QPixmap pix((tag * dpr).toSize());
                pix.setDevicePixelRatio(dpr);
                pix.fill(Qt::transparent);
                QPainter painter(&pix);
                painter.setRenderHint(QPainter::Antialiasing);
                painter.setPen(Qt::NoPen);
                painter.setBrush(QColor(0xc9, 0x6f, 0x00));
                painter.drawRoundedRect(QRectF(QPointF(0, 0), tag), tag.height() / 2, tag.height() / 2);
                QFont font = invert->font();
                font.setPixelSize(11);
                font.setBold(true);
                painter.setFont(font);
                const QFontMetricsF metrics(font);
                // icon + gap + "NC", centered in the tag; a narrow box (small system font) scales it down to fit
                const qreal content = 16 + 2 + metrics.horizontalAdvance(tr("NC"));
                const qreal fit = qMin<qreal>(1, (tag.width() - 6) / content);
                painter.translate(tag.width() / 2, tag.height() / 2);
                painter.scale(fit, fit);
                // icon: the symbol of a switch contact in white - a line, two circles and the lever rising
                // from the left circle to rest on the right one (the same drawing as the WebApp's style.css)
                QSvgRenderer(QByteArray("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 3.5 24 9'><g fill='none' stroke='white' stroke-width='1.6' stroke-linecap='round'>"
                                        "<path d='M0.5 8H4M20 8H23.5M8 8L18.8 4.6'/><circle cx='6' cy='8' r='2'/><circle cx='18' cy='8' r='2'/></g></svg>"))
                    .render(&painter, QRectF(-content / 2, -3, 16, 6));
                painter.setPen(Qt::white);
                painter.drawText(QPointF(-content / 2 + 18, metrics.capHeight() / 2), tr("NC"));
                painter.end();
                QIcon icon(pix);
                icon.addPixmap(pix, QIcon::Disabled);
                invert->setStyleSheet(QString("QCheckBox { %1: 8px; spacing: 0px; } QCheckBox::indicator { width: 0px; height: 0px; border: none; image: none; }").arg(margin));
                invert->setText(QString());
                invert->setIcon(icon);
                invert->setIconSize(tag.toSize());
                invert->setFixedSize(box);
            } else {
                invert->setStyleSheet(QString("QCheckBox { %1: 8px; }").arg(margin));
                invert->setText(tr("NC"));
                invert->setIcon(QIcon());
                invert->setMinimumSize(0, 0);
                invert->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
            }
        }
    }
}


void guiWindow::on_customPinsEnabled_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::customPins] = arg1;
    BoxesUpdate();

    // back as loaded: the saved inversions; otherwise the pins changed, so none
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::inputInvertMask] =
        (static_cast<bool>(arg1) == App_Common::boolSettings[App_Common::dataOrig][OF_Const::customPins]) ?
            App_Common::settingsTable[App_Common::dataOrig][OF_Const::inputInvertMask] : 0;

    ui->actionExport_Custom_Layout->setEnabled(arg1);

    // rumble and solenoid toggles without a pin are turned off by BoxesUpdate (or at the end of a preset or an import)
    ui->solenoidFFBox->setEnabled(App_Common::inputsMap.value(OF_Const::solenoidPin) > OF_Const::btnUnmapped);
    ui->rumbleFFBox->setEnabled(App_Common::inputsMap.value(OF_Const::rumblePin) > OF_Const::btnUnmapped);

    DiffUpdate();
}


void guiWindow::on_presetsBox_currentIndexChanged(int index)
{
    if(index > -1) {
        const QScopedValueRollback<int> bulk(pinBoxesDepth, pinBoxesDepth+1);

        // presets are inherently custom layouts
        if(!ui->customPinsEnabled->isChecked())
            ui->customPinsEnabled->setChecked(true);

        // clear pinBoxes to be safe
        for(uint8_t i = 0; i < pinBoxes.count(); ++i)
            pinBoxes.at(i)->setCurrentIndex(OF_Const::btnUnmapped+1);

        // set pinboxes to alt preset values (and let the index changed signal handle the rest)
        auto preset = App_Common::OFPresets.boardsAltPresets.equal_range(App_Common::board.type.toStdString()).first;
        for(int i = 0; i < index; ++i)
            preset++;

        for(int i = 0; i < pinBoxes.count(); ++i)
            pinBoxes.at(i)->setCurrentIndex(preset->second.pin[i]+1);

        FeedbackOffWithoutPins();

        // every function was taken off its pin
        for(const auto &input : OF_Const::invertibleInputs)
            SetInverted(input, false);

        DiffUpdate();
    }
}


void guiWindow::on_rumbleToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::rumble] = arg1;

    if(arg1) {
        ui->rumbleSettingsBox->setEnabled(true);
        ui->rumbleTestBtn->setEnabled(true);
    } else {
        ui->rumbleFFToggle->setChecked(false);
        ui->rumbleSettingsBox->setEnabled(false);
        ui->rumbleTestBtn->setEnabled(false);
    }

    if(!(arg1 && App_Common::boolSettings[App_Common::dataCurrent][OF_Const::rumbleFF]) && !App_Common::boolSettings[App_Common::dataCurrent][OF_Const::solenoid]) {
        ui->autofireToggle->setChecked(false);
        ui->autofireToggle->setEnabled(false);
    } else {
        ui->autofireToggle->setEnabled(true);
    }

    DiffUpdate();
}


void guiWindow::on_solenoidToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::solenoid] = arg1;

    if(arg1) {
        ui->rumbleFFToggle->setChecked(false);
        ui->solenoidSettingsBox->setEnabled(true);
        ui->solenoidTestBtn->setEnabled(true);
    } else {
        ui->solenoidSettingsBox->setEnabled(false);
        ui->solenoidTestBtn->setEnabled(false);
    }

    if(!arg1 && !(App_Common::boolSettings[App_Common::dataCurrent][OF_Const::rumble] && App_Common::boolSettings[App_Common::dataCurrent][OF_Const::rumbleFF])) {
        ui->autofireToggle->setChecked(false);
        ui->autofireToggle->setEnabled(false);
    } else {
        ui->autofireToggle->setEnabled(true);
    }

    DiffUpdate();
}


void guiWindow::on_autofireToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::autofire] = arg1;

    DiffUpdate();
}


void guiWindow::on_simplePauseToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::simplePause] = arg1;
    DiffUpdate();
}


void guiWindow::on_holdToPauseToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::holdToPause] = arg1;

    if(arg1) ui->holdToPauseLengthBox->setEnabled(true);
    else     ui->holdToPauseLengthBox->setEnabled(false);

    DiffUpdate();
}


void guiWindow::on_commonAnodeToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::commonAnode] = arg1;
    DiffUpdate();
}


void guiWindow::on_lowButtonsToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::lowButtonsMode] = arg1;
    DiffUpdate();
}


void guiWindow::on_pedalWirelessToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::pedalWireless] = arg1;

    // A wireless pedal has no pin, but it is still an input whose outputs are
    // worth setting: its mapping rows follow the box.
    for(int i = 0; i < BUTTON_COUNT-1; ++i)
        if(IsPedal(i))
            btnFuncGBoxes.at(i)->setEnabled(InputAvailable(i));

    DiffUpdate();
}


void guiWindow::on_bootOutputModeBox_currentIndexChanged(int index)
{
    // A value the box cannot show (an unknown mode from a newer board) leaves the
    // combo at -1: the stored value is kept as it is rather than overwritten.
    if(index < 0)
        return;

    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::bootOutputMode] = index;

    DiffUpdate();
}


void guiWindow::on_rumbleFFToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::rumbleFF] = arg1;
    if(arg1) ui->solenoidToggle->setChecked(false);

    if(!(arg1 && App_Common::boolSettings[App_Common::dataCurrent][OF_Const::rumble]) && !App_Common::boolSettings[App_Common::dataCurrent][OF_Const::solenoid]) {
        ui->autofireToggle->setChecked(false);
        ui->autofireToggle->setEnabled(false);
    } else {
        ui->autofireToggle->setEnabled(true);
    }

    DiffUpdate();
}


void guiWindow::on_tempWarningBox_valueChanged(int arg1)
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::tempWarning] = arg1;
    DiffUpdate();
}


void guiWindow::on_tempShutoffBox_valueChanged(int arg1)
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::tempShutdown] = arg1;
    DiffUpdate();
}


void guiWindow::on_rumbleIntensityBox_valueChanged(int arg1)
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::rumbleStrength] = arg1;
    DiffUpdate();
}


void guiWindow::on_rumbleLengthBox_valueChanged(int arg1)
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::rumbleInterval] = arg1;
    DiffUpdate();
}


void guiWindow::on_holdToPauseLengthBox_valueChanged(int arg1)
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::holdToPauseLength] = arg1;
    DiffUpdate();
}


void guiWindow::on_solenoidOnLengthBox_valueChanged(int arg1)
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::solenoidOnLength] = arg1;
    DiffUpdate();
}


void guiWindow::on_solenoidOffLengthBox_valueChanged(int arg1)
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::solenoidOffLength] = arg1;
    DiffUpdate();
}


void guiWindow::on_solenoidHoldLengthBox_valueChanged(int arg1)
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::solenoidHoldLength] = arg1;
    DiffUpdate();
}


void guiWindow::on_neopixelStrandLengthBox_valueChanged(int arg1)
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcount] = arg1;
    if(arg1 < App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDstatic]) {
        ui->customLEDstaticSpinbox->setValue(arg1);
    }

    // show NeoPixel notice if values are updated
    PixelsDiff();

    DiffUpdate();
}


void guiWindow::on_customLEDstaticSpinbox_valueChanged(int arg1)
{
    if(arg1 > App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcount]) { ui->customLEDstaticSpinbox->setValue(App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcount]); }
    else { App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDstatic] = arg1; }
    if(OF_Const::customLEDstatic) {
        switch(App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDstatic]) {
        case 1:
            ui->customLEDstaticBtn1->setEnabled(true);
            ui->customLEDstaticBtn2->setEnabled(false);
            ui->customLEDstaticBtn3->setEnabled(false);
            break;
        case 2:
            ui->customLEDstaticBtn1->setEnabled(true);
            ui->customLEDstaticBtn2->setEnabled(true);
            ui->customLEDstaticBtn3->setEnabled(false);
            break;
        case 3:
            ui->customLEDstaticBtn1->setEnabled(true);
            ui->customLEDstaticBtn2->setEnabled(true);
            ui->customLEDstaticBtn3->setEnabled(true);
            break;
        default:
            ui->customLEDstaticBtn1->setEnabled(false);
            ui->customLEDstaticBtn2->setEnabled(false);
            ui->customLEDstaticBtn3->setEnabled(false);
            break;
        }
    }

    // show NeoPixel notice if values are updated
    PixelsDiff();

    DiffUpdate();
}


void guiWindow::on_customLEDstaticBtn1_clicked()
{
    QColor colorPick = QColorDialog::getColor(App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcolor1]);
    if(colorPick.isValid()) {
        int *red = new int;
        int *green = new int;
        int *blue = new int;
        colorPick.getRgb(red, green, blue);
        uint32_t packedColor = 0;
        packedColor |= *red << 16;
        packedColor |= *green << 8;
        packedColor |= *blue;
        App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcolor1] = packedColor;
        ui->customLEDstaticBtn1->setStyleSheet(QString("background-color: #%1").arg(packedColor, 6, 16, QLatin1Char('0')));

        // show NeoPixel notice if values are updated
        PixelsDiff();

        DiffUpdate();
    }
}


void guiWindow::on_customLEDstaticBtn2_clicked()
{
    QColor colorPick = QColorDialog::getColor(App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcolor2]);
    if(colorPick.isValid()) {
        int *red = new int;
        int *green = new int;
        int *blue = new int;
        colorPick.getRgb(red, green, blue);
        uint32_t packedColor = 0;
        packedColor |= *red << 16;
        packedColor |= *green << 8;
        packedColor |= *blue;
        App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcolor2] = packedColor;
        ui->customLEDstaticBtn2->setStyleSheet(QString("background-color: #%1").arg(packedColor, 6, 16, QLatin1Char('0')));

        // show NeoPixel notice if values are updated
        PixelsDiff();

        DiffUpdate();
    }
}


void guiWindow::on_customLEDstaticBtn3_clicked()
{
    QColor colorPick = QColorDialog::getColor(App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcolor3]);
    if(colorPick.isValid()) {
        int *red = new int;
        int *green = new int;
        int *blue = new int;
        colorPick.getRgb(red, green, blue);
        uint32_t packedColor = 0;
        packedColor |= *red << 16;
        packedColor |= *green << 8;
        packedColor |= *blue;
        App_Common::settingsTable[App_Common::dataCurrent][OF_Const::customLEDcolor3] = packedColor;
        ui->customLEDstaticBtn3->setStyleSheet(QString("background-color: #%1").arg(packedColor, 6, 16, QLatin1Char('0')));

        // show NeoPixel notice if values are updated
        PixelsDiff();

        DiffUpdate();
    }
}


void guiWindow::on_invertStaticPixelsBox_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::invertStaticPixels] = arg1;

    if(arg1) ui->customLEDstaticSpinbox->setPrefix(tr("Last "));
    else     ui->customLEDstaticSpinbox->setPrefix(tr("First "));

    DiffUpdate();
}


void guiWindow::on_i2cOLEDtoggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::i2cOLED] = arg1;
    ui->oledGroup->setEnabled(arg1);

    DiffUpdate();
}


void guiWindow::on_oledAltAddrsToggle_stateChanged(int arg1)
{
    App_Common::boolSettings[App_Common::dataCurrent][OF_Const::i2cOLEDaltAddr] = arg1;

    DiffUpdate();
}

void guiWindow::on_cameraModelUnlockToggle_stateChanged(int arg1)
{
    bool enabled = (arg1 != 0);
    ui->camModel_DFRobot->setEnabled(enabled);
    ui->camModel_PAJ7025R2->setEnabled(enabled);
    ui->camModel_PAJ7025R3->setEnabled(enabled);
}

void guiWindow::on_camModel_DFRobot_clicked()
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::cameraModel] = OF_Const::DFRobot_SEN0158;
    DiffUpdate();
}

void guiWindow::on_camModel_PAJ7025R2_clicked()
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::cameraModel] = OF_Const::PixArt_PAJ7025R2;
    DiffUpdate();
}

void guiWindow::on_camModel_PAJ7025R3_clicked()
{
    App_Common::settingsTable[App_Common::dataCurrent][OF_Const::cameraModel] = OF_Const::PixArt_PAJ7025R3;
    DiffUpdate();
}

void guiWindow::on_tinyUSBLayoutToggle_stateChanged(int arg1)
{
    if(arg1) {
        ui->tUSBLayoutSimple->setVisible(false);
        ui->tUSBLayoutAdvanced->setVisible(true);
    } else {
        ui->tUSBLayoutAdvanced->setVisible(false);
        ui->tUSBLayoutSimple->setVisible(true);
    }
}


void guiWindow::on_tUSB_p1_toggled(bool checked)
{
    if(checked && !serialActive) {
        App_Common::tinyUSBtable.tinyUSBid = 1;
        memset(App_Common::tinyUSBtable.tinyUSBname, 0, sizeof(App_Common::tinyUSBtable_s::tinyUSBname));
        strcpy(App_Common::tinyUSBtable.tinyUSBname, "FIRECon P1");
        ui->productIdInput->setValue(App_Common::tinyUSBtable.tinyUSBid);
        ui->productNameInput->setText(App_Common::tinyUSBtable.tinyUSBname);

        DiffUpdate();
    }
}


void guiWindow::on_tUSB_p2_toggled(bool checked)
{
    if(checked && !serialActive) {
        App_Common::tinyUSBtable.tinyUSBid = 2;
        memset(App_Common::tinyUSBtable.tinyUSBname, 0, sizeof(App_Common::tinyUSBtable_s::tinyUSBname));
        strcpy(App_Common::tinyUSBtable.tinyUSBname, "FIRECon P2");
        ui->productIdInput->setValue(App_Common::tinyUSBtable.tinyUSBid);
        ui->productNameInput->setText(App_Common::tinyUSBtable.tinyUSBname);

        DiffUpdate();
    }
}


void guiWindow::on_tUSB_p3_toggled(bool checked)
{
    if(checked && !serialActive) {
        App_Common::tinyUSBtable.tinyUSBid = 3;
        memset(App_Common::tinyUSBtable.tinyUSBname, 0, sizeof(App_Common::tinyUSBtable_s::tinyUSBname));
        strcpy(App_Common::tinyUSBtable.tinyUSBname, "FIRECon P3");
        ui->productIdInput->setValue(App_Common::tinyUSBtable.tinyUSBid);
        ui->productNameInput->setText(App_Common::tinyUSBtable.tinyUSBname);

        DiffUpdate();
    }
}


void guiWindow::on_tUSB_p4_toggled(bool checked)
{
    if(checked && !serialActive) {
        App_Common::tinyUSBtable.tinyUSBid = 4;
        memset(App_Common::tinyUSBtable.tinyUSBname, 0, sizeof(App_Common::tinyUSBtable_s::tinyUSBname));
        strcpy(App_Common::tinyUSBtable.tinyUSBname, "FIRECon P4");
        ui->productIdInput->setValue(App_Common::tinyUSBtable.tinyUSBid);
        ui->productNameInput->setText(App_Common::tinyUSBtable.tinyUSBname);

        DiffUpdate();
    }
}


void guiWindow::on_productIdInput_valueChanged(int arg1)
{
    App_Common::tinyUSBtable.tinyUSBid = arg1;
    if(ui->productNameInput->text().isEmpty()) {
        switch(arg1) {
        case 1:
            ui->tUSB_p1->setChecked(true);
            break;
        case 2:
            ui->tUSB_p2->setChecked(true);
            break;
        case 3:
            ui->tUSB_p3->setChecked(true);
            break;
        case 4:
            ui->tUSB_p4->setChecked(true);
            break;
        default:
            ui->tUSB_p1->setChecked(false);
            ui->tUSB_p2->setChecked(false);
            ui->tUSB_p3->setChecked(false);
            ui->tUSB_p4->setChecked(false);
            break;
        }
    }

    DiffUpdate();
}


void guiWindow::on_productNameInput_textEdited(const QString &arg1)
{
    bool badInput = false;
    // Very unga-bunga way of doing this.
    // if someone is aware of a validator for this, feel free to replace this.
    for(int i = 0; i < arg1.length(); ++i) {
        if(arg1.at(i).unicode() > 255) {
            badInput = true;
            break;
        }
    }

    if(badInput) {
        ui->productNameInput->setText(App_Common::tinyUSBtable.tinyUSBname);
        ui->productNameInput->setStyleSheet("color: red");
    } else {
        if(!ui->productNameInput->styleSheet().isEmpty())
            ui->productNameInput->setStyleSheet("");
        strncpy(App_Common::tinyUSBtable.tinyUSBname, arg1.toLocal8Bit().constData(), sizeof(App_Common::tinyUSBtable_s::tinyUSBname)-1);
        DiffUpdate();
    }
}


/// cali profiles
void guiWindow::profileBoxes_activated(int index)
{
    switch(sender()->property("type").toInt()) {
    case App_Common::pBoxIRsens:
        App_Common::profilesTable[sender()->property("slot").toInt()].irSensitivity = index;
        break;
    case App_Common::pBoxRunMode:
        App_Common::profilesTable[sender()->property("slot").toInt()].runMode = index;
        break;
    case App_Common::pBoxLayout:
        App_Common::profilesTable[sender()->property("slot").toInt()].layoutType = index;
        break;
    case App_Common::pBoxAR:
        App_Common::profilesTable[sender()->property("slot").toInt()].aspectRatio = index;
        break;
    default:
        break;
    }

    DiffUpdate();
}


void guiWindow::renameBoxes_clicked()
{
    // TODO: limit character length in the text dialog - for now, just use up to 15 characters.
    QString newLabel = QInputDialog::getText(this,
                                             tr("Input Name"),
                                             tr("Set name for Calibration Profile %1").arg(sender()->property("slot").toInt()+1));

    if(!newLabel.isEmpty()) {
        selectedProfile[sender()->property("slot").toInt()]->setText(QString("%1. %2").arg(sender()->property("slot").toInt()+1).arg(newLabel.left(15)));
        memset(App_Common::profilesTable[sender()->property("slot").toInt()].profName, 0, sizeof(App_Common::profilesTable_s::profName));
        strncpy(App_Common::profilesTable[sender()->property("slot").toInt()].profName, newLabel.toLocal8Bit().constData(), sizeof(App_Common::profilesTable_s::profName)-1);
    }

    DiffUpdate();
}


void guiWindow::colorBoxes_clicked()
{
    QColor colorPick = QColorDialog::getColor(App_Common::profilesTable[sender()->property("slot").toInt()].color);
    if(colorPick.isValid()) {
        int red;
        int green;
        int blue;
        colorPick.getRgb(&red, &green, &blue);
        uint32_t packedColor = 0;
        packedColor |= red << 16;
        packedColor |= green << 8;
        packedColor |= blue;
        App_Common::profilesTable[sender()->property("slot").toInt()].color = packedColor;
        color[sender()->property("slot").toInt()]->setStyleSheet(QString("background-color: #%1").arg(packedColor, 6, 16, QLatin1Char('0')));
        DiffUpdate();
    }
}

void guiWindow::selectedProfile_isChecked(bool isChecked)
{
    // Apparently we get two signals at once, so only process the checked one.
    if(isChecked && !serialActive) {
        const int profile = sender()->property("slot").toInt();

        if(profile != App_Common::board.selectedProfile) {
            const char payload[] = {
                (char)OF_Const::sCaliProfile, // Subcommand: select profile only
                (char)profile
            };

            serial.SendCommand(OF_Const::sCaliProfile, payload, sizeof(payload));
        }
    }
}

void guiWindow::caliBtns_clicked()
{
    const int profile = sender()->property("slot").toInt();

    NewCaliWindow(AppCaliWindow::modeCalibrate);

    const char payload[] = {
        (char)OF_Const::sCaliStart,   // Subcommand: start calibration
        (char)profile,
        (char)(App_Common::profilesTable.at(profile).irSensitivity +
                (App_Common::profilesTable.at(profile).layoutType << 4)),
        (char)OF_Const::caliFlagIrView // Optional: IR emitters during calibration (older firmware ignores it)
    };

    if(!serial.SendCommand(OF_Const::sCaliProfile, payload, sizeof(payload)))
        serial.FailOperation();
}

/// gun tests
void guiWindow::on_rumbleTestBtn_clicked()
{
    if(serial.SendCommand(OF_Const::sTestRumble))
        ui->statusBar->showMessage(tr("Sent a rumble test pulse."), 2500);
}


void guiWindow::on_solenoidTestBtn_clicked()
{
    if(serial.SendCommand(OF_Const::sTestSolenoid))
        ui->statusBar->showMessage(tr("Sent a solenoid test pulse."), 2500);
}


void guiWindow::on_redLedTestBtn_clicked()
{
    if(serial.SendCommand(OF_Const::sTestLEDR))
        ui->statusBar->showMessage(tr("Set LED to Red."), 2500);
}


void guiWindow::on_greenLedTestBtn_clicked()
{
    if(serial.SendCommand(OF_Const::sTestLEDG))
        ui->statusBar->showMessage(tr("Set LED to Green."), 2500);
}


void guiWindow::on_blueLedTestBtn_clicked()
{
    if(serial.SendCommand(OF_Const::sTestLEDB))
        ui->statusBar->showMessage(tr("Set LED to Blue."), 2500);
}


void guiWindow::on_testBtn_clicked()
{
    bool startTest = true;

    if(App_Common::profilesTable.at(App_Common::board.selectedProfile).layoutType != App_Common::profilesTable_orig.at(App_Common::board.selectedProfile).layoutType) {
        startTest = QMessageBox::information(
                        this,
                        tr("Warning: Unsaved Changes"),
                        tr("Your current calibration profile has an unsaved IR Layout type change.\n"
                           "Test Mode relies on the profile's settings since the last save, and may not work as expected.\n"
                           "It is recommended that you save your changes before continuing.\n\n"
                           "Continue to Test Mode?"),
                        QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;
    }

    const char enabled = true;
    if(startTest && serial.SendCommand(OF_Const::sIRTest, &enabled, sizeof(enabled)))
        NewCaliWindow(AppCaliWindow::modeIRTest);
}


void guiWindow::on_clearEepromBtn_clicked()
{
    // Do we really need all this msgbox setup?
    QMessageBox messageBox;
    messageBox.setText(tr("Really delete saved data?"));
    messageBox.setInformativeText(tr("This operation will delete all saved data, including:\n\n"
                                     " - Calibration Profiles\n"
                                     " - Toggles\n - Settings\n"
                                     " - Custom Identifiers\n\n"
                                     "Are you sure about this?"));
    messageBox.setWindowTitle(tr("Delete Confirmation"));
    messageBox.setIcon(QMessageBox::Warning);
    messageBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    messageBox.setDefaultButton(QMessageBox::No);

    if(messageBox.exec() == QMessageBox::Yes) {
        ui->statusBar->showMessage(tr("Board reset to initial settings."));
        serial.SendCommand(OF_Const::sClearFlash);
        QThread::msleep(50);
        serial.port.close();
        ui->comPortSelector->setCurrentIndex(0);
    } else ui->statusBar->showMessage(tr("Clear operation canceled."), 3000);
}


void guiWindow::on_baudResetBtn_clicked()
{
    const bool sent = serial.RebootToBootldr();

    /* test stuff for potential app FW update functionality
    // At least on my system, the Bootloader device takes ~7s to appear
    QThread::msleep(7000);
    // Class-ify this function, maybe.
    QString picoPath;
    foreach(const QStorageInfo &storageDevices, QStorageInfo::mountedVolumes()) {
        if(storageDevices.isValid() && storageDevices.isReady() && storageDevices.displayName() == "RPI-RP2") {
            picoPath = storageDevices.device();
            qDebug() << "Found a Pico bootloader!";
            break;
        } else {
            qDebug() << "nope";
        }
    }
    qDebug() << picoPath;
    // QFile::copy("file", picoPath+"file");
*/

    if(!sent) {
        ui->statusBar->showMessage(tr("The restart command could not be sent: reconnect and try again."), 10000);
    } else if (App_Common::board.arch == App_Common::OFPresets.boardArchs[OF_Const::boardRP]) {
        ui->statusBar->showMessage(tr("Board reset to bootloader."), 5000);
    } else {
        ui->statusBar->showMessage(tr("Board restarted."), 5000);
    }
    ui->comPortSelector->setCurrentIndex(0);
}


/// System/background: AppSerial is the only code that reads QSerialPort.
void guiWindow::serialPort_readyRead(const uint8_t command, const QByteArray &payload)
{
    #ifdef OFAPP_DEBUG
    debugWindow.AppendText(payload);
    #endif

    // Transaction responses are consumed by AppSerial itself.  Keep ordinary
    // asynchronous UI traffic quiet while a transaction is in progress, but
    // never hide a validated device error.
    if(serialActive && command != OF_Const::sError)
        return;

    switch(command) {
    case OF_Const::sBtnPressed:
        if(payload.size() == 1) {
            const uint8_t btn = (uint8_t)payload.at(0);
            if(btn < testLabel.count())
                testLabel.at(btn)->setStyleSheet("background-color: #FF0000; font: bold");
        }
        break;

    case OF_Const::sBtnReleased:
        if(payload.size() == 1) {
            const uint8_t btn = (uint8_t)payload.at(0);
            if(btn < testLabel.count())
                testLabel.at(btn)->setStyleSheet("");
        }
        break;

    case OF_Const::sTemperatureUpd:
        if(payload.size() == 1) {
            const unsigned int temp = (uint8_t)payload.at(0);
            if(temp == OF_Const::TEMPERATURE_SENSOR_ERROR_VALUE) {
                ui->tmp36Label->setText(tr("Temperature: FAULT"));
                ui->tmp36Label->setStyleSheet("color: white; background-color: #FF0000; font: bold");
                break;
            }

            ui->tmp36Label->setText(tr("Temperature: %1°C").arg(temp));
            if(temp > App_Common::settingsTable[App_Common::dataOrig][OF_Const::tempShutdown])
                ui->tmp36Label->setStyleSheet("color: white;      background-color: #FF0000; font: bold");
            else if(temp > App_Common::settingsTable[App_Common::dataOrig][OF_Const::tempWarning])
                ui->tmp36Label->setStyleSheet("color: light-gray; background-color: #EABD2B; font: bold");
            else
                ui->tmp36Label->setStyleSheet("color: black;      background-color: #11D00A; font: bold");
        }
        break;

    case OF_Const::sAnalogPosUpd:
        if(payload.size() == 4) {
            uint16_t oriPosX;
            uint16_t oriPosY;
            memcpy(&oriPosX, payload.constData(), sizeof(oriPosX));
            memcpy(&oriPosY, payload.constData() + sizeof(oriPosX), sizeof(oriPosY));
            uint8_t posX = ~(oriPosX / 16);
            uint8_t posY = ~(oriPosY / 16);
            analogPos->setRect(posX - 8, posY - 8, 16, 16);
            ui->aPosLabel->setText(QString("%1 , %2").arg(oriPosX).arg(oriPosY));
        }
        break;

    case OF_Const::sCurrentProf:
        if(payload.size() == 1) {
            const uint8_t selection = (uint8_t)payload.at(0);
            if(selection != App_Common::board.selectedProfile && selection < App_Common::profilesTable.size()) {
                App_Common::board.selectedProfile = selection;
                selectedProfile[selection]->setChecked(true);
            }
            if(caliWindow != nullptr && caliWindow->GetWindowMode() == AppCaliWindow::modeIRTest)
                caliWindow->SetIrTestDiamond(App_Common::profilesTable_orig.at(App_Common::board.selectedProfile).layoutType == OF_Const::layoutDiamond);
            DiffUpdate();
        }
        break;

    case OF_Const::sError:
        if(!payload.isEmpty() &&
           (uint8_t)payload.at(0) == AppSerial::APP_SERIAL_ERR_COMMIT_RETRY) {
            // Queued error delivery must not undo a later successful retry.
            if(!serial.CommitNeedsRetry())
                break;
            DiffUpdate();
            ui->statusBar->showMessage(tr("Save not confirmed. Keep the board powered and retry Save."));
            serial.ShowError(tr("Save not confirmed"),
                tr("Saving failed or its result could not be confirmed. Your edits remain in this App; completed calibration remains in the powered board.\n\n"
                   "Check the connection or the reported storage error, then press Save and Send Settings again. No restart is required for a recoverable save error. Hardware tests stay disabled until a save is confirmed.\n\n"
                   "Disconnecting/reloading can replace unsent App edits with the board's partly updated settings. Restarting loses unsaved board RAM."));
            break;
        }
        if(payload.isEmpty()) {
            if(caliWindow != nullptr)
                caliWindow->Shutdown();
            ui->comPortSelector->setCurrentIndex(0);
            ui->statusBar->showMessage(tr("Operation not confirmed: restart the microcontroller and reconnect."));
            serial.ShowError(tr("Operation not confirmed"),
                             tr("The operation failed or its result could not be confirmed. Restart the microcontroller and reconnect before continuing. If saving had already started, verify the settings and save them again after resolving the error."));
            break;
        }
        if(payload.size() < 1)
            break;
        switch((uint8_t)payload.at(0)) {
        case OF_Const::sErrCam:
            if(caliWindow != nullptr)
                caliWindow->Shutdown();
            serial.ShowError(tr("Device Error: Camera not available!"),
                             tr("<p>Data received from the board indicates that the camera is in a bad state.<br>"
                                "This can happen if, for example, the camera wires are crossed<br>"
                                "(data wire to clock pin, clock wire to data pin),<br>"
                                "or the camera pins are wired to a different component,<br>"
                                "such as a button or Force Feedback output.</p>"
                                "<p>You are able to change the camera pins in the <i>Boards Layout</i> tab<br>"
                                "if they should be mapped different GPIO;<br>"
                                "Otherwise, the camera wires must be resoldered to resolve this error.</p>"
                                "<p>IR Testing and Calibration will not be available while in this state.</p>"),
                             QMessageBox::Critical);
            break;
        case OF_Const::sErrPeriphGeneric:
            serial.ShowError(tr("Peripheral Device Error!"),
                             tr("<p>Data received from the board indicates that an I2C peripheral device failed to initialize.<br>"
                                "This can happen if, for example, the peripheral's wires are crossed<br>"
                                "(data wire to clock pin, clock wire to data pin),<br>"
                                "or the pins for the peripheral are set to a different component,<br>"
                                "such as a button or Force Feedback output.</p>"
                                "<p>Confirm that the wires for the peripheral are connected to the correct <i>Peripheral I2C</i> pins<br>"
                                "in the <i>Boards Layout</i> tab.</p>"),
                             QMessageBox::Critical);
            break;
        default:
            break;
        }
        break;

    case OF_Const::sCaliStageUpd:
        if(payload.size() == 1 && caliWindow != nullptr &&
           caliWindow->GetWindowMode() == AppCaliWindow::modeCalibrate)
            caliWindow->CaliModeSet((uint8_t)payload.at(0));
        break;

    case OF_Const::sCaliInfoUpd:
        if(payload.size() == 5 && caliWindow != nullptr &&
           caliWindow->GetWindowMode() == AppCaliWindow::modeCalibrate)
            caliWindow->CaliModeTextUpdate((uint8_t)payload.at(0), payload.constData() + 1);
        break;

    case OF_Const::sTestBlobs:
        // IR test, and calibration with the IR view (caliFlagIrView)
        if(caliWindow != nullptr && caliWindow->GetWindowMode() != AppCaliWindow::modeAlignment)
            caliWindow->TestModeBlobs((const uint8_t*)payload.constData(), (int)payload.size());
        break;

    case OF_Const::sCaliIrWarning:
        if(payload.size() == 1 && caliWindow != nullptr &&
           caliWindow->GetWindowMode() == AppCaliWindow::modeCalibrate)
            caliWindow->CaliIrWarning((uint8_t)payload.at(0));
        break;

    case OF_Const::sTestCoords:
        if(payload.size() == (int)(sizeof(int) * 12) && caliWindow != nullptr) {
            int coordsList[12];
            memcpy(coordsList, payload.constData(), sizeof(coordsList));
            caliWindow->TestModeDraw(coordsList);
        }
        break;

    case OF_Const::sClearFlash:
        ui->comPortSelector->setCurrentIndex(0);
        QMessageBox::information(this,
                                 tr("Successfully reset board settings"),
                                 tr("Please unplug the board and reinsert it into the PC."));
        break;
    }
}


void guiWindow::serialPort_SearchFinished()
{

    serialSearchPending = false;

    // if ports have changed
    if(serialSearchWatcher.result()) {

        const QList<QSerialPortInfo> ports =
            serial.CurrentPortsSnapshot();

        // ports have changed, update COM ports list
        // if comPort only has "Nothing", safe to add items
        if(ui->comPortSelector->count() == 0) {
            if(ports.count()) {
                ui->comPortSelector->addItem(tr("[Select a Device to Configure]"));
                ui->comPortSelector->setCurrentIndex(0);
                for(auto &port : std::as_const(ports))
                    ui->comPortSelector->addItem(port.portName()+" (" + port.description() + ')');
            }
        // if comPort is filled
        // TODO: find some way to add new items without removing
        } else {
            if(ports.count()) {
                // if no active comPort
                if(ui->comPortSelector->currentIndex() <= 0) {
                    while(ui->comPortSelector->count() > 1)
                        ui->comPortSelector->removeItem(1);

                    for(const auto &newPort : std::as_const(ports))
                        ui->comPortSelector->addItem(newPort.portName()+" (" + newPort.description() + ')');
                // if comPort is active
                } else {
                    // remove all other comPorts
                    int i = 1;
                    while(ui->comPortSelector->count() > 2) {
                        if(i == ui->comPortSelector->currentIndex())
                            ++i;
                        else ui->comPortSelector->removeItem(i);
                    }

                    // check if current comPort is still in devices list
                    // TODO: probably a better way of doing this, meh
                    bool inList = false;
                    for(const auto &newPort : std::as_const(ports))
                        if(ui->comPortSelector->currentText() == newPort.portName()+" (" + newPort.description() + ')')
                            inList = true;
                    if(!inList) {
                        statusBar()->showMessage(tr("Current board has been disconnected."));
                        ui->comPortSelector->removeItem(1);
                    }

                    // append new items to list
                    for(const auto &newPort : std::as_const(ports))
                        if(ui->comPortSelector->currentText() != newPort.portName()+" (" + newPort.description() + ')')
                            ui->comPortSelector->addItem(newPort.portName()+" (" + newPort.description() + ')');
                }
            // if ports list is cleared, assume no board can be connected.
            // TODO: for whatever reason, this path specifically doesn't kick in under Windows VM?
            } else {
                if(ui->comPortSelector->currentIndex() > 0)
                    statusBar()->showMessage(tr("Current board has been disconnected."));
                ui->comPortSelector->clear();
            }
        }
    }
}


void guiWindow::serialPort_progressSet(const int &range)
{
    if(range) {
        if(!statusProgressBar->isVisible())
            statusProgressBar->setVisible(true);
        statusProgressBar->setRange(0, range);
    } else {
        statusProgressBar->setVisible(false);
        statusProgressBar->setValue(0);
    }
}


void guiWindow::serialPort_progressUpdate(const int &pos, const QString& statusText)
{
    if(statusProgressBar->isVisible())
        statusProgressBar->setValue(pos);

    if(statusText != nullptr)
        ui->statusBar->showMessage(statusText, 5000);
}





void guiWindow::CaliWindowExiting(const int &mode,
                                  const int &topOffsetNew,
                                  const int &bottomOffsetNew,
                                  const int &leftOffsetNew,
                                  const int &rightOffsetNew,
                                  const float &topLeftLedNew,
                                  const float &topRightLedNew)
{
    switch(mode) {
    case AppCaliWindow::modeCalibrate:
    {
        uint8_t selection = caliWindow->property("profile").toInt();

        if(!(topOffsetNew  == -1 && bottomOffsetNew == -1 &&
             leftOffsetNew == -1 && rightOffsetNew  == -1 &&
             topLeftLedNew == -1 && topRightLedNew  == -1)) {
                App_Common::profilesTable[selection].topOffset = topOffsetNew;
                topOffset[selection]->setText(QString::number(topOffsetNew));

                App_Common::profilesTable[selection].bottomOffset = bottomOffsetNew;
                bottomOffset[selection]->setText(QString::number(bottomOffsetNew));

                App_Common::profilesTable[selection].leftOffset = leftOffsetNew;
                leftOffset[selection]->setText(QString::number(leftOffsetNew));

                App_Common::profilesTable[selection].rightOffset = rightOffsetNew;
                rightOffset[selection]->setText(QString::number(rightOffsetNew));

                App_Common::profilesTable[selection].TLled = topLeftLedNew;
                TLled[selection]->setText(QString::number(topLeftLedNew));

                App_Common::profilesTable[selection].TRled = topRightLedNew;
                TRled[selection]->setText(QString::number(topRightLedNew));

                if(topOffsetNew    >= -32768 && topOffsetNew    <= 32768 &&
                   bottomOffsetNew >= -32768 && bottomOffsetNew <= 32768 &&
                   leftOffsetNew   >= -32768 && leftOffsetNew   <= 32768 &&
                   rightOffsetNew  >= -32768 && rightOffsetNew  <= 32768 &&
                   topLeftLedNew   >= -32768 && topLeftLedNew   <= 32768 &&
                   topRightLedNew  >= -32768 && topRightLedNew  <= 32768)
                    ui->statusBar->showMessage(tr("Calibration for Profile ") + QString::number(selection+1) + tr(" successful"), 5000);
                else ui->statusBar->showMessage(tr("Calibration for Profile ") + QString::number(selection+1) + tr(" returned with malformed values."), 10000);

                DiffUpdate();
        } else ui->statusBar->showMessage(tr("Cancelled Calibration for Profile ") + QString::number(selection+1), 10000);
        break;
    }
    case AppCaliWindow::modeIRTest:
    {
        const char enabled = false;
        if(!serial.port.isOpen() ||
           serial.SendCommand(OF_Const::sIRTest, &enabled, sizeof(enabled))) {
            ui->buttonsTestArea->setEnabled(true);
            ui->pinsTab->setEnabled(true);
            ui->settingsTab->setEnabled(true);
            ui->profilesTab->setEnabled(true);
            ui->feedbackTestsBox->setEnabled(true);
            ui->dangerZoneBox->setEnabled(true);
        }
        break;
    }
    case AppCaliWindow::modeAlignment:
    default:
        break;
    }

    // for some reason, caliWindows can leave a lingering pointer???
    // so make sure it's deleted.
    caliWindow->close();
    if(caliWindow != nullptr)
        caliWindow = nullptr;
}


void guiWindow::CaliWindowRequestedExit()
{
    if(!serial.SendCommand(OF_Const::serialTerminator))
        serial.FailOperation();
}


void guiWindow::on_tabWidget_currentChanged(int index)
{
    // TODO: use enums
    // ...buuut those have to be manually changed in the ui file. bleh.
    switch(index) {
    // button mappings tab
    case 1:
        ui->btnFuncDescBox->setTitle("");
        ui->btnFuncDescText->setText(ui->btnFuncDescText->whatsThis());
        break;
    // settings tab
    case 2:
        ui->settingsDescBox->setTitle("");
        ui->settingsDescText->setText(ui->settingsDescText->whatsThis());
        break;
    // profiles tab
    case 3:
        ui->profilesDescBox->setTitle("");
        ui->profilesDescText->setText(ui->profilesDescText->whatsThis());
        break;
    // test tab (no use yet)
    case App_Common::trackTestItem:
    // pins tab (doesn't have any)
    case App_Common::trackPinbox:
    default:
        break;
    }
}


/// actions
void guiWindow::on_actionShow_Unsafe_Settings_toggled(bool arg1)
{
    if(arg1) ui->solenoidTempBox->setVisible(true);
    else     ui->solenoidTempBox->setVisible(false);
}


void guiWindow::on_actionCompatible_Boards_triggered()
{
    boardsWindow.show();
}


void guiWindow::on_actionAbout_UI_triggered()
{
    AppAbout *about = new AppAbout();
    about->show();
}


void guiWindow::on_actionOpenFIRE_Documentation_triggered()
{
    // the manual of this project, in the App's language (the Italian link is in the translation), as in the WebApp
    QDesktopServices::openUrl(QUrl(tr("https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32/blob/main/lightgun/src/README.md#english-version")));
}


void guiWindow::on_actionOpenFIRE_Serial_Usage_triggered()
{
    // the serial commands page of this project's wiki, in the App's language, as in the WebApp
    QDesktopServices::openUrl(QUrl(tr("https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32/wiki/Serial_Commands_OpenFIRE_EN")));
}


void guiWindow::on_actionOpen_IR_Emitter_Alignment_Assistant_triggered()
{
    NewCaliWindow(AppCaliWindow::modeAlignment);
}


// Custom layout file (.ofl): board type + '\n', then records of (name, NUL, one byte):
// - (function name, NUL, GPIO) for every mapped function;
// - ("NC:" + function name, NUL, 0) for every inverted input;
// - ("Checksum", NUL, CRC-8 of every byte before this record), then ("End", NUL, 0) as the last bytes.
// Readers skip the records they don't know, so later versions can add records without changing
// this layout. Same format as the web app (lightgun/webapp/js/app/state.js in the firmware).
static constexpr char OFL_NC_PREFIX[] = "NC:";
static constexpr char OFL_CHECKSUM[]  = "Checksum";
static constexpr char OFL_END[]       = "End";

// CRC-8 (polynomial 0x07, initial value 0) of the .ofl bytes before the checksum record.
static uint8_t OflCrc8(const QByteArray &bytes)
{
    uint8_t crc = 0;
    for(const char byte : bytes) {
        crc ^= static_cast<uint8_t>(byte);
        for(int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}


void guiWindow::on_actionImport_Custom_Layout_triggered()
{
    QString path = QFileDialog::getOpenFileName(this,
                                                tr("Open New Layout"),
                                                QDir::homePath(),
                                                "OpenFIRE Layout Files (*.ofl)");

    if(!path.isEmpty()) {
        QFile fileIn(path);
        if(fileIn.open(QFile::ReadOnly)) {
            const QByteArray bytes = fileIn.readAll();
            fileIn.close();

            const qsizetype newline = bytes.indexOf('\n');
            if(newline < 0 || bytes.left(newline).trimmed() != App_Common::board.type) {
                QMessageBox::warning(this, tr("Board Doesn't Match"),
                                           tr("Custom layout file is not compatible with this board."));
                return;
            }

            // read the whole file first: an incomplete or damaged file changes nothing
            QVector<int> layout(pinBoxes.count(), OF_Const::btnUnmapped);
            QVector<int> inverted;
            bool valid = false;
            qsizetype checked = -1;   // offset just after a valid checksum record
            qsizetype offset = newline + 1;
            while(offset < bytes.size()) {
                const qsizetype start = offset;
                const qsizetype end = bytes.indexOf('\0', offset);
                if(end < 0 || end + 1 >= bytes.size())
                    break;
                const QByteArray name = bytes.mid(offset, end - offset);
                const uint8_t value = static_cast<uint8_t>(bytes.at(end + 1));
                offset = end + 2;

                if(name == OFL_END) {
                    // the end record must follow the checksum record and be the last bytes of the file
                    valid = (start == checked && offset == bytes.size() && value == 0);
                    break;
                }
                if(name == OFL_CHECKSUM) {
                    if(value != OflCrc8(bytes.left(start)))
                        break;
                    checked = offset;
                    continue;
                }
                if(name.startsWith(OFL_NC_PREFIX)) {
                    const QByteArray func = name.mid(sizeof(OFL_NC_PREFIX)-1);
                    const auto found = App_Common::OFPresets.boardInputs_Strings.find(std::string_view(func.constData(), func.size()));
                    if(found != App_Common::OFPresets.boardInputs_Strings.cend())
                        inverted << found->second;
                    continue;
                }
                const auto found = App_Common::OFPresets.boardInputs_Strings.find(std::string_view(name.constData(), name.size()));
                if(found != App_Common::OFPresets.boardInputs_Strings.cend() && value < layout.size())
                    layout[value] = found->second;
            }

            if(!valid) {
                QMessageBox::warning(this, tr("File Read Error"),
                                           tr("Custom layout file could not be read."));
                return;
            }

            const QScopedValueRollback<int> bulk(pinBoxesDepth, pinBoxesDepth+1);
            if(!ui->customPinsEnabled->isChecked()) ui->customPinsEnabled->setChecked(true);
            // clear current mapping
            for(const auto &box : std::as_const(pinBoxes))
                box->setCurrentIndex(OF_Const::btnUnmapped+1);

            // import new maps
            for(int i = 0; i < layout.size(); ++i)
                if(layout.at(i) > OF_Const::btnUnmapped)
                    pinBoxes.at(i)->setCurrentIndex(layout.at(i)+1);

            FeedbackOffWithoutPins();

            // the inversions come from the file only: none in the file = none inverted
            // (and only for the functions that have a pin)
            for(const auto &input : OF_Const::invertibleInputs)
                SetInverted(input, inverted.contains(input) && App_Common::inputsMap.value(input) > OF_Const::btnUnmapped);

            // recover previous toggles that may have been turned off in the process
            if(App_Common::inputsMap.value(OF_Const::solenoidPin) >= 0)
                ui->solenoidToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::solenoid]);
            if(App_Common::inputsMap.value(OF_Const::rumblePin) >= 0)
                ui->rumbleToggle->setChecked(App_Common::boolSettings[App_Common::dataOrig][OF_Const::rumble]);

            DiffUpdate();
            ui->statusBar->showMessage(tr("Successfully imported custom layout!"), 5000);
        } else QMessageBox::warning(this, tr("File Read Error"),
                                          tr("Custom layout file could not be read."));
    } else ui->statusBar->showMessage(tr("Canceled custom layout load operation."), 5000);
}


void guiWindow::on_actionExport_Custom_Layout_triggered()
{
    QString path = QFileDialog::getSaveFileName(this,
                                                tr("Save New Layout"),
                                                QDir::homePath(),
                                                "OpenFIRE Layout Files (*.ofl)");

    if(!path.isEmpty()) {
        QFile fileOut(path);
        if(fileOut.open(QFile::WriteOnly)) {
            QByteArray bytes = App_Common::board.type + '\n';
            QVector<std::string_view> inverted;

            // in function order, like the WebApp
            for(int input = 0; input < OF_Const::boardInputsCount; ++input) {
                if(App_Common::inputsMap.value(input) > OF_Const::btnUnmapped) {
                    const std::string_view name(App_Common::OFPresets.boardInputs_sortedStr[input+1]);
                    bytes.append(name.data(), name.length());
                    bytes.append('\0');
                    bytes.append(static_cast<char>(App_Common::inputsMap.value(input)));
                    if(Invertible(input) && Inverted(input))
                        inverted << name;
                }
            }

            for(const auto &name : std::as_const(inverted)) {
                bytes.append(OFL_NC_PREFIX);
                bytes.append(name.data(), name.length());
                bytes.append('\0');
                bytes.append('\0');
            }

            const char crc = static_cast<char>(OflCrc8(bytes));
            bytes.append(OFL_CHECKSUM);
            bytes.append('\0');
            bytes.append(crc);
            bytes.append(OFL_END);
            bytes.append('\0');
            bytes.append('\0');

            const bool written = (fileOut.write(bytes) == bytes.size());
            fileOut.close();
            if(written)
                ui->statusBar->showMessage(tr("Custom layout export successful!"), 5000);
            else QMessageBox::warning(this, tr("File Write Error"),
                                            tr("Custom layout file could not be written."));
        } else QMessageBox::warning(this, tr("File Write Error"),
                                          tr("Custom layout file could not be written."));
    } else ui->statusBar->showMessage(tr("Canceled custom layout save operation."), 5000);
}


void guiWindow::on_actionDebug_Window_triggered()
{
    #ifdef OFAPP_DEBUG
    debugWindow.show();
    #endif
}
