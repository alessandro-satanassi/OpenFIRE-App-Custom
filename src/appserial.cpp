/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    Serial input/output routines.

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

#include "appserial.h"
#include "appcommon.h"
#include "../boards/OpenFIREshared.h"
#include <QDebug>
#include <QMessageBox>
#include <QMutexLocker>
#include <cstring>
#include <utility>

static QString AppSerialHex(const uint8_t value)
{
    return QStringLiteral("0x%1").arg(value, 2, 16, QLatin1Char('0')).toUpper();
}

static QString AppSerialFieldName(const std::string_view value)
{
    return QString::fromUtf8(value.data(), (int)value.length());
}

AppSerial::AppSerial(QObject *parent)
    : QObject(parent), port(this)
{
    // Keep asynchronous events, but queue the slot so it cannot drain the
    // QSerialPort buffer from inside waitForReadyRead().  The synchronous
    // transaction loops below then wake immediately, read the bytes themselves
    // and never pay a full timeout for a frame that was already received.
    connect(&port, &QSerialPort::readyRead,
            this, &AppSerial::ProcessIncoming,
             Qt::QueuedConnection);
}

void AppSerial::ShowError(const QString &titleText, const QString &text,
                          QMessageBox::Icon icon)
{
    if(syncError.isVisible())
        return;

    syncError.setWindowTitle(titleText);
    syncError.setText(text);
    syncError.setIcon(icon);
    syncError.show();
}

// ===== Frame encoding and incoming stream ====================================

uint8_t AppSerial::AppSerialCRC8(const uint8_t *data, int length)
{
    uint8_t crc = 0;

    while(length-- > 0) {
        crc ^= *data++;
        for(uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x9B) : (uint8_t)(crc << 1);
    }

    return crc;
}

uint8_t AppSerial::NextSequence()
{
    ++appSerialTxSequence;
    if(appSerialTxSequence == 0)
        appSerialTxSequence = 1;

    return appSerialTxSequence;
}

void AppSerial::ResetProtocolState()
{
    appSerialRxBuffer.clear();
    appSerialResponses.clear();
    appSerialRxTimer.invalidate();
    appSerialTxSequence = 0;
    appSerialLastRequest = 0;
    appSerialAckReceived = false;
    appSerialLastRxValid = false;

    appSerialSessionActive = false;
    appSerialCommitInProgress = false;
    appSerialCommitNeedsRetry = false;
    appSerialCommitError = false;
    appSerialLastStartSequence = 0;
    appSerialLastSaveSequence = 0;
}

QByteArray AppSerial::BuildFrame(uint8_t typeFlags,
                                 uint8_t command,
                                 uint8_t sequence,
                                 const char *payload,
                                 uint8_t length)
{
    if(length > APP_SERIAL_MAX_PAYLOAD || (length > 0 && payload == nullptr)) {
        qWarning().noquote() << "[AppSerial] Cannot build frame: invalid payload for command"
                             << AppSerialHex(command) << "length" << (unsigned int)length;
        return QByteArray();
    }

    QByteArray frame;
    frame.resize(APP_SERIAL_OVERHEAD + length);
    frame[0] = (char)APP_SERIAL_START_1;
    frame[1] = (char)APP_SERIAL_START_2;
    frame[2] = (char)typeFlags;
    frame[3] = (char)command;
    frame[4] = (char)sequence;
    frame[5] = (char)length;

    if(length > 0)
        memcpy(frame.data() + 6, payload, length);

    frame[6 + length] = (char)AppSerialCRC8((const uint8_t*)frame.constData() + 2, 4 + length);
    return frame;
}

bool AppSerial::SendAck(uint8_t command, uint8_t sequence)
{
    const QByteArray frame = BuildFrame(APP_SERIAL_TYPE_ACK, command, sequence, nullptr, 0);
    if(frame.isEmpty() || !port.isOpen()) {
        qWarning().noquote() << "[AppSerial] Cannot send ACK for command"
                             << AppSerialHex(command) << "sequence" << (unsigned int)sequence
                             << "because the serial port is not available.";
        return false;
    }

    const qint64 written = port.write(frame);
    if(written != frame.size()) {
        qWarning().noquote() << "[AppSerial] Incomplete ACK write for command"
                             << AppSerialHex(command) << "sequence" << (unsigned int)sequence
                             << "written" << written << "of" << frame.size() << "bytes.";
        return false;
    }

    // QSerialPort::flush() is non-blocking and may leave the ACK queued in the
    // App while the firmware is already waiting for it.  Make the tiny ACK
    // leave Qt immediately; this keeps the stop-and-wait protocol fast without
    // dropping per-frame acknowledgement or retry protection.
    if(port.bytesToWrite() > 0 &&
       !port.waitForBytesWritten(APP_SERIAL_ACK_TIMEOUT) &&
       port.bytesToWrite() > 0) {
        qWarning().noquote() << "[AppSerial] ACK write timeout for command"
                             << AppSerialHex(command) << "sequence" << (unsigned int)sequence << '.';
        return false;
    }

    return true;
}

void AppSerial::ProcessIncoming()
{

    // A readyRead notification may already be queued when the port is closed.
    if(!port.isOpen())
        return;

    if(!appSerialRxBuffer.isEmpty() && appSerialRxTimer.isValid() &&
       appSerialRxTimer.elapsed() > APP_SERIAL_FRAME_TIMEOUT) {
        qWarning().noquote() << "[AppSerial] Partial frame timed out; discarded"
                             << appSerialRxBuffer.size() << "byte(s).";
        appSerialRxBuffer.clear();
    }

    const QByteArray incoming = port.readAll();
    if(!incoming.isEmpty()) {
        appSerialRxBuffer.append(incoming);
        appSerialRxTimer.restart();
    }

    ParseIncoming();
}

void AppSerial::ParseIncoming()
{
    for(;;) {
        if(appSerialRxBuffer.size() < 2)
            return;

        int start = -1;
        for(int i = 0; i + 1 < appSerialRxBuffer.size(); ++i) {
            if((uint8_t)appSerialRxBuffer.at(i) == APP_SERIAL_START_1 &&
               (uint8_t)appSerialRxBuffer.at(i + 1) == APP_SERIAL_START_2) {
                start = i;
                break;
            }
        }

        if(start < 0) {
            #ifdef OFAPP_DEBUG
            qDebug().noquote() << "[AppSerial] Discarding"
                               << appSerialRxBuffer.size()
                               << "byte(s) without a complete frame start marker.";
            #endif
            if((uint8_t)appSerialRxBuffer.at(appSerialRxBuffer.size() - 1) == APP_SERIAL_START_1)
                appSerialRxBuffer = QByteArray(1, (char)APP_SERIAL_START_1);
            else appSerialRxBuffer.clear();
            return;
        }

        if(start > 0) {
            #ifdef OFAPP_DEBUG
            qDebug().noquote() << "[AppSerial] Resynchronizing after"
                               << start << "unexpected byte(s).";
            #endif
            appSerialRxBuffer.remove(0, start);
        }

        if(appSerialRxBuffer.size() < 6)
            return;

        const uint8_t typeFlags = (uint8_t)appSerialRxBuffer.at(2);
        const uint8_t type = typeFlags & APP_SERIAL_TYPE_MASK;
        const uint8_t command = (uint8_t)appSerialRxBuffer.at(3);
        const uint8_t sequence = (uint8_t)appSerialRxBuffer.at(4);
        const uint8_t length = (uint8_t)appSerialRxBuffer.at(5);
        const bool invalidFlags = (typeFlags & (uint8_t)~(APP_SERIAL_TYPE_MASK | APP_SERIAL_FLAG_FINAL)) != 0;
        const bool invalidFinal = (typeFlags & APP_SERIAL_FLAG_FINAL) &&
                                  (type != APP_SERIAL_TYPE_RESPONSE || length != 0);
        const bool invalidSequence = type == APP_SERIAL_TYPE_EVENT ? sequence != 0 : sequence == 0;
        const bool invalidAck = type == APP_SERIAL_TYPE_ACK && length != 0;

        if(invalidFlags || invalidFinal || invalidSequence || invalidAck || length > APP_SERIAL_MAX_PAYLOAD) {
            qWarning().noquote() << "[AppSerial] Invalid frame header: type/flags"
                                 << AppSerialHex(typeFlags) << "command"
                                 << AppSerialHex(command) << "sequence" << (unsigned int)sequence
                                 << "length" << (unsigned int)length << "; resynchronizing.";
            appSerialRxBuffer.remove(0, 1);
            continue;
        }

        const int frameLength = APP_SERIAL_OVERHEAD + length;
        if(appSerialRxBuffer.size() < frameLength)
            return;

        const uint8_t receivedCRC = (uint8_t)appSerialRxBuffer.at(6 + length);
        const uint8_t calculatedCRC = AppSerialCRC8((const uint8_t*)appSerialRxBuffer.constData() + 2,
                                                    4 + length);
        if(receivedCRC != calculatedCRC) {
            qWarning().noquote() << "[AppSerial] CRC mismatch for command"
                                 << AppSerialHex(command) << "sequence" << (unsigned int)sequence
                                 << "received" << AppSerialHex(receivedCRC)
                                 << "expected" << AppSerialHex(calculatedCRC)
                                 << "; frame discarded.";
            appSerialRxBuffer.remove(0, 1);
            continue;
        }

        AppSerialFrame_s frame;
        frame.typeFlags = typeFlags;
        frame.command = command;
        frame.sequence = sequence;
        frame.payload = appSerialRxBuffer.mid(6, length);
        frame.crc = receivedCRC;
        appSerialRxBuffer.remove(0, frameLength);

        HandleFrame(frame);
    }
}

// ===== Response routing and recovery ========================================

bool AppSerial::IsTransactionResponse(uint8_t command)
{
    switch(command) {
    case OF_Const::sDock2:
    case OF_Const::sGetPins:
    case OF_Const::sGetToggles:
    case OF_Const::sGetSettings:
    case OF_Const::sGetProfile:
    case OF_Const::sGetBtns:
    case OF_Const::sCommitStart:
    case OF_Const::sSave:
    case OF_Const::serialTerminator:
        return true;
    default:
        return false;
    }
}

void AppSerial::HandleFrame(const AppSerialFrame_s &frame)
{
    const uint8_t type = frame.typeFlags & APP_SERIAL_TYPE_MASK;
    bool deliverEvent = type == APP_SERIAL_TYPE_EVENT;

    if(type == APP_SERIAL_TYPE_ACK) {
        appSerialAckCommand = frame.command;
        appSerialAckSequence = frame.sequence;
        appSerialAckReceived = true;
        return;
    }

    // Reliable responses are ACKed even when duplicate or stale. Only an
    // accepted response reaches the transaction queue or the common event path.
    if(type == APP_SERIAL_TYPE_RESPONSE) {
        const uint8_t length = (uint8_t)frame.payload.size();
        const bool duplicate =
            appSerialLastRxValid &&
            appSerialLastRxTypeFlags == frame.typeFlags &&
            appSerialLastRxCommand == frame.command &&
            appSerialLastRxSequence == frame.sequence &&
            appSerialLastRxLength == length &&
            appSerialLastRxCRC == frame.crc;

        if(!duplicate) {
            appSerialLastRxValid = true;
            appSerialLastRxTypeFlags = frame.typeFlags;
            appSerialLastRxCommand = frame.command;
            appSerialLastRxSequence = frame.sequence;
            appSerialLastRxLength = length;
            appSerialLastRxCRC = frame.crc;
        }

        SendAck(frame.command, frame.sequence);

        if(duplicate)
            return;

        // Commit results are tied to the request, not merely its command.
        if(frame.command == OF_Const::sError &&
           frame.payload.size() == 2 &&
           (uint8_t)frame.payload.at(0) == APP_SERIAL_ERR_COMMIT_RETRY) {
            if((uint8_t)frame.payload.at(1) != appSerialLastRequest ||
               frame.sequence != appSerialTxSequence)
                return; // ACKed, but belongs to an older attempt.
            appSerialCommitNeedsRetry = true;
            appSerialCommitError = appSerialCommitInProgress;
            if(!appSerialCommitInProgress)
                emit Serial_EventReceived(frame.command, frame.payload);
            return;
        }

        const bool transactionResponse =
            IsTransactionResponse(frame.command);

        const bool expectedResponse =
            transactionResponse &&
            (frame.command == appSerialLastRequest ||
             (frame.command == OF_Const::sDock2 &&
              appSerialLastRequest == 0)) &&
            ((frame.command != OF_Const::sCommitStart &&
              frame.command != OF_Const::sSave) ||
             frame.sequence == appSerialTxSequence);

        if(expectedResponse) {
            if(frame.command == OF_Const::sDock2)
                appSerialSessionActive = true;

            appSerialResponses.append(frame);
        } else if(transactionResponse) {
#ifdef OFAPP_DEBUG
            qDebug().noquote()
                << "[AppSerial] Ignoring stale transaction response"
                << AppSerialHex(frame.command)
                << "while waiting for"
                << AppSerialHex(appSerialLastRequest) << '.';
#endif
        } else
            deliverEvent = true;
    }

    // Both wire events and non-transaction responses use this GUI delivery path.
    if(!deliverEvent)
        return;

    if(frame.command == OF_Const::sError) {
        qWarning().noquote()
            << (type == APP_SERIAL_TYPE_EVENT ?
                "[AppSerial] Device reported an asynchronous error; payload:" :
                "[AppSerial] Device reported an error; payload:")
            << frame.payload.toHex(' ');
    }

    if(frame.command != OF_Const::sError || !frame.payload.isEmpty()) {
        emit Serial_EventReceived(frame.command, frame.payload);
        return;
    }

    if(appSerialCommitInProgress)
        appSerialCommitError = true;
    else if(appSerialCommitNeedsRetry)
        emit Serial_EventReceived(OF_Const::sError,
                                  QByteArray(1, (char)APP_SERIAL_ERR_COMMIT_RETRY));
    else
        FailOperation();
}

// Commit failure is recoverable while both peers retain their RAM.
bool AppSerial::CommitFailed()
{
    appSerialCommitInProgress = false;
    appSerialCommitNeedsRetry = true;
    qWarning().noquote() << "[AppSerial] Save failed or was not confirmed. RAM is retained; retry Save without restarting the board.";
    emit Serial_EventReceived(OF_Const::sError,
                             QByteArray(1, (char)APP_SERIAL_ERR_COMMIT_RETRY));
    return false;
}

// Fatal errors outside a commit retain the existing restart policy.
bool AppSerial::FailOperation()
{
    const bool notifyError = port.isOpen() || !port.portName().isEmpty();
    if(port.isOpen())
        port.close();
    ResetProtocolState();
    port.setPortName("");

    if(notifyError) {
        qWarning().noquote() << "[AppSerial] Operation failed or not confirmed. Restart the board and reconnect before continuing.";
        // Reuse the existing generic error; the GUI clears its provisional data.
        emit Serial_EventReceived(OF_Const::sError, QByteArray());
    }
    return false;
}

// ===== Reliable command delivery and response waits ==========================

bool AppSerial::WaitForAck(uint8_t command, uint8_t sequence)
{
    QElapsedTimer timer;
    timer.start();

    for(;;) {
        ProcessIncoming();
        if(!port.isOpen() || (appSerialCommitInProgress && appSerialCommitError))
            return false;
        if(appSerialAckReceived && appSerialAckCommand == command && appSerialAckSequence == sequence) {
            appSerialAckReceived = false;
            return true;
        }

        // A valid response can exist only if the firmware has already received
        // and executed this request. It therefore also proves request delivery
        // when the small request ACK was lost.
        for(const auto &response : std::as_const(appSerialResponses)) {
            if(response.command == command)
                return true;
        }

        const int remaining = APP_SERIAL_ACK_TIMEOUT - (int)timer.elapsed();
        if(remaining <= 0)
            break;

        // ProcessIncoming() is queued while this call is blocked, so the port
        // buffer remains intact and waitForReadyRead() wakes without the old
        // full-timeout race. Loop back to parse and validate the actual ACK.
        port.waitForReadyRead(remaining);
    }

    return false;
}

bool AppSerial::SendCommand(uint8_t command, const char *payload, size_t length)
{
    if(length > APP_SERIAL_MAX_PAYLOAD || (length > 0 && payload == nullptr)) {
        qWarning().noquote() << "[AppSerial] SendCommand rejected invalid payload for command"
                             << AppSerialHex(command) << "length" << (qulonglong)length << '.';
        return false;
    }

    if(!port.isOpen()) {
        qWarning().noquote() << "[AppSerial] Cannot send command"
                             << AppSerialHex(command) << ": serial port is closed.";
        return false;
    }

    uint8_t sequence = NextSequence();
    // Even if record traffic wraps the byte counter, do not reuse the last
    // start/save ID for the very next attempt of the same command.
    if(command == OF_Const::sCommitStart) {
        if(sequence == appSerialLastStartSequence)
            sequence = NextSequence();
        appSerialLastStartSequence = sequence;
    } else if(command == OF_Const::sSave) {
        if(sequence == appSerialLastSaveSequence)
            sequence = NextSequence();
        appSerialLastSaveSequence = sequence;
    }
    const QByteArray frame = BuildFrame(APP_SERIAL_TYPE_REQUEST, command, sequence, payload, (uint8_t)length);
    if(frame.isEmpty()) {
        qWarning().noquote() << "[AppSerial] Frame creation failed for command"
                             << AppSerialHex(command) << "sequence" << (unsigned int)sequence << '.';
        return false;
    }

    for(int i = appSerialResponses.size() - 1; i >= 0; --i) {
        if(appSerialResponses.at(i).command == command)
            appSerialResponses.removeAt(i);
    }

    appSerialLastRequest = command;

    for(uint8_t attempt = 0; attempt <= APP_SERIAL_MAX_RETRIES; ++attempt) {
        if(appSerialCommitInProgress && appSerialCommitError)
            return false;
        appSerialAckReceived = false;
        const qint64 written = port.write(frame);
        if(written != frame.size()) {
            qWarning().noquote() << "[AppSerial] Incomplete frame write for command"
                                 << AppSerialHex(command) << "sequence" << (unsigned int)sequence
                                 << "written" << written << "of" << frame.size() << "bytes.";
            return false;
        }

        if(port.bytesToWrite() > 0 && !port.waitForBytesWritten(APP_SERIAL_ACK_TIMEOUT) && port.bytesToWrite() > 0) {
            qWarning().noquote() << "[AppSerial] Serial write timeout for command"
                                 << AppSerialHex(command) << "sequence" << (unsigned int)sequence
                                 << "attempt" << (attempt + 1) << "of"
                                 << (APP_SERIAL_MAX_RETRIES + 1) << "; retrying.";
            continue;
        }

        if(WaitForAck(command, sequence))
            return true;
        if(!port.isOpen() || (appSerialCommitInProgress && appSerialCommitError))
            return false;

        qWarning().noquote() << "[AppSerial] ACK timeout for command"
                             << AppSerialHex(command) << "sequence" << (unsigned int)sequence
                             << "attempt" << (attempt + 1) << "of"
                             << (APP_SERIAL_MAX_RETRIES + 1)
                             << (attempt < APP_SERIAL_MAX_RETRIES ? "; retrying." : ".");
    }

    qWarning().noquote() << "[AppSerial] Command" << AppSerialHex(command)
                         << "sequence" << (unsigned int)sequence << "failed after"
                         << (APP_SERIAL_MAX_RETRIES + 1) << "transmission attempts.";
    return false;
}

bool AppSerial::WaitForResponse(uint8_t command, AppSerialFrame_s *frame, int timeout)
{
    QElapsedTimer timer;
    timer.start();

    for(;;) {
        ProcessIncoming();
        if(!port.isOpen() || (appSerialCommitInProgress && appSerialCommitError))
            return false;

        for(int i = 0; i < appSerialResponses.size(); ++i) {
            if(appSerialResponses.at(i).command == command) {
                if(frame != nullptr)
                    *frame = appSerialResponses.takeAt(i);
                return true;
            }
        }

        const int remaining = timeout - (int)timer.elapsed();
        if(remaining <= 0)
            break;

        // As above, ProcessIncoming() owns parsing after the blocking wait.
        port.waitForReadyRead(remaining);
    }

    qWarning().noquote() << (frame != nullptr ?
                             "[AppSerial] Timed out while retrieving response for command" :
                             "[AppSerial] Response timeout for command")
                         << AppSerialHex(command) << "after" << timeout << "ms.";
    return false;
}

// ===== Dock handshake: raw request, framed response ==========================

bool AppSerial::BeginDock(QByteArray &deviceInfo)
{
    ResetProtocolState();
    port.readAll();

    const char dock[] = {
        (char)OF_Const::sDock1,
        (char)OF_Const::sDock2
    };

    // Local Dock policy (ms), independent of framed command/response timeouts.
    const int dockTimeout = 3000;
    const int retryInterval = 750;
    const int writeTimeout = 250;

    QElapsedTimer timer;
    timer.start();

    qint64 nextAttempt = 0;

    for(;;) {
        ProcessIncoming();

        for(int i = 0; i < appSerialResponses.size(); ++i) {
            if(appSerialResponses.at(i).command != OF_Const::sDock2)
                continue;

            const AppSerialFrame_s frame =
                appSerialResponses.takeAt(i);

            if((frame.typeFlags & APP_SERIAL_FLAG_FINAL) ||
                frame.payload.isEmpty()) {
                qWarning().noquote()
                << "[AppSerial] Invalid board-information response:"
                << "unexpected final flag or empty payload.";
                return false;
            }

            deviceInfo = frame.payload;
            return true;
        }

        const qint64 elapsed = timer.elapsed();
        if(elapsed >= dockTimeout)
            break;

        if(elapsed >= nextAttempt) {
            const qint64 written = port.write(dock, sizeof(dock));

            if(written != sizeof(dock)) {
                qWarning().noquote()
                << "[AppSerial] Incomplete initial Dock write:"
                << written << "of" << sizeof(dock) << "bytes.";
            } else if(port.bytesToWrite() > 0 &&
                       !port.waitForBytesWritten(writeTimeout) &&
                       port.bytesToWrite() > 0) {
                qWarning().noquote()
                << "[AppSerial] Initial Dock write timed out;"
                << "the request will be retried.";
            }

            nextAttempt = timer.elapsed() + retryInterval;
        }

        int waitTime = dockTimeout - (int)timer.elapsed();
        int untilRetry = (int)(nextAttempt - timer.elapsed());

        if(untilRetry < 1)
            untilRetry = 1;
        if(untilRetry < waitTime)
            waitTime = untilRetry;

        if(waitTime > 0)
            port.waitForReadyRead(waitTime);
    }

    qWarning().noquote()
        << "[AppSerial] No valid board-information frame received during Dock.";
    return false;
}

// ===== Device discovery and initial synchronization ==========================

bool AppSerial::SearchPorts()
{
    QList<QSerialPortInfo> serialFoundList = QSerialPortInfo::availablePorts();

    // Keep only OpenFIRE devices.
    for(int i = serialFoundList.size() - 1; i >= 0; --i) {
        if(serialFoundList.at(i).vendorIdentifier() != 0xF143)
            serialFoundList.removeAt(i);
    }

    QMutexLocker locker(&currentPortsMutex);

    bool portsChanged = currentPorts.size() != serialFoundList.size();

    if(!portsChanged) {
        for(const auto &foundPort : std::as_const(serialFoundList)) {
            if(!currentPortsNames.contains(foundPort.portName())) {
                portsChanged = true;
                break;
            }
        }
    }

    if(!portsChanged)
        return false;

    currentPorts = serialFoundList;
    currentPortsNames.clear();
    currentPortsNames.reserve(currentPorts.size());
    for(const auto &currentPort : std::as_const(currentPorts))
        currentPortsNames.append(currentPort.portName());

#ifdef OFAPP_DEBUG
    qDebug().noquote() << "[AppSerial] OpenFIRE device list changed:"
                       << currentPortsNames.join(", ");
#endif

    return true;
}

QList<QSerialPortInfo> AppSerial::CurrentPortsSnapshot() const
{
    QMutexLocker locker(&currentPortsMutex);
    return currentPorts;
}

bool AppSerial::GetSettings(const QString &portName)
{
    if(port.isOpen())
        Disconnect();

    const QList<QSerialPortInfo> ports = CurrentPortsSnapshot();

    for(const auto &curPort : ports) {
        if(portName == curPort.portName() + " (" + curPort.description() + ')') {
            port.setPort(curPort);
            break;
        }
    }

    if(port.portName().isEmpty())
        return false;

    port.setBaudRate(QSerialPort::Baud9600);
    if(!port.open(QIODevice::ReadWrite)) {
        QMessageBox::warning(nullptr,
                             tr("Serial port is already in use!"),
                             tr("This usually indicates that the port is being used by something else, e.g. Arduino IDE's serial monitor, or another command line app (stty, screen).\n\n"
                                "Please close the offending application and try selecting this port again."));
        return false;
    }

    port.setDataTerminalReady(true);

    QByteArray boardInfo;
    if(!BeginDock(boardInfo)) {
        QMessageBox::warning(nullptr,
                             tr("Data hasn't arrived! (Stale state?)"),
                             tr("Device was detected, but initial settings request wasn't received in time!\n"
                                "This can happen if the app was unexpectedly closed and the gun is in a stale docked state.\n\n"
                                "Try selecting the device again."));
        RequestToReboot();
        return false;
    }

    const char separator = (char)OF_Const::serialTerminator;
    const int firstSeparator = boardInfo.indexOf(separator);
    const int secondSeparator = firstSeparator >= 0 ? boardInfo.indexOf(separator, firstSeparator + 1) : -1;
    const int usbOffset = secondSeparator + 1;

    if(firstSeparator <= 0 || secondSeparator <= firstSeparator + 1 ||
       boardInfo.size() < usbOffset + (int)sizeof(App_Common::tinyUSBtable_s)) {
        printf("Port did not respond with a valid board-information payload.\n");
        RequestToReboot();
        return false;
    }

    emit Serial_SetProgressRange(6);
    emit Serial_ProgressUpdate(1, tr("Getting Board Info"));

    App_Common::board.version = boardInfo.left(firstSeparator);
    App_Common::board.type = boardInfo.mid(firstSeparator + 1,
                                           secondSeparator - firstSeparator - 1);
    printf("Version number: %s\n", App_Common::board.version.constData());
    printf("Board type: %s\n", App_Common::board.type.constData());

    if(App_Common::board.type.contains("esp32-s3"))
        App_Common::board.arch = App_Common::OFPresets.boardArchs[OF_Const::boardESP32_S3];
    else App_Common::board.arch = App_Common::OFPresets.boardArchs[OF_Const::boardRP];

    memcpy(&App_Common::tinyUSBtable,
           boardInfo.constData() + usbOffset,
           sizeof(App_Common::tinyUSBtable_s));
    memcpy(&App_Common::tinyUSBtable_orig,
           &App_Common::tinyUSBtable,
           sizeof(App_Common::tinyUSBtable_s));

    const int tailOffset = usbOffset + sizeof(App_Common::tinyUSBtable_s);
    if(boardInfo.size() >= tailOffset + 2 &&
       boardInfo.at(tailOffset) == separator &&
       (uint8_t)boardInfo.at(tailOffset + 1) == OF_Const::sError) {
        ShowError(tr("Device Error: Camera not available!"),
                  tr("<p>Data received from the board indicates that the camera is in a bad state.<br>"
                     "This can happen if, for example, the camera wires are crossed<br>"
                     "(data wire to clock pin, clock wire to data pin),<br>"
                     "or the camera pins are wired to a different component,<br>"
                     "such as a button or Force Feedback output.</p>"
                     "<p>You are able to change the camera pins in the <i>Boards Layout</i> tab<br>"
                     "if they should be mapped different GPIO;<br>"
                     "Otherwise, the camera wires must be resoldered to resolve this error.</p>"),
                  QMessageBox::Warning);
    }

    // For the fixed tables below, wait for the first record before clearing the
    // destination. ReceiveSettingsRecords() then consumes that queued record.
    if(!SendCommand(OF_Const::sGetToggles) ||
       !WaitForResponse(OF_Const::sGetToggles, nullptr, APP_SERIAL_SETTINGS_TIMEOUT))
        return false;
    memset(App_Common::boolSettings, false, sizeof(App_Common::boolSettings));
    if(!ReceiveSettingsRecords(OF_Const::sGetToggles,
                               App_Common::boolSettings[App_Common::dataCurrent],
                               App_Common::OFPresets.boolTypes_Strings,
                               sizeof(App_Common::boolSettings[App_Common::dataCurrent]) / OF_Const::boolTypesCount))
        return false;
    memcpy(App_Common::boolSettings[App_Common::dataOrig],
           App_Common::boolSettings[App_Common::dataCurrent],
           sizeof(App_Common::boolSettings[App_Common::dataCurrent]));

    emit Serial_ProgressUpdate(2, tr("Getting Settings (1)"));

    if(App_Common::boolSettings[App_Common::dataCurrent][OF_Const::customPins]) {
        if(!SendCommand(OF_Const::sGetPins) ||
           !WaitForResponse(OF_Const::sGetPins, nullptr, APP_SERIAL_SETTINGS_TIMEOUT))
            return false;
        App_Common::inputsMap_orig.clear();
        App_Common::inputsMap.clear();
        if(!ReceiveSettingsRecords(OF_Const::sGetPins,
                                   &App_Common::inputsMap_orig,
                                   App_Common::OFPresets.boardInputs_Strings,
                                   0))
            return false;
    } else {
        for(int i = 0; i < OF_Const::boardInputsCount; ++i)
            App_Common::inputsMap_orig[i] = OF_Const::btnUnmapped;
    }
    App_Common::inputsMap = App_Common::inputsMap_orig;

    emit Serial_ProgressUpdate(3, tr("Getting Settings (2)"));

    if(!SendCommand(OF_Const::sGetSettings) ||
       !WaitForResponse(OF_Const::sGetSettings, nullptr, APP_SERIAL_SETTINGS_TIMEOUT))
        return false;
    memset(App_Common::settingsTable, 0, sizeof(App_Common::settingsTable));
    if(!ReceiveSettingsRecords(OF_Const::sGetSettings,
                               App_Common::settingsTable[App_Common::dataCurrent],
                               App_Common::OFPresets.settingsTypes_Strings,
                               sizeof(App_Common::settingsTable[App_Common::dataCurrent]) / OF_Const::settingsTypesCount))
        return false;
    memcpy(App_Common::settingsTable[App_Common::dataOrig],
           App_Common::settingsTable[App_Common::dataCurrent],
           sizeof(App_Common::settingsTable[App_Common::dataCurrent]));

    emit Serial_ProgressUpdate(4, tr("Getting Button Mappings"));

    if(!SendCommand(OF_Const::sGetBtns) ||
       !WaitForResponse(OF_Const::sGetBtns, nullptr, APP_SERIAL_SETTINGS_TIMEOUT))
        return false;
    memset(App_Common::inputFuncTable, 0, sizeof(App_Common::inputFuncTable));
    if(!ReceiveSettingsRecords(OF_Const::sGetBtns,
                               App_Common::inputFuncTable[App_Common::dataCurrent],
                               App_Common::OFPresets.boardInputs_Strings,
                               sizeof(App_Common::inputFuncTable[App_Common::dataCurrent]) / BUTTON_COUNT))
        return false;
    memcpy(App_Common::inputFuncTable[App_Common::dataOrig],
           App_Common::inputFuncTable[App_Common::dataCurrent],
           sizeof(App_Common::inputFuncTable[App_Common::dataCurrent]));

    emit Serial_ProgressUpdate(5, tr("Getting Profiles Data"));
    App_Common::profilesTable.clear();
    App_Common::profilesTable_orig.clear();

    if(!SendCommand(OF_Const::sGetProfile) ||
       !WaitForResponse(OF_Const::sGetProfile, nullptr, APP_SERIAL_SETTINGS_TIMEOUT))
        return false;
    if(!ReceiveSettingsRecords(OF_Const::sGetProfile,
                               nullptr,
                               App_Common::OFPresets.profSettingTypes_Strings,
                               sizeof(float)))
        return false;

    App_Common::profilesTable_orig = App_Common::profilesTable;
    App_Common::board.previousProfile = App_Common::board.selectedProfile;
    emit Serial_ProgressUpdate(6, tr("Successfully synced data!"));
    return true;
}

// ===== Settings record decoding =============================================
// Payload: field name + NUL, value size, optional profile number, value bytes.
// CurrentProf has no profile-number byte; an empty FINAL ends the record stream.

bool AppSerial::ReceiveSettingsRecords(
    uint8_t command,
    void *dataPtr,
    const std::unordered_map<std::string_view, int> &dataMap,
    size_t dataSize)
{
    for(;;) {
        AppSerialFrame_s frame;
        if(!WaitForResponse(command, &frame, APP_SERIAL_SETTINGS_TIMEOUT))
            return false;

        if(frame.typeFlags & APP_SERIAL_FLAG_FINAL) {
            if(!frame.payload.isEmpty()) {
                qWarning().noquote() << "[AppSerial] Malformed final record for command"
                                     << AppSerialHex(command)
                                     << ": final payload must be empty.";
                return false;
            }

            if(command == OF_Const::sGetProfile) {
                if(App_Common::profilesTable.isEmpty() ||
                    App_Common::profilesTable.size() > APP_SERIAL_MAX_PROFILE_COUNT ||
                    App_Common::board.selectedProfile < 0 ||
                    App_Common::board.selectedProfile >= App_Common::profilesTable.size()) {
                    qWarning().noquote()
                    << "[AppSerial] Incomplete or invalid profile set received.";
                    return false;
                }
            }

            return true;
        }

        const QByteArray &payload = frame.payload;
        const int nullPos = payload.indexOf('\0');
        if(nullPos < 0 || nullPos + 1 >= payload.size()) {
            qWarning().noquote() << "[AppSerial] Malformed settings record for command"
                                 << AppSerialHex(command)
                                 << ": missing field terminator or value length; payload size"
                                 << payload.size() << '.';
            return false;
        }

        // The view is used only while this frame's payload remains alive.
        const std::string_view fieldName(payload.constData(), (size_t)nullPos);
        int pos = nullPos + 1;
        const uint8_t valueSize = (uint8_t)payload.at(pos++);

        const auto field = dataMap.find(fieldName);
        if(field == dataMap.end()) {
            #ifdef OFAPP_DEBUG
            qDebug().noquote() << "[AppSerial] Ignoring unknown settings field"
                               << AppSerialFieldName(fieldName) << "for forward compatibility.";
            #endif
            continue;
        }

        const int index = field->second;
        if(index < 0) {
            #ifdef OFAPP_DEBUG
            qDebug().noquote() << "[AppSerial] Ignoring disabled settings field"
                               << AppSerialFieldName(fieldName) << '.';
            #endif
            continue;
        }

        if(command == OF_Const::sGetPins) {
            if(valueSize != sizeof(int8_t) || pos + valueSize != payload.size()) {
                qWarning().noquote() << "[AppSerial] Malformed pin record"
                                     << AppSerialFieldName(fieldName) << ": declared value size"
                                     << (unsigned int)valueSize << "payload size" << payload.size() << '.';
                return false;
            }
            int8_t pin = 0;
            memcpy(&pin, payload.constData() + pos, sizeof(pin));
            App_Common::inputsMap_orig[index] = pin;
        } else if(command == OF_Const::sGetProfile) {
            if(index == OF_Const::profCurrent) {
                // CurrentProf is intentionally a one-byte wire value; the UI
                // stores it in a native int only after decoding it.
                if(valueSize != sizeof(uint8_t) || pos + valueSize != payload.size()) {
                    qWarning().noquote() << "[AppSerial] Malformed current-profile record: declared value size"
                                         << (unsigned int)valueSize << "payload size" << payload.size() << '.';
                    return false;
                }
                const uint8_t selectedProfile = (uint8_t)payload.at(pos);
                if(selectedProfile >= APP_SERIAL_MAX_PROFILE_COUNT) {
                    qWarning().noquote()
                    << "[AppSerial] Invalid selected profile:"
                    << (unsigned int)selectedProfile << '.';
                    return false;
                }

                App_Common::board.selectedProfile = selectedProfile;

            } else {
                if(pos >= payload.size()) {
                    qWarning().noquote() << "[AppSerial] Malformed profile record"
                                         << AppSerialFieldName(fieldName) << ": missing profile number.";
                    return false;
                }
                const uint8_t profNum = (uint8_t)payload.at(pos++);

                if(profNum >= APP_SERIAL_MAX_PROFILE_COUNT) {
                    qWarning().noquote()
                    << "[AppSerial] Invalid profile number:"
                    << (unsigned int)profNum << '.';
                    return false;
                }

                while(profNum >= App_Common::profilesTable.size())
                    App_Common::profilesTable.append(App_Common::profilesTable_s());

                const size_t expectedSize = index == OF_Const::profName ?
                                            sizeof(App_Common::profilesTable_s::profName) : dataSize;
                if(valueSize != expectedSize || pos + valueSize != payload.size()) {
                    qWarning().noquote() << "[AppSerial] Malformed profile record"
                                         << AppSerialFieldName(fieldName) << "profile" << (unsigned int)profNum
                                         << ": declared value size" << (unsigned int)valueSize
                                         << "expected" << expectedSize
                                         << "payload size" << payload.size() << '.';
                    return false;
                }

                memcpy((char*)&App_Common::profilesTable[profNum] + (dataSize * index),
                       payload.constData() + pos,
                       valueSize);
            }
        } else {
            if(valueSize != dataSize || pos + valueSize != payload.size()) {
                qWarning().noquote() << "[AppSerial] Malformed settings record"
                                     << AppSerialFieldName(fieldName) << ": declared value size"
                                     << (unsigned int)valueSize << "expected" << dataSize
                                     << "payload size" << payload.size() << '.';
                return false;
            }
            memcpy((char*)dataPtr + (dataSize * index), payload.constData() + pos, valueSize);
        }

        #ifdef OFAPP_DEBUG
        qDebug() << payload;
        #endif
    }
}

// ===== Settings commit ======================================================

bool AppSerial::CommitSettings()
{
    appSerialCommitInProgress = true;
    appSerialCommitNeedsRetry = true;
    appSerialCommitError = false;

    if(!port.isOpen()) {
        qWarning().noquote() << "[AppSerial] Cannot commit settings: serial port is closed.";
        return CommitFailed();
    }

    emit Serial_SetProgressRange(8);
    emit Serial_ProgressUpdate(0, tr("Waiting for board..."));

    // Preserve both phases: wait without consuming, then retrieve and validate.
    if(!SendCommand(OF_Const::sCommitStart) ||
       !WaitForResponse(OF_Const::sCommitStart, nullptr, APP_SERIAL_COMMIT_START_TIMEOUT)) {
        qWarning().noquote() << "[AppSerial] Commit failed while starting the transaction.";
        return CommitFailed();
    }

    AppSerialFrame_s ready;
    if(!WaitForResponse(OF_Const::sCommitStart, &ready, APP_SERIAL_COMMIT_READY_TIMEOUT)) {
        qWarning().noquote() << "[AppSerial] Commit-start confirmation was not available.";
        return CommitFailed();
    }

    if(!ready.payload.isEmpty() || (ready.typeFlags & APP_SERIAL_FLAG_FINAL))
        return CommitFailed();

    emit Serial_ProgressUpdate(1, tr("Sending Toggles..."));
    if(!SendSettingsRecords(OF_Const::sCommitToggles,
                            App_Common::boolSettings[App_Common::dataCurrent],
                            App_Common::OFPresets.boolTypes_Strings,
                            sizeof(App_Common::boolSettings[App_Common::dataCurrent]) / OF_Const::boolTypesCount)) {
        return CommitFailed();
    }

    // Include the pin group when custom pins are being disabled as well.
    if(App_Common::boolSettings[App_Common::dataCurrent][OF_Const::customPins] ||
       App_Common::boolSettings[App_Common::dataOrig][OF_Const::customPins]) {
        emit Serial_ProgressUpdate(2, tr("Sending Pins Map..."));
        int8_t inputMapFW[OF_Const::boardInputsCount];
        for(size_t i = 0; i < OF_Const::boardInputsCount; ++i)
            inputMapFW[i] = App_Common::inputsMap.value(i);

        if(!SendSettingsRecords(OF_Const::sCommitPins,
                                inputMapFW,
                                App_Common::OFPresets.boardInputs_Strings,
                                sizeof(inputMapFW) / OF_Const::boardInputsCount)) {
            return CommitFailed();
        }
    }

    emit Serial_ProgressUpdate(3, tr("Sending Settings..."));
    if(!SendSettingsRecords(OF_Const::sCommitSettings,
                            App_Common::settingsTable[App_Common::dataCurrent],
                            App_Common::OFPresets.settingsTypes_Strings,
                            sizeof(App_Common::settingsTable[App_Common::dataCurrent]) / OF_Const::settingsTypesCount)) {
        return CommitFailed();
    }

    emit Serial_ProgressUpdate(4, tr("Sending Buttons..."));
    if(!SendSettingsRecords(OF_Const::sCommitBtns,
                            App_Common::inputFuncTable[App_Common::dataCurrent],
                            App_Common::OFPresets.boardInputs_Strings,
                            sizeof(App_Common::inputFuncTable[App_Common::dataCurrent]) / BUTTON_COUNT)) {
        return CommitFailed();
    }

    emit Serial_ProgressUpdate(5, tr("Sending Profile Data..."));
    for(size_t i = 0; i < App_Common::profilesTable.count(); ++i) {
        if(!SendSettingsRecords(OF_Const::sCommitProfile,
                                &App_Common::profilesTable[i],
                                App_Common::OFPresets.profSettingTypes_Strings,
                                sizeof(float),
                                i)) {
            return CommitFailed();
        }
    }

    emit Serial_ProgressUpdate(6, tr("Sending TinyUSB ID Data..."));
    if(!SendCommand(OF_Const::sCommitID,
                    (const char*)&App_Common::tinyUSBtable,
                    sizeof(App_Common::tinyUSBtable_s))) {
        return CommitFailed();
    }

    emit Serial_ProgressUpdate(7, tr("Saving..."));
    if(!SendCommand(OF_Const::sSave))
        return CommitFailed();

    // A delivery ACK is not a save result: wait for the device's actual outcome.
    AppSerialFrame_s saved;
    if(!WaitForResponse(OF_Const::sSave, &saved, APP_SERIAL_SAVE_TIMEOUT)) {
        qWarning().noquote() << "[AppSerial] No final save result received from the device.";
        return CommitFailed();
    }
    if(saved.payload.isEmpty() || (uint8_t)saved.payload.at(0) > 1) {
        qWarning().noquote() << "[AppSerial] Malformed save result.";
        return CommitFailed();
    }
    if(saved.payload.at(0) == false) {
        qWarning().noquote() << "[AppSerial] Device reported that settings could not be saved:"
                             << saved.payload.mid(1);
        return CommitFailed();
    }

    appSerialCommitInProgress = false;
    appSerialCommitNeedsRetry = false;
    appSerialCommitError = false;
    emit Serial_ProgressUpdate(8);
    return true;
}

// ===== Settings record encoding =============================================

bool AppSerial::SendSettingsRecords(
    uint8_t command,
    const void *dataPtr,
    const std::unordered_map<std::string_view, int> &dataMap,
    size_t dataSize,
    size_t profNum)
{
    char payload[APP_SERIAL_MAX_PAYLOAD];
    bool profCurrentMarked = false;

    for(const auto &pair : dataMap) {
        if(pair.second < 0)
            continue;

        size_t length = 0;
        if(pair.first.length() + 1 > sizeof(payload)) {
            qWarning().noquote() << "[AppSerial] Settings field name is too long for command"
                                 << AppSerialHex(command) << ':' << AppSerialFieldName(pair.first) << '.';
            return false;
        }
        memcpy(payload, pair.first.data(), pair.first.length());
        length += pair.first.length();
        payload[length++] = '\0';

        if(command == OF_Const::sCommitProfile) {
            // Calibration geometry stays in the firmware; send App-writable fields.
            if(pair.second < OF_Const::profIrSens)
                continue;
            if(pair.second == OF_Const::profCurrent) {
                // Preserve the existing one-per-batch record and ignore aliases.
                if(profCurrentMarked)
                    continue;
                if(length + 2 > sizeof(payload)) {
                    qWarning().noquote() << "[AppSerial] Current-profile record exceeds the payload limit.";
                    return false;
                }
                payload[length++] = sizeof(uint8_t);
                payload[length++] = (char)App_Common::board.selectedProfile;
                profCurrentMarked = true;
            } else {
                const size_t valueSize = pair.second == OF_Const::profName ?
                                         sizeof(App_Common::profilesTable_s::profName) : dataSize;
                if(length + 2 + valueSize > sizeof(payload)) {
                    qWarning().noquote() << "[AppSerial] Profile record exceeds the payload limit:"
                                         << AppSerialFieldName(pair.first) << "profile" << profNum << '.';
                    return false;
                }
                payload[length++] = (char)valueSize;
                payload[length++] = (char)profNum;
                memcpy(&payload[length], (const uint8_t*)dataPtr + (dataSize * pair.second), valueSize);
                length += valueSize;
            }
        } else {
            if(command == OF_Const::sCommitBtns && pair.second >= BUTTON_COUNT - 1)
                continue;
            if(length + 1 + dataSize > sizeof(payload)) {
                qWarning().noquote() << "[AppSerial] Settings record exceeds the payload limit:"
                                     << AppSerialFieldName(pair.first) << '.';
                return false;
            }
            payload[length++] = (char)dataSize;
            memcpy(&payload[length], (const uint8_t*)dataPtr + (dataSize * pair.second), dataSize);
            length += dataSize;
        }

        if(!SendCommand(command, payload, length)) {
            qWarning().noquote() << "[AppSerial] Failed to send settings field"
                                 << AppSerialFieldName(pair.first) << "using command"
                                 << AppSerialHex(command) << '.';
            return false;
        }
    }

    return true;
}

// ===== Disconnect and bootloader ============================================

void AppSerial::Disconnect()
{
    if(port.isOpen() && appSerialSessionActive) {
        // Together with the command byte this preserves the FE FE FE
        // full-disconnect marker used by the firmware.
        const char payload[] = {
            (char)OF_Const::serialTerminator,
            (char)OF_Const::serialTerminator
        };

        if(SendCommand(OF_Const::serialTerminator, payload, sizeof(payload))) {
            AppSerialFrame_s finalResponse;

            if(WaitForResponse(OF_Const::serialTerminator,
                             &finalResponse,
                             APP_SERIAL_DISCONNECT_TIMEOUT)) {
                if(!(finalResponse.typeFlags & APP_SERIAL_FLAG_FINAL) ||
                    !finalResponse.payload.isEmpty()) {
                    qWarning().noquote()
                    << "[AppSerial] Invalid final disconnect response.";
                }
            }
        }
    }

    if(port.isOpen())
        port.close();

    ResetProtocolState();
    port.setPortName("");
}

void AppSerial::RequestToReboot()
{
    QString title;
    QString text;
    if(App_Common::board.arch == App_Common::OFPresets.boardArchs[OF_Const::boardRP]) {
        title = tr("Reset Board to Bootloader?");
        text = tr("<p>The board you selected did not respond to the app properly.</p>"
                  "<p>This can usually be resolved by rebooting the microcontroller to its bootloader, and then updating the board to the latest firmware, which can be found at:</p>"
                  "<p><a href='https://github.com/TeamOpenFIRE/OpenFIRE-Firmware/releases/latest'><span style=' text-decoration: underline; color:#8ab4f8;'>https://github.com/TeamOpenFIRE/OpenFIRE-Firmware/releases/latest</span></a></p>"
                  "<p>Would you like to reboot this board to apply an update?</p>");
    } else {
        title = tr("Reboot Microcontroller?");
        text = tr("<p>The board you selected did not respond to the app properly.</p>"
                  "<p>This can usually be resolved by rebooting the microcontroller, and then updating the board to the latest firmware, which can be found at:</p>"
                  "<p><a href='https://github.com/TeamOpenFIRE/OpenFIRE-Firmware/releases/latest'><span style=' text-decoration: underline; color:#8ab4f8;'>https://github.com/TeamOpenFIRE/OpenFIRE-Firmware/releases/latest</span></a></p>"
                  "<p>Would you like to reboot this board to apply an update?</p>");
    }

    if(QMessageBox::critical(nullptr, title, text, QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes)
        RebootToBootldr();
}

void AppSerial::RebootToBootldr()
{
    if(port.isOpen()) {
        if(App_Common::board.arch ==
            App_Common::OFPresets.boardArchs[OF_Const::boardRP]) {
            port.setBaudRate(QSerialPort::Baud1200);
            port.setDataTerminalReady(false);
        } else {
            // Local write deadline only (ms): bootloader entry does not wait for ACK.
            const int writeTimeout = 500;
            const QByteArray frame = BuildFrame(
                APP_SERIAL_TYPE_REQUEST,
                OF_Const::sRebootToBootloader,
                NextSequence(),
                nullptr,
                0);

            if(port.write(frame) != frame.size()) {
                qWarning() << "[AppSerial] Incomplete reboot command write.";
            } else {
                QElapsedTimer timer;
                timer.start();

                // Wait only for local transmission, never for an ACK.
                while(port.bytesToWrite() > 0) {
                    const int remaining = writeTimeout - (int)timer.elapsed();

                    if(remaining <= 0 ||
                        (!port.waitForBytesWritten(remaining) &&
                         port.bytesToWrite() > 0)) {
                        qWarning()
                        << "[AppSerial] Reboot command write did not complete.";
                        break;
                    }
                }
            }
        }

        port.close();
    }

    ResetProtocolState();
}
