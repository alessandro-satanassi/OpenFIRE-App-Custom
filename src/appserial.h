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

#ifndef APPSERIAL_H
#define APPSERIAL_H

#include <QObject>
#include <QByteArray>
#include <QElapsedTimer>
#include <QList>
#include <QMessageBox>
#include <QMutex>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QString>
#include <QStringList>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <unordered_map>

class AppSerial : public QObject
{
    Q_OBJECT

public:
    static constexpr uint8_t APP_SERIAL_START_1       = 0xA5;
    static constexpr uint8_t APP_SERIAL_START_2       = 0x5A;
    static constexpr uint8_t APP_SERIAL_MAX_PAYLOAD   = 200;

    static constexpr uint8_t APP_SERIAL_MAX_PROFILE_COUNT = 20;  // massimo numero di profili che l'app può gestire

    static constexpr uint8_t APP_SERIAL_OVERHEAD      = 7;
    static constexpr int     APP_SERIAL_FRAME_TIMEOUT = 250;
    static constexpr int     APP_SERIAL_ACK_TIMEOUT   = 500;
    static constexpr uint8_t APP_SERIAL_MAX_RETRIES   = 3;

    enum AppSerialType_e : uint8_t {
        APP_SERIAL_TYPE_REQUEST  = 0x00,
        APP_SERIAL_TYPE_RESPONSE = 0x01,
        APP_SERIAL_TYPE_EVENT    = 0x02,
        APP_SERIAL_TYPE_ACK      = 0x03,
        APP_SERIAL_TYPE_MASK     = 0x03,
        APP_SERIAL_FLAG_FINAL    = 0x80
    };

    explicit AppSerial(QObject *parent = nullptr);

    QSerialPort port;

    /// @brief      Searches for new serial devices
    /// @returns    Whether devices list has changed (true) or not (false)
    /// @note       If found devices are different, this list is merged into the new currentPorts.
    bool SearchPorts();

    QList<QSerialPortInfo> CurrentPortsSnapshot() const;

    /// @brief      Loads the connected device's settings into App_Common.
    /// @returns    Success (true) or failure (false)
    /// @param      portName Displayed name from the device selector.
    bool GetSettings(const QString &portName);

    /// @brief      Sends one framed command to the connected device.
    /// @note       Waits for delivery acknowledgement, not operation completion.
    /// @returns    Success (true) or failure (false)
    bool SendCommand(uint8_t command, const char *payload = nullptr, size_t length = 0);

    /// @brief      Sends App_Common settings and waits for the device's save result.
    /// @returns    Success (true) or failure (false)
    bool CommitSettings();
    bool CommitNeedsRetry() const { return appSerialCommitNeedsRetry; }

    // sError payload: [retry required] [originating request command].
    static constexpr uint8_t APP_SERIAL_ERR_COMMIT_RETRY = 0x82;

    /// @brief Close on a fatal error; require a board restart. Returns false.
    bool FailOperation();

    /// @brief      Disconnects current serial device and clears port name
    void Disconnect();

    /// @brief      Enters the bootloader using RP2040's 1200-baud touch or the ESP32 command.
    void RebootToBootldr();

    /// @brief      Show error message popup regarding serial
    void ShowError(const QString &titleText, const QString &text,
                   QMessageBox::Icon icon = QMessageBox::Warning);

private:
    typedef struct AppSerialFrame_t {
        uint8_t typeFlags;
        uint8_t command;
        uint8_t sequence;
        QByteArray payload;
        uint8_t crc;
    } AppSerialFrame_s;

    // Framing, delivery and response routing.
    static uint8_t AppSerialCRC8(const uint8_t *data, int length);
    static QByteArray BuildFrame(uint8_t typeFlags, uint8_t command,
                                 uint8_t sequence, const char *payload, uint8_t length);
    bool SendAck(uint8_t command, uint8_t sequence);
    bool WaitForAck(uint8_t command, uint8_t sequence);
    // With a destination, remove and return the response; nullptr only waits.
    bool WaitForResponse(uint8_t command, AppSerialFrame_s *frame, int timeout);
    bool BeginDock(QByteArray &deviceInfo);
    void ResetProtocolState();
    void ParseIncoming();
    void HandleFrame(const AppSerialFrame_s &frame);
    bool CommitFailed();
    static bool IsTransactionResponse(uint8_t command);
    uint8_t NextSequence();
    void RequestToReboot();

    // One settings record per frame; field names and wire values are unchanged.
    bool ReceiveSettingsRecords(uint8_t command, void *dataPtr,
                                const std::unordered_map<std::string_view, int> &dataMap,
                                size_t dataSize);
    bool SendSettingsRecords(uint8_t command, const void *dataPtr,
                             const std::unordered_map<std::string_view, int> &dataMap,
                             size_t dataSize, size_t profNum = 0);

    /// @brief Primary list of OpenFIRE serial devices, protected for the scan worker.
    QList<QSerialPortInfo> currentPorts;
    QStringList currentPortsNames;

    QByteArray appSerialRxBuffer;
    QList<AppSerialFrame_s> appSerialResponses;
    QElapsedTimer appSerialRxTimer;

    uint8_t appSerialTxSequence = 0;
    uint8_t appSerialLastRequest = 0;
    bool appSerialAckReceived = false;
    uint8_t appSerialAckCommand = 0;
    uint8_t appSerialAckSequence = 0;

    bool appSerialLastRxValid = false;
    uint8_t appSerialLastRxTypeFlags = 0;
    uint8_t appSerialLastRxCommand = 0;
    uint8_t appSerialLastRxSequence = 0;
    uint8_t appSerialLastRxLength = 0;
    uint8_t appSerialLastRxCRC = 0;

    QMessageBox syncError;

    mutable QMutex currentPortsMutex;
    bool appSerialSessionActive = false;
    bool appSerialCommitInProgress = false;
    bool appSerialCommitNeedsRetry = false;
    bool appSerialCommitError = false;
    uint8_t appSerialLastStartSequence = 0;
    uint8_t appSerialLastSaveSequence = 0;

private slots:
    void ProcessIncoming();

signals:
    /// @brief
    void Serial_SetProgressRange(const int &);

    /// @brief
    void Serial_ProgressUpdate(const int &, const QString& = nullptr);

    /// @brief      Delivers a complete validated asynchronous App event.
    void Serial_EventReceived(const uint8_t, const QByteArray&);
};

#endif // APPSERIAL_H
