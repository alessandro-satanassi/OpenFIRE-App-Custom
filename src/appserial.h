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
#include <QSerialPort>
#include <QSerialPortInfo>
#include <cstdint>
#include <string_view>
#include <unordered_map>

#include <QMutex>

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

    enum {
        Serial_SearchPorts = 0,
        Serial_GetSettings,
        Serial_OneShot,
        Serial_Sync,
        Serial_Disconnect
    } serialReturnTypes_e;

    QSerialPort port;

    /// @brief      Primary list of found serial devices
    /// @note       Only to be updated when new found ports list is different from this one.
    QList<QSerialPortInfo> currentPorts;

    QStringList currentPortsNames;

    /// @brief      Searches for new serial devices
    /// @returns    Whether devices list has changed (true) or not (false)
    /// @note       If found devices are different, this list is merged into the new currentPorts.
    bool SearchPorts();

    QList<QSerialPortInfo> CurrentPortsSnapshot() const;

    /// @brief      Creates a string list of ports based on given list of serial devices
    /// @returns    New list of port names
    /// @param      QList
    ///             Serial ports list
    QStringList GeneratePortsList(const QList<QSerialPortInfo> &);

    /// @brief      Grabs settings from connected Serial device to App_Const
    /// @returns    Success (true) or failure (false)
    /// @param      Index of currentPorts list to pull from
    bool GetSettings(const QString &);

    /// @brief      Macro for saving batches of settings being received over serial
    /// @returns    Success (true) or failure (false)
    /// @param      void*
    ///             Pointer to data array
    /// @param      unordered_map
    ///             Map from OF_Const::OFPresets to reference for string names and respective indices
    /// @param      size_t
    ///             Size of data blocks that void* points to, for correct seeks through array elements by address
    bool BatchStoreSettings(void*, const std::unordered_map<std::string_view, int>&, const size_t&);

    /// @brief      Sends serial message to currently connected Serial device
    /// @returns    Success (true) or failure (false)
    /// @param      QString
    ///             String to send to device
    bool OneShotSend(const char*, const unsigned int & = 0, const bool & = false);
    bool OneShotSend(const char &, const bool & = false);

    /// @brief      Commits settings (App_Const) to currently connected Serial device
    /// @returns    Success (true) or failure (false)
    bool CommitSettings();
    bool CommitNeedsRetry() const { return appSerialCommitNeedsRetry; }

    // sError payload: [retry required] [originating request command].
    static constexpr uint8_t APP_SERIAL_ERR_COMMIT_RETRY = 0x82;

    /// @brief Close on a fatal error; require a board restart. Returns false.
    bool FailOperation();

    /// @brief      Macro for committing batches of data to board
    /// @returns    Success (true) or failure (false)
    /// @param      void*
    ///             Pointer to data array
    /// @param      unordered_map
    ///             Map from OF_Const::OFPresets to reference for string names and respective indices
    /// @param      size_t
    ///             Size of data blocks that void* points to, for correct seeks through array elements by address
    bool BatchSendSettings(void*, const std::unordered_map<std::string_view, int>&, const size_t&, const size_t& = 0);

    /// @brief      Disconnects current serial device and clears port name
    void Disconnect();

    /// @brief      Prompts user to reboot to bootloader
    void RequestToReboot();

    /// @brief      Sends magic baud 1200 signal to reset connected board to bootloader
    void RebootToBootldr();

    /// @brief      Show error message popup regarding serial
    void ShowError(const QString& titleText, const QString& text, const QMessageBox::Icon icon = QMessageBox::Warning) {
        if(!syncError.isVisible()) {
            syncError.setWindowTitle(titleText);
            syncError.setText(text);
            syncError.setIcon(icon);
            syncError.show();
        }
    }

private:
    typedef struct AppSerialFrame_t {
        uint8_t typeFlags;
        uint8_t command;
        uint8_t sequence;
        QByteArray payload;
        uint8_t crc;
    } AppSerialFrame_s;

    static uint8_t AppSerialCRC8(const uint8_t*, int);
    QByteArray BuildFrame(uint8_t, uint8_t, uint8_t, const char*, uint8_t) const;
    bool SendReliableFrame(uint8_t, const char*, uint8_t);
    bool SendAck(uint8_t, uint8_t);
    bool WaitForAck(uint8_t, uint8_t);
    bool WaitForResponse(uint8_t, int = 1000);
    bool TakeResponse(uint8_t, AppSerialFrame_s&, int = 1000);
    bool BeginDock(QByteArray&);
    void ResetProtocolState();
    void ParseIncoming();
    void HandleFrame(const AppSerialFrame_s&);
    bool CommitFailed();
    bool IsTransactionResponse(uint8_t) const;
    uint8_t NextSequence();

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

    char TXbuf[APP_SERIAL_MAX_PAYLOAD + 1];

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
