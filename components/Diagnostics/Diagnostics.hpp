#ifndef DIAGNOSTICS_HPP
#define DIAGNOSTICS_HPP

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "esp_http_server.h"
#include "esp_log.h"

class DiagnosticsService {
public:
    struct LogSnapshotEntry {
        uint64_t seq;
        uint64_t timestampUs;
        std::string message;
    };

    DiagnosticsService();

    void init(size_t maxLines = 400, size_t maxLineLength = 192);
    void setAuthToken(const std::string& token);

    void installLogSink();

    void onWifiConnected();
    void onWifiDisconnected();

    bool isServerRunning() const;

    uint64_t droppedCount() const;
    size_t bufferSize() const;

private:
    static constexpr const char* TAG = "Diagnostics";

    struct BufferItem {
        uint64_t seq;
        uint64_t timestampUs;
        std::string message;
    };

    size_t maxLines_;
    size_t maxLineLength_;
    std::deque<BufferItem> buffer_;
    mutable std::mutex bufferMutex_;
    std::atomic<uint64_t> nextSeq_;
    std::atomic<uint64_t> droppedCount_;

    mutable std::mutex serverMutex_;
    httpd_handle_t server_;
    std::string authToken_;

    vprintf_like_t previousLogSink_;

    static DiagnosticsService* instance_;

    static int logSink(const char* fmt, va_list args);
    int logSinkInternal(const char* fmt, va_list args);

    void appendLogLine(const char* line, size_t lineLen);

    void startServer();
    void stopServer();

    bool isAuthorized(httpd_req_t* req) const;

    bool parseCursorAndLimit(httpd_req_t* req, uint64_t* cursor, size_t* limit) const;
    std::vector<LogSnapshotEntry> snapshotFrom(uint64_t cursor, size_t maxLines, uint64_t* nextCursor) const;

    static esp_err_t handleLogs(httpd_req_t* req);
    static esp_err_t handleStatus(httpd_req_t* req);
    static esp_err_t handleCoreInfo(httpd_req_t* req);

    esp_err_t handleLogsInternal(httpd_req_t* req);
    esp_err_t handleStatusInternal(httpd_req_t* req);
    esp_err_t handleCoreInfoInternal(httpd_req_t* req);
};

#endif
