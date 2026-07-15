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

/**
 * @brief Diagnostics service for ESP32 system monitoring and logging.
 * 
 * Provides HTTP-based access to system diagnostics including:
 * - Buffered log viewing with cursor-based pagination
 * - System status (uptime, reset reason, buffer statistics)
 * - Core dump information
 * 
 * Features token-based authentication and non-blocking HTTP handlers that yield
 * to LVGL task to prevent watchdog timeouts.
 */
class DiagnosticsService {
public:
    /** @brief A single log entry snapshot with sequence number and timestamp */
    struct LogSnapshotEntry {
        uint64_t seq;              ///< Sequential log number
        uint64_t timestampUs;      ///< Timestamp in microseconds (from esp_timer)
        std::string message;       ///< Log message text
    };

    /** @brief Construct diagnostics service instance */
    DiagnosticsService();

    /**
     * @brief Initialize the diagnostics service with buffer configuration
     * @param maxLines Maximum number of log lines to buffer (default 400)
     * @param maxLineLength Maximum length of each log line (default 192)
     */
    void init(size_t maxLines = 400, size_t maxLineLength = 192);

    /**
     * @brief Set authentication token for HTTP endpoints
     * @param token Bearer token to validate X-Diag-Token header
     */
    void setAuthToken(const std::string& token);

    /**
     * @brief Install this service as the ESP-IDF log sink
     * Intercepts all log output and buffers it for HTTP access
     */
    void installLogSink();

    /** @brief Start HTTP server when WiFi connects */
    void onWifiConnected();

    /** @brief Stop HTTP server when WiFi disconnects */
    void onWifiDisconnected();

    /** @brief Check if diagnostics HTTP server is currently running */
    bool isServerRunning() const;

    /** @brief Get count of log lines dropped due to buffer overflow */
    uint64_t droppedCount() const;

    /** @brief Get current number of buffered log lines */
    size_t bufferSize() const;

private:
    /** @brief Log tag for ESP_LOG macros */
    static constexpr const char* TAG = "Diagnostics";

    /** @brief Internal log buffer entry with metadata */
    struct BufferItem {
        uint64_t seq;              ///< Sequential log number for pagination
        uint64_t timestampUs;      ///< Timestamp when logged (from esp_timer)
        std::string message;       ///< Log message text (trimmed)
    };

    // Configuration
    size_t maxLines_;              ///< Maximum number of log lines to keep in buffer
    size_t maxLineLength_;         ///< Maximum characters per log line

    // Logging buffer (thread-safe via bufferMutex_)
    std::deque<BufferItem> buffer_;     ///< Circular buffer of log entries
    mutable std::mutex bufferMutex_;    ///< Protects buffer_ and related atomics
    std::atomic<uint64_t> nextSeq_;     ///< Next sequence number for log entries
    std::atomic<uint64_t> droppedCount_; ///< Count of log lines lost to overflow

    // HTTP server (thread-safe via serverMutex_)
    mutable std::mutex serverMutex_;    ///< Protects server_ and authToken_
    httpd_handle_t server_;             ///< ESP HTTP server handle (nullptr if stopped)
    std::string authToken_;             ///< Bearer token for X-Diag-Token header validation

    // Previous log sink for chaining
    vprintf_like_t previousLogSink_;    ///< Previous vprintf handler (for log chaining)

    // Reset reason tracking
    std::string lastResetReason_;       ///< Last reset reason retrieved from NVS

    // Singleton instance
    static DiagnosticsService* instance_; ///< Singleton instance for static callback access

    // Log sink installation (static callbacks)
    /** @brief Static entry point for log sink (calls logSinkInternal via instance_) */
    static int logSink(const char* fmt, va_list args);

    /** @brief Internal log sink that buffers log messages with timestamps */
    int logSinkInternal(const char* fmt, va_list args);

    /**
     * @brief Append formatted log line to buffer with auto-trim and overflow handling
     * @param line Log message text
     * @param lineLen Length of message
     */
    void appendLogLine(const char* line, size_t lineLen);

    /** @brief Initialize and start HTTP server (thread-safe, idempotent) */
    void startServer();

    /** @brief Shut down and stop HTTP server (thread-safe, idempotent) */
    void stopServer();

    /**
     * @brief Verify X-Diag-Token header matches configured auth token
     * @param req HTTP request to validate
     * @return true if token matches or no token required, false otherwise
     */
    bool isAuthorized(httpd_req_t* req) const;

    /**
     * @brief Parse cursor and limit query parameters from HTTP request
     * @param req HTTP request with optional ?cursor=N&limit=N query string
     * @param cursor Output cursor value (defaults to 0)
     * @param limit Output limit value (defaults to 80, max 200)
     * @return true on successful parse (or valid defaults used)
     */
    bool parseCursorAndLimit(httpd_req_t* req, uint64_t* cursor, size_t* limit) const;

    /**
     * @brief Get snapshot of log entries from cursor position
     * @param cursor Start from entries after this sequence number
     * @param maxLines Maximum entries to return
     * @param nextCursor Output for next cursor value (for pagination)
     * @return Vector of log entries in chronological order
     */
    std::vector<LogSnapshotEntry> snapshotFrom(uint64_t cursor, size_t maxLines, uint64_t* nextCursor) const;

    /**
     * @brief Load last reset reason from NVS storage
     * Retrieves the reset reason that was saved on previous boot
     */
    void loadLastResetReason();

    /**
     * @brief Save current reset reason to NVS storage
     * Persists the current reset reason so it can be retrieved on next boot
     */
    void saveCurrentResetReason();

    // HTTP handler static entry points
    /** @brief Static HTTP handler for /diag/logs endpoint (wraps handleLogsInternal) */
    static esp_err_t handleLogs(httpd_req_t* req);

    /** @brief Static HTTP handler for /diag/status endpoint (wraps handleStatusInternal) */
    static esp_err_t handleStatus(httpd_req_t* req);

    /** @brief Static HTTP handler for /diag/coreinfo endpoint (wraps handleCoreInfoInternal) */
    static esp_err_t handleCoreInfo(httpd_req_t* req);

    // HTTP handler implementations
    /**
     * @brief Handle GET /diag/logs - return paginated log buffer as JSON
     * Yields to allow LVGL task to run (prevents watchdog timeout)
     */
    esp_err_t handleLogsInternal(httpd_req_t* req);

    /**
     * @brief Handle GET /diag/status - return system uptime, current and last reset reason, buffer stats as JSON
     * Returns both current reset reason (why system is running) and last reset reason (from previous boot)
     * Yields to allow LVGL task to run (prevents watchdog timeout)
     */
    esp_err_t handleStatusInternal(httpd_req_t* req);

    /**
     * @brief Handle GET /diag/coreinfo - return core dump partition info as JSON
     * Yields to allow LVGL task to run (prevents watchdog timeout)
     */
    esp_err_t handleCoreInfoInternal(httpd_req_t* req);
};

#endif // DIAGNOSTICS_HPP
