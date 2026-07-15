#include "Diagnostics.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <sstream>

#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/task.h"

DiagnosticsService* DiagnosticsService::instance_ = nullptr;

namespace {
std::string jsonEscape(const std::string& in)
{
    std::string out;
    out.reserve(in.size() + 8);

    for (char c : in) {
        switch (c) {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buff[7];
                std::snprintf(buff, sizeof(buff), "\\u%04x", c);
                out += buff;
            } else {
                out += c;
            }
            break;
        }
    }

    return out;
}

const char* resetReasonToString(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_UNKNOWN:
        return "unknown";
    case ESP_RST_POWERON:
        return "poweron";
    case ESP_RST_EXT:
        return "external";
    case ESP_RST_SW:
        return "software";
    case ESP_RST_PANIC:
        return "panic";
    case ESP_RST_INT_WDT:
        return "int_wdt";
    case ESP_RST_TASK_WDT:
        return "task_wdt";
    case ESP_RST_WDT:
        return "wdt";
    case ESP_RST_DEEPSLEEP:
        return "deepsleep";
    case ESP_RST_BROWNOUT:
        return "brownout";
    case ESP_RST_SDIO:
        return "sdio";
    default:
        return "other";
    }
}
} // namespace

DiagnosticsService::DiagnosticsService()
    : maxLines_(400)
    , maxLineLength_(192)
    , nextSeq_(1)
    , droppedCount_(0)
    , server_(nullptr)
    , previousLogSink_(nullptr)
{
}

void DiagnosticsService::init(size_t maxLines, size_t maxLineLength)
{
    // Ensure reasonable minimum buffer sizes
    maxLines_ = std::max<size_t>(50, maxLines);
    maxLineLength_ = std::max<size_t>(80, maxLineLength);
    instance_ = this;
    
    // Load previous reset reason from NVS and save current one
    loadLastResetReason();
    saveCurrentResetReason();
}

void DiagnosticsService::setAuthToken(const std::string& token)
{
    std::lock_guard<std::mutex> lock(serverMutex_);
    authToken_ = token;
}

void DiagnosticsService::loadLastResetReason()
{
    // Open NVS namespace for diagnostics
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open("diagnostics", NVS_READONLY, &nvsHandle);
    
    if (err != ESP_OK) {
        lastResetReason_ = "unknown";
        return;
    }

    // Read last reset reason from NVS (max 32 bytes)
    char reasonBuffer[32] = {0};
    size_t length = sizeof(reasonBuffer);
    err = nvs_get_str(nvsHandle, "last_reset", reasonBuffer, &length);
    
    if (err == ESP_OK) {
        lastResetReason_ = std::string(reasonBuffer);
    } else {
        lastResetReason_ = "unknown";
    }
    
    nvs_close(nvsHandle);
}

void DiagnosticsService::saveCurrentResetReason()
{
    // Get current reset reason
    esp_reset_reason_t reason = esp_reset_reason();
    const char* reasonStr = resetReasonToString(reason);
    
    // Open NVS namespace for diagnostics (read-write)
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open("diagnostics", NVS_READWRITE, &nvsHandle);
    
    if (err != ESP_OK) {
        return;
    }

    // Save current reset reason to NVS
    nvs_set_str(nvsHandle, "last_reset", reasonStr);
    nvs_commit(nvsHandle);
    nvs_close(nvsHandle);
}

void DiagnosticsService::installLogSink()
{
    if (instance_ == nullptr) {
        instance_ = this;
    }
    // Chain previous log sink so we can call it after buffering
    previousLogSink_ = esp_log_set_vprintf(&DiagnosticsService::logSink);
}

int DiagnosticsService::logSink(const char* fmt, va_list args)
{
    if (instance_ == nullptr) {
        return vprintf(fmt, args);
    }
    return instance_->logSinkInternal(fmt, args);
}

int DiagnosticsService::logSinkInternal(const char* fmt, va_list args)
{
    // Format log message into temporary buffer first
    char stackBuffer[256];

    va_list formatCopy;
    va_copy(formatCopy, args);
    int needed = std::vsnprintf(stackBuffer, sizeof(stackBuffer), fmt, formatCopy);
    va_end(formatCopy);

    // If message fits in stack buffer, use it directly; otherwise allocate on heap
    if (needed > 0) {
        if (static_cast<size_t>(needed) < sizeof(stackBuffer)) {
            appendLogLine(stackBuffer, static_cast<size_t>(needed));
        } else {
            std::string dynamicLine;
            dynamicLine.resize(static_cast<size_t>(needed));

            va_list longCopy;
            va_copy(longCopy, args);
            std::vsnprintf(dynamicLine.data(), dynamicLine.size() + 1, fmt, longCopy);
            va_end(longCopy);

            appendLogLine(dynamicLine.c_str(), dynamicLine.size());
        }
    }

    // Chain to previous sink (usually stdout)
    if (previousLogSink_ != nullptr) {
        return previousLogSink_(fmt, args);
    }

    return vprintf(fmt, args);
}

void DiagnosticsService::appendLogLine(const char* line, size_t lineLen)
{
    // Validate input
    if (line == nullptr || lineLen == 0) {
        return;
    }

    // Cap line length and create std::string
    size_t cappedLen = std::min(lineLen, maxLineLength_);
    std::string msg(line, cappedLen);
    
    // Remove trailing newlines for cleaner storage
    while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r')) {
        msg.pop_back();
    }

    if (msg.empty()) {
        return;  // Skip empty lines
    }

    // Create buffer item with timestamp
    BufferItem item;
    item.seq = nextSeq_.fetch_add(1);
    item.timestampUs = static_cast<uint64_t>(esp_timer_get_time());
    item.message = std::move(msg);

    // Add to circular buffer (thread-safe)
    std::lock_guard<std::mutex> lock(bufferMutex_);
    if (buffer_.size() >= maxLines_) {
        buffer_.pop_front();  // Drop oldest entry if full
        droppedCount_.fetch_add(1);
    }
    buffer_.push_back(std::move(item));
}

void DiagnosticsService::onWifiConnected()
{
    startServer();
}

void DiagnosticsService::onWifiDisconnected()
{
    stopServer();
}

bool DiagnosticsService::isServerRunning() const
{
    std::lock_guard<std::mutex> lock(serverMutex_);
    return server_ != nullptr;
}

uint64_t DiagnosticsService::droppedCount() const
{
    return droppedCount_.load();
}

size_t DiagnosticsService::bufferSize() const
{
    std::lock_guard<std::mutex> lock(bufferMutex_);
    return buffer_.size();
}

void DiagnosticsService::startServer()
{
    // Prevent concurrent server start attempts
    std::lock_guard<std::mutex> lock(serverMutex_);
    if (server_ != nullptr) {
        return;  // Already running
    }

    // Configure HTTP server with reasonable defaults
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;  // Support up to 8 URI handlers
    config.stack_size = 6144;     // Sufficient for JSON response building

    if (httpd_start(&server_, &config) != ESP_OK) {
        server_ = nullptr;
        ESP_LOGE(TAG, "Failed to start diagnostics HTTP server");
        return;
    }

    // Register /diag/logs endpoint (paginated log buffer)
    httpd_uri_t logsUri = {};
    logsUri.uri = "/diag/logs";
    logsUri.method = HTTP_GET;
    logsUri.handler = &DiagnosticsService::handleLogs;
    logsUri.user_ctx = this;

    // Register /diag/status endpoint (system status)
    httpd_uri_t statusUri = {};
    statusUri.uri = "/diag/status";
    statusUri.method = HTTP_GET;
    statusUri.handler = &DiagnosticsService::handleStatus;
    statusUri.user_ctx = this;

    // Register /diag/coreinfo endpoint (core dump info)
    httpd_uri_t coreUri = {};
    coreUri.uri = "/diag/coreinfo";
    coreUri.method = HTTP_GET;
    coreUri.handler = &DiagnosticsService::handleCoreInfo;
    coreUri.user_ctx = this;

    httpd_register_uri_handler(server_, &logsUri);
    httpd_register_uri_handler(server_, &statusUri);
    httpd_register_uri_handler(server_, &coreUri);

    ESP_LOGI(TAG, "Diagnostics HTTP server started");
}

void DiagnosticsService::stopServer()
{
    // Prevent concurrent server stop attempts
    std::lock_guard<std::mutex> lock(serverMutex_);
    if (server_ == nullptr) {
        return;  // Already stopped
    }

    httpd_stop(server_);
    server_ = nullptr;
    ESP_LOGI(TAG, "Diagnostics HTTP server stopped");
}

bool DiagnosticsService::isAuthorized(httpd_req_t* req) const
{
    if (authToken_.empty()) {
        return true;
    }

    size_t valueLen = httpd_req_get_hdr_value_len(req, "X-Diag-Token");
    if (valueLen == 0 || valueLen > 128) {
        return false;
    }

    std::string token;
    token.resize(valueLen + 1);
    if (httpd_req_get_hdr_value_str(req, "X-Diag-Token", token.data(), token.size()) != ESP_OK) {
        return false;
    }

    token.resize(valueLen);
    return token == authToken_;
}

bool DiagnosticsService::parseCursorAndLimit(httpd_req_t* req, uint64_t* cursor, size_t* limit) const
{
    *cursor = 0;
    *limit = 80;

    size_t queryLen = httpd_req_get_url_query_len(req);
    if (queryLen == 0 || queryLen >= 128) {
        return true;
    }

    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return true;
    }

    char cursorStr[24] = {0};
    if (httpd_query_key_value(query, "cursor", cursorStr, sizeof(cursorStr)) == ESP_OK) {
        *cursor = strtoull(cursorStr, nullptr, 10);
    }

    char limitStr[8] = {0};
    if (httpd_query_key_value(query, "limit", limitStr, sizeof(limitStr)) == ESP_OK) {
        size_t requestedLimit = static_cast<size_t>(strtoul(limitStr, nullptr, 10));
        if (requestedLimit > 0) {
            *limit = std::min<size_t>(requestedLimit, 200);
        }
    }

    return true;
}

std::vector<DiagnosticsService::LogSnapshotEntry>
DiagnosticsService::snapshotFrom(uint64_t cursor, size_t maxLines, uint64_t* nextCursor) const
{
    std::vector<LogSnapshotEntry> out;
    out.reserve(maxLines);

    uint64_t localNext = cursor;

    // Snapshot buffer under lock to avoid concurrent modification
    std::lock_guard<std::mutex> lock(bufferMutex_);
    for (const auto& item : buffer_) {
        // Skip entries up to and including cursor
        if (item.seq <= cursor) {
            continue;
        }

        out.push_back({item.seq, item.timestampUs, item.message});
        localNext = item.seq;

        if (out.size() >= maxLines) {
            break;
        }
    }

    *nextCursor = localNext;
    return out;
}

esp_err_t DiagnosticsService::handleLogs(httpd_req_t* req)
{
    // Static entry point - get instance and call instance method
    auto* self = static_cast<DiagnosticsService*>(req->user_ctx);
    return self->handleLogsInternal(req);
}

esp_err_t DiagnosticsService::handleStatus(httpd_req_t* req)
{
    // Static entry point - get instance and call instance method
    auto* self = static_cast<DiagnosticsService*>(req->user_ctx);
    return self->handleStatusInternal(req);
}

esp_err_t DiagnosticsService::handleCoreInfo(httpd_req_t* req)
{
    // Static entry point - get instance and call instance method
    auto* self = static_cast<DiagnosticsService*>(req->user_ctx);
    return self->handleCoreInfoInternal(req);
}

esp_err_t DiagnosticsService::handleLogsInternal(httpd_req_t* req)
{
    // Validate authentication token from X-Diag-Token header
    if (!isAuthorized(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        esp_err_t result = httpd_resp_send(req, "unauthorized", HTTPD_RESP_USE_STRLEN);
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to LVGL
        return result;
    }

    // Parse pagination parameters: ?cursor=N&limit=M
    uint64_t cursor = 0;
    size_t limit = 80;
    parseCursorAndLimit(req, &cursor, &limit);

    // Get snapshot of log entries from cursor position
    uint64_t nextCursor = cursor;
    auto rows = snapshotFrom(cursor, limit, &nextCursor);

    // Build JSON response body with log entries
    std::ostringstream body;
    body << "{\"cursor\":" << cursor
         << ",\"next_cursor\":" << nextCursor
         << ",\"count\":" << rows.size()
         << ",\"dropped\":" << droppedCount() << ",\"logs\":[";

    for (size_t i = 0; i < rows.size(); ++i) {
        const auto& row = rows[i];
        body << "{\"seq\":" << row.seq
             << ",\"ts_us\":" << row.timestampUs
             << ",\"msg\":\"" << jsonEscape(row.message) << "\"}";
        if (i + 1 < rows.size()) {
            body << ",";
        }
    }
    body << "]}";

    std::string out = body.str();

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_send(req, out.c_str(), out.size());
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to LVGL
    return result;
}

esp_err_t DiagnosticsService::handleStatusInternal(httpd_req_t* req)
{
    // Validate authentication token from X-Diag-Token header
    if (!isAuthorized(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        esp_err_t result = httpd_resp_send(req, "unauthorized", HTTPD_RESP_USE_STRLEN);
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to LVGL
        return result;
    }

    // Get current system metrics and retrieve last reset reason from member
    esp_reset_reason_t reason = esp_reset_reason();

    // Build JSON response with system status including both current and last reset reasons
    std::ostringstream body;
    body << "{\"uptime_us\":" << static_cast<uint64_t>(esp_timer_get_time())
         << ",\"reset_reason\":\"" << resetReasonToString(reason)
         << "\",\"last_reset_reason\":\"" << lastResetReason_
         << "\",\"buffered_logs\":" << bufferSize()
         << ",\"dropped_logs\":" << droppedCount()
         << ",\"server_running\":" << (isServerRunning() ? "true" : "false")
         << "}";

    std::string out = body.str();

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_send(req, out.c_str(), out.size());
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to LVGL
    return result;
}

esp_err_t DiagnosticsService::handleCoreInfoInternal(httpd_req_t* req)
{
    // Validate authentication token from X-Diag-Token header
    if (!isAuthorized(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        esp_err_t result = httpd_resp_send(req, "unauthorized", HTTPD_RESP_USE_STRLEN);
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to LVGL
        return result;
    }

    // Find core dump partition in flash
    const esp_partition_t* corePart = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_COREDUMP,
        nullptr);

    bool partitionFound = (corePart != nullptr);
    bool hasDataGuess = false;

    // Check if partition contains data (not erased to 0xFF)
    if (partitionFound) {
        uint8_t header[16] = {0};
        if (esp_partition_read(corePart, 0, header, sizeof(header)) == ESP_OK) {
            hasDataGuess = false;
            for (size_t i = 0; i < sizeof(header); ++i) {
                if (header[i] != 0xFF) {
                    hasDataGuess = true;
                    break;
                }
            }
        }
    }

    // Format core dump partition address as hex string
    char addressStr[16] = "0x0";
    if (partitionFound) {
        std::snprintf(addressStr, sizeof(addressStr), "0x%" PRIx32, corePart->address);
    }

    std::ostringstream body;
    body << "{\"coredump_partition_found\":" << (partitionFound ? "true" : "false")
         << ",\"coredump_has_data_guess\":" << (hasDataGuess ? "true" : "false")
         << ",\"partition_address\":\"" << addressStr << "\""
         << ",\"partition_size\":" << (partitionFound ? corePart->size : 0)
         << ",\"note\":\"Decode core dump with matching firmware ELF and espcoredump.py\"}";

    std::string out = body.str();

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_send(req, out.c_str(), out.size());
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to LVGL
    return result;
}
