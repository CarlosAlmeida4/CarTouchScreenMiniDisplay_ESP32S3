#include "Diagnostics.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <sstream>

#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"

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
    maxLines_ = std::max<size_t>(50, maxLines);
    maxLineLength_ = std::max<size_t>(80, maxLineLength);
    instance_ = this;
}

void DiagnosticsService::setAuthToken(const std::string& token)
{
    std::lock_guard<std::mutex> lock(serverMutex_);
    authToken_ = token;
}

void DiagnosticsService::installLogSink()
{
    if (instance_ == nullptr) {
        instance_ = this;
    }
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
    char stackBuffer[256];

    va_list formatCopy;
    va_copy(formatCopy, args);
    int needed = std::vsnprintf(stackBuffer, sizeof(stackBuffer), fmt, formatCopy);
    va_end(formatCopy);

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

    if (previousLogSink_ != nullptr) {
        return previousLogSink_(fmt, args);
    }

    return vprintf(fmt, args);
}

void DiagnosticsService::appendLogLine(const char* line, size_t lineLen)
{
    if (line == nullptr || lineLen == 0) {
        return;
    }

    size_t cappedLen = std::min(lineLen, maxLineLength_);

    std::string msg(line, cappedLen);
    while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r')) {
        msg.pop_back();
    }

    if (msg.empty()) {
        return;
    }

    BufferItem item;
    item.seq = nextSeq_.fetch_add(1);
    item.timestampUs = static_cast<uint64_t>(esp_timer_get_time());
    item.message = std::move(msg);

    std::lock_guard<std::mutex> lock(bufferMutex_);
    if (buffer_.size() >= maxLines_) {
        buffer_.pop_front();
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
    std::lock_guard<std::mutex> lock(serverMutex_);
    if (server_ != nullptr) {
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.stack_size = 6144;

    if (httpd_start(&server_, &config) != ESP_OK) {
        server_ = nullptr;
        ESP_LOGE(TAG, "Failed to start diagnostics HTTP server");
        return;
    }

    httpd_uri_t logsUri = {};
    logsUri.uri = "/diag/logs";
    logsUri.method = HTTP_GET;
    logsUri.handler = &DiagnosticsService::handleLogs;
    logsUri.user_ctx = this;

    httpd_uri_t statusUri = {};
    statusUri.uri = "/diag/status";
    statusUri.method = HTTP_GET;
    statusUri.handler = &DiagnosticsService::handleStatus;
    statusUri.user_ctx = this;

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
    std::lock_guard<std::mutex> lock(serverMutex_);
    if (server_ == nullptr) {
        return;
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

    std::lock_guard<std::mutex> lock(bufferMutex_);
    for (const auto& item : buffer_) {
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
    auto* self = static_cast<DiagnosticsService*>(req->user_ctx);
    return self->handleLogsInternal(req);
}

esp_err_t DiagnosticsService::handleStatus(httpd_req_t* req)
{
    auto* self = static_cast<DiagnosticsService*>(req->user_ctx);
    return self->handleStatusInternal(req);
}

esp_err_t DiagnosticsService::handleCoreInfo(httpd_req_t* req)
{
    auto* self = static_cast<DiagnosticsService*>(req->user_ctx);
    return self->handleCoreInfoInternal(req);
}

esp_err_t DiagnosticsService::handleLogsInternal(httpd_req_t* req)
{
    if (!isAuthorized(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        return httpd_resp_send(req, "unauthorized", HTTPD_RESP_USE_STRLEN);
    }

    uint64_t cursor = 0;
    size_t limit = 80;
    parseCursorAndLimit(req, &cursor, &limit);

    uint64_t nextCursor = cursor;
    auto rows = snapshotFrom(cursor, limit, &nextCursor);

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
    return httpd_resp_send(req, out.c_str(), out.size());
}

esp_err_t DiagnosticsService::handleStatusInternal(httpd_req_t* req)
{
    if (!isAuthorized(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        return httpd_resp_send(req, "unauthorized", HTTPD_RESP_USE_STRLEN);
    }

    esp_reset_reason_t reason = esp_reset_reason();

    std::ostringstream body;
    body << "{\"uptime_us\":" << static_cast<uint64_t>(esp_timer_get_time())
         << ",\"reset_reason\":\"" << resetReasonToString(reason)
         << "\",\"buffered_logs\":" << bufferSize()
         << ",\"dropped_logs\":" << droppedCount()
         << ",\"server_running\":" << (isServerRunning() ? "true" : "false")
         << "}";

    std::string out = body.str();

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out.c_str(), out.size());
}

esp_err_t DiagnosticsService::handleCoreInfoInternal(httpd_req_t* req)
{
    if (!isAuthorized(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        return httpd_resp_send(req, "unauthorized", HTTPD_RESP_USE_STRLEN);
    }

    const esp_partition_t* corePart = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_COREDUMP,
        nullptr);

    bool partitionFound = (corePart != nullptr);
    bool hasDataGuess = false;

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

    std::ostringstream body;
    body << "{\"coredump_partition_found\":" << (partitionFound ? "true" : "false")
         << ",\"coredump_has_data_guess\":" << (hasDataGuess ? "true" : "false")
         << ",\"partition_size\":" << (partitionFound ? corePart->size : 0)
         << ",\"note\":\"Decode core dump with matching firmware ELF and espcoredump.py\"}";

    std::string out = body.str();

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out.c_str(), out.size());
}
