/**
 * @file logging.h
 * @brief Tesseract Logging
 *
 * Improved Tesseract logging inspired by Robot Raconteur log system
 * 
 * @author John Wason
 * @date August 14, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #include <tesseract_common/logging.h>

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iomanip>
#include <ctime>
#include <filesystem>
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

 namespace tesseract_common
 {
    std::ostream& operator<<(std::ostream& out, const TesseractLogRecord& record)
    {
        std::time_t t = std::chrono::system_clock::to_time_t(record.time);
        std::tm tm = *std::localtime(&t);
        out << "[" << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << "] [" << tesseractLogLevelToString(record.level) 
        << "," << record.error_ident 
        << "] ["
        << record.thread_id << "] [" 
        << record.component;
        if (!record.subcomponent.empty())
        {
            out << "," << record.subcomponent;
        }
        if (!record.object_instance.empty())
        {
            out << "," << record.object_instance;
        }
        out << "] ";

        if (!record.object_path.empty())
        {
            out << " [";
            // comman-separated list of object paths
            for (const auto& path : record.object_path)
            {
                out << path << ",";
            }
            // Remove the last comma
            if (!record.object_path.empty())
                out.seekp(-1, std::ios_base::end);
            out << "] ";
        }

        if (!record.source_file.empty())
        {
            out << "[" << std::filesystem::path(record.source_file).filename().string() << ":" << record.source_line
                << "] ";
        }

        out << record.message;
        return out;
    }

    std::string tesseractLogLevelToString(LogLevel level)
    {
        switch (level)
        {
        case LogLevel::Trace:
            return "trace";
        case LogLevel::Debug:
            return "debug";
        case LogLevel::Info:
            return "info";
        case LogLevel::Warning:
            return "warning";
        case LogLevel::Error:
            return "error";
        case LogLevel::Fatal:
            return "fatal";
        default:
            return "unknown";
        }
    }

    TesseractRecordStream::TesseractRecordStream(std::weak_ptr<LogManager> log_manager)
    {
        this->log_manager = log_manager;
        record.time = std::chrono::system_clock::now();
    }
    TesseractRecordStream::TesseractRecordStream(std::weak_ptr<LogManager> log_manager, LogLevel level, const std::string& error_ident,
                            const std::string& component, const std::string& subcomponent,
                            const std::string& object_instance, const std::vector<std::string>& object_path,
                            const std::string& source_file, uint32_t source_line, const std::string& thread_id)
        : record{level, error_ident, component, subcomponent, object_instance, object_path, {}, {}, source_file, source_line, thread_id}
    {
        record.time = std::chrono::system_clock::now();
        this->log_manager = log_manager;
    }
    TesseractRecordStream::~TesseractRecordStream()
    {
        record.message = ss.str();
        try
        {
            auto log_manager_ptr = log_manager.lock();
            if (!log_manager_ptr)
            {
                // If log manager is not available, we just ignore the record
                return;
            }
            log_manager_ptr->logRecord(record);
        }
        catch (std::exception& exp)
        {
            // If logging fails, we just ignore it
            // This is to prevent logging from throwing exceptions that could crash the application
            std::cerr << "Failed to log record: " << exp.what() << std::endl;
        }
    }
    std::stringstream& TesseractRecordStream::stream()
    {
        return ss;
    }

    boost::intrusive_ptr<TesseractRecordStream> openRecordStream(std::weak_ptr<LogManager> log_manager,
        LogLevel level, const std::string& error_ident, const std::string& component,
        const std::string& subcomponent, const std::string& object_instance,
        const std::vector<std::string>& object_path, const std::string& source_file,
        uint32_t source_line, const std::string& thread_id)
    {
        auto log_manager_ptr = log_manager.lock();
        if (!log_manager_ptr)
        {
            return nullptr;
        }

        if (!log_manager_ptr->compareLogLevel(level))
        {
            return nullptr;  // If the log level is not enabled, return nullptr
        }

        return new TesseractRecordStream(log_manager, level, error_ident, component, subcomponent,
                                         object_instance, object_path, source_file, source_line, thread_id);

    }

    void LogManager::logRecord(const tesseract_common::TesseractLogRecord& record)
    {

    }
    std::shared_ptr<LogManager> LogManager::getInstance()
    {

    }
    bool LogManager::compareLogLevel(LogLevel level) const
    {
        return true;
    }

    /** @brief Set the log level */
    void LogManager::setLogLevel(LogLevel level)
    {

    }

    /** @brief Get the current log level */
    LogLevel LogManager::getLogLevel() const
    {
        
    }
 }