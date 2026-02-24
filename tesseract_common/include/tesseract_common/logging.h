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
#ifndef TESSERACT_LOGGING_H
#define TESSERACT_LOGGING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <chrono>
#include <unordered_map>
#include <vector>
#include <any>
#include <sstream>
#include <memory>
#include <boost/intrusive_ptr.hpp>
#include <boost/smart_ptr/intrusive_ref_counter.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
    /**
     * @brief Available log levels for Tesseract
     * 
     */
    enum class LogLevel
    {
        /** @brief `trace` log level */
        Trace,
        /** @brief `debug` log level */
        Debug,
        /** @brief `info` log level */
        Info,
        /** @brief `warning` log level */
        Warning,
        /** @brief `error` log level */
        Error,
        /** @brief `fatal` log level */
        Fatal,
        /** @brief `disabled` log level */
        Disable = 1000
    };

    class LogManager;


    /**
     * @brief Tesseract log record
     * 
     * Records information about a logging event
     * 
     */
    struct TesseractLogRecord
    {
        /** The log record error level */
        LogLevel level{LogLevel::Warning};
        /** An identifier for an error. Either a gcc style "diagnostic ID" or a UUID in hex string  */
        std::string error_ident;
        /** The component that generated the message (typically package name or namespace) */
        std::string component;
        /** The subcomponent that generate the message (typically class name) */
        std::string subcomponent;
        /** The instance ID of the object that generated the message (pointer or UUID)*/
        std::string object_instance;
        /** The path of the object. Use string UUIDs of taskflow nodes */
        std::vector<std::string> object_path;
        /** Human readable log message */
        std::string message;
        /** Time of logging event */
        std::chrono::system_clock::time_point time;
        /** The sourcecode filename */
        std::string source_file;
        /** The line within the sourcecode file */
        uint32_t source_line{0};
        /** The source thread */
        std::string thread_id;
        /** Machine readable parameters attached to event */
        std::unordered_map<std::string, std::any> parameters;
    };

    /** Write a TesseractLogRecord to stream */
    std::ostream& operator<<(std::ostream& out, const TesseractLogRecord& record);

    /** Convert a LogLevel to string  */
    std::string tesseractLogLevelToString(LogLevel level);

    
    class TesseractRecordStream : public boost::intrusive_ref_counter<TesseractRecordStream>
    {
        protected:
            TesseractLogRecord record;
            std::stringstream ss;
            std::weak_ptr<LogManager> log_manager;

        public:
            TesseractRecordStream(std::weak_ptr<LogManager> log_manager);
            TesseractRecordStream(std::weak_ptr<LogManager> log_manager, LogLevel level, const std::string& error_ident,
                                  const std::string& component, const std::string& subcomponent,
                                  const std::string& object_instance, const std::vector<std::string>& object_path,
                                  const std::string& source_file, uint32_t source_line, const std::string& thread_id);
            ~TesseractRecordStream();
            std::stringstream& stream();

            static boost::intrusive_ptr<TesseractRecordStream> openRecordStream(std::weak_ptr<LogManager> log_manager,
                LogLevel level, const std::string& error_ident, const std::string& component,
                const std::string& subcomponent, const std::string& object_instance,
                const std::vector<std::string>& object_path, const std::string& source_file,
                uint32_t source_line, const std::string& thread_id="-");
    };

    class LogRecordHandler
    {
        public:
            virtual ~LogRecordHandler() = default;

            virtual void handleRecord(const TesseractLogRecord& record) = 0;
    };

    class LogManager
    {
        public:
            LogManager() = default;
            /** @brief Log a record */
            void logRecord(const tesseract_common::TesseractLogRecord& record);
            /** @brief Get the default log manager instance */
            static std::shared_ptr<LogManager> getInstance();
            /** @brief Compare the log level with the current log level */
            bool compareLogLevel(LogLevel level) const;

            /** @brief Set the log level */
            void setLogLevel(LogLevel level);

            /** @brief Get the current log level */
            LogLevel getLogLevel() const;

            /** @brief Add a log record handler */
            void addLogRecordHandler(std::shared_ptr<LogRecordHandler> handler);

            /** @brief Remove a log record handler */
            void removeLogRecordHandler(std::shared_ptr<LogRecordHandler> handler);

            /** @brief Remove all log record handlers */
            void clearLogRecordHandlers();

        protected:
            std::atomic<LogLevel> current_log_level{LogLevel::Warning};
            boost::shared_mutex config_mutex;
            std::vector<std::shared_ptr<LogRecordHandler>> handlers;
    };

    /*
    #define ROBOTRACONTEUR_LOG(node, lvl, component, component_name, component_object_id, ep, service_path, member, args)  \
    {                                                                                                                  \
        boost::intrusive_ptr<RobotRaconteur::RRLogRecordStream> ____rr_log_record_stream____ =                         \
            RobotRaconteur::RRLogRecordStream::OpenRecordStream(                                                       \
                node, RobotRaconteur::RobotRaconteur_LogLevel_##lvl,                                                   \
                RobotRaconteur::RobotRaconteur_LogComponent_##component, component_name, component_object_id, ep,      \
                service_path, member, __FILE__, __LINE__);                                                             \
        if (____rr_log_record_stream____)                                                                              \
        {                                                                                                              \
            ____rr_log_record_stream____->Stream() << args;                                                            \
        }                                                                                                              \
    }
    */

    #define TESSERACT_LOG(log_manager, lvl, error_ident, component, subcomponent, object_instance, object_path, args)  \
    {                                                                                                                  \
        boost::intrusive_ptr<TesseractRecordStream> ____tesseract_log_record_stream____ =                              \
            TesseractRecordStream::openRecordStream(log_manager, lvl, error_ident, component, subcomponent,            \
                object_instance, object_path,  __FILE__, __LINE__);                                                    \
        if (____tesseract_log_record_stream____)                                                                       \
        {                                                                                                              \
            ____tesseract_log_record_stream____->stream() << args;                                                     \
        }                                                                                                              \
    }
    
    // TODO: Fill in various macros, see https://github.com/robotraconteur/robotraconteur/blob/master/RobotRaconteurCore/include/RobotRaconteur/Logging.h
    // for examples
}

#endif
