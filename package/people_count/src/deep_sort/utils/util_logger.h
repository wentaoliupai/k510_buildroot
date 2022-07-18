#ifndef UTIL_LOGGER_H
#define UTIL_LOGGER_H

#include<string>
#include<thread>
#include"spdlog/spdlog.h"           //v1版本
#include"spdlog/sinks/rotating_file_sink.h"

using std::string;
using std::shared_ptr;

class SingleRotatingLog
{
private:
    SingleRotatingLog()=default;
    class AuxiliarySingleRelease
    {
        public:
        ~AuxiliarySingleRelease();
    };
public:
    void log_init(string path="logs/deep_sort.txt",int file_size=1024*1024,int files_num=3,spdlog::level::level_enum level=spdlog::level::info);
    static SingleRotatingLog* get_instance();
    shared_ptr<spdlog::logger> logger();   //应为#define中用到了，预编译的时候需要知道它的实际返回类型，不能是auto

private:
    shared_ptr<spdlog::logger> logger_ptr;
public:
    static SingleRotatingLog *log_instance;
    static AuxiliarySingleRelease log_release;              //程序结束时自动释放
};

#define SINGLE_ROTATING_LOG_INIT(path,file_size,files_num,level) SingleRotatingLog::get_instance()->log_init(path,file_size,files_num,level);
#define SINGLE_RATATING_LOG_WITH_FILE_LINE(logger,level,...) \
                (logger)->log(spdlog::source_loc{__FILE__,__LINE__,__func__},level,__VA_ARGS__)

#define LOG_WITH_TRACE_LEVEL(...) \
                SINGLE_RATATING_LOG_WITH_FILE_LINE(SingleRotatingLog::get_instance()->logger(), \
                spdlog::level::trace,__VA_ARGS__)

#define LOG_WITH_DEBUG_LEVEL(...) \
                SINGLE_RATATING_LOG_WITH_FILE_LINE(SingleRotatingLog::get_instance()->logger(), \
                spdlog::level::debug,__VA_ARGS__)

#define LOG_WITH_INFO_LEVEL(...) \
                SINGLE_RATATING_LOG_WITH_FILE_LINE(SingleRotatingLog::get_instance()->logger(), \
                spdlog::level::info,__VA_ARGS__)

#define LOG_WITH_WARN_LEVEL(...) \
                SINGLE_RATATING_LOG_WITH_FILE_LINE(SingleRotatingLog::get_instance()->logger(), \
                spdlog::level::warn,__VA_ARGS__)    
 
 #define LOG_WITH_ERROR_LEVEL(...) \
                SINGLE_RATATING_LOG_WITH_FILE_LINE(SingleRotatingLog::get_instance()->logger(), \
                spdlog::level::err,__VA_ARGS__)      

 #define LOG_CRITICAL_ERROR_LEVEL(...) \
                SINGLE_RATATING_LOG_WITH_FILE_LINE(SingleRotatingLog::get_instance()->logger(), \
                spdlog::level::critical,__VA_ARGS__)        
#endif