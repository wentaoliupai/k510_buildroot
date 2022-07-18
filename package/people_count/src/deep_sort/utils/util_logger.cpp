//定义宏使输出文件名和行号
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#define SPDLOG_TRACE_ON

#include"util_logger.h"

/*
    使用示例:
    string path="logs/deep_sort.txt";
    int file_size=1024*1024,files_num=3;
    SINGLE_ROTATING_LOG_INIT(path,file_size,files_num);
    string str="msg";
    LOG_WITH_INFO_LEVEL("This is a test,{}",str);
*/
SingleRotatingLog *SingleRotatingLog::log_instance=nullptr;
SingleRotatingLog::AuxiliarySingleRelease SingleRotatingLog::log_release;

SingleRotatingLog::AuxiliarySingleRelease::~AuxiliarySingleRelease()
{
    if(log_instance)
        delete log_instance;
}

void SingleRotatingLog::log_init(string path,int file_size,int files_num,spdlog::level::level_enum level)
{
     //sink
    logger_ptr=spdlog::rotating_logger_st("deep sort",path,file_size,files_num);
    logger_ptr->set_level(level);
    //[%l] 日志级别 [%s] 文件 [%#] 行号 [%v] 实际文本
    logger_ptr->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");
    //设置当出发 err 或更严重的错误时立刻刷新日志到  disk
    logger_ptr->flush_on(spdlog::level::trace);
}

SingleRotatingLog* SingleRotatingLog::get_instance()
{
    if(log_instance==nullptr)
    {
        log_instance=new SingleRotatingLog;
    }
    return log_instance;
}

shared_ptr<spdlog::logger> SingleRotatingLog::logger()
{
    return logger_ptr;
}

 