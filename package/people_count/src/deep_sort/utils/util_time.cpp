#include"util_time.h"
#include"util_logger.h"

void UtilTime::set_start_time()
{
    start_time=steady_clock::now();
}

void UtilTime::set_end_time()
{
    end_time=steady_clock::now();
}

void UtilTime::show_time_span(string oper)
{
    auto span=duration_cast<microseconds>(end_time-start_time);
    //cout<<oper<<"用时:"<<double(span.count())*microseconds::period::num  / microseconds::period::den<<"s"<<endl;
    LOG_WITH_INFO_LEVEL("\"{}\" elapsed time {} ms",oper,double(span.count())  / 1000);
}