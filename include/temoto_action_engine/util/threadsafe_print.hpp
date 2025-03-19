#ifndef TEMOTO_ACTION_ENGINE__THREADSAFE_PRINT_H
#define TEMOTO_ACTION_ENGINE__THREADSAFE_PRINT_H

#include <iostream>
#include <sstream>
#include <mutex>

class PrintThread: public std::ostringstream
{
public:
  PrintThread() = default;

  ~PrintThread()
  {
    //std::lock_guard<std::mutex> guard(_mutexPrint);
    std::cout << this->str();
  }

private:
  //static std::mutex _mutexPrint;
};

//std::mutex PrintThread::_mutexPrint{};

#endif