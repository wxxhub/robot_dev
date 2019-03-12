#ifndef _TIMER_HPP_
#define _TIMER_HPP_
 
#include <iostream>
#include <ratio>
#include <chrono>
#include <thread>

class Timer
{
public:
  Timer() : m_begin(std::chrono::high_resolution_clock::now()) {}

  void reset();
  void stop();

  double elapsed();

  void show();
  void sleep(double ms);
  void wait(double ms);
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> m_begin;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_end;
};

void Timer::reset() 
{ 
  m_begin = std::chrono::high_resolution_clock::now();
}

void Timer::stop() 
{ 
  m_end = std::chrono::high_resolution_clock::now();
}

double Timer::elapsed() 
{
  return (std::chrono::duration_cast<std::chrono::duration<double>>(m_end - m_begin)).count()*1000;
}

void Timer::show() 
{
  std::cout << "Time elapsed: " << elapsed() << " ms" << std::endl;
}

void Timer::sleep(double ms) 
{
  std::this_thread::sleep_for(std::chrono::duration<double>(ms/1000));
}

void Timer::wait(double ms)
{
  stop(); sleep(ms - elapsed()); stop();
}

#endif //_TIMER_HPP_