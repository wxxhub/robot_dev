#ifndef ROBOTIS_FRAMEWORK_COMMON_SINGLETON_H_
#define ROBOTIS_FRAMEWORK_COMMON_SINGLETON_H_

namespace robotis_framework
{

template <class T>
class Singleton
{
private:
  static T *unique_instance_;

protected:
  Singleton() { }
  Singleton(Singleton const&) { }
  Singleton& operator=(Singleton const&) { return *this; }

public:
  static T* getInstance()
  {
    if (unique_instance_ == NULL)
      unique_instance_ = new T;
    return unique_instance_;
  }

  static void destoryInstance()
  {
    if (unique_instance_)
    {
      delete unique_instance_;
      unique_instance_ = NULL;
    }
  }

};

template <class T> T* Singleton<T>::unique_instance_ = NULL;

}


#endif /* ROBOTIS_FRAMEWORK_COMMON_SINGLETON_H_ */
