#pragma once

#include <mutex>
#include <condition_variable>

class Semaphore
{
private:
    std::mutex m_lock;
    std::condition_variable m_cond;

    size_t m_count;

public:
    Semaphore(size_t count = 0): m_count(count) {}

    void Wait()
    {
        std::unique_lock<std::mutex> ulock(m_lock);

        while (m_count == 0)
            m_cond.wait(ulock);

        m_count--;
    }

    bool TryWait()
    {
        std::unique_lock<std::mutex> ulock(m_lock);

        if (m_count == 0)
            return false;

        m_count--;
        return true;
    }

    void Post()
    {
        std::unique_lock<std::mutex> ulock(m_lock);
        m_count++;
        m_cond.notify_one();
    }
};

