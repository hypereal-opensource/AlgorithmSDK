#pragma once
#include <mutex>
#include <condition_variable>

/**
 * A thread-safe flag for multiple threads to wait for some condition.
 * References:
 *     http://en.cppreference.com/w/cpp/thread/condition_variable
 *     http://stackoverflow.com/questions/14920725/waiting-for-an-atomic-bool
 */
class SafeFlag
{
	mutable std::mutex mtx;
	mutable std::condition_variable cv;
	bool flag;

public:
	SafeFlag()
		: flag(false)
	{}

	bool is_set() const
	{
		std::lock_guard<std::mutex> lock(mtx);
		return flag;
	}

	void set()
	{
		{
			std::lock_guard<std::mutex> lock(mtx);
			flag = true;
		}
		cv.notify_all();
	}

	void reset()
	{
		{
			std::lock_guard<std::mutex> lock(mtx);
			flag = false;
		}
		cv.notify_all();
	}

	// Block until flag is set.
	void wait() const
	{
		std::unique_lock<std::mutex> lock(mtx);
		cv.wait(lock, [this] { return flag; });
	}

	template <typename Rep, typename Period>
	bool wait_for(const std::chrono::duration<Rep, Period>& rel_time) const
	{
		std::unique_lock<std::mutex> lock(mtx);
		return cv.wait_for(lock, rel_time, [this] { return flag; });
	}

	template <typename Rep, typename Period>
	bool wait_until(const std::chrono::duration<Rep, Period>& rel_time) const
	{
		std::unique_lock<std::mutex> lock(mtx);
		return cv.wait_until(lock, rel_time, [this] { return flag; });
	}
};
