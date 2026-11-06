/**
 * @file AtomicLock.h
 * @brief Atomic spinlock for thread synchronization
 */

#ifndef ATOMIC_LOCK_H
#define ATOMIC_LOCK_H

#include <atomic>

// Lightweight spinlock using std::atomic_flag
class AFLock
{
	public:
		AFLock() = default;
		~AFLock() = default;

		// Acquire lock (blocking)
		void Lock()
		{
			while (mAFL.test_and_set(std::memory_order_acquire));
		}

		// Try to acquire lock (non-blocking)
		bool TryLock()
		{
			return mAFL.test_and_set(std::memory_order_acquire) ? false : true;
		}

		// Release lock
		void Unlock()
		{
			mAFL.clear(std::memory_order_release);
		}

	private:
		std::atomic_flag mAFL = ATOMIC_FLAG_INIT;
};

// RAII scoped lock wrapper
template<typename L>
class ScopedLock
{
	public:
		explicit ScopedLock(L& lock) :
			mLock(lock)
	{
		mLock.Lock();
	}

		~ScopedLock()
		{
			mLock.Unlock();
		}

	private:
		L& mLock;
};
#endif
