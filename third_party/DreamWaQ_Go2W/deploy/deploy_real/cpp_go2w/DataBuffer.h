/**
 * @file DataBuffer.h
 * @brief Thread-safe data buffer for LowState sharing
 */

#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H

#include <memory>
#include "AtomicLock.h"

// Thread-safe buffer using AFLock and shared_ptr
template<typename T>
class DataBuffer
{
	public:
		explicit DataBuffer() = default;
		~DataBuffer() = default;

		// Set data pointer directly
		void SetDataPtr(const std::shared_ptr<T>& dataPtr)
		{
			ScopedLock<AFLock> lock(mLock);
			mDataPtr = dataPtr;
		}

		// Get data pointer (recommended for zero-copy access)
		std::shared_ptr<T> GetDataPtr(bool clear = false)
		{
			ScopedLock<AFLock> lock(mLock);
			if (clear)
			{
				std::shared_ptr<T> dataPtr = mDataPtr;
				mDataPtr.reset();
				return dataPtr;
			}
			else
			{
				return mDataPtr;
			}
		}

		// Swap data pointer
		void SwapDataPtr(std::shared_ptr<T>& dataPtr)
		{
			ScopedLock<AFLock> lock(mLock);
			mDataPtr.swap(dataPtr);
		}

		// Store data by copy (use in DDS callback)
		void SetData(const T& data)
		{
			ScopedLock<AFLock> lock(mLock);
			mDataPtr = std::shared_ptr<T>(new T(data));
		}

		// Get data by copy
		bool GetData(T& data, bool clear = false)
		{
			ScopedLock<AFLock> lock(mLock);
			if (mDataPtr == NULL)
			{
				return false;
			}

			data = *mDataPtr;
			if (clear)
			{
				mDataPtr.reset();
			}

			return true;
		}

		// Clear buffer
		void Clear()
		{
			ScopedLock<AFLock> lock(mLock);
			mDataPtr.reset();
		}

	private:
		std::shared_ptr<T> mDataPtr;
		AFLock mLock;
};
#endif
