#pragma once

#include "multithreading/thread_safe_buffer.h"

// TODO: Rename to "Subscriber" (and rename "Observable" to "Publisher"?)

// TODO: TEST THIS CLASS
/**
 * This class observes an "Observable<T>". That is, it can be registered with an
 * "Observable<T>" to receive new instances of type T when they are available
 *
 * @tparam T The type of object this class is observing
 */
template <typename T>
class Observer
{
   public:
    Observer();

    /**
     * Add the given value to the internal buffer
     *
     * @param val The value to add to th internal buffer
     */
    virtual void receiveValue(T val) final;

    virtual ~Observer() = default;

   protected:
    /**
     * Get the most recent value from the buffer
     *
     * If no value is available, this will block until a value is available.
     *
     * @return The value most recently added to the buffer
     */
    virtual T getMostRecentValueFromBuffer() final;

   private:
    const size_t DEFAULT_BUFFER_SIZE = 1;

    ThreadSafeBuffer<T> buffer;
};

// TODO: all this implementation can probably be in a `.cpp` file...same for Observable

template <typename T>
Observer<T>::Observer() : buffer(DEFAULT_BUFFER_SIZE)
{
}

template <typename T>
void Observer<T>::receiveValue(T val)
{
    buffer.push(std::move(val));
}

template <typename T>
T Observer<T>::getMostRecentValueFromBuffer()
{
    return std::move(buffer.pullMostRecentlyAddedValue());
}
