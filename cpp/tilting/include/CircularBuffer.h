#ifndef H_CIRCBUFFER
#define H_CIRCBUFFER
/* A circular buffer template class implementation, which can store data of any type needed

code source: https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/circular_buffer.cpp
more info: https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/

usage instructions (copied from link above):

    To instantiate a circular buffer, we just declare an object and specify the templated type for our buffer. Here’s an example using a buffer of 10 uint32_t entries:
        CircularBuffer<uint32_t> buffer(10);

    Adding data is easy:
        uint32_t x = 100;
        buffer.put(x);

    And getting data is equally easy:
        x = buffer.get()

*/

#include <cstdio>
#include <memory>

template <class T>
class CircularBuffer {
public:
	explicit CircularBuffer(size_t size) :
		buf_(std::unique_ptr<T[]>(new T[size])),
		max_size_(size)
	{

	}

	void push(T item) {

		buf_[head_] = item;

		if(full_) {
			tail_ = (tail_ + 1) % max_size_;
		}

		head_ = (head_ + 1) % max_size_;

		full_ = head_ == tail_;
	}

	T pop()
	{
		if(empty())
		{
			return T();
		}

		//Read data and advance the tail (we now have a free space)
		auto val = buf_[tail_];
		full_ = false;
		tail_ = (tail_ + 1) % max_size_;

		return val;
	}

	//FUTURE: add offset as for peek
	void del()
	{
		if(empty())
		{
			return;
		}

		//advance the tail (we now have a free space)
		full_ = false;
		tail_ = (tail_ + 1) % max_size_;

		return;
	}

	//read the value at the tail, or offset from the tail without removing it from the buffer
	T peek(unsigned int offset=0)
	{
		//make sure buffer large enough to contain item that's offset
		if (size() <= offset){
			return T();
		}

		auto val = buf_[tail_ + offset];

		//do not advance the tail, makes no change to buffer state
		return val;
	}

	void reset()
	{
		// std::lock_guard<std::mutex> lock(mutex_);
		head_ = tail_;
		full_ = false;
	}

	bool empty() const
	{
		//if head and tail are equal, we are empty
		return (!full_ && (head_ == tail_));
	}

	bool full() const
	{
		//If tail is ahead the head by 1, we are full
		return full_;
	}

	size_t capacity() const
	{
		return max_size_;
	}

	size_t size() const
	{
		size_t size = max_size_;

		if(!full_)
		{
			if(head_ >= tail_)
			{
				size = head_ - tail_;
			}
			else
			{
				size = max_size_ + head_ - tail_;
			}
		}

		return size;
	}

private:
	// std::mutex mutex_;
	std::unique_ptr<T[]> buf_;
	size_t head_ = 0;
	size_t tail_ = 0;
	const size_t max_size_;
	bool full_ = 0;
};

#endif