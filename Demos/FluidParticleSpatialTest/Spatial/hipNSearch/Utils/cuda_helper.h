#pragma once
#include <thrust/device_vector.h>

class CUDAException : public std::runtime_error
{
public:
	CUDAException(const char *_const_Message);
};

class CUDAMallocException : public std::runtime_error
{
public:
	CUDAMallocException(const char *_const_Message);
};

class CUDAMemCopyException : public std::runtime_error
{
public:
	CUDAMemCopyException(const char *const_Message);
};

/*static*/ class CudaHelper
{
public:
	/** Synchronizes the device work with the current thread and throws any errors as exception.
	*/
	static void DeviceSynchronize();


	/** Throws the last error as exception.
	*/
	static void CheckLastError();

	static void GetThreadBlocks(unsigned int numberOfElements, unsigned int alignment, /*out*/ unsigned int &numberOfThreadBlocks, /*out*/ unsigned int &numberOfThreads);


	/** Gets a raw pointer from a thrust vector
	*/
	template<typename T>
	static T* GetPointer(thrust::device_vector<T> &vector)
	{
		return thrust::raw_pointer_cast(&vector[0]);
	}

	/** Gets the size of the device_vector data in bytes.
	*/
	template<typename T>
	static size_t GetSizeInBytes(const thrust::device_vector<T> &vector)
	{
		return sizeof(T) * vector.size();
	}

	/** Copies data from host to device.
	*/
	static void MemcpyHostToDevice(void* host, void* device, size_t size);

	/** Copies data from host to device.
	*/
	template<typename T>
	static void MemcpyHostToDevice(T* host, T* device, size_t elements)
	{
		MemcpyHostToDevice((void*)host, (void*)device, elements * sizeof(T));
	}

	/** Copies data from device to host.
	*/
	static void MemcpyDeviceToHost(void* device, void* host, size_t size);

	/** Copies data from device to host.
	*/
	template<typename T>
	static void MemcpyDeviceToHost(T* device, T* host, size_t elements)
	{
		MemcpyDeviceToHost((void*)device, (void*)host, elements * sizeof(T));
	}

};