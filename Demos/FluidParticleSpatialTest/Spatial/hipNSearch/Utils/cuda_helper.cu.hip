#include "cuda_helper.h"
#include <hip/hip_runtime.h>

CUDAException::CUDAException(const char *_const_Message) : std::runtime_error(_const_Message)
{

}

CUDAMallocException::CUDAMallocException(const char *_const_Message) : std::runtime_error(_const_Message)
{

}

CUDAMemCopyException::CUDAMemCopyException(const char *_const_Message) : std::runtime_error(_const_Message)
{

}

void CudaHelper::DeviceSynchronize()
{
	hipError_t cudaStatus = hipDeviceSynchronize();
	if (cudaStatus != hipSuccess)
	{
		auto temp = hipGetErrorString(cudaStatus);
		throw CUDAException(temp);
	}
}

void CudaHelper::GetThreadBlocks(unsigned int numberOfElements, unsigned int alignment, /*out*/ unsigned int &numberOfThreadBlocks, /*out*/ unsigned int &numberOfThreads)
{
	numberOfThreads = (numberOfElements / alignment) * alignment;
	numberOfThreadBlocks = (numberOfElements / alignment);
	if (numberOfElements % alignment != 0)
	{
		numberOfThreads += alignment;
		numberOfThreadBlocks++;
	}
}

void CudaHelper::MemcpyHostToDevice(void* host, void* device, size_t size)
{
	hipError_t cudaStatus = hipMemcpy(device, host, size, hipMemcpyHostToDevice);
	if (cudaStatus != hipSuccess)
	{
		throw CUDAMemCopyException("cudaMemcpy() failed!");
	}
}

void CudaHelper::MemcpyDeviceToHost(void* device, void* host, size_t size)
{
	hipError_t cudaStatus = hipMemcpy(host, device, size, hipMemcpyDeviceToHost);
	if (cudaStatus != hipSuccess)
	{
		throw CUDAMemCopyException("cudaMemcpy() failed!");
	}
}

void CudaHelper::CheckLastError()
{
	hipError_t cudaStatus = hipGetLastError();
	if (cudaStatus != hipSuccess)
	{
		auto temp = hipGetErrorString(cudaStatus);
		throw CUDAException(temp);
	}
}