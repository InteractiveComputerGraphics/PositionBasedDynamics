//
// cuda_call_check.h
// cuda texture tester 
//
// created by ruanjm on 12/03/15
// Copyright (c) 2015 ruanjm. All rights reserved.
//

#ifndef _CUDA_CALL_CHECK_H
#define _CUDA_CALL_CHECK_H

#include <hip/hip_runtime.h>
#include <iostream>

#define CUDA_SAFE_CALL(err)     cuda_safe_call_(err, __FILE__, __LINE__)
#define CUDA_KERNEL_CHECK(err)  cuda_kernel_check_(err, __FILE__, __LINE__)

inline void cuda_safe_call_(hipError_t err, const char *file_name, const int num_line)
{
    if (hipSuccess != err)
    {
        std::cerr << "An error encountered: \"" << hipGetErrorString(err) << "\" at " \
        << *file_name << ':' << num_line << std::endl;
        exit(0);
    }
}

inline void cuda_kernel_check_(const char *error_msg, const char *file_name, const int num_line)
{
    hipError_t err = hipDeviceSynchronize();
    if (hipSuccess != err)
    {
        std::cerr << file_name << "[" << num_line << "]: "
            << (error_msg == nullptr ? "NONE" : error_msg)
            << "[" << (int)err << "]: "
            << hipGetErrorString(err) << std::endl;
        exit(0);
    }
}

#endif/*_CUDA_CALL_CHECK_H*/