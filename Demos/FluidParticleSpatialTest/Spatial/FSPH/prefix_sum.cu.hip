/*
 * Copyright 1993-2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and 
 * proprietary rights in and to this software and related documentation and 
 * any modifications thereto.  Any use, reproduction, disclosure, or distribution 
 * of this software and related documentation without an express license 
 * agreement from NVIDIA Corporation is strictly prohibited.
 * 
 */

#ifndef _SCAN_BEST_KERNEL_CU_
#define _SCAN_BEST_KERNEL_CU_

// Define this to more rigorously avoid bank conflicts, 
// even at the lower (root) levels of the tree
// Note that due to the higher addressing overhead, performance 
// is lower with ZERO_BANK_CONFLICTS enabled.  It is provided
// as an example.
//#define ZERO_BANK_CONFLICTS 

#define LOG_NUM_BANKS	 4

#ifdef ZERO_BANK_CONFLICTS
#define CONFLICT_FREE_OFFSET(index) ((index) >> LOG_NUM_BANKS + (index) >> (2*LOG_NUM_BANKS))
#else
#define CONFLICT_FREE_OFFSET(index) ((index) >> LOG_NUM_BANKS)
#endif

#include <hip/hip_runtime.h>

///////////////////////////////////////////////////////////////////////////////
// Work-efficient compute implementation of scan, one thread per 2 elements
// Work-efficient: O(log(n)) steps, and O(n) adds.
// Also shared storage efficient: Uses n + n/NUM_BANKS shared memory -- no ping-ponging
// Also avoids most bank conflicts using single-element offsets every NUM_BANKS elements.
//
// In addition, If ZERO_BANK_CONFLICTS is defined, uses 
//     n + n/NUM_BANKS + n/(NUM_BANKS*NUM_BANKS) 
// shared memory. If ZERO_BANK_CONFLICTS is defined, avoids ALL bank conflicts using 
// single-element offsets every NUM_BANKS elements, plus additional single-element offsets 
// after every NUM_BANKS^2 elements.
//
// Uses a balanced tree type algorithm.  See Blelloch, 1990 "Prefix Sums 
// and Their Applications", or Prins and Chatterjee PRAM course notes:
// https://www.cs.unc.edu/~prins/Classes/633/Handouts/pram.pdf
// 
// This work-efficient version is based on the algorithm presented in Guy Blelloch's
// excellent paper "Prefix sums and their applications".
// http://www.cs.cmu.edu/~blelloch/papers/Ble93.pdf
//
// Pro: Work Efficient, very few bank conflicts (or zero if ZERO_BANK_CONFLICTS is defined)
// Con: More instructions to compute bank-conflict-free shared memory addressing,
// and slightly more shared memory storage used.
//

template <bool isNP2> __device__ void loadSharedChunkFromMem (float *s_data, const float *g_idata, int n, int baseIndex, int& ai, int& bi, int& mem_ai, int& mem_bi, int& bankOffsetA, int& bankOffsetB )
{
    int thid = threadIdx.x;
    mem_ai = baseIndex + threadIdx.x;
    mem_bi = mem_ai + blockDim.x;

    ai = thid;
    bi = thid + blockDim.x;
    bankOffsetA = CONFLICT_FREE_OFFSET(ai);		    // compute spacing to avoid bank conflicts
    bankOffsetB = CONFLICT_FREE_OFFSET(bi);
    
	s_data[ai + bankOffsetA] = g_idata[mem_ai];		// Cache the computational window in shared memory pad values beyond n with zeros
    
    if (isNP2) { // compile-time decision
        s_data[bi + bankOffsetB] = (bi < n) ? g_idata[mem_bi] : 0; 
    } else {
        s_data[bi + bankOffsetB] = g_idata[mem_bi]; 
    }
}


template <bool isNP2> __device__ void loadSharedChunkFromMemInt (int *s_data, const int *g_idata, int n, int baseIndex, int& ai, int& bi, int& mem_ai, int& mem_bi, int& bankOffsetA, int& bankOffsetB )
{
    int thid = threadIdx.x;
    mem_ai = baseIndex + threadIdx.x;
    mem_bi = mem_ai + blockDim.x;

    ai = thid;
    bi = thid + blockDim.x;
    bankOffsetA = CONFLICT_FREE_OFFSET(ai);		    // compute spacing to avoid bank conflicts
    bankOffsetB = CONFLICT_FREE_OFFSET(bi);
    
	s_data[ai + bankOffsetA] = g_idata[mem_ai];		// Cache the computational window in shared memory pad values beyond n with zeros
    
    if (isNP2) { // compile-time decision
        s_data[bi + bankOffsetB] = (bi < n) ? g_idata[mem_bi] : 0; 
    } else {
        s_data[bi + bankOffsetB] = g_idata[mem_bi]; 
    }
}

template <bool isNP2> __device__ void storeSharedChunkToMem(float* g_odata, const float* s_data, int n, int ai, int bi, int mem_ai, int mem_bi,int bankOffsetA, int bankOffsetB)
{
    __syncthreads();

    g_odata[mem_ai] = s_data[ai + bankOffsetA];			// write results to global memory
    if (isNP2) { // compile-time decision
        if (bi < n) g_odata[mem_bi] = s_data[bi + bankOffsetB]; 
    } else {
        g_odata[mem_bi] = s_data[bi + bankOffsetB]; 
    }
}
template <bool isNP2> __device__ void storeSharedChunkToMemInt (int* g_odata, const int* s_data, int n, int ai, int bi, int mem_ai, int mem_bi,int bankOffsetA, int bankOffsetB)
{
    __syncthreads();

    g_odata[mem_ai] = s_data[ai + bankOffsetA];			// write results to global memory
    if (isNP2) { // compile-time decision
        if (bi < n) g_odata[mem_bi] = s_data[bi + bankOffsetB]; 
    } else {
        g_odata[mem_bi] = s_data[bi + bankOffsetB]; 
    }
}


template <bool storeSum> __device__ void clearLastElement( float* s_data, float *g_blockSums, int blockIndex)
{
    if (threadIdx.x == 0) {
        int index = (blockDim.x << 1) - 1;
        index += CONFLICT_FREE_OFFSET(index);        
        if (storeSum) { // compile-time decision
            // write this block's total sum to the corresponding index in the blockSums array
            g_blockSums[blockIndex] = s_data[index];
        }
        s_data[index] = 0;		// zero the last element in the scan so it will propagate back to the front
    }
}

template <bool storeSum> __device__ void clearLastElementInt ( int* s_data, int *g_blockSums, int blockIndex)
{
    if (threadIdx.x == 0) {
        int index = (blockDim.x << 1) - 1;
        index += CONFLICT_FREE_OFFSET(index);        
        if (storeSum) { // compile-time decision
            // write this block's total sum to the corresponding index in the blockSums array
            g_blockSums[blockIndex] = s_data[index];
        }
        s_data[index] = 0;		// zero the last element in the scan so it will propagate back to the front
    }
}


__device__  static unsigned int buildSum(float *s_data)
{
    unsigned int thid = threadIdx.x;
    unsigned int stride = 1;
    
    // build the sum in place up the tree
    for (int d = blockDim.x; d > 0; d >>= 1) {
        __syncthreads();

        if (thid < d) {
            int i  = __mul24(__mul24(2, stride), thid);
            int ai = i + stride - 1;
            int bi = ai + stride;
            ai += CONFLICT_FREE_OFFSET(ai);
            bi += CONFLICT_FREE_OFFSET(bi);
            s_data[bi] += s_data[ai];
        }
        stride *= 2;
    }
    return stride;
}
__device__ static unsigned int buildSumInt(int *s_data)
{
    unsigned int thid = threadIdx.x;
    unsigned int stride = 1;
    
    // build the sum in place up the tree
    for (int d = blockDim.x; d > 0; d >>= 1) {
        __syncthreads();
        if (thid < d) {
            int i  = __mul24(__mul24(2, stride), thid);
            int ai = i + stride - 1;
            int bi = ai + stride;
            ai += CONFLICT_FREE_OFFSET(ai);
            bi += CONFLICT_FREE_OFFSET(bi);
            s_data[bi] += s_data[ai];
        }
        stride *= 2;
    }
    return stride;
}

__device__ static void scanRootToLeaves(float *s_data, unsigned int stride)
{
     unsigned int thid = threadIdx.x;

    // traverse down the tree building the scan in place
    for (int d = 1; d <= blockDim.x; d *= 2) {
        stride >>= 1;
        __syncthreads();

        if (thid < d) {
            int i  = __mul24(__mul24(2, stride), thid);
            int ai = i + stride - 1;
            int bi = ai + stride;
            ai += CONFLICT_FREE_OFFSET(ai);
            bi += CONFLICT_FREE_OFFSET(bi);
            float t = s_data[ai];
            s_data[ai] = s_data[bi];
            s_data[bi] += t;
        }
    }
}

__device__ static void scanRootToLeavesInt(int *s_data, unsigned int stride)
{
     unsigned int thid = threadIdx.x;

    // traverse down the tree building the scan in place
    for (int d = 1; d <= blockDim.x; d *= 2) {
        stride >>= 1;
        __syncthreads();

        if (thid < d) {
            int i  = __mul24(__mul24(2, stride), thid);
            int ai = i + stride - 1;
            int bi = ai + stride;
            ai += CONFLICT_FREE_OFFSET(ai);
            bi += CONFLICT_FREE_OFFSET(bi);
            int t = s_data[ai];
            s_data[ai] = s_data[bi];
            s_data[bi] += t;
        }
    }
}

template <bool storeSum> __device__ void prescanBlock(float *data, int blockIndex, float *blockSums)
{
    int stride = buildSum (data);               // build the sum in place up the tree
    clearLastElement<storeSum> (data, blockSums, (blockIndex == 0) ? blockIdx.x : blockIndex);
    scanRootToLeaves (data, stride);            // traverse down tree to build the scan 
}
template <bool storeSum> __device__ void prescanBlockInt (int *data, int blockIndex, int *blockSums)
{
    int stride = buildSumInt (data);               // build the sum in place up the tree
    clearLastElementInt <storeSum>(data, blockSums, (blockIndex == 0) ? blockIdx.x : blockIndex);
    scanRootToLeavesInt (data, stride);            // traverse down tree to build the scan 
}

__global__ static void uniformAdd(float *g_data, float *uniforms, int n, int blockOffset, int baseIndex)
{
    __shared__ float uni;
    if (threadIdx.x == 0) uni = uniforms[blockIdx.x + blockOffset];    
    unsigned int address = __mul24(blockIdx.x, (blockDim.x << 1)) + baseIndex + threadIdx.x; 

    __syncthreads();    
    // note two adds per thread
    g_data[address]              += uni;
    g_data[address + blockDim.x] += (threadIdx.x + blockDim.x < n) * uni;
}
__global__ static void uniformAddInt(int *g_data, int *uniforms, int n, int blockOffset, int baseIndex)
{
    __shared__ int uni;
    if (threadIdx.x == 0) uni = uniforms[blockIdx.x + blockOffset];    
    unsigned int address = __mul24(blockIdx.x, (blockDim.x << 1)) + baseIndex + threadIdx.x; 

    __syncthreads();    
    // note two adds per thread
    g_data[address]              += uni;
    g_data[address + blockDim.x] += (threadIdx.x + blockDim.x < n) * uni;
}


#endif // #ifndef _SCAN_BEST_KERNEL_CU_

