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

// includes, kernels

#include <hip/hip_runtime.h>
#include "scan.cuh.hip"
#include <assert.h>
#include <math.h>
#include "cuda_call_check.h.hip"
#include "scan_kern.cuh.hip"

inline bool isPowerOfTwo(int n) { return ((n&(n - 1)) == 0); }

inline int floorPow2(int n) {
#ifdef WIN32
    return 1 << (int)logb((float)n);
#else
    int exp;
    frexp((float)n, &exp);
    return 1 << (exp - 1);
#endif
}

float**			g_scanBlockSums = 0;
int**			g_scanBlockSumsInt = 0;
unsigned int	g_numEltsAllocated = 0;
unsigned int	g_numLevelsAllocated = 0;

void preallocBlockSums(unsigned int maxNumElements)
{
    assert(g_numEltsAllocated == 0); // shouldn't be called 

    g_numEltsAllocated = maxNumElements;
    unsigned int blockSize = BLOCK_SIZE; // max size of the thread blocks
    unsigned int numElts = maxNumElements;
    int level = 0;

    do {
        unsigned int numBlocks = max(1, (int)ceil((float)numElts / (2.f * blockSize)));
        if (numBlocks > 1) level++;
        numElts = numBlocks;
    } while (numElts > 1);

    g_scanBlockSums = (float**)malloc(level * sizeof(float*));
    g_numLevelsAllocated = level;

    numElts = maxNumElements;
    level = 0;

    do {
        unsigned int numBlocks = max(1, (int)ceil((float)numElts / (2.f * blockSize)));
        if (numBlocks > 1)
            CUDA_SAFE_CALL(hipMalloc((void**)&g_scanBlockSums[level++], numBlocks * sizeof(float)));
        numElts = numBlocks;
    } while (numElts > 1);

}
void preallocBlockSumsInt(unsigned int maxNumElements)
{
    assert(g_numEltsAllocated == 0); // shouldn't be called 

    g_numEltsAllocated = maxNumElements;
    unsigned int blockSize = BLOCK_SIZE; // max size of the thread blocks
    unsigned int numElts = maxNumElements;
    int level = 0;

    do {
        unsigned int numBlocks = max(1, (int)ceil((float)numElts / (2.f * blockSize)));
        if (numBlocks > 1) level++;
        numElts = numBlocks;
    } while (numElts > 1);

    g_scanBlockSumsInt = (int**)malloc(level * sizeof(int*));
    g_numLevelsAllocated = level;

    numElts = maxNumElements;
    level = 0;

    do {
        unsigned int numBlocks = max(1, (int)ceil((float)numElts / (2.f * blockSize)));
        if (numBlocks > 1) CUDA_SAFE_CALL(hipMalloc((void**)&g_scanBlockSumsInt[level++], numBlocks * sizeof(int)));
        numElts = numBlocks;
    } while (numElts > 1);
}

void deallocBlockSums()
{
    if (g_scanBlockSums != 0x0) {
        for (unsigned int i = 0; i < g_numLevelsAllocated; i++)
            CUDA_SAFE_CALL(hipFree(g_scanBlockSums[i]));

        free((void**)g_scanBlockSums);
    }

    g_scanBlockSums = 0;
    g_numEltsAllocated = 0;
    g_numLevelsAllocated = 0;
}
void deallocBlockSumsInt()
{
    if (g_scanBlockSums != 0x0) {
        for (unsigned int i = 0; i < g_numLevelsAllocated; i++)
            CUDA_SAFE_CALL(hipFree(g_scanBlockSumsInt[i]));
        free((void**)g_scanBlockSumsInt);
    }

    g_scanBlockSumsInt = 0;
    g_numEltsAllocated = 0;
    g_numLevelsAllocated = 0;
}



void prescanArrayRecursive(float *outArray, const float *inArray, int numElements, int level)
{
    unsigned int blockSize = BLOCK_SIZE; // max size of the thread blocks
    unsigned int numBlocks = max(1, (int)ceil((float)numElements / (2.f * blockSize)));
    unsigned int numThreads;

    if (numBlocks > 1)
        numThreads = blockSize;
    else if (isPowerOfTwo(numElements))
        numThreads = numElements / 2;
    else
        numThreads = floorPow2(numElements);

    unsigned int numEltsPerBlock = numThreads * 2;

    // if this is a non-power-of-2 array, the last block will be non-full
    // compute the smallest power of 2 able to compute its scan.
    unsigned int numEltsLastBlock = numElements - (numBlocks - 1) * numEltsPerBlock;
    unsigned int numThreadsLastBlock = max(1, numEltsLastBlock / 2);
    unsigned int np2LastBlock = 0;
    unsigned int sharedMemLastBlock = 0;

    if (numEltsLastBlock != numEltsPerBlock) {
        np2LastBlock = 1;
        if (!isPowerOfTwo(numEltsLastBlock)) numThreadsLastBlock = floorPow2(numEltsLastBlock);
        unsigned int extraSpace = (2 * numThreadsLastBlock) / NUM_BANKS;
        sharedMemLastBlock = sizeof(float) * (2 * numThreadsLastBlock + extraSpace);
    }

    // padding space is used to avoid shared memory bank conflicts
    unsigned int extraSpace = numEltsPerBlock / NUM_BANKS;
    unsigned int sharedMemSize = sizeof(float) * (numEltsPerBlock + extraSpace);

#ifdef DEBUG
    if (numBlocks > 1) assert(g_numEltsAllocated >= numElements);
#endif

    // setup execution parameters
    // if NP2, we process the last block separately
    dim3  grid(max(1, numBlocks - np2LastBlock), 1, 1);
    dim3  threads(numThreads, 1, 1);

    // execute the scan
    if (numBlocks > 1) {
        prescan<true, false><<<grid, threads, sharedMemSize>>>(outArray, inArray, g_scanBlockSums[level], numThreads * 2, 0, 0);
        if (np2LastBlock) {
            prescan<true, true><<<1, numThreadsLastBlock, sharedMemLastBlock>>>(outArray, inArray, g_scanBlockSums[level], numEltsLastBlock, numBlocks - 1, numElements - numEltsLastBlock);
        }

        // After scanning all the sub-blocks, we are mostly done.  But now we 
        // need to take all of the last values of the sub-blocks and scan those.  
        // This will give us a new value that must be added to each block to 
        // get the final results.
        // recursive (CPU) call
        prescanArrayRecursive(g_scanBlockSums[level], g_scanBlockSums[level], numBlocks, level + 1);

        uniformAdd<<<grid, threads>>>(outArray, g_scanBlockSums[level], numElements - numEltsLastBlock, 0, 0);
        if (np2LastBlock) {
            uniformAdd<<<1, numThreadsLastBlock>>>(outArray, g_scanBlockSums[level], numEltsLastBlock, numBlocks - 1, numElements - numEltsLastBlock);
        }
    }
    else if (isPowerOfTwo(numElements)) {
        prescan<false, false><<<grid, threads, sharedMemSize>>>(outArray, inArray, 0, numThreads * 2, 0, 0);
    }
    else {
        prescan<false, true><<<grid, threads, sharedMemSize>>>(outArray, inArray, 0, numElements, 0, 0);
    }
}

void prescanArrayRecursiveInt(int *outArray, const int *inArray, int numElements, int level)
{
    unsigned int blockSize = BLOCK_SIZE; // max size of the thread blocks
    unsigned int numBlocks = max(1, (int)ceil((float)numElements / (2.f * blockSize)));
    unsigned int numThreads;

    if (numBlocks > 1)
        numThreads = blockSize;
    else if (isPowerOfTwo(numElements))
        numThreads = numElements / 2;
    else
        numThreads = floorPow2(numElements);

    unsigned int numEltsPerBlock = numThreads * 2;

    // if this is a non-power-of-2 array, the last block will be non-full
    // compute the smallest power of 2 able to compute its scan.
    unsigned int numEltsLastBlock = numElements - (numBlocks - 1) * numEltsPerBlock;
    unsigned int numThreadsLastBlock = max(1, numEltsLastBlock / 2);
    unsigned int np2LastBlock = 0;
    unsigned int sharedMemLastBlock = 0;

    if (numEltsLastBlock != numEltsPerBlock) {
        np2LastBlock = 1;
        if (!isPowerOfTwo(numEltsLastBlock)) numThreadsLastBlock = floorPow2(numEltsLastBlock);
        unsigned int extraSpace = (2 * numThreadsLastBlock) / NUM_BANKS;
        sharedMemLastBlock = sizeof(float) * (2 * numThreadsLastBlock + extraSpace);
    }

    // padding space is used to avoid shared memory bank conflicts
    unsigned int extraSpace = numEltsPerBlock / NUM_BANKS;
    unsigned int sharedMemSize = sizeof(float) * (numEltsPerBlock + extraSpace);

#ifdef DEBUG
    if (numBlocks > 1) assert(g_numEltsAllocated >= numElements);
#endif

    // setup execution parameters
    // if NP2, we process the last block separately
    dim3  grid(max(1, numBlocks - np2LastBlock), 1, 1);
    dim3  threads(numThreads, 1, 1);

    // execute the scan
    if (numBlocks > 1) {
        prescanInt<true, false><<<grid, threads, sharedMemSize>>>(outArray, inArray, g_scanBlockSumsInt[level], numThreads * 2, 0, 0);
        if (np2LastBlock) {
            prescanInt<true, true><<<1, numThreadsLastBlock, sharedMemLastBlock>>>(outArray, inArray, g_scanBlockSumsInt[level], numEltsLastBlock, numBlocks - 1, numElements - numEltsLastBlock);
        }

        // After scanning all the sub-blocks, we are mostly done.  But now we 
        // need to take all of the last values of the sub-blocks and scan those.  
        // This will give us a new value that must be added to each block to 
        // get the final results.
        // recursive (CPU) call
        prescanArrayRecursiveInt(g_scanBlockSumsInt[level], g_scanBlockSumsInt[level], numBlocks, level + 1);

        uniformAddInt<<<grid, threads>>>(outArray, g_scanBlockSumsInt[level], numElements - numEltsLastBlock, 0, 0);
        if (np2LastBlock) {
            uniformAddInt<<<1, numThreadsLastBlock>>>(outArray, g_scanBlockSumsInt[level], numEltsLastBlock, numBlocks - 1, numElements - numEltsLastBlock);
        }
    }
    else if (isPowerOfTwo(numElements)) {
        prescanInt<false, false><<<grid, threads, sharedMemSize>>>(outArray, inArray, 0, numThreads * 2, 0, 0);
    }
    else {
        prescanInt<false, true><<<grid, threads, sharedMemSize>>>(outArray, inArray, 0, numElements, 0, 0);
    }
}


void prescanArray(float *d_odata, float *d_idata, int num)
{
    // preform prefix sum
    preallocBlockSums(num);
    prescanArrayRecursive(d_odata, d_idata, num, 0);
    deallocBlockSums();
}
void prescanArrayInt(int *d_odata, int *d_idata, int num)
{
    // preform prefix sum
    preallocBlockSumsInt(num);
    prescanArrayRecursiveInt(d_odata, d_idata, num, 0);
    deallocBlockSumsInt();
}

char* d_idata = NULL;
char* d_odata = NULL;

void prefixSum(int num)
{
    prescanArray((float*)d_odata, (float*)d_idata, num);
}

void prefixSumInt(int num)
{
    prescanArrayInt((int*)d_odata, (int*)d_idata, num);
}

void prefixSumToGPU(char* inArray, int num, int siz)
{
    CUDA_SAFE_CALL(hipMalloc((void**)&d_idata, num*siz));
    CUDA_SAFE_CALL(hipMalloc((void**)&d_odata, num*siz));
    CUDA_SAFE_CALL(hipMemcpy(d_idata, inArray, num*siz, hipMemcpyHostToDevice));
}
void prefixSumFromGPU(char* outArray, int num, int siz)
{
    CUDA_SAFE_CALL(hipMemcpy(outArray, d_odata, num*siz, hipMemcpyDeviceToHost));
    CUDA_SAFE_CALL(hipFree(d_idata));
    CUDA_SAFE_CALL(hipFree(d_odata));
    d_idata = NULL;
    d_odata = NULL;
}
