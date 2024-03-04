//
// scan_kern.cuh
// prefix_sum 
//
// created by ruanjm on 16/12/15
// Copyright (c) 2015 ruanjm. All rights reserved.
//


#ifndef _SCAN_KERN_CUH
#define _SCAN_KERN_CUH

#define max(a,b)            (((a) > (b)) ? (a) : (b))

#define NUM_BANKS   32
#define BLOCK_SIZE  256

/* prefix sum */
#include "prefix_sum.cu.hip"
// NOTE: Template functions must be defined in the header
template <bool storeSum, bool isNP2> __global__ void prescan(float *g_odata, const float *g_idata, float *g_blockSums, int n, int blockIndex, int baseIndex) {
    int ai, bi, mem_ai, mem_bi, bankOffsetA, bankOffsetB;
    extern __shared__ float s_data[];
    loadSharedChunkFromMem<isNP2>(s_data, g_idata, n, (baseIndex == 0) ? __mul24(blockIdx.x, (blockDim.x << 1)) : baseIndex, ai, bi, mem_ai, mem_bi, bankOffsetA, bankOffsetB);
    prescanBlock<storeSum>(s_data, blockIndex, g_blockSums);
    storeSharedChunkToMem<isNP2>(g_odata, s_data, n, ai, bi, mem_ai, mem_bi, bankOffsetA, bankOffsetB);
}
template <bool storeSum, bool isNP2> __global__ void prescanInt(int *g_odata, const int *g_idata, int *g_blockSums, int n, int blockIndex, int baseIndex) {
    int ai, bi, mem_ai, mem_bi, bankOffsetA, bankOffsetB;
    extern __shared__ int s_dataInt[];
    loadSharedChunkFromMemInt <isNP2>(s_dataInt, g_idata, n, (baseIndex == 0) ? __mul24(blockIdx.x, (blockDim.x << 1)) : baseIndex, ai, bi, mem_ai, mem_bi, bankOffsetA, bankOffsetB);
    prescanBlockInt<storeSum>(s_dataInt, blockIndex, g_blockSums);
    storeSharedChunkToMemInt <isNP2>(g_odata, s_dataInt, n, ai, bi, mem_ai, mem_bi, bankOffsetA, bankOffsetB);
}
__global__ void uniformAddInt(int*  g_data, int *uniforms, int n, int blockOffset, int baseIndex);
__global__ void uniformAdd(float*g_data, float *uniforms, int n, int blockOffset, int baseIndex);


#endif/*_SCAN_KERN_CUH*/