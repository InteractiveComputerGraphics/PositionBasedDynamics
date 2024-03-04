//
// scan.cuh
// prefix_sum 
//
// created by ruanjm on 16/12/15
// Copyright (c) 2015 ruanjm. All rights reserved.
//

#ifndef _SCAN_CUH
#define _SCAN_CUH

extern "C"
{

    void prefixSumToGPU(char* inArray, int num, int siz);
    void prefixSumFromGPU(char* outArray, int num, int siz);
    void prefixSum(int num);
    void prefixSumInt(int num);
    void preallocBlockSumsInt(unsigned int num);
    void deallocBlockSumsInt();
    void prescanArray(float* outArray, float* inArray, int numElements);
    void prescanArrayInt(int* outArray, int* inArray, int numElements);
    void prescanArrayRecursiveInt(int *outArray, const int *inArray, int numElements, int level);

}

#endif/*_SCAN_CUH*/