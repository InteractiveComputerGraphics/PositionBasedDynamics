//
// gpu_model.h
// Hybrid_Parallel_SPH
//
// created by ruanjm on 2016/04/10
// Copyright (c) 2016 ruanjm. All rights reserved.
//

#ifndef _GPU_MODEL_H
#define _GPU_MODEL_H

#include <hip/hip_runtime_api.h>

#define I_ADD_SUB   0   // add.s32, sub.s32
#define I_MAD_MUL   1   // mul.xx.s32, mad.xx.s32, xx=lo
#define I_DIV_REM   2   // div.u32, rem.u32, div.s32, rem.s32
#define I_MIN_MAX   3   // min.u32, max.u32, min.s32, max.s32
#define I_ABS       4   // abs.s32
#define I_MUL24     5   // mul24.xx.u32, mul24.xx.s32, xx=lo
#define I_LOGICAL   6   // AND, OR and XOR
#define I_SHL_SHR   7   // shl.b32, shr.u32
#define I_SAD       8   // sad.u32, sad.s32
#define F_ADD_SUB   9   // add.f32, sub.f32
#define F_MAD_MUL   10  // mul.f32, fma.xx.f32, xx=rn
#define F_DIV       11  // div.xx.f32, xx=rn
#define F_DIVIDEF   12  // div.approx.f32
#define F_EXP2      13  // ex2.approx.f32
#define F_LOG2      14  // lg2.approx.f32
#define F_SIN_COS   15  // sin.approx.f32, cos.approx.f32
#define F_SQRT      16  // sqrt.xx.f32, xx=rn
#define F_RSQRT     17  // rsqrt.approx.f32
#define F_MIN_MAX   18  // min.f32, max.f32
#define F_RCP       19  // rcp.xx.f32, xx=rn

#define ARI_STAT_SIZE   20  // arithmetical insts
#define DEFAULT_INST    ARI_STAT_SIZE

#define GLOBAL_ACC	0
#define SHARED_ACC  1
#define LOCAL_ACC	2
#define CONST_ACC   3

#define MEM_STAT_SIZE   4

#define NO_RECOMMENDATION  -1

struct PTXBlockStatistic
{
    __host__ __device__
    inline PTXBlockStatistic operator*(unsigned int t) const{
        PTXBlockStatistic result;

        result.recommended_times = 1;

        result.num_insts = num_insts * t;
        result.num_unknown = num_unknown * t;
        result.num_sync = num_sync * t;
        result.num_bra = num_bra * t;
        for (size_t i = 0; i < ARI_STAT_SIZE; ++i){
            result.num_stat[i] = num_stat[i] * t;
        }
        for (size_t i = 0; i < MEM_STAT_SIZE; ++i){
            result.num_mem[i] = num_mem[i] * t;
        }

        return result;
    }

    __host__ __device__
    inline void operator*=(unsigned int t) {
        recommended_times = 1;
        num_insts *= t;
        num_unknown *= t;
        num_sync *= t;
        num_bra *= t;
        for (size_t i = 0; i < ARI_STAT_SIZE; ++i){
            num_stat[i] *= t;
        }
        for (size_t i = 0; i < MEM_STAT_SIZE; ++i){
            num_mem[i] *= t;
        }
    }

    __host__ __device__
    inline PTXBlockStatistic operator+(const PTXBlockStatistic &a) const{
        PTXBlockStatistic result;

        result.recommended_times = NO_RECOMMENDATION;

        result.num_insts = num_insts + a.num_insts;
        result.num_unknown = num_unknown + a.num_unknown;
        result.num_sync = num_sync + a.num_sync;
        result.num_bra = num_bra + a.num_bra;
        for (size_t i = 0; i < ARI_STAT_SIZE; ++i){
            result.num_stat[i] = num_stat[i] + a.num_stat[i];
        }
        for (size_t i = 0; i < MEM_STAT_SIZE; ++i){
            result.num_mem[i] = num_mem[i] + a.num_mem[i];
        }

        return result;
    }

    __host__ __device__
    inline void operator +=(const PTXBlockStatistic &a){
        recommended_times = NO_RECOMMENDATION;

        num_insts += a.num_insts;
        num_unknown += a.num_unknown;
        num_sync += a.num_sync;
        num_bra += a.num_bra;
        for (size_t i = 0; i < ARI_STAT_SIZE; ++i){
            num_stat[i] += a.num_stat[i];
        }
        for (size_t i = 0; i < MEM_STAT_SIZE; ++i){
            num_mem[i] += a.num_mem[i];
        }
    }

    int recommended_times;

    // statistics
    unsigned int num_insts;
    unsigned int num_unknown;
    unsigned int num_sync;
    unsigned int num_bra;
    unsigned int num_stat[ARI_STAT_SIZE];
    unsigned int num_mem[MEM_STAT_SIZE];
};

struct KernelRelatedParas
{
    float dram_lat;             // baseline DRAM access latency
    float delta;                // transaction departure delay
    float default_inst_lat;     // default instruction latency
};

struct GPUDeviceInfo
{
    float simd_width;           // #SPs per SM
    float warp_size;            // #threads in a warp
    float gamma;                // machine dependent parameter(for thread sync)
    float ilp;
    float mlp;
};

struct InstructionInfo
{
    float inst_info[ARI_STAT_SIZE];

    __device__ __host__
    float operator[](unsigned int i) const {
        return inst_info[i];
    }

    __device__ __host__
    float &operator[](unsigned int i){
        return inst_info[i];
    }
};

#endif/*_GPU_MODEL_H*/
