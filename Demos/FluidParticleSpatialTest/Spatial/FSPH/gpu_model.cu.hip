//
// gpu_model.cu
// Hybrid_Parallel_SPH
//
// created by ruanjm on 2016/04/09
// Copyright (c) 2016 ruanjm. All rights reserved.
//


#include <hip/hip_runtime.h>
#include "gpu_model.cuh.hip"
//#include <device_launch_parameters.h>
#include "cuda_call_check.h.hip"
#include "cuda_math.cuh.hip"
#include "gpu_model.h.hip"
#include "gpu_model_reader.h.hip"
#include "sph_utils.cuh.hip"

namespace gpu_model
{

using namespace sph;

const char *kLatencyFileName = "insts_latency.json";
const char *kPTXStatisticsFileNameSMS = "sph_sms_arti_block_statistics.json";
const char *kPTXStatisticsFileNameTRA = "sph_tra_arti_block_statistics.json";
const char *kFunNameDensitySMS = "_ZN3sph19knComputeDensitySMSENS_18ParticleBufferListEPiS1_PNS_9BlockTaskE";
const char *kFunNameForceSMS = "_ZN3sph17knComputeForceSMSENS_18ParticleBufferListEPiS1_PNS_9BlockTaskE";
const char *kFunNameDensityTRA = "_Z21knBmComputeDensityTRAN3SPH18ParticleBufferListEPjS1_jS1_";
const char *kFunNameForceTRA = "_Z19knBmComputeForceTRAN3SPH18ParticleBufferListEPjS1_jS1_";

#define kDensityTRA     0
#define kForceTRA       1
#define kDensitySMS     2
#define kForceSMS       3

const int kSelfID = 13U;
const int kNumNeighbor = 27U;
//const float v1_32 = 1.f / 32.f;
#define MIN(A, B) (A > B ? B : A)

const int kDefaultNumThread = 256;
__constant__ GPUDeviceInfo kDevGPUInfo;
__constant__ InstructionInfo kDevInstsLatency;

struct GPUModel
{
    PTXBlockStatistic *bs_tra_density;
    PTXBlockStatistic *bs_tra_force;
    PTXBlockStatistic *bs_sms_density;
    PTXBlockStatistic *bs_sms_force;

    PTXBlockStatistic *static_block;

    KernelRelatedParas *kn_paras;
};

/****************************** Kernel ******************************/

__global__
void knCalculateBlockRequirementSMSMode(int *block_req, int *cell_start, int *cell_end, int block_size, int numc)
{
    unsigned int idx = threadIdx.x + __umul24(blockDim.x, blockIdx.x);

    if (idx >= numc) return;
    
    int cs = cell_start[idx];
    int ce = cell_end[idx];
    block_req[idx] = cs == kInvalidCellIdx ? 0 : ceil_int(ce - cs, block_size);
}

__device__
float knCalculateAvgLatency(const PTXBlockStatistic &insts_count, float default_lat)
{
    float total_time = 0.0f;
    for (size_t i = 0; i < ARI_STAT_SIZE; ++i)
    {
        total_time += (float)insts_count.num_stat[i] * kDevInstsLatency[i];
    }
    for (size_t i = 0; i < MEM_STAT_SIZE; ++i)
    {
        total_time += (float)insts_count.num_mem[i] * default_lat;
    }
    total_time += (float)insts_count.num_bra * default_lat;
    total_time += (float)insts_count.num_unknown * default_lat;

    return total_time / (insts_count.num_insts - insts_count.num_sync);
}

__device__
float knCalculateKernelClock(PTXBlockStatistic *blocks, int *times, int num_inst_block, PTXBlockStatistic &basic_block, 
                             KernelRelatedParas &kn_para, float num_uncoal_per_warp, 
                             int num_blocks, int num_warps, int num_sms)
{
    PTXBlockStatistic insts_count = basic_block;
    for (size_t i = 0; i < num_inst_block; ++i)
    {
        if (NO_RECOMMENDATION == blocks[i].recommended_times)
        {
            insts_count += blocks[i] * times[i];
        }
    }

    float avg_inst_lat = knCalculateAvgLatency(insts_count, kn_para.default_inst_lat);
    float dram_lat = kn_para.dram_lat + (num_uncoal_per_warp - 1) * kn_para.delta;

    // compute ITILP
    float itilp_max = 8 / (kDevGPUInfo.warp_size / kDevGPUInfo.simd_width);     // Eq.2-5
    float itilp = MIN(kDevGPUInfo.ilp * num_warps, itilp_max);                   // Eq.2-4

    // compute ITMLP
    float itmlp = 2;

    // compute execution time
    float f_sync = kDevGPUInfo.gamma * dram_lat * insts_count.num_mem[GLOBAL_ACC] / insts_count.num_insts;  // Eq.2-8
    float o_sync = ((insts_count.num_sync * num_blocks) / num_sms) * f_sync;                                // Eq.2-7
    if (1 >= num_warps) o_sync = 0;

    float w_serial = o_sync;
    float w_parallel = (((insts_count.num_insts - insts_count.num_sync) * num_warps) / num_sms) * (avg_inst_lat / itilp);   // Eq.2-3

    float t_comp = w_serial + w_parallel;   // Eq.2-2

    float t_mem = (insts_count.num_mem[GLOBAL_ACC] * num_warps) / (num_warps * itmlp) * dram_lat;   // Eq.2-11

    float t_overlap = MIN(t_comp, t_mem);

    return t_comp + t_mem - t_overlap;
}

__global__
void knCalculateBlockRequirementHybridMode(int *cell_type, int *d_cell_num, int *block_req, GPUModel gm, int *cell_offset, int *cell_num, ushort3 grid_size, int block_size)
{
    unsigned int idx = threadIdx.x + blockDim.x*blockIdx.x;
    int numc = grid_size.x * grid_size.y * grid_size.z;
    if (idx >= numc) return;
    int nump_self = d_cell_num[idx];
   
    
    
    int totaln = nump_self;
	ushort3 self_pos = CellIdx2CellPos(idx, grid_size);
    int nidx = CellPos2CellIdx(self_pos + make_ushort3(-1, 0, 0), grid_size);
    if (kInvalidCellIdx != nidx) totaln += d_cell_num[nidx];
    nidx = CellPos2CellIdx(self_pos + make_ushort3(1, 0, 0), grid_size);
    if (kInvalidCellIdx != nidx) totaln += d_cell_num[nidx];
    nidx = CellPos2CellIdx(self_pos + make_ushort3(0, 1, 0), grid_size);
    if (kInvalidCellIdx != nidx) totaln += d_cell_num[nidx];
    nidx = CellPos2CellIdx(self_pos + make_ushort3(0, -1, 0), grid_size);
    if (kInvalidCellIdx != nidx) totaln += d_cell_num[nidx];
    nidx = CellPos2CellIdx(self_pos + make_ushort3(0, 0, -1), grid_size);
    if (kInvalidCellIdx != nidx) totaln += d_cell_num[nidx];
    nidx = CellPos2CellIdx(self_pos + make_ushort3(0, 0, 1), grid_size);
    if (kInvalidCellIdx != nidx) totaln += d_cell_num[nidx];
               
    if (totaln < 96 && nump_self < 15){ block_req[idx] = 0; }else if (totaln < 60){ block_req[idx] = (nump_self + 27) >> 5; }else{ block_req[idx] = (nump_self + 31) >> 5; }
}

/****************************** Interface ******************************/

void calculateStaticBlock(PTXBlockStatistic &static_block, PTXBlockStatistic *blocks, size_t num_blocks)
{
    std::memset(&static_block, 0, sizeof(PTXBlockStatistic));

    for (size_t i = 0; i < num_blocks; ++i)
    {
        if (NO_RECOMMENDATION != blocks[i].recommended_times)
        {
            static_block += blocks[i] * blocks[i].recommended_times;
        }
    }
}

void setKernelParameters(KernelRelatedParas &kn_para, unsigned int type)
{
    switch (type)
    {
    case kDensityTRA:
        kn_para.dram_lat = 250;
        kn_para.delta = 0;
        kn_para.default_inst_lat = 12;
        break;
    case kForceTRA:
        kn_para.dram_lat = 230;
        kn_para.delta = 0;
        kn_para.default_inst_lat = 12;
        break;
    case kDensitySMS:
        kn_para.dram_lat = 240;
        kn_para.delta = 10;
        kn_para.default_inst_lat = 14;
        break;
    case kForceSMS:
        kn_para.dram_lat = 240;
        kn_para.delta = 10;
        kn_para.default_inst_lat = 10;
        break;
    default:
        break;
    }
}

void allocateGPUModel(GPUModel *&gpu_model)
{
    unsigned int data_len;
    PTXBlockStatistic *bs_tra_density = nullptr;
    PTXBlockStatistic *bs_tra_force = nullptr;
    PTXBlockStatistic *bs_sms_density = nullptr;
    PTXBlockStatistic *bs_sms_force = nullptr;
    PTXBlockStatistic static_block[4];
    KernelRelatedParas kn_paras[4];
    InstructionInfo insts_latency;
    GPUDeviceInfo device_info;

    gpu_model = new GPUModel;

    // read block data and transfer to device
    data_len = readPTXStatisticsFromFile(bs_tra_density, kFunNameDensityTRA, kPTXStatisticsFileNameTRA);
    CUDA_SAFE_CALL(hipMalloc(&(gpu_model->bs_tra_density), data_len * sizeof(PTXBlockStatistic)));
    CUDA_SAFE_CALL(hipMemcpy(gpu_model->bs_tra_density, bs_tra_density, data_len * sizeof(PTXBlockStatistic), hipMemcpyHostToDevice));
    data_len = readPTXStatisticsFromFile(bs_tra_force, kFunNameForceTRA, kPTXStatisticsFileNameTRA);
    CUDA_SAFE_CALL(hipMalloc(&(gpu_model->bs_tra_force), data_len * sizeof(PTXBlockStatistic)));
    CUDA_SAFE_CALL(hipMemcpy(gpu_model->bs_tra_force, bs_tra_force, data_len * sizeof(PTXBlockStatistic), hipMemcpyHostToDevice));
    data_len = readPTXStatisticsFromFile(bs_sms_density, kFunNameDensitySMS, kPTXStatisticsFileNameSMS);
    CUDA_SAFE_CALL(hipMalloc(&(gpu_model->bs_sms_density), data_len * sizeof(PTXBlockStatistic)));
    CUDA_SAFE_CALL(hipMemcpy(gpu_model->bs_sms_density, bs_sms_density, data_len * sizeof(PTXBlockStatistic), hipMemcpyHostToDevice));
    data_len = readPTXStatisticsFromFile(bs_sms_force, kFunNameForceSMS, kPTXStatisticsFileNameSMS);
    CUDA_SAFE_CALL(hipMalloc(&(gpu_model->bs_sms_force), data_len * sizeof(PTXBlockStatistic)));
    CUDA_SAFE_CALL(hipMemcpy(gpu_model->bs_sms_force, bs_sms_force, data_len * sizeof(PTXBlockStatistic), hipMemcpyHostToDevice));

    // calculate static block and transfer to device
    calculateStaticBlock(static_block[kDensityTRA], bs_tra_density, 5);
    calculateStaticBlock(static_block[kForceTRA], bs_tra_force, 5);
    calculateStaticBlock(static_block[kDensitySMS], bs_sms_density, 7);
    calculateStaticBlock(static_block[kForceSMS], bs_sms_force, 7);
    CUDA_SAFE_CALL(hipMalloc(&(gpu_model->static_block), 4 * sizeof(PTXBlockStatistic)));
    CUDA_SAFE_CALL(hipMemcpy(gpu_model->static_block, static_block, 4 * sizeof(PTXBlockStatistic), hipMemcpyHostToDevice));

    // read instruction latency and transfer to device
    readInstructionLatencyFromFile(insts_latency, kLatencyFileName);
    size_t iiSize = sizeof(InstructionInfo);
    (hipMemcpyToSymbol(HIP_SYMBOL(kDevInstsLatency), &insts_latency, iiSize));

    // set device info and transfer to device
    device_info.simd_width = 64;
    device_info.warp_size = 32;
    device_info.gamma = 64;
    device_info.ilp = 2;
    device_info.mlp = 2;
    (hipMemcpyToSymbol(HIP_SYMBOL(kDevGPUInfo), &device_info, sizeof(GPUDeviceInfo)));

    // set kernel related paras and transfer to device
    setKernelParameters(kn_paras[kDensityTRA], kDensityTRA);
    setKernelParameters(kn_paras[kForceTRA], kForceTRA);
    setKernelParameters(kn_paras[kDensitySMS], kDensitySMS);
    setKernelParameters(kn_paras[kForceSMS], kForceSMS);
    CUDA_SAFE_CALL(hipMalloc(&(gpu_model->kn_paras), 4 * sizeof(KernelRelatedParas)));
    CUDA_SAFE_CALL(hipMemcpy(gpu_model->kn_paras, kn_paras, 4 * sizeof(KernelRelatedParas), hipMemcpyHostToDevice));

    if (nullptr != bs_tra_density) delete[]bs_tra_density;
    if (nullptr != bs_tra_force) delete[]bs_tra_force;
    if (nullptr != bs_sms_density) delete[]bs_sms_density;
    if (nullptr != bs_sms_force) delete[]bs_sms_force;
}

void freeGPUModel(GPUModel *gpu_model)
{
    CUDA_SAFE_CALL(hipFree(gpu_model->bs_tra_density));
    CUDA_SAFE_CALL(hipFree(gpu_model->bs_tra_force));
    CUDA_SAFE_CALL(hipFree(gpu_model->bs_sms_density));
    CUDA_SAFE_CALL(hipFree(gpu_model->bs_sms_force));
    CUDA_SAFE_CALL(hipFree(gpu_model->kn_paras));
    CUDA_SAFE_CALL(hipFree(gpu_model->static_block));

    delete gpu_model;
}

void calculateBlockRequirementHybridMode(int *cell_type, int *d_cell_num, int *block_req, GPUModel *gm, int *cell_offset, int *cell_num, ushort3 grid_size, int block_size)
{
    int numc = grid_size.x * grid_size.y * grid_size.z;

    int num_thread = kDefaultNumThread;
    int num_block = ceil_int(numc, num_thread);

    knCalculateBlockRequirementHybridMode <<<num_block, num_thread >>>(cell_type, d_cell_num, block_req, *gm, cell_offset, cell_num, grid_size, block_size);
}

}