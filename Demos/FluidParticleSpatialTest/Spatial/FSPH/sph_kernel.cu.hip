//
// sph_kernel.cu
// Hybrid_Parallel_SPH
//
// created by ruanjm on 2016/04/05
// Copyright (c) 2016 ruanjm. All rights reserved.
//


#include <hip/hip_runtime.h>
#include "sph_kernel.cuh.hip"
#include "sph_kernel_shared_data.cuh.hip"

#include <thrust/count.h>
#include <thrust/execution_policy.h>
#include <numeric>

namespace sph
{

const int kDefaultNumThreadSMS = 32;
const int kDefulatMinBlocksSMS = 10;
const int kDefaultNumThreadSMS2 = 64;
const int kDefaultNumThreadTRA = 128;
const int kNumberNeighborCells = 27;

__constant__ SystemParameter kDevSysPara;

__device__ __host__
inline float3 cal_rePos(const float3 &pos4, const float3 &pos3) {
    return make_float3(pos3.x - pos4.x, pos3.y - pos4.y, pos3.z - pos4.z);
}


__device__ __host__
inline unsigned int ceil_uint(unsigned int a, unsigned int b) { return (a + b - 1) / b; }

__device__ __host__
inline float powf_2(float base) { return base * base; }

__device__ __host__
inline float powf_3(float base) { return base * base * base; }

__device__ __host__
inline float powf_7(float base) { return base * base * base * base * base * base * base; }

typedef unsigned int uint;

#define COLORA(r,g,b,a)	( (uint((a)*255.0f)<<24) | (uint((b)*255.0f)<<16) | (uint((g)*255.0f)<<8) | uint((r)*255.0f) )

/****************************** Kernel ******************************/

//sf 计算density 一个cell

__global__ //__launch_bounds__(kDefaultNumThreadSMS, kDefulatMinBlocksSMS)
void knInit(ParticleBufferList buff_list, int nump)
{
    unsigned int idx = threadIdx.x + __umul24(blockIdx.x, blockDim.x);
    if (idx >= nump) return;
    /*float4 ev;
    ev.x = 0; ev.y = 0; ev.z = 0; ev.w = 0;
    buff_list.evaluated_velocity[idx] = ev;
    buff_list.acceleration[idx] = make_float3(0.f, 0.f, 0.f);*/
}


void BuffInit(ParticleBufferList buff_list_n, int nm){
    if (nm <= 0) return;
    int num_thread = 256;
    int number_block = ceil_int(nm, num_thread);
    knInit <<<number_block, num_thread >>>(buff_list_n, nm);
}

//__device__
//inline void knComputeCellDensitySMS64(const int& isSame, SimDenSharedData128 *sdata, CDAPData *self_data, int read_num)
//{
//    int kk = (1-isSame)*(threadIdx.x>>5);
//    for (size_t i = (kk <<5); i < (kk<<5) + read_num; ++i)
//    {
//        float3 neighbor_position = sdata->getPos(i);
//        float dis_2 = distance_square(self_data->pos, neighbor_position);
//        if (kDevSysPara.kernel_2 < dis_2 || kFloatSmall > dis_2)
//            continue;
//        self_data->pos.w += powf_3(kDevSysPara.kernel_2 - dis_2);
//    }
//}

//__device__
//inline void knComputeCellForceSMS64(const int& isSame, float3 *pres_kn, float3 *vis_kn, SimForSharedData128 *sdata, CFData *self_data, int read_num)
//{
//    //#pragma unroll 16
//	int kk = (1 - isSame)*(threadIdx.x>>5);
//    for (size_t i = (kk <<5); i < (kk <<5) + read_num; ++i)
//    {
//
//        float3 neighbor_position = sdata->getPosition(i);
//        float3 rel_pos = cal_rePos(neighbor_position, self_data->pos);
//
//        float dis_2 = rel_pos.x * rel_pos.x + rel_pos.y * rel_pos.y + rel_pos.z * rel_pos.z;
//
//        if (kDevSysPara.kernel_2 < dis_2 || kFloatSmall > dis_2)
//            continue;
//
//        float dis = sqrtf(dis_2);
//        float V = 1 / neighbor_position.w;
//        float kernel_r = kDevSysPara.kernel - dis;
//
//        // pressure force
//        float temp_pres_kn = V * (self_data->ev.w + sdata->getEV(i).w) * kernel_r * kernel_r;
//        *pres_kn = *pres_kn - rel_pos * __fdividef(temp_pres_kn, dis);
//
//        // viscosity force
//        float3 rel_vel = cal_rePos(self_data->ev, sdata->getEV(i));
//        float temp_vis_kn = V * kernel_r;
//        *vis_kn = *vis_kn + rel_vel * temp_vis_kn;
//
//        // surface force
//        float temp = V * powf_2(kDevSysPara.kernel_2 - dis_2);
//        self_data->grad_color = self_data->grad_color + rel_pos * temp;
//        self_data->lplc_color = self_data->lplc_color + V * (kDevSysPara.kernel_2 - dis_2) *
//            (dis_2 - 3 / 4 * (kDevSysPara.kernel_2 - dis_2));
//    }
//}

__device__
inline void knComputeCellNeighbourCountSMS64(int& count, const int& isSame, SimNeighSharedData128* sdata, CFData* self_data, int read_num)
{
    int kk = (1 - isSame) * (threadIdx.x >> 5);

    for (size_t i = (kk << 5); i < (kk << 5) + read_num; ++i)
    {
        float3 neighbor_position = sdata->getPosition(i);
        float3 rel_pos = cal_rePos(neighbor_position, self_data->pos);

        float dis_2 = rel_pos.x * rel_pos.x + rel_pos.y * rel_pos.y + rel_pos.z * rel_pos.z;

        if (kDevSysPara.kernel_2 < dis_2 || kFloatSmall > dis_2)
            continue;

        count++;
    }
}

__device__
inline void knComputeCellNeighbourSMS64(int& count, const int& isSame, SimNeighSharedData128* sdata, CFData* self_data, int read_num, int* neighbors, int writeOffset)
{
    int kk = (1 - isSame) * (threadIdx.x >> 5);

    for (size_t i = (kk << 5); i < (kk << 5) + read_num; ++i)
    {

        float3 neighbor_position = sdata->getPosition(i);
        float3 rel_pos = cal_rePos(neighbor_position, self_data->pos);

        float dis_2 = rel_pos.x * rel_pos.x + rel_pos.y * rel_pos.y + rel_pos.z * rel_pos.z;

        if (kDevSysPara.kernel_2 < dis_2 || kFloatSmall > dis_2)
            continue;

        neighbors[writeOffset + count] = sdata->getIndex(i);
        count++;
    }
}

__global__
void knManualSetting(ParticleBufferList buff_list, unsigned int nump, int step)
{
    unsigned int idx = threadIdx.x + __umul24(blockIdx.x, blockDim.x);

    if (idx >= nump) return;
}

//__device__
//inline float4 addfloat4(const float4& vec4, const float3 &vec3){
//    float4 nVec4;
//    nVec4.x = vec4.x + vec3.x;
//    nVec4.y = vec4.y + vec3.y;
//    nVec4.z = vec4.z + vec3.z;
//    nVec4.w = vec4.w;
//    return nVec4;
//}
//
//__device__
//inline float4 floathalf4add3(float3 ra, const float4& vec4){
//    float4 v4;
//    v4.x = (ra.x + vec4.x)*0.5;;
//    v4.y = (ra.y + vec4.y)*0.5;
//    v4.z = (vec4.z + ra.z)*0.5;
//    v4.w = vec4.w;
//    return v4;
//}
//__device__
//inline float3 float4m3(float3 ra, const float4& vec4){
//
//    return make_float3(ra.x*vec4.x, ra.y*vec4.y, vec4.z*ra.z);
//}
//__device__ 
//inline float dotV34(float3 v3, float4 v4){
//    return v3.x*v4.x + v3.y*v4.y + v3.z*v4.z;
//}

__device__
inline int cdMax(int a, int b)
{
    return a>b ? a : b;
}

__global__
void find_max(sumGrad *id_value, int numbers, int iSize)
{
    int x_id = __umul24(blockDim.x, blockIdx.x) + threadIdx.x;
    x_id++;
    if (x_id <= numbers)
    {
        int P = x_id & (iSize - 1);
        if (0 == P)
            P = iSize;
        if (P > (iSize >> 1))
        {
            x_id--;
            id_value[x_id].num_neigh = cdMax(id_value[x_id].num_neigh, id_value[x_id + (iSize >> 1) - P].num_neigh);
        }
    }
}

//sf PCISPH over -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/****************************** Interface ******************************/

hipStream_t sms_stream;
hipEvent_t sms_density_event;
hipEvent_t sms_force_event;

void transSysParaToDevice(const SystemParameter *host_para)
{
    CUDA_SAFE_CALL(hipMemcpyToSymbol(HIP_SYMBOL(kDevSysPara), host_para, sizeof(SystemParameter)));
    printf("testing self_density %f\n", kDevSysPara.mass);
}

//void initializeKernel()
//{
//    CUDA_SAFE_CALL(hipStreamCreateWithFlags(&sms_stream, hipStreamDefault));
//    CUDA_SAFE_CALL(hipEventCreateWithFlags(&sms_density_event, hipEventDisableTiming));
//    CUDA_SAFE_CALL(hipEventCreateWithFlags(&sms_force_event, hipEventDisableTiming));
//}
//
//void releaseKernel()
//{
//    CUDA_SAFE_CALL(hipStreamDestroy(sms_stream));
//    CUDA_SAFE_CALL(hipEventDestroy(sms_density_event));
//    CUDA_SAFE_CALL(hipEventDestroy(sms_force_event));
//}

//__device__
//void knComputeCellDensityTRA9(ParticleBufferList &buff_list, CDAPData *self_data, int cell_offset, int cell_num)
//{
//    if (0 == cell_num) return;
//    int end_idx = cell_offset + cell_num;
//    for (size_t i = cell_offset; i < end_idx; ++i)
//    {
//        float4 neighbor_pos = buff_list.position_d[i];
//
//
//        float3 rel_pos = cal_rePos(neighbor_pos, self_data->pos);
//        float dis_2 = rel_pos.x * rel_pos.x + rel_pos.y * rel_pos.y + rel_pos.z * rel_pos.z;
//
//        if (dis_2 < kFloatSmall || dis_2 > kDevSysPara.kernel_2) continue;
//
//        self_data->pos.w += __powf(kDevSysPara.kernel_2 - dis_2, 3);
//    }
//}
//
//__device__
//void knComputeCellForceTRA9(float3 *pres_kn, float3 *vis_kn, ParticleBufferList &buff_list, CFData *self_data, int cell_offset, int cell_num)
//{
//
//    if (0 == cell_num) return;
//    int end_idx = cell_offset + cell_num;
//    for (size_t i = cell_offset; i < end_idx; ++i)
//    {
//
//        float4 neighbor_pos = buff_list.position_d[i];
//        float4 neighbor_ev = buff_list.evaluated_velocity[i];
//
//        float3 rel_pos = cal_rePos(neighbor_pos, self_data->pos);
//        float dis_2 = rel_pos.x * rel_pos.x + rel_pos.y * rel_pos.y + rel_pos.z * rel_pos.z;
//
//        if (dis_2 < kFloatSmall || dis_2 > kDevSysPara.kernel_2) continue;
//
//        float dis = sqrtf(dis_2);
//        float V = 1 / (neighbor_pos.w);
//        float kernel_r = kDevSysPara.kernel - dis;
//
//
//        float temp_pres_kn = V * (self_data->ev.w + neighbor_ev.w) * kernel_r * kernel_r;
//        *pres_kn = *pres_kn - rel_pos * __fdividef(temp_pres_kn, dis);
//
//        float3 rel_vel = cal_rePos(self_data->ev, neighbor_ev);
//        float temp_vis_kn = V * kernel_r;
//        *vis_kn = *vis_kn + rel_vel * temp_vis_kn;
//
//        float temp = V * powf_2(kDevSysPara.kernel_2 - dis_2);
//        self_data->grad_color = self_data->grad_color + rel_pos * temp;
//        self_data->lplc_color += V * (kDevSysPara.kernel_2 - dis_2) *
//            (dis_2 - 3 / 4 * (kDevSysPara.kernel_2 - dis_2));
//    }
//}

__device__
void knComputeCellNeighbourCountTRA9(int& count, ParticleBufferList& buff_list, CFData* self_data, int cell_offset, int cell_num, int* neighborCounts)
{
    if (0 == cell_num) return;
    int end_idx = cell_offset + cell_num;
    for (size_t i = cell_offset; i < end_idx; ++i)
    {
        float3 neighbor_pos = buff_list.position_d[i];

        float3 rel_pos = cal_rePos(neighbor_pos, self_data->pos);
        float dis_2 = rel_pos.x * rel_pos.x + rel_pos.y * rel_pos.y + rel_pos.z * rel_pos.z;

        if (dis_2 < kFloatSmall || dis_2 > kDevSysPara.kernel_2) continue;

        count++;
    }
}

__device__
void knComputeCellNeighbourTRA9(int& count, int self_idx, ParticleBufferList& buff_list, CFData* self_data, int cell_offset, int cell_num, int* neighbors, int writeOffset)
{
    if (0 == cell_num) return;
    int end_idx = cell_offset + cell_num;
    for (size_t i = cell_offset; i < end_idx; ++i)
    {
        float3 neighbor_pos = buff_list.position_d[i];

        float3 rel_pos = cal_rePos(neighbor_pos, self_data->pos);
        float dis_2 = rel_pos.x * rel_pos.x + rel_pos.y * rel_pos.y + rel_pos.z * rel_pos.z;

        if (dis_2 < kFloatSmall || dis_2 > kDevSysPara.kernel_2) continue;

        neighbors[writeOffset + count] = i;
        count++;
    }
}

__device__
ushort3 calCI(const ushort3& in){
	ushort3 outd;
	outd.x = in.x >> 2;
	outd.y = in.y >> 2;
	outd.z = in.z >> 2;
	return outd;
}
//__global__ //__launch_bounds__(64, 10)
//void kncomputeDensityHybrid128n(int *cell_offset_M, ParticleIdxRange range, ParticleBufferList buff_list, int *cindex, int *cell_offset, int *cell_num, BlockTask *block_task, int bt_offset)
//{
//    if (blockIdx.x < bt_offset) {
//        int self_idx = threadIdx.x + __umul24(blockIdx.x, blockDim.x) + range.begin;
//        if (self_idx >= range.end) return;
//        self_idx = cindex[self_idx];
//
//        CDAPData self_data;
//        self_data.pos = buff_list.position_d[self_idx];
//        self_data.pos.w = 0;
//        ushort3 cell_posc = ParticlePos2CellPosM(self_data.pos, kDevSysPara.cell_size);
//
//		ushort3 cell_pos = calCI(cell_posc);
//		int xxx = (cell_posc.x) & 0x03;
//
//        ushort3 grid_size = kDevSysPara.grid_size;
//        int cell_offset_;
//        int cell_nump_;
//
//        for (int i = 0; i < 9; i++){
//            ushort3 neighbor_pos = cell_pos + make_ushort3(-1, i % 3 - 1, i / 3 % 3 - 1);
//            if (neighbor_pos.y < 0 || neighbor_pos.y >= grid_size.y ||
//                neighbor_pos.z < 0 || neighbor_pos.z >= grid_size.z) {
//                continue;
//            }
//            else {
//                int nid_left, nid_mid, nid_right;
//                nid_left = CellPos2CellIdx(neighbor_pos, grid_size);
//                ++neighbor_pos.x;
//                nid_mid = CellPos2CellIdx(neighbor_pos, grid_size);
//                ++neighbor_pos.x;
//                nid_right = CellPos2CellIdx(neighbor_pos, grid_size);
//
//				cell_offset_ =
//					kInvalidCellIdx == nid_left ? cell_offset[nid_mid] : cell_offset_M[(nid_left << 6) + (xxx << 4)];
//				cell_nump_ = cell_num[nid_mid];
//				if (kInvalidCellIdx != nid_left) cell_nump_ += cell_offset[nid_mid] - cell_offset_M[(nid_left << 6) + (xxx << 4)];//cell_nump[nid_left];
//				if (xxx == 3){
//					if (kInvalidCellIdx != nid_right) cell_nump_ += cell_num[nid_right];
//				}
//				else{
//					if (kInvalidCellIdx != nid_right) cell_nump_ += cell_offset_M[(nid_right << 6) + ((xxx + 1) << 4)] - cell_offset_M[(nid_right << 6)];//cell_nump[nid_right];
//				}
//
//                knComputeCellDensityTRA9(buff_list, &self_data, cell_offset_, cell_nump_);
//            }
//        }
//
//        self_data.pos.w *= kDevSysPara.mass * kDevSysPara.poly6_value;
//        self_data.pos.w += kDevSysPara.self_density;
//        buff_list.position_d[self_idx].w = self_data.pos.w;
//        buff_list.evaluated_velocity[self_idx].w = (__powf(__fdividef(self_data.pos.w, kDevSysPara.rest_density), 7) - 1) * kDevSysPara.gas_constant;
//
//		float denv = (5000 - buff_list.position_d[self_idx].w) / 6000;
//		buff_list.color[self_idx] = COLORA(1.0f*denv, 0.f, 1.0*denv, 1.0);
//    }
//    else{
//
//        int t = blockIdx.x - bt_offset;
//		int n = (t << 1) + (threadIdx.x >> 5);
//        BlockTask bt = block_task[n];
//		int isSame = bt.isSame;
//	
//        int cell_id = bt.cellid;
//		ushort3 cellpos = CellIdx2CellPos(cell_id, kDevSysPara.grid_size);
//
//        int self_idx = cell_offset[cell_id] + bt.p_offset + threadIdx.x % 32;
//
//        int temp_cell_end = cell_offset[cell_id] + cell_num[cell_id];
//
//        float total_density = 0.0f;
//        CDAPData data;
//
//        if (self_idx < temp_cell_end)   // initialize self data
//        {
//            data.pos = buff_list.position_d[self_idx];
//            data.pos.w = 0;
//        }
//        __shared__ SimDenSharedData128 sdata;
//		sdata.initialize(bt.zzi, bt.zzz, bt.xxi, bt.xxx, cell_offset_M, isSame, cell_offset, cell_num, cellpos, kDevSysPara.grid_size);
//        while (true)
//        {
//			__syncthreads();
//			int r = sdata.read32Data(cell_offset_M, isSame, buff_list);
//			__syncthreads();
//            if (0 == r) break;  // neighbor cells read complete
//            if (self_idx < temp_cell_end)
//            {
//				knComputeCellDensitySMS64(isSame, &sdata, &data, r);
//            }
//        }
//        if (self_idx < temp_cell_end)
//        {
//            data.pos.w *= kDevSysPara.mass * kDevSysPara.poly6_value;
//            data.pos.w += kDevSysPara.self_density;
//            buff_list.position_d[self_idx].w = data.pos.w < kFloatSmall ? kDevSysPara.rest_density : data.pos.w;
//            buff_list.evaluated_velocity[self_idx].w = (powf_7(__fdividef(data.pos.w, kDevSysPara.rest_density)) - 1) * kDevSysPara.gas_constant;
//
//			float denv = (5000 - buff_list.position_d[self_idx].w) / 6000;
//			if (isSame == 1){
//				buff_list.color[self_idx] = COLORA(0.f, 1.0f*denv, 1.0*denv, 1.0);
//			}
//			else{
//				buff_list.color[self_idx] = COLORA(1.0f*denv, 1.0f*denv, 0.f, 1.0);
//			}
//
//
//        }
//    }
//}
//__global__ //__launch_bounds__(64, 10)
//void kncomputeForceHybrid128n(int *cell_offset_M,ParticleIdxRange range, ParticleBufferList buff_list, int *cindex, int *cell_offset, int *cell_num, BlockTask *block_task, int bt_offset)
//{
//    if (blockIdx.x < bt_offset){
//        int self_idx = threadIdx.x + __umul24(blockIdx.x, blockDim.x) + range.begin;
//        if (self_idx >= range.end) return;
//        self_idx = cindex[self_idx];
//
//        CFData self_data;
//        self_data.pos = buff_list.position_d[self_idx];
//        self_data.ev = buff_list.evaluated_velocity[self_idx];
//       
//
//        self_data.grad_color = make_float3(0.0f, 0.0f, 0.0f);
//        self_data.lplc_color = 0.0f;
//
//
//		ushort3 cell_posc = ParticlePos2CellPosM(self_data.pos, kDevSysPara.cell_size);
//
//		ushort3 cell_pos = calCI(cell_posc);
//		int xxx = (cell_posc.x) & 0x03;
//        float3 pres_kn = make_float3(0.0f, 0.0f, 0.0f);
//        float3 vis_kn = make_float3(0.0f, 0.0f, 0.0f);
//
//
//
//
//        ushort3 grid_size = kDevSysPara.grid_size;
//        int cell_offset_;
//        int cell_nump_;
//
//        for (int i = 0; i < 9; i++){
//            ushort3 neighbor_pos = cell_pos + make_ushort3(-1, i % 3 - 1, i / 3 % 3 - 1);
//            if (neighbor_pos.y < 0 || neighbor_pos.y >= grid_size.y ||
//                neighbor_pos.z < 0 || neighbor_pos.z >= grid_size.z) {
//                continue;
//            }
//            else {
//                int nid_left, nid_mid, nid_right;
//                nid_left = CellPos2CellIdx(neighbor_pos, grid_size);
//                ++neighbor_pos.x;
//                nid_mid = CellPos2CellIdx(neighbor_pos, grid_size);
//                ++neighbor_pos.x;
//                nid_right = CellPos2CellIdx(neighbor_pos, grid_size);
//
//				cell_offset_ =
//					kInvalidCellIdx == nid_left ? cell_offset[nid_mid] : cell_offset_M[(nid_left << 6) + (xxx << 4)];
//				cell_nump_ = cell_num[nid_mid];
//				if (kInvalidCellIdx != nid_left) cell_nump_ += cell_offset[nid_mid] - cell_offset_M[(nid_left << 6) + (xxx << 4)];//cell_nump[nid_left];
//				if (xxx == 3){
//					if (kInvalidCellIdx != nid_right) cell_nump_ += cell_num[nid_right];
//				}
//				else{
//					if (kInvalidCellIdx != nid_right) cell_nump_ += cell_offset_M[(nid_right << 6) + ((xxx + 1) << 4)] - cell_offset_M[(nid_right << 6)];//cell_nump[nid_right];
//				}
//
//                knComputeCellForceTRA9(&pres_kn, &vis_kn, buff_list, &self_data, cell_offset_, cell_nump_);
//            }
//        }
//
//        float3 total_force = pres_kn * kDevSysPara.spiky_value / 2 + vis_kn * kDevSysPara.viscosity * kDevSysPara.visco_value;
//
//        self_data.grad_color *= kDevSysPara.grad_poly6 * kDevSysPara.mass;
//        self_data.lplc_color *= kDevSysPara.lplc_poly6 * kDevSysPara.mass;
//
//        self_data.lplc_color = __fdividef(self_data.lplc_color, self_data.pos.w);
//        float sur_nor = sqrtf(self_data.grad_color.x * self_data.grad_color.x +
//                              self_data.grad_color.y * self_data.grad_color.y +
//                              self_data.grad_color.z * self_data.grad_color.z);
//
//        float3 force;
//        if (sur_nor > kDevSysPara.surface_normal)
//        {
//            force = self_data.grad_color * kDevSysPara.surface_coe * self_data.lplc_color / sur_nor;
//        }
//        else
//        {
//            force = make_float3(0.0f, 0.0f, 0.0f);
//        }
//
//        total_force *= kDevSysPara.mass;
//        buff_list.acceleration[self_idx] = total_force + force;
//    }
//    else{
//
//        int t = blockIdx.x - bt_offset;
//		int n = (t<<1) + (threadIdx.x>>5);
//		BlockTask bt = block_task[n];
//		int isSame = bt.isSame;
//
//		int cell_id = bt.cellid;
//		ushort3 cellpos = CellIdx2CellPos(cell_id, kDevSysPara.grid_size);
//
//        int self_idx = cell_offset[cell_id] + bt.p_offset + threadIdx.x % 32;
//
//        int temp_cell_end = cell_offset[cell_id] + cell_num[cell_id];
//
//        float3 pres_kn = make_float3(0.0f, 0.0f, 0.0f);
//        float3 vis_kn = make_float3(0.0f, 0.0f, 0.0f);
//        CFData self_data;
//
//        if (self_idx < temp_cell_end)   // init self data
//        {
//            self_data.pos = buff_list.position_d[self_idx];
//            self_data.ev = buff_list.evaluated_velocity[self_idx];
//            self_data.grad_color = make_float3(0.0f, 0.0f, 0.0f);
//            self_data.lplc_color = 0.0f;
//        }
//
//        __shared__ SimForSharedData128 sdata;
//		sdata.initialize(bt.zzi, bt.zzz, bt.xxi, bt.xxx, cell_offset_M, isSame, cell_offset, cell_num, cellpos, kDevSysPara.grid_size);
//        while (true)
//        {
//			__syncthreads();
//			int r = sdata.read32Data(cell_offset_M, isSame, buff_list);
//			__syncthreads();
//            if (0 == r) break;  // neighbor cells read complete
//            
//            if (self_idx < temp_cell_end)
//            {
//                knComputeCellForceSMS64(isSame, &pres_kn, &vis_kn, &sdata, &self_data, r);
//            }
//        }
//        if (self_idx < temp_cell_end)
//        {
//            float3 total_force = pres_kn * kDevSysPara.spiky_value / 2 + vis_kn * kDevSysPara.viscosity * kDevSysPara.visco_value;
//
//            self_data.grad_color *= kDevSysPara.grad_poly6 * kDevSysPara.mass;
//            self_data.lplc_color *= kDevSysPara.lplc_poly6 * kDevSysPara.mass;
//
//            self_data.lplc_color = __fdividef(self_data.lplc_color, self_data.pos.w);
//            float sur_nor = sqrtf(self_data.grad_color.x * self_data.grad_color.x +
//                                  self_data.grad_color.y * self_data.grad_color.y +
//                                  self_data.grad_color.z * self_data.grad_color.z);
//
//            float3 force;
//            if (sur_nor > kDevSysPara.surface_normal)
//            {
//                force = self_data.grad_color * kDevSysPara.surface_coe * self_data.lplc_color / sur_nor;
//            }
//            else
//            {
//                force = make_float3(0.0f, 0.0f, 0.0f);
//            }
//
//            total_force *= kDevSysPara.mass;
//            buff_list.acceleration[self_idx] = total_force + force;
//        }
//    }
//}

__global__
void kncomputeNeighboursCount(int* cell_offset_M, ParticleIdxRange range, ParticleBufferList buff_list, int* cindex, int* cell_offset, int* cell_num, BlockTask* block_task, int bt_offset, int* neighborCounts)
{
    if (blockIdx.x < bt_offset) {
        int self_idx = threadIdx.x + __umul24(blockIdx.x, blockDim.x) + range.begin;
        if (self_idx >= range.end) return;
        self_idx = cindex[self_idx];

        int count = 0;

        CFData self_data;
        self_data.pos = buff_list.position_d[self_idx];

        ushort3 cell_posc = ParticlePos2CellPosM(self_data.pos, kDevSysPara.cell_size);

        ushort3 cell_pos = calCI(cell_posc);
        int xxx = (cell_posc.x) & 0x03;

        ushort3 grid_size = kDevSysPara.grid_size;
        int cell_offset_;
        int cell_nump_;

        for (int i = 0; i < 9; i++) {
            ushort3 neighbor_pos = cell_pos + make_ushort3(-1, i % 3 - 1, i / 3 % 3 - 1);
            if (neighbor_pos.y < 0 || neighbor_pos.y >= grid_size.y ||
                neighbor_pos.z < 0 || neighbor_pos.z >= grid_size.z) {
                continue;
            }
            else {
                int nid_left, nid_mid, nid_right;
                nid_left = CellPos2CellIdx(neighbor_pos, grid_size);
                ++neighbor_pos.x;
                nid_mid = CellPos2CellIdx(neighbor_pos, grid_size);
                ++neighbor_pos.x;
                nid_right = CellPos2CellIdx(neighbor_pos, grid_size);

                cell_offset_ = kInvalidCellIdx == nid_left ? cell_offset[nid_mid] : cell_offset_M[(nid_left << 6) + (xxx << 4)];
                cell_nump_ = cell_num[nid_mid];
                if (kInvalidCellIdx != nid_left) cell_nump_ += cell_offset[nid_mid] - cell_offset_M[(nid_left << 6) + (xxx << 4)];//cell_nump[nid_left];
                if (xxx == 3) {
                    if (kInvalidCellIdx != nid_right) cell_nump_ += cell_num[nid_right];
                }
                else {
                    if (kInvalidCellIdx != nid_right) cell_nump_ += cell_offset_M[(nid_right << 6) + ((xxx + 1) << 4)] - cell_offset_M[(nid_right << 6)];//cell_nump[nid_right];
                }

                knComputeCellNeighbourCountTRA9(count, buff_list, &self_data, cell_offset_, cell_nump_, neighborCounts);
            }
        }
        neighborCounts[self_idx] = count;
    }
    else {
        int t = blockIdx.x - bt_offset;
        int n = (t << 1) + (threadIdx.x >> 5);
        BlockTask bt = block_task[n];
        int isSame = bt.isSame;

        int cell_id = bt.cellid;
        ushort3 cellpos = CellIdx2CellPos(cell_id, kDevSysPara.grid_size);

        int self_idx = cell_offset[cell_id] + bt.p_offset + threadIdx.x % 32;

        int temp_cell_end = cell_offset[cell_id] + cell_num[cell_id];

        CFData self_data;

        if (self_idx < temp_cell_end)   // init self data
        {
            self_data.pos = buff_list.position_d[self_idx];
        }

        int count = 0;

        __shared__ SimNeighSharedData128 sdata;
        sdata.initialize(bt.zzi, bt.zzz, bt.xxi, bt.xxx, cell_offset_M, isSame, cell_offset, cell_num, cellpos, kDevSysPara.grid_size);
        while (true)
        {
            __syncthreads();
            int r = sdata.read32Data(cell_offset_M, isSame, buff_list);
            __syncthreads();
            if (0 == r) break;  // neighbor cells read complete

            if (self_idx < temp_cell_end)
            {
                knComputeCellNeighbourCountSMS64(count, isSame, &sdata, &self_data, r);
            }
        }
        neighborCounts[self_idx] = count;
    }
    //if (threadIdx.x + __umul24(blockIdx.x, blockDim.x) + range.begin == 18630 - 1) neighborCounts[threadIdx.x + __umul24(blockIdx.x, blockDim.x) + range.begin] = 22;
}

__global__ 
void kncomputeNeighbours(int* cell_offset_M, ParticleIdxRange range, ParticleBufferList buff_list, int* cindex, int* cell_offset, int* cell_num, BlockTask* block_task, int bt_offset, int* neighbours, int* neighboursOffset)
{
    if (blockIdx.x < bt_offset) {
        int self_idx = threadIdx.x + __umul24(blockIdx.x, blockDim.x) + range.begin;
        if (self_idx >= range.end) return;
        self_idx = cindex[self_idx];

        int count = 0;
        const int writeOffset = neighboursOffset[self_idx];

        CFData self_data;
        self_data.pos = buff_list.position_d[self_idx];

        ushort3 cell_posc = ParticlePos2CellPosM(self_data.pos, kDevSysPara.cell_size);

        ushort3 cell_pos = calCI(cell_posc);
        int xxx = (cell_posc.x) & 0x03;

        ushort3 grid_size = kDevSysPara.grid_size;
        int cell_offset_;
        int cell_nump_;

        for (int i = 0; i < 9; i++) {
            ushort3 neighbor_pos = cell_pos + make_ushort3(-1, i % 3 - 1, i / 3 % 3 - 1);
            if (neighbor_pos.y < 0 || neighbor_pos.y >= grid_size.y ||
                neighbor_pos.z < 0 || neighbor_pos.z >= grid_size.z) {
                continue;
            }
            else {
                int nid_left, nid_mid, nid_right;
                nid_left = CellPos2CellIdx(neighbor_pos, grid_size);
                ++neighbor_pos.x;
                nid_mid = CellPos2CellIdx(neighbor_pos, grid_size);
                ++neighbor_pos.x;
                nid_right = CellPos2CellIdx(neighbor_pos, grid_size);

                cell_offset_ = kInvalidCellIdx == nid_left ? cell_offset[nid_mid] : cell_offset_M[(nid_left << 6) + (xxx << 4)];
                cell_nump_ = cell_num[nid_mid];
                if (kInvalidCellIdx != nid_left) cell_nump_ += cell_offset[nid_mid] - cell_offset_M[(nid_left << 6) + (xxx << 4)];//cell_nump[nid_left];
                if (xxx == 3) {
                    if (kInvalidCellIdx != nid_right) cell_nump_ += cell_num[nid_right];
                }
                else {
                    if (kInvalidCellIdx != nid_right) cell_nump_ += cell_offset_M[(nid_right << 6) + ((xxx + 1) << 4)] - cell_offset_M[(nid_right << 6)];//cell_nump[nid_right];
                }

                knComputeCellNeighbourTRA9(count, self_idx, buff_list, &self_data, cell_offset_, cell_nump_, neighbours, writeOffset);
            }
        }

    }
    else {
        int t = blockIdx.x - bt_offset;
        int n = (t << 1) + (threadIdx.x >> 5);
        BlockTask bt = block_task[n];
        int isSame = bt.isSame;

        int cell_id = bt.cellid;
        ushort3 cellpos = CellIdx2CellPos(cell_id, kDevSysPara.grid_size);

        int self_idx = cell_offset[cell_id] + bt.p_offset + threadIdx.x % 32;

        int temp_cell_end = cell_offset[cell_id] + cell_num[cell_id];

        CFData self_data;
        int count = 0;
        int writeOffset = 0;

        if (self_idx < temp_cell_end)   // init self data
        {
            self_data.pos = buff_list.position_d[self_idx];
            writeOffset = neighboursOffset[self_idx];
        }

        __shared__ SimNeighSharedData128 sdata;
        sdata.initialize(bt.zzi, bt.zzz, bt.xxi, bt.xxx, cell_offset_M, isSame, cell_offset, cell_num, cellpos, kDevSysPara.grid_size);
        while (true)
        {
            __syncthreads();
            int r = sdata.read32Data(cell_offset_M, isSame, buff_list);
            __syncthreads();
            if (0 == r) break;  // neighbor cells read complete

            if (self_idx < temp_cell_end)
            {
                knComputeCellNeighbourSMS64(count, isSame, &sdata, &self_data, r, neighbours, writeOffset);
            }
        }
    }
}

void computeNeighbours(int* cell_offset_M, ParticleIdxRange range, ParticleBufferList buff_list_n, int* cindex, int* cell_offset, int* cell_num, BlockTask* block_task, int num_block, Neighbours* neigh) 
{
    int total_thread = range.end - range.begin;
    int num_thread = 64;
    int bt_offset = 0;
    int number_blocks = ceil_int(num_block, 2);
    if (total_thread > 0) {
        bt_offset = ceil_int(total_thread, 64);
        number_blocks += bt_offset;
    }
    if (number_blocks <= 0) return;

    neigh->dNeighborCounts.resize(neigh->particleCount);

    kncomputeNeighboursCount <<<number_blocks, num_thread>>> (cell_offset_M, range, buff_list_n, cindex, cell_offset, cell_num, block_task, bt_offset, thrust::raw_pointer_cast(neigh->dNeighborCounts.data())); //thrust::raw_pointer_cast(&(neigh.dNeighborCounts)[0]));
    CUDA_SAFE_CALL(hipDeviceSynchronize());
    neigh->dNeighborOffsets.resize(neigh->particleCount);

    CUDA_SAFE_CALL(hipDeviceSynchronize());

    //Exclusive scan over to figure out the new size of the neigbour counts
    thrust::exclusive_scan(
        neigh->dNeighborCounts.begin(),
        neigh->dNeighborCounts.end(),
        neigh->dNeighborOffsets.begin());

    CUDA_SAFE_CALL(hipDeviceSynchronize());

    //Compute total amount of neighbors
    uint lastOffset = 0;
    //hipMemcpy(thrust::raw_pointer_cast(&(neigh.dNeighborOffsets)[0]) + neigh.particleCount - 1, &lastOffset, 1, hipMemcpyDeviceToHost);
    CUDA_SAFE_CALL(hipMemcpy(&lastOffset, thrust::raw_pointer_cast(neigh->dNeighborOffsets.data()) + neigh->particleCount - 1, sizeof(int), hipMemcpyDeviceToHost));
    uint lastParticleNeighborCount = 0;
    CUDA_SAFE_CALL(hipMemcpy(&lastParticleNeighborCount, thrust::raw_pointer_cast(&(neigh->dNeighborCounts)[0]) + neigh->particleCount - 1, sizeof(int), hipMemcpyDeviceToHost));
    uint totalNeighborCount = lastOffset + lastParticleNeighborCount;
    neigh->dNeighbors.resize(totalNeighborCount);

    CUDA_SAFE_CALL(hipDeviceSynchronize());

    kncomputeNeighbours <<<number_blocks, num_thread >>> (cell_offset_M, range, buff_list_n, cindex, cell_offset, cell_num, block_task, bt_offset, thrust::raw_pointer_cast(&neigh->dNeighbors[0]), thrust::raw_pointer_cast(&neigh->dNeighborOffsets[0]));

    CUDA_SAFE_CALL(hipDeviceSynchronize());

    //Transfer device data to host
    if (neigh->neighborsSize < sizeof(int) * totalNeighborCount)
    {
        if (neigh->neighborsSize != 0)
        {
            hipHostFree(neigh->neighbors);
        }

        neigh->neighborsSize = sizeof(int) * static_cast<int>(totalNeighborCount * 1.5 * 64); //Increase allocation size to force hip to actually allocate. What the hell HIP?
        hipHostMalloc(&neigh->neighbors, neigh->neighborsSize);
    }
    if (neigh->countsSize < sizeof(int) * neigh->particleCount)
    {
        if (neigh->countsSize != 0)
        {
            hipHostFree(neigh->offsets);
            hipHostFree(neigh->counts);
        }

        neigh->countsSize = sizeof(int) * static_cast<int>(neigh->particleCount * 1.5 * 64); //Increase allocation size to force hip to actually allocate. What the hell HIP?
        hipHostMalloc(&neigh->offsets, neigh->countsSize);
        hipHostMalloc(&neigh->counts, neigh->countsSize);
    }

    

    CUDA_SAFE_CALL(hipMemcpy(neigh->neighbors, thrust::raw_pointer_cast(&neigh->dNeighbors[0]), sizeof(int) * totalNeighborCount, hipMemcpyDeviceToHost));
    CUDA_SAFE_CALL(hipMemcpy(neigh->counts, thrust::raw_pointer_cast(&neigh->dNeighborCounts[0]), sizeof(int) * neigh->particleCount, hipMemcpyDeviceToHost));
    CUDA_SAFE_CALL(hipMemcpy(neigh->offsets, thrust::raw_pointer_cast(&neigh->dNeighborOffsets[0]), sizeof(int) * neigh->particleCount, hipMemcpyDeviceToHost));

    //Try to sum all the values on the gpu and see if it corresponds to the data we get from our CPU values.
    int n0 = thrust::reduce(thrust::device, thrust::raw_pointer_cast(&neigh->dNeighbors[0]), thrust::raw_pointer_cast(&neigh->dNeighbors[0]) + totalNeighborCount);
    printf("GPU neighbor sum: %d\n", n0);
    int n1 = std::accumulate(neigh->neighbors, neigh->neighbors + totalNeighborCount, 0);
    printf("CPU neighbor sum: %d\n", n1);

    n0 = thrust::reduce(thrust::device, thrust::raw_pointer_cast(&neigh->dNeighborCounts[0]), thrust::raw_pointer_cast(&neigh->dNeighborCounts[0]) + neigh->particleCount);
    printf("GPU counts sum: %d\n", n0);
    n1 = std::accumulate(neigh->counts, neigh->counts + neigh->particleCount, 0);
    printf("CPU counts sum: %d\n", n1);

    n0 = thrust::reduce(thrust::device, thrust::raw_pointer_cast(&neigh->dNeighborOffsets[0]), thrust::raw_pointer_cast(&neigh->dNeighborOffsets[0]) + neigh->particleCount);
    printf("GPU offsets sum: %d\n", n0);
    n1 = std::accumulate(neigh->offsets, neigh->offsets + neigh->particleCount, 0);
    printf("CPU offsets sum: %d\n", n1);
    
    //Return host arrays neighbours per particle, number of neighbours per particle, offset for each particle
}

//void computeDensityHybrid128n(int *cell_offset_M, ParticleIdxRange range, ParticleBufferList buff_list_n, int* cindex, int *cell_offset, int *cell_num, BlockTask *block_task, int num_block){
//
//    int total_thread = range.end - range.begin;
//    int num_thread = 64;
//    int bt_offset = 0;
//    int number_blocks = ceil_int(num_block, 2);
//    if (total_thread > 0){
//        bt_offset = ceil_int(total_thread, 64);
//        number_blocks += bt_offset;
//    }
//    if (number_blocks <= 0) return;
//
//    size_t offset = 0;
//
//	kncomputeDensityHybrid128n <<<number_blocks, num_thread >>>(cell_offset_M, range, buff_list_n, cindex, cell_offset, cell_num, block_task, bt_offset);
//}
//
//void computeForceHybrid128n(int *cell_offset_M, ParticleIdxRange range, ParticleBufferList buff_list_n, int* cindex, int *cell_offset, int *cell_num, BlockTask *block_task, int num_block){
//    int total_thread = range.end - range.begin;
//    int num_thread = 64;
//    int bt_offset = 0;
//    int number_blocks = ceil_int(num_block, 2);
//    if (total_thread > 0){
//        bt_offset = ceil_int(total_thread, 64);
//        number_blocks += bt_offset;
//    }
//    if (number_blocks <= 0) return;
//
//    size_t offset = 0;
//
//	kncomputeForceHybrid128n <<<number_blocks, num_thread >>>(cell_offset_M, range, buff_list_n, cindex, cell_offset, cell_num, block_task, bt_offset);
//}

//void manualSetting(ParticleBufferList buff_list, int nump, int step)
//{
//    int num_thread = kDefaultNumThreadTRA;
//    int num_block = ceil_int(nump, num_thread);
//
//    knManualSetting <<<num_block, num_thread >>>(buff_list, nump, step);
//}
//
//sf 更新位置速度
//void advance(ParticleBufferList buff_list, int nump)
//{
//    int num_thread = kDefaultNumThreadTRA;
//    int num_block = ceil_int(nump, num_thread);
//
//    knIntegrateVelocityE <<<num_block, num_thread >>>(buff_list, nump);
//}

void find_max_P(int blocks, int tds, sumGrad *id_value, int numbers)
{
    int iSize = 1;
    while (iSize < numbers)
    {
        iSize <<= 1;
        find_max <<<blocks, tds >>>(id_value, numbers, iSize);
    }
}

//pscisph over--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//sf finally namespace sph-------------------------------------------------
}