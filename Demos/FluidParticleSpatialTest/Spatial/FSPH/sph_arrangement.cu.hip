//
// sph_arrangement.cu
// Hybrid_Parallel_SPH
//
// created by ruanjm on 2016/04/05
// Copyright (c) 2016 ruanjm. All rights reserved.
//


//#include <device_launch_parameters.h>
#include <thrust/scan.h>
#include <thrust/device_ptr.h>
#include <hip/hip_runtime.h>
#include "sph_arrangement.cuh.hip"
#include "scan.cuh.hip"
#include "gpu_model.cuh.hip"
#include "sph_utils.cuh.hip"

#include<fstream>

#include <thrust/count.h>
#include <thrust/execution_policy.h>
#include <numeric>

namespace sph
{


/****************************GPU_COUNT_SORT**************************/

__global__
void CountingSort_Offset(int *cell_offset, int cellNum, int iSize)
{
    int x_id = __umul24(blockDim.x, blockIdx.x) + threadIdx.x;
    x_id++;
    if (x_id <= cellNum)
    {
        int P = x_id & (iSize - 1);
        if (0 == P)
            P = iSize;
        if (P > (iSize >> 1))
        {
            x_id--;
            cell_offset[x_id] = cell_offset[x_id] + cell_offset[x_id + (iSize >> 1) - P];
        }
    }
}

__global__ void CountingSort_Cell_Sum_two_M(int* hashp, int *p_offset, int *hashId, int *cell_numbers, int iSize, int *block_reqs, int numc)
{
	int x_id = __umul24(blockDim.x, blockIdx.x) + threadIdx.x;
	if (x_id < iSize)
	{
		int selfHash = (hashId[x_id] >> 6);


		selfHash = block_reqs[selfHash] <<5 > p_offset[x_id] ? (selfHash + numc) : selfHash;

		hashp[x_id] = selfHash;
		p_offset[x_id] = atomicAdd(cell_numbers + selfHash, 1);
	}
}

void CountingSort_Offest_P(int block_cell, int cell_s, int *cell_offset, int cellNum)
{
    int iSize = 1;
    while (iSize < cellNum)
    {
        iSize <<= 1;
        CountingSort_Offset <<<block_cell, cell_s >>>(cell_offset, cellNum, iSize);
    }
}

__global__ void CountingSort_Result_two9(int *p_offset, int *hash, int *hash_new, int *cell_offset, int *index,int num)
{
    int id = __umul24(blockDim.x, blockIdx.x) + threadIdx.x;
    if (id < num)
    {
        int iStart = 0;
        int x_id = hash[id];
        if (0 != x_id)
            iStart = cell_offset[x_id - 1];
        int p_id = iStart + p_offset[id];
        index[p_id] = id;
        hash_new[p_id] = x_id;
    }
}

__device__
inline int hash2cellid(int base, int num) { return base % num; }
__device__
inline int hash2blockeq(int base, int num) { return base / num; }

__global__
void knFindHybridModeMiddleValue(int numc, int *mid_val, int *hash, unsigned int nump)
{
    extern __shared__ int shared_hash[];
    unsigned int idx = threadIdx.x + __umul24(blockDim.x, blockIdx.x);
    if (idx == 0){ *mid_val = -1; }
    int self_hash;
    if (idx < nump){
        self_hash = hash[idx];
        shared_hash[threadIdx.x + 1] = self_hash;
        if (idx > 0 && threadIdx.x == 0){
            shared_hash[0] = hash[idx - 1];
        }
    }
    __syncthreads();
    if (idx < nump) {
        int prior_hash = idx == 0 ? -1 : shared_hash[threadIdx.x];
        if (self_hash != prior_hash){
            if (hash2blockeq(self_hash, numc) > 0) {
                if (idx == 0 || hash2blockeq(prior_hash, numc) == 0){
                    *mid_val = idx;
                }
            }
        }
    }
}

/**********************************************************************************************************/


/****************************** Kernel ******************************/

#define HASH2CELLIDX(X) (X & 0x3fffffff)
#define HASH2BLOCKREQ(X) (X >> 30)

//__global__
//void countCellNum(int *hash, float3 *position, float cell_size, ushort3 grid_size, int *cell_num, unsigned int nump)
//{
//    unsigned int idx = threadIdx.x + __umul24(blockDim.x, blockIdx.x);
//    if (idx >= nump) return;
//    int hav = ParticlePos2CellIdx(position[idx], grid_size, cell_size);
//    hash[idx] = hav;
//    atomicAdd(cell_num + hav, 1);
//}

/****************************** Arrangement ******************************/

Arrangement::Arrangement(ParticleBufferObject &buff_list, ParticleBufferObject &buff_temp,unsigned int nump, unsigned int nump_capacity, float cell_size, ushort3 grid_size)
    : buff_list_(buff_list), buff_temp_(buff_temp),  nump_(nump), nump_capacity_(nump_capacity), cell_size_(cell_size), grid_size_(grid_size)
{
    numc_ = grid_size.x * grid_size.y * grid_size.z;
	printf("Number of cells: %d\n", numc_);

    CUDA_SAFE_CALL(hipMalloc(&d_hash_, nump_capacity * sizeof(int)));

    CUDA_SAFE_CALL(hipMalloc(&d_index_, nump_capacity * sizeof(int)));

    // SMS
	CUDA_SAFE_CALL(hipMalloc(&d_hash_p, nump_capacity * sizeof(int)));

    CUDA_SAFE_CALL(hipMalloc(&hashp, nump_ * sizeof(int)));

    CUDA_SAFE_CALL(hipMalloc(&d_block_reqs_, numc_ * sizeof(int)));



    CUDA_SAFE_CALL(hipMalloc(&d_task_array_offset_32_, numc_ * sizeof(int)));


    CUDA_SAFE_CALL(hipMalloc(&d_breqs_offset_, numc_ * sizeof(int)));

    CUDA_SAFE_CALL(hipMalloc(&cell_num_two, (2 * numc_+1) * sizeof(int)));

    CUDA_SAFE_CALL(hipMalloc(&cell_type, numc_ * sizeof(int)));


    CUDA_SAFE_CALL(hipMalloc(&d_block_task_, numc_ * 10 * sizeof(BlockTask)));

    CUDA_SAFE_CALL(hipMalloc(&d_num_block_, sizeof(int)));

    CUDA_SAFE_CALL(hipMalloc(&d_middle_value_, sizeof(int)));


    CUDA_SAFE_CALL(hipMalloc(&d_num_cta_, sizeof(int)));

    CUDA_SAFE_CALL(hipMalloc(&d_cell_offset_, (numc_+1) * sizeof(int)));

    CUDA_SAFE_CALL(hipMalloc(&d_cell_nump_, numc_ * sizeof(int)));


    CUDA_SAFE_CALL(hipMalloc(&d_p_offset_, nump_ * sizeof(int)));

    CUDA_SAFE_CALL(hipMalloc(&d_p_offset_p, nump_ * sizeof(int)));

	CUDA_SAFE_CALL(hipMalloc(&d_cell_offset_M, (numc_ * 64 + 1) * sizeof(int)));

	CUDA_SAFE_CALL(hipMalloc(&d_cell_nump_M, (numc_ * 64 + 1) * sizeof(int)));


	CUDA_SAFE_CALL(hipMalloc(&part2Idx, nump_ * sizeof(int)));
	CUDA_SAFE_CALL(hipMalloc(&idx2Part, nump_ * sizeof(int)));


    preallocBlockSumsInt(numc_);

    CUDA_SAFE_CALL(hipMemset(d_num_cta_, 0, sizeof(int)));

    gpu_model::allocateGPUModel(p_gpu_model_);
}

Arrangement::~Arrangement()
{
    CUDA_SAFE_CALL(hipFree(cell_type));
    CUDA_SAFE_CALL(hipFree(d_task_array_offset_32_));
    CUDA_SAFE_CALL(hipFree(d_p_offset_));


    CUDA_SAFE_CALL(hipFree(d_p_offset_p));

    CUDA_SAFE_CALL(hipFree(hashp));
    CUDA_SAFE_CALL(hipFree(d_hash_));
	CUDA_SAFE_CALL(hipFree(d_hash_p));
    CUDA_SAFE_CALL(hipFree(d_index_));
    CUDA_SAFE_CALL(hipFree(d_block_reqs_));
    CUDA_SAFE_CALL(hipFree(d_breqs_offset_));
    CUDA_SAFE_CALL(hipFree(d_block_task_));
    CUDA_SAFE_CALL(hipFree(d_num_block_));
    CUDA_SAFE_CALL(hipFree(d_middle_value_));
    CUDA_SAFE_CALL(hipFree(cell_num_two));

    CUDA_SAFE_CALL(hipFree(d_num_cta_));
    CUDA_SAFE_CALL(hipFree(d_cell_offset_));
    CUDA_SAFE_CALL(hipFree(d_cell_nump_));
	CUDA_SAFE_CALL(hipFree(d_cell_offset_M));
	CUDA_SAFE_CALL(hipFree(d_cell_nump_M));

    deallocBlockSumsInt();

    gpu_model::freeGPUModel(p_gpu_model_);
}

__global__
void knArrangeTasksFixedM(int *hash, int* celloff, int *cellnum, BlockTask *block_tasks, int *num_block, int *block_reqs, int *breqs_offset, ushort3 grid_size, int cta_size, int numc) {
	unsigned int idx = threadIdx.x + __umul24(blockDim.x, blockIdx.x);

	if (idx >= numc) return;
	int start = celloff[idx];
	int offset = breqs_offset[idx];
	int numb = block_reqs[idx];
	int p_offset = 0;
	int nump = cellnum[idx];

	for (int i = offset; i < offset + numb; ++i) {
		int hashA = hash[start + p_offset];
		int xxi = ((hashA & 0x030) >> 4);
		int zzi = ((hashA & 0x0c) >> 2);
		BlockTask bt;
		bt.p_offset = p_offset;
		bt.cellid = idx;
		p_offset += cta_size;
		int  hashB;
		if (p_offset >= nump){
			hashB = hash[start + nump - 1];

		}
		else{
			hashB = hash[start + p_offset - 1];
		}
		int xxx = ((hashB & 0x030) >> 4);
		int zzz = ((hashB & 0x0c) >> 2);
		bt.xxi = xxi;
		bt.xxx = xxx;

		if (xxi == xxx){
			bt.zzi = zzi;
			bt.zzz = zzz;
		}
		else{
			bt.zzi = 0;
			bt.zzz = 3;
		}
		block_tasks[i] = bt;
	}

	if (idx == numc - 1) {
		*num_block = offset + numb;
	}
}

__global__
void judgeTask(BlockTask *block_tasks, int *num_block) {
	unsigned int idx = threadIdx.x + __umul24(blockDim.x, blockIdx.x);
	int numb = num_block[0];
	if (idx >= numb) return;
	if (numb % 2 == 0){
		if (idx % 2 == 0){
			if (block_tasks[idx].cellid == block_tasks[idx + 1].cellid){
				block_tasks[idx].isSame = 1;
				block_tasks[idx + 1].isSame = 1;
				if (block_tasks[idx].xxi == block_tasks[idx + 1].xxx){
					block_tasks[idx].zzz = block_tasks[idx + 1].zzz;
				}
				else{
					block_tasks[idx].zzi = 0;
					block_tasks[idx].zzz = 3;
				}
				block_tasks[idx].xxx = block_tasks[idx + 1].xxx;
			}
			else{
				block_tasks[idx].isSame = 0;
				block_tasks[idx + 1].isSame = 0;
			}
		}
	}
	else{
		if (idx % 2 == 0 && idx<numb - 1){
			if (block_tasks[idx].cellid == block_tasks[idx + 1].cellid){
				block_tasks[idx].isSame = 1;
				block_tasks[idx + 1].isSame = 1;
				if (block_tasks[idx].xxi == block_tasks[idx + 1].xxx){
					block_tasks[idx].zzz = block_tasks[idx + 1].zzz;
				}
				else{
					block_tasks[idx].zzi = 0;
					block_tasks[idx].zzz = 3;
				}
				block_tasks[idx].xxx = block_tasks[idx + 1].xxx;
			}
			else{
				block_tasks[idx].isSame = 0;
				block_tasks[idx + 1].isSame = 0;
			}
		}
		if (idx == numb - 1){
			block_tasks[idx].isSame = 1;
			block_tasks[idx + 1].isSame = 1;
			block_tasks[idx + 1].cellid = block_tasks[idx].cellid;
			block_tasks[idx + 1].p_offset = block_tasks[idx].p_offset + 32;
		}
	}
}

void Arrangement::arrangeBlockTasksFixedM(int *hash, int *celloff, int *cellnum, BlockTask* d_task_array, int* d_cta_reqs, int* d_task_array_offset, int cta_size) {
	int num_thread = 128;
	int num_block = ceil_int(numc_, num_thread);

	knArrangeTasksFixedM <<<num_block, num_thread >>>(hash, celloff, cellnum, d_task_array, d_num_cta_, d_cta_reqs, d_task_array_offset, grid_size_, cta_size, numc_);

	CUDA_SAFE_CALL(hipMemcpy(&h_num_cta_, d_num_cta_, sizeof(int), hipMemcpyDeviceToHost));

	judgeTask <<<ceil_int(h_num_cta_, num_thread), num_thread >>>(d_task_array, d_num_cta_);
}


int xixi = 0;

__global__ void CountingSort_Result_M(int *p_offset_p, int *p_offset, int *hash, int *hash_new, int *cell_offset, int num, ParticleBufferList old_data, ParticleBufferList new_data, int* p2i, int* i2p)
{
	int id = __umul24(blockDim.x, blockIdx.x) + threadIdx.x;
	if (id < num)
	{
		int x_id = hash[id];
		int iStart = cell_offset[x_id];
		int p_id = iStart + p_offset[id];
		hash_new[p_id] = x_id;
		int ici = (x_id & 0xffffffc0);
		p_offset_p[p_id] = p_offset[id] + iStart - cell_offset[ici];

		p2i[p_id] = id; //FIX reverse here, should hopefully work?
		i2p[id] = p_id;

		new_data.position_d[p_id] = old_data.position_d[id];
		//new_data.velocity[p_id] = old_data.velocity[id];
		//new_data.evaluated_velocity[p_id] = old_data.evaluated_velocity[id];
		//new_data.color[p_id] = old_data.color[id];
	}
}
__global__ void CountingSort_Cell_SumM(int *p_offset, int *hashId, int *cell_numbers, int iSize, float3 *position, float cell_size, ushort3 grid_size)
{
	int x_id = __umul24(blockDim.x, blockIdx.x) + threadIdx.x;

	if (x_id < iSize)
	{
		int hid = ParticlePos2CellIdxM(position[x_id], grid_size, cell_size);
		hashId[x_id] = hid;
		p_offset[x_id] = atomicAdd(cell_numbers + hid, 1);
	}
}
__global__ void calculate_cell_info(int *cell_Numx, int *cell_Offset, int *cell_Offset_M, int numc)
{
	int id = __umul24(blockDim.x, blockIdx.x) + threadIdx.x;
	if (id < numc)
	{
		int idi = (id << 6);
		cell_Numx[id] = cell_Offset_M[idi + 64] - cell_Offset_M[idi];
		cell_Offset[id] = cell_Offset_M[idi];
	}
}
void Arrangement::CountingSort_O_M()
{
	/*float3 f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("Beginning GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/
	int num_thread = 256;
	int num_block = ceil_int(nump_, num_thread);
	int numCN = (numc_ <<6);
	int num_blockc = ceil_int(numCN + 1, num_thread);

	hipMemset(d_cell_nump_M, 0x00, sizeof(int)* (numCN+1));
	
	CountingSort_Cell_SumM <<<num_block, num_thread >>>(d_p_offset_, d_hash_, d_cell_nump_M, nump_, buff_list_.get_buff_list().position_d, cell_size_, grid_size_);
	/*f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("CountingSort_Cell_SumM GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/

	int* tempHostHash = new int[nump_];
	CUDA_SAFE_CALL(hipMemcpy(tempHostHash, d_hash_, sizeof(int)*nump_, hipMemcpyDeviceToHost));
	for (int i = 0; i < nump_; i++)
	{
		if (tempHostHash[i] == kInvalidCellIdx)
		{
			printf("%d: Has an invalid index\n", i);
		}
	}
	delete[] tempHostHash;

	thrust::exclusive_scan(thrust::device_ptr<int>(d_cell_nump_M), thrust::device_ptr<int>(d_cell_nump_M) +numCN + 1, thrust::device_ptr<int>(d_cell_offset_M));
	/*f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("exclusive_scan GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/

	CountingSort_Result_M <<<num_block, num_thread >>>(d_p_offset_p, d_p_offset_, d_hash_, hashp, d_cell_offset_M, nump_, buff_list_.get_buff_list(), buff_temp_.get_buff_list(), part2Idx, idx2Part);
	
	/*f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("CountingSort_Result_M GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);
	f0 = thrust::reduce(thrust::device, buff_temp_.get_buff_list().position_d, buff_temp_.get_buff_list().position_d + nump_);
	printf("\tCountingSort_Result_M TEMP GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/

	calculate_cell_info <<<ceil_int(numc_, num_thread), num_thread >>>(d_cell_nump_, d_cell_offset_, d_cell_offset_M, numc_);
	/*f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("calculate_cell_info GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/

	int *p = d_hash_;
	d_hash_ = hashp;
	hashp = p;
	buff_list_.swapObj(buff_temp_);
	/*f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("Buffer swap GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/
}

void Arrangement::CountingSortCUDA_Two9_M()
{
	int numCell = numc_ << 1;
	int num_thread = 256;
	int num_block = ceil_int(nump_, num_thread);
	int num_blockc = ceil_int(numCell, num_thread);

	hipMemset(cell_num_two, 0x00, sizeof(int)* (numCell));
	
	CountingSort_Cell_Sum_two_M <<<num_block, num_thread >>>(d_hash_p, d_p_offset_p, d_hash_, cell_num_two, nump_, d_block_reqs_, numc_);
	CountingSort_Offest_P(num_blockc, num_thread, cell_num_two, numCell);


	CountingSort_Result_two9 <<<num_block, num_thread >>>(d_p_offset_p, d_hash_p, hashp, cell_num_two, d_index_, nump_);
	unsigned int shared_mem_size = (num_thread + 1) * sizeof(int);
	knFindHybridModeMiddleValue <<<num_block, num_thread, shared_mem_size >>>(numc_, d_middle_value_, hashp, nump_);
	CUDA_SAFE_CALL(hipMemcpy(&middle_value_, d_middle_value_, sizeof(int), hipMemcpyDeviceToHost));
}

int Arrangement::arrangeHybridMode9M(){
	/*float3 f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("Beginning GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/

	CountingSort_O_M();
	/*f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("CountingSort_O_M GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/

	gpu_model::calculateBlockRequirementHybridMode(cell_type, d_cell_nump_, d_block_reqs_, p_gpu_model_, d_cell_offset_, d_cell_nump_, grid_size_, 32);
	/*f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("calculateBlockRequirementHybridMode GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/

	CountingSortCUDA_Two9_M();
	/*f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("CountingSortCUDA_Two9_M GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/
    
	thrust::exclusive_scan(thrust::device_ptr<int>(d_block_reqs_), thrust::device_ptr<int>(d_block_reqs_) +numc_, thrust::device_ptr<int>(d_task_array_offset_32_));
	/*f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("exclusive_scan GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/
	
	arrangeBlockTasksFixedM(d_hash_, d_cell_offset_, d_cell_nump_, d_block_task_, d_block_reqs_, d_task_array_offset_32_, 32);
	/*f0 = thrust::reduce(thrust::device, buff_list_.get_buff_list().position_d, buff_list_.get_buff_list().position_d + nump_);
	printf("arrangeBlockTasksFixedM GPU sum: (%f,%f,%f)\n", f0.x, f0.y, f0.z);*/
	
	return (middle_value_ > nump_ || middle_value_ < 0) ? nump_ : middle_value_;
}

int* Arrangement::getDevCellStartIdx()
{
    return d_start_index_;
}

int* Arrangement::getDevCellEndIdx()
{
    return d_end_index_;
}

int Arrangement::getNumBlockSMSMode()
{
    return h_num_cta_;
}

BlockTask * Arrangement::getBlockTasks()
{
    return d_block_task_;
}

void Arrangement::resetNumParticle(unsigned int nump)
{
	nump_ = nump;

	if (nump_capacity_ < nump_)
	{
		CUDA_SAFE_CALL(hipFree(d_hash_));
		CUDA_SAFE_CALL(hipFree(d_index_));
		CUDA_SAFE_CALL(hipMalloc(&d_hash_, nump_ * sizeof(int)));
		CUDA_SAFE_CALL(hipMalloc(&d_index_, nump_ * sizeof(int)));
		nump_capacity_ = nump_;
	}
}

}