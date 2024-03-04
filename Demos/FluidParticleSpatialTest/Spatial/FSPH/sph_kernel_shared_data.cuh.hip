//
// sph_kernel_shared_data.cuh
// Hybrid_Parallel_SPH
//
// created by ruanjm on 2016/04/05
// Copyright (c) 2016 ruanjm. All rights reserved.
//

#ifndef _SPH_KERNEL_SHARED_DATA_CUH
#define _SPH_KERNEL_SHARED_DATA_CUH


#define LOG_NUM_BANKS_MINE	 5
#define CONFLICT_FREE_OFFSET_MINE(index) ((index) >> LOG_NUM_BANKS)


#include <hip/hip_runtime.h>
//#include <device_launch_parameters.h>
#include "cuda_math.cuh.hip"
#include "sph_particle.h.hip"
#include "sph_utils.cuh.hip"

namespace sph
{

const int kNumSharedData = 32;
const int kNumNeighborCells = 64;

const int rate = 2;

struct CDAPData
{
    float3 pos;
};

struct CFData
{
    float3  pos;
    /*float4  ev;
    float3  grad_color;
    float   lplc_color;*/
};

class SimDenSharedData128
{
public:
    __device__ SimDenSharedData128(){}

	__device__ void initialize(const int& minz, const int& maxz, const int& min, const int& max, int *celloffM, const int& isSame, int *cell_offset, int *cell_nump, ushort3 &cell_pos, const ushort3 &grid_size) {
        unsigned int idx = threadIdx.x;
        int bi = (idx>>5);
        int bj = idx % 32+(isSame)*(bi<<5);
        if (bj < 9) {
            int kk = bi * 9 + bj;
            ushort3 neighbor_pos = cell_pos + make_ushort3(-1, bj % 3 - 1, bj / 3 % 3 - 1);
            if (neighbor_pos.y < 0 || neighbor_pos.y >= grid_size.y ||
                neighbor_pos.z < 0 || neighbor_pos.z >= grid_size.z) {
                cell_offset_[kk] = 0;
                cell_nump_[kk] = 0;
            }
			else {
				int nid_left, nid_mid, nid_right;
				nid_left = CellPos2CellIdx(neighbor_pos, grid_size);
				++neighbor_pos.x;
				nid_mid = CellPos2CellIdx(neighbor_pos, grid_size);
				++neighbor_pos.x;
				nid_right = CellPos2CellIdx(neighbor_pos, grid_size);
				cell_offset_[kk] =
					kInvalidCellIdx == nid_left ? cell_offset[nid_mid] : celloffM[(nid_left << 6) + (min << 4)];
				int nc = 1;
				int my_cell_nump = cell_nump[nid_mid];
				if (kInvalidCellIdx != nid_left) {
					my_cell_nump += cell_offset[nid_mid] - celloffM[(nid_left << 6) + (min << 4)];
					nc++;
				}//cell_nump[nid_left];
				if (max == 3){
					if (kInvalidCellIdx != nid_right) {
						my_cell_nump += cell_nump[nid_right];
						nc++;
					}
				}
				else{
					if (kInvalidCellIdx != nid_right) {
						my_cell_nump += celloffM[(nid_right << 6) + ((max + 1) << 4)] - celloffM[(nid_right << 6)];//cell_nump[nid_right];
						nc++;
					}
				}
				int leftl = (kInvalidCellIdx == nid_left ? 0 : 4 - min);
				int rightl = (kInvalidCellIdx == nid_right ? 0 : 1 + max);
				int leng = 4 + leftl + rightl;

				if (my_cell_nump > leng * 22){
					if (((minz > 0) && ((kk % 9) < 3))){
						int begin_ = kInvalidCellIdx == nid_left ? (nid_mid << 6) : (nid_left << 6) + (min << 4);
						cell_begin[kk] = begin_;
						if (my_cell_nump>0){
							int lay = 0;
							while ((celloffM[begin_ + ((lay + 1) << 4)] - celloffM[begin_ + (minz << 2) + (lay << 4)]) == 0){
								lay++;
								if (lay >= leng){
									my_cell_nump = 0;
								}
							}
						}
					}
					else if (((maxz < 3) && ((kk % 9) > 5))){
						int begin_ = kInvalidCellIdx == nid_left ? (nid_mid << 6) : (nid_left << 6) + (min << 4);
						cell_begin[kk] = begin_;
						if (my_cell_nump > 0){
							int lay = 0;
							while ((celloffM[begin_ + (lay << 4) + ((maxz + 1) << 2)] - celloffM[begin_ + (lay << 4)]) == 0){
								lay++;
								if (lay >= leng){
									my_cell_nump = 0;
								}
							}
						}
					}
					else{
						cell_begin[kk] = -1;
					}
				}
				else{
					cell_begin[kk] = -1;
				}
				cell_nump_[kk] = my_cell_nump;
			}
        }
		if (bj == 31){
			ushort3 neighbor_pos = cell_pos + make_ushort3(-1, 0, 0);
			int nid_left, nid_mid, nid_right;
			nid_left = CellPos2CellIdx(neighbor_pos, grid_size);
			++neighbor_pos.x;
			nid_mid = CellPos2CellIdx(neighbor_pos, grid_size);
			++neighbor_pos.x;
			nid_right = CellPos2CellIdx(neighbor_pos, grid_size);
			int leftl = (kInvalidCellIdx == nid_left ? 0 : 4 - min);
			int rightl = (kInvalidCellIdx == nid_right ? 0 : 1+max);
			length[bi] = 4 + leftl + rightl;
			iminz[bi] = minz;
			imaxz[bi] = maxz;
		}
		depth[idx] = 0;
		int noCombie = (1 - isSame);
		current_cell_index_[idx] = 9 + bi * 9 * noCombie;
        offset_in_cell_[idx] = 0;
		__syncthreads();
		for (int i = bi * 9 * noCombie; i < bi * 9 * noCombie + 9; i += 1) {
            if (0 != cell_nump_[i]) {
                current_cell_index_[idx] = i;
                break;
            }
        }
    }

	__device__ int read32Data(int *celloffM, const int& isSame, const ParticleBufferList& buff_list) {
		unsigned int idx = threadIdx.x;
		int bi = (idx >> 5);
		int bj = idx % 32 + (isSame)*(bi << 5);
		int noCombine = (1 - isSame);
		int readDataSize = kNumSharedData*(1 + isSame);
		int curr_cell_index = current_cell_index_[idx];
		if (9 + bi * 9 * noCombine <= curr_cell_index) return 0;
		int cell_begin_ = cell_begin[curr_cell_index];
		if (cell_begin_ == -1){
			int offset_in_cell = offset_in_cell_[idx];
			int remain_nump = cell_nump_[curr_cell_index] - offset_in_cell;
			int num_read = remain_nump > readDataSize ? readDataSize : remain_nump;
			if (num_read > bj) {
				int read_idx = cell_offset_[curr_cell_index] + offset_in_cell + bj;
				position_[idx] = buff_list.position_d[read_idx];
			}
			if (remain_nump > readDataSize) offset_in_cell_[idx] += readDataSize;
			else {
				int next_cell_idx = curr_cell_index + 1;
				while (next_cell_idx < 9 + bi * 9 * noCombine) {
					if (0 != cell_nump_[next_cell_idx]) break;
					++next_cell_idx;
				}
				current_cell_index_[idx] = next_cell_idx;
				offset_in_cell_[idx] = 0;
			}
			return num_read;
		}
		else{
			int min = iminz[bi*(1 - isSame)];
			int max = imaxz[bi*(1 - isSame)];
			int lgth = length[bi*(1 - isSame)];
			if (((min > 0) && ((curr_cell_index % 9) < 3))){
				int lay = depth[idx];
				while ((celloffM[cell_begin_ + ((lay + 1) << 4)] - celloffM[cell_begin_ + (min << 2) + (lay << 4)]) == 0){
					lay++;
				}
				depth[idx] = lay;
				int celloff = celloffM[cell_begin_ + (min << 2) + (lay << 4)];
				int offset_in_cell = offset_in_cell_[idx];
				int remain_nump = celloffM[cell_begin_ + ((lay + 1) << 4)] - celloff - offset_in_cell;
				int num_read = remain_nump > readDataSize ? readDataSize : remain_nump;
				if (num_read > bj) {
					int read_idx = celloff + offset_in_cell + bj;
					position_[idx] = buff_list.position_d[read_idx];
				}
				if (remain_nump > readDataSize) offset_in_cell_[idx] += readDataSize;
				else {
					depth[idx]++;
					if (depth[idx] < lgth){
						while ((celloffM[cell_begin_ + ((depth[idx] + 1) << 4)] - celloffM[cell_begin_ + (min << 2) + (depth[idx] << 4)]) == 0){
							depth[idx]++;
							if (depth[idx] >= lgth) break;
						}
					}
					if (depth[idx] >= lgth){
						int next_cell_idx = curr_cell_index + 1;
						while (next_cell_idx < 9 + bi * 9 * noCombine) {
							if (0 != cell_nump_[next_cell_idx]) break;
							++next_cell_idx;
						}
						current_cell_index_[idx] = next_cell_idx;
						depth[idx] = 0;
					}
					offset_in_cell_[idx] = 0;
				}
				return num_read;
			}
			else if (((max < 3) && ((curr_cell_index % 9) > 5))){
				int lay = depth[idx];
				while ((celloffM[cell_begin_ + (lay << 4) + ((max + 1) << 2)] - celloffM[cell_begin_ + (lay << 4)]) == 0){
					lay++;
				}
				depth[idx] = lay;


				int celloff = celloffM[cell_begin_ + (lay << 4)];

				int offset_in_cell = offset_in_cell_[idx];
				int remain_nump = celloffM[cell_begin_ + (lay << 4) + ((max + 1) << 2)] - celloff - offset_in_cell;

				int num_read = remain_nump > readDataSize ? readDataSize : remain_nump;
				if (num_read > bj) {
					int read_idx = celloff + offset_in_cell + bj;
					position_[idx] = buff_list.position_d[read_idx];
				}
				if (remain_nump > readDataSize) offset_in_cell_[idx] += readDataSize;
				else {
					depth[idx]++;
					if (depth[idx] < lgth){
						while ((celloffM[cell_begin_ + (depth[idx] << 4) + ((max + 1) << 2)] - celloffM[cell_begin_ + (depth[idx] << 4)]) == 0){
							depth[idx]++;
							if (depth[idx] >= lgth) break;
						}
					}
					if (depth[idx] >= lgth){
						int next_cell_idx = curr_cell_index + 1;
						while (next_cell_idx < 9 + bi * 9 * noCombine) {
							if (0 != cell_nump_[next_cell_idx]) break;
							++next_cell_idx;
						}
						current_cell_index_[idx] = next_cell_idx;
						depth[idx] = 0;
					}
					offset_in_cell_[idx] = 0;
				}
				return num_read;
			}
		}
	}
    __device__  float3 & getPos(uint idx) {
		return position_[idx];
    }

private:
    float3 position_[kNumSharedData * rate];
    int cell_offset_[9 * rate];
    int cell_nump_[9 * rate];
	int cell_begin[9 * rate];
	uint offset_in_cell_[kNumSharedData * rate];
    char current_cell_index_[kNumSharedData * rate];
	char depth[kNumSharedData * rate];
	char length[rate];
	char iminz[rate];
	char imaxz[rate];
};


class SimForSharedData128
{
public:
    __device__ SimForSharedData128(){}

	__device__ void initialize(const int& minz, const int& maxz, const int& min, const int& max, int *celloffM, const int& isSame, int *cell_offset, int *cell_nump, const ushort3 &cell_pos, const ushort3 &grid_size) {
        unsigned int idx = threadIdx.x;
		int bi = (idx>>5);
		int bj = idx % 32 + (isSame)*(bi<<5);

        if (bj < 9) {

            int kk = bi * 9 + bj;

			ushort3 neighbor_pos = cell_pos + make_ushort3(-1, bj % 3 - 1, bj / 3 % 3 - 1);
            if (neighbor_pos.y < 0 || neighbor_pos.y >= grid_size.y ||
                neighbor_pos.z < 0 || neighbor_pos.z >= grid_size.z) {
                cell_offset_[kk] = 0;
                cell_nump_[kk] = 0;
            }
            else {
				int nid_left, nid_mid, nid_right;
				nid_left = CellPos2CellIdx(neighbor_pos, grid_size);
				++neighbor_pos.x;
				nid_mid = CellPos2CellIdx(neighbor_pos, grid_size);
				++neighbor_pos.x;
				nid_right = CellPos2CellIdx(neighbor_pos, grid_size);
				cell_offset_[kk] =
					kInvalidCellIdx == nid_left ? cell_offset[nid_mid] : celloffM[(nid_left << 6) + (min << 4)];
				int nc = 1;
				int my_cell_nump = cell_nump[nid_mid];
				if (kInvalidCellIdx != nid_left) {
					my_cell_nump += cell_offset[nid_mid] - celloffM[(nid_left << 6) + (min << 4)];
					nc++;
				}
				if (max == 3){
					if (kInvalidCellIdx != nid_right) {
						my_cell_nump += cell_nump[nid_right];
						nc++;
					}
				}
				else{
					if (kInvalidCellIdx != nid_right) {
						my_cell_nump += celloffM[(nid_right << 6) + ((max + 1) << 4)] - celloffM[(nid_right << 6)];
						nc++;
					}
				}
				int leftl = (kInvalidCellIdx == nid_left ? 0 : 4 - min);
				int rightl = (kInvalidCellIdx == nid_right ? 0 : 1 + max);
				int leng = 4 + leftl + rightl;

				if (my_cell_nump > leng * 22){
					if (((minz > 0) && ((kk % 9) < 3))){
						int begin_ = kInvalidCellIdx == nid_left ? (nid_mid << 6) : (nid_left << 6) + (min << 4);
						cell_begin[kk] = begin_;
						if (my_cell_nump>0){
							int lay = 0;
							while ((celloffM[begin_ + ((lay + 1) << 4)] - celloffM[begin_ + (minz << 2) + (lay << 4)]) == 0){
								lay++;
								if (lay >= leng){
									my_cell_nump = 0;
								}
							}
						}
					}
					else if (((maxz < 3) && ((kk % 9) > 5))){
						int begin_ = kInvalidCellIdx == nid_left ? (nid_mid << 6) : (nid_left << 6) + (min << 4);
						cell_begin[kk] = begin_;
						if (my_cell_nump > 0){
							int lay = 0;
							while ((celloffM[begin_ + (lay << 4) + ((maxz + 1) << 2)] - celloffM[begin_ + (lay << 4)]) == 0){
								lay++;
								if (lay >= leng){
									my_cell_nump = 0;
								}
							}
						}
					}
					else{
						cell_begin[kk] = -1;
					}
				}
				else{
					cell_begin[kk] = -1;
				}
				cell_nump_[kk] = my_cell_nump;
			}
        }

		if (bj == 31){
			ushort3 neighbor_pos = cell_pos + make_ushort3(-1, 0, 0);
			int nid_left, nid_mid, nid_right;
			nid_left = CellPos2CellIdx(neighbor_pos, grid_size);
			++neighbor_pos.x;
			nid_mid = CellPos2CellIdx(neighbor_pos, grid_size);
			++neighbor_pos.x;
			nid_right = CellPos2CellIdx(neighbor_pos, grid_size);
			int leftl = (kInvalidCellIdx == nid_left ? 0 : 4 - min);
			int rightl = (kInvalidCellIdx == nid_right ? 0 : 1 + max);
			length[bi] = 4 + leftl + rightl;
			iminz[bi] = minz;
			imaxz[bi] = maxz;
		}
		depth[idx] = 0;

		int noCombine = 1 - isSame;

		current_cell_index_[idx] = 9 + bi * 9 * noCombine;
        offset_in_cell_[idx] = 0;
		
		__syncthreads();
		for (int i = bi * 9 * noCombine; i < bi * 9 * noCombine + 9; i += 1) {
            if (0 != cell_nump_[i]) {
                current_cell_index_[idx] = i;
                break;
            }
        }
    }

	__device__ int read32Data(int *celloffM, const int& isSame, const ParticleBufferList& buff_list) {

		
        unsigned int idx = threadIdx.x;

		int bi = (idx >> 5);
		int bj = idx % 32 + (isSame)*(bi << 5);

        int curr_cell_index = current_cell_index_[idx];
		int noCombine = 1 - isSame;
		int readDataSize = kNumSharedData*(1 + isSame);
        // neighbors read complete
		if (9 + bi * 9 * noCombine <= curr_cell_index) return 0;
		int cell_begin_ = cell_begin[curr_cell_index];
		if (cell_begin_ == -1){
			int offset_in_cell = offset_in_cell_[idx];
			int remain_nump = cell_nump_[curr_cell_index] - offset_in_cell;
			int num_read = remain_nump > readDataSize ? readDataSize : remain_nump;
			if (num_read > bj) {
				int read_idx = cell_offset_[curr_cell_index] + offset_in_cell + bj;
				position_d[idx] = buff_list.position_d[read_idx];
				//ev_[idx] = buff_list.evaluated_velocity[read_idx];
			}
			if (remain_nump > readDataSize) offset_in_cell_[idx] += readDataSize;
			else {
				int next_cell_idx = curr_cell_index + 1;
				while (next_cell_idx < 9 + bi * 9 * noCombine) {
					if (0 != cell_nump_[next_cell_idx]) break;
					++next_cell_idx;
				}
				current_cell_index_[idx] = next_cell_idx;
				offset_in_cell_[idx] = 0;
			}
			return num_read;
		}
		else{
			int min = iminz[bi*(1 - isSame)];
			int max = imaxz[bi*(1 - isSame)];
			int lgth = length[bi*(1 - isSame)];
			if (((min > 0) && ((curr_cell_index % 9) < 3))){
				int lay = depth[idx];
				while ((celloffM[cell_begin_ + ((lay + 1) << 4)] - celloffM[cell_begin_ + (min << 2) + (lay << 4)]) == 0){
					lay++;
				}
				depth[idx] = lay;
				int celloff = celloffM[cell_begin_ + (min << 2) + (lay << 4)];
				int offset_in_cell = offset_in_cell_[idx];
				int remain_nump = celloffM[cell_begin_ + ((lay + 1) << 4)] - celloff - offset_in_cell;
				int num_read = remain_nump > readDataSize ? readDataSize : remain_nump;
				if (num_read > bj) {
					int read_idx = celloff + offset_in_cell + bj;
					position_d[idx] = buff_list.position_d[read_idx];
					//ev_[idx] = buff_list.evaluated_velocity[read_idx];
				}
				if (remain_nump > readDataSize) offset_in_cell_[idx] += readDataSize;
				else {
					depth[idx]++;
					if (depth[idx] < lgth){
						while ((celloffM[cell_begin_ + ((depth[idx] + 1) << 4)] - celloffM[cell_begin_ + (min << 2) + (depth[idx] << 4)]) == 0){
							depth[idx]++;
							if (depth[idx] >= lgth) break;
						}
					}
					if (depth[idx] >= lgth){
						int next_cell_idx = curr_cell_index + 1;
						while (next_cell_idx < 9 + bi * 9 * noCombine) {
							if (0 != cell_nump_[next_cell_idx]) break;
							++next_cell_idx;
						}
						current_cell_index_[idx] = next_cell_idx;
						depth[idx] = 0;
					}
					offset_in_cell_[idx] = 0;
				}
				return num_read;
			}
			else if (((max < 3) && ((curr_cell_index % 9) > 5))){
				int lay = depth[idx];
				while ((celloffM[cell_begin_ + (lay << 4) + ((max + 1) << 2)] - celloffM[cell_begin_ + (lay << 4)]) == 0){
					lay++;
				}
				depth[idx] = lay;


				int celloff = celloffM[cell_begin_ + (lay << 4)];

				int offset_in_cell = offset_in_cell_[idx];
				int remain_nump = celloffM[cell_begin_ + (lay << 4) + ((max + 1) << 2)] - celloff - offset_in_cell;

				int num_read = remain_nump > readDataSize ? readDataSize : remain_nump;
				if (num_read > bj) {
					int read_idx = celloff + offset_in_cell + bj;
					position_d[idx] = buff_list.position_d[read_idx];
					//ev_[idx] = buff_list.evaluated_velocity[read_idx];
				}
				if (remain_nump > readDataSize) offset_in_cell_[idx] += readDataSize;
				else {
					depth[idx]++;
					if (depth[idx] < lgth){
						while ((celloffM[cell_begin_ + (depth[idx] << 4) + ((max + 1) << 2)] - celloffM[cell_begin_ + (depth[idx] << 4)]) == 0){
							depth[idx]++;
							if (depth[idx] >= lgth) break;
						}
					}
					if (depth[idx] >= lgth){
						int next_cell_idx = curr_cell_index + 1;
						while (next_cell_idx < 9 + bi * 9 * noCombine) {
							if (0 != cell_nump_[next_cell_idx]) break;
							++next_cell_idx;
						}
						current_cell_index_[idx] = next_cell_idx;
						depth[idx] = 0;
					}
					offset_in_cell_[idx] = 0;
				}
				return num_read;
			}
		}
    }

	__device__ float3 & getPosition(unsigned int idx) { return position_d[idx]; }

	//__device__ float4 & getEV(unsigned int idx) { return ev_[idx]; }

private:
    float3 position_d[kNumSharedData * rate];
	//float4  ev_[kNumSharedData * rate];
    int cell_offset_[9 * rate];
    int cell_nump_[9 * rate];
	int cell_begin[9 * rate];
	uint offset_in_cell_[kNumSharedData * rate];
	char current_cell_index_[kNumSharedData * rate];
	char depth[kNumSharedData * rate];
	char length[rate];
	char iminz[rate];
	char imaxz[rate];
};

class SimNeighSharedData128
{
public:
	__device__ SimNeighSharedData128() {}

	__device__ void initialize(const int& minz, const int& maxz, const int& min, const int& max, int* celloffM, const int& isSame, int* cell_offset, int* cell_nump, const ushort3& cell_pos, const ushort3& grid_size) {
		unsigned int idx = threadIdx.x;
		int bi = (idx >> 5);
		int bj = idx % 32 + (isSame) * (bi << 5);

		if (bj < 9) {

			int kk = bi * 9 + bj;

			ushort3 neighbor_pos = cell_pos + make_ushort3(-1, bj % 3 - 1, bj / 3 % 3 - 1);
			if (neighbor_pos.y < 0 || neighbor_pos.y >= grid_size.y ||
				neighbor_pos.z < 0 || neighbor_pos.z >= grid_size.z) {
				cell_offset_[kk] = 0;
				cell_nump_[kk] = 0;
			}
			else {
				int nid_left, nid_mid, nid_right;
				nid_left = CellPos2CellIdx(neighbor_pos, grid_size);
				++neighbor_pos.x;
				nid_mid = CellPos2CellIdx(neighbor_pos, grid_size);
				++neighbor_pos.x;
				nid_right = CellPos2CellIdx(neighbor_pos, grid_size);
				cell_offset_[kk] =
					kInvalidCellIdx == nid_left ? cell_offset[nid_mid] : celloffM[(nid_left << 6) + (min << 4)];
				int nc = 1;
				int my_cell_nump = cell_nump[nid_mid];
				if (kInvalidCellIdx != nid_left) {
					my_cell_nump += cell_offset[nid_mid] - celloffM[(nid_left << 6) + (min << 4)];
					nc++;
				}
				if (max == 3) {
					if (kInvalidCellIdx != nid_right) {
						my_cell_nump += cell_nump[nid_right];
						nc++;
					}
				}
				else {
					if (kInvalidCellIdx != nid_right) {
						my_cell_nump += celloffM[(nid_right << 6) + ((max + 1) << 4)] - celloffM[(nid_right << 6)];//cell_nump[nid_right];
						nc++;
					}
				}
				int leftl = (kInvalidCellIdx == nid_left ? 0 : 4 - min);
				int rightl = (kInvalidCellIdx == nid_right ? 0 : 1 + max);
				int leng = 4 + leftl + rightl;

				if (my_cell_nump > leng * 22) {
					if (((minz > 0) && ((kk % 9) < 3))) {
						int begin_ = kInvalidCellIdx == nid_left ? (nid_mid << 6) : (nid_left << 6) + (min << 4);
						cell_begin[kk] = begin_;
						if (my_cell_nump > 0) {
							int lay = 0;
							while ((celloffM[begin_ + ((lay + 1) << 4)] - celloffM[begin_ + (minz << 2) + (lay << 4)]) == 0) {
								lay++;
								if (lay >= leng) {
									my_cell_nump = 0;
								}
							}
						}
					}
					else if (((maxz < 3) && ((kk % 9) > 5))) {
						int begin_ = kInvalidCellIdx == nid_left ? (nid_mid << 6) : (nid_left << 6) + (min << 4);
						cell_begin[kk] = begin_;
						if (my_cell_nump > 0) {
							int lay = 0;
							while ((celloffM[begin_ + (lay << 4) + ((maxz + 1) << 2)] - celloffM[begin_ + (lay << 4)]) == 0) {
								lay++;
								if (lay >= leng) {
									my_cell_nump = 0;
								}
							}
						}
					}
					else {
						cell_begin[kk] = -1;
					}
				}
				else {
					cell_begin[kk] = -1;
				}
				cell_nump_[kk] = my_cell_nump;
			}
		}

		if (bj == 31) {
			ushort3 neighbor_pos = cell_pos + make_ushort3(-1, 0, 0);
			int nid_left, nid_mid, nid_right;
			nid_left = CellPos2CellIdx(neighbor_pos, grid_size);
			++neighbor_pos.x;
			nid_mid = CellPos2CellIdx(neighbor_pos, grid_size);
			++neighbor_pos.x;
			nid_right = CellPos2CellIdx(neighbor_pos, grid_size);
			int leftl = (kInvalidCellIdx == nid_left ? 0 : 4 - min);
			int rightl = (kInvalidCellIdx == nid_right ? 0 : 1 + max);
			length[bi] = 4 + leftl + rightl;
			iminz[bi] = minz;
			imaxz[bi] = maxz;
		}
		depth[idx] = 0;

		int noCombine = 1 - isSame;

		current_cell_index_[idx] = 9 + bi * 9 * noCombine;
		offset_in_cell_[idx] = 0;

		__syncthreads();
		for (int i = bi * 9 * noCombine; i < bi * 9 * noCombine + 9; i += 1) {
			if (0 != cell_nump_[i]) {
				current_cell_index_[idx] = i;
				break;
			}
		}
	}

	__device__ int read32Data(int* celloffM, const int& isSame, const ParticleBufferList& buff_list) {


		unsigned int idx = threadIdx.x;

		int bi = (idx >> 5);
		int bj = idx % 32 + (isSame) * (bi << 5);

		int curr_cell_index = current_cell_index_[idx];
		int noCombine = 1 - isSame;
		int readDataSize = kNumSharedData * (1 + isSame);
		// neighbors read complete
		if (9 + bi * 9 * noCombine <= curr_cell_index) return 0;
		int cell_begin_ = cell_begin[curr_cell_index];
		if (cell_begin_ == -1) {
			int offset_in_cell = offset_in_cell_[idx];
			int remain_nump = cell_nump_[curr_cell_index] - offset_in_cell;
			int num_read = remain_nump > readDataSize ? readDataSize : remain_nump;
			if (num_read > bj) {
				int read_idx = cell_offset_[curr_cell_index] + offset_in_cell + bj;
				position_d[idx] = buff_list.position_d[read_idx];
				partIdx[idx] = read_idx;
			}
			if (remain_nump > readDataSize) offset_in_cell_[idx] += readDataSize;
			else {
				int next_cell_idx = curr_cell_index + 1;
				while (next_cell_idx < 9 + bi * 9 * noCombine) {
					if (0 != cell_nump_[next_cell_idx]) break;
					++next_cell_idx;
				}
				current_cell_index_[idx] = next_cell_idx;
				offset_in_cell_[idx] = 0;
			}
			return num_read;
		}
		else {
			int min = iminz[bi * (1 - isSame)];
			int max = imaxz[bi * (1 - isSame)];
			int lgth = length[bi * (1 - isSame)];
			if (((min > 0) && ((curr_cell_index % 9) < 3))) {
				int lay = depth[idx];
				while ((celloffM[cell_begin_ + ((lay + 1) << 4)] - celloffM[cell_begin_ + (min << 2) + (lay << 4)]) == 0) {
					lay++;
				}
				depth[idx] = lay;
				int celloff = celloffM[cell_begin_ + (min << 2) + (lay << 4)];
				int offset_in_cell = offset_in_cell_[idx];
				int remain_nump = celloffM[cell_begin_ + ((lay + 1) << 4)] - celloff - offset_in_cell;
				int num_read = remain_nump > readDataSize ? readDataSize : remain_nump;
				if (num_read > bj) {
					int read_idx = celloff + offset_in_cell + bj;
					position_d[idx] = buff_list.position_d[read_idx];
					partIdx[idx] = read_idx;
				}
				if (remain_nump > readDataSize) offset_in_cell_[idx] += readDataSize;
				else {
					depth[idx]++;
					if (depth[idx] < lgth) {
						while ((celloffM[cell_begin_ + ((depth[idx] + 1) << 4)] - celloffM[cell_begin_ + (min << 2) + (depth[idx] << 4)]) == 0) {
							depth[idx]++;
							if (depth[idx] >= lgth) break;
						}
					}
					if (depth[idx] >= lgth) {
						int next_cell_idx = curr_cell_index + 1;
						while (next_cell_idx < 9 + bi * 9 * noCombine) {
							if (0 != cell_nump_[next_cell_idx]) break;
							++next_cell_idx;
						}
						current_cell_index_[idx] = next_cell_idx;
						depth[idx] = 0;
					}
					offset_in_cell_[idx] = 0;
				}
				return num_read;
			}
			else if (((max < 3) && ((curr_cell_index % 9) > 5))) {
				int lay = depth[idx];
				while ((celloffM[cell_begin_ + (lay << 4) + ((max + 1) << 2)] - celloffM[cell_begin_ + (lay << 4)]) == 0) {
					lay++;
				}
				depth[idx] = lay;


				int celloff = celloffM[cell_begin_ + (lay << 4)];

				int offset_in_cell = offset_in_cell_[idx];
				int remain_nump = celloffM[cell_begin_ + (lay << 4) + ((max + 1) << 2)] - celloff - offset_in_cell;

				int num_read = remain_nump > readDataSize ? readDataSize : remain_nump;
				if (num_read > bj) {
					int read_idx = celloff + offset_in_cell + bj;
					position_d[idx] = buff_list.position_d[read_idx];
					partIdx[idx] = read_idx;

				}
				if (remain_nump > readDataSize) offset_in_cell_[idx] += readDataSize;
				else {
					depth[idx]++;
					if (depth[idx] < lgth) {
						while ((celloffM[cell_begin_ + (depth[idx] << 4) + ((max + 1) << 2)] - celloffM[cell_begin_ + (depth[idx] << 4)]) == 0) {
							depth[idx]++;
							if (depth[idx] >= lgth) break;
						}
					}
					if (depth[idx] >= lgth) {
						int next_cell_idx = curr_cell_index + 1;
						while (next_cell_idx < 9 + bi * 9 * noCombine) {
							if (0 != cell_nump_[next_cell_idx]) break;
							++next_cell_idx;
						}
						current_cell_index_[idx] = next_cell_idx;
						depth[idx] = 0;
					}
					offset_in_cell_[idx] = 0;
				}
				return num_read;
			}
		}
	}

	__device__ float3& getPosition(unsigned int idx) { return position_d[idx]; }

	__device__ int& getIndex(unsigned int idx) { return partIdx[idx]; }

private:
	float3 position_d[kNumSharedData * rate];
	int partIdx[kNumSharedData * rate];

	int cell_offset_[9 * rate];
	int cell_nump_[9 * rate];
	int cell_begin[9 * rate];
	uint offset_in_cell_[kNumSharedData * rate];
	char current_cell_index_[kNumSharedData * rate];
	char depth[kNumSharedData * rate];
	char length[rate];
	char iminz[rate];
	char imaxz[rate];
};


//sf class PCISPH over-------------------------------------------------------------------

}

#endif/*_SPH_KERNEL_SHARED_DATA_CUH*/