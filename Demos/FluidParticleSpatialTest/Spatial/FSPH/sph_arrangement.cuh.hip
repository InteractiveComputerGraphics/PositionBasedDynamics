//
// sph_arrangement.cuh
// Hybrid_Parallel_SPH
//
// created by ruanjm on 2016/04/05
// Copyright (c) 2016 ruanjm. All rights reserved.
//

#ifndef _SPH_ARRANGEMENT_CUH
#define _SPH_ARRANGEMENT_CUH

#include "sph_particle.h.hip"

namespace gpu_model { struct GPUModel; }

namespace sph
{

class Arrangement
{
public:
    Arrangement(ParticleBufferObject &buff_list,
                ParticleBufferObject &buff_temp,
               
                unsigned int nump, 
				unsigned int nump_capacity,
                float cell_size, 
                ushort3 grid_size);

    ~Arrangement();

    // return middle value of 2 parallel framework
    int arrangeTRAMode();
    void arrangeSMSMode();
	int arrangeHybridMode();
    int arrangeHybridMode9();
    void test();

    void sortParticles();
    void assignTasksFixedCTA();

    int* getDevCellStartIdx();
    int* getDevCellEndIdx();
    int getNumBlockSMSMode();
    BlockTask *getBlockTasks();

	void resetNumParticle(unsigned int nump);

    
    int* getDevOffsetData() { return d_cell_offset_data; }
    int* getDevCellOffset() { return d_cell_offset_; }
	int* getDevCellOffsetM() { return d_cell_offset_M; }
    int* getDevCellIndex() { return d_index_; }

    int* getDevCellNumP() { return d_cell_nump_; }

    unsigned int getNumC() { return numc_; }


    void CountingSortCUDA();
    void CountingSort_O();

    void CountingSortCUDA_Two();
    void CountingSortCUDA_Two9();
    void countNum();

	void CountingSort_O_M();
	void CountingSortCUDA_Two9_M();
	int arrangeHybridMode9M();

    int* part2Idx;
    int* idx2Part;

private:
    void calculateHash();
    void calculateHashWithBlockReq();
    void sortHash();
    void sortIndexByHash();
    void reindexParticles();    // use index sorted by hash to reindex
    void reindexParticles2();   // use "particle offset in cell" and "prefix summed cell offset" to reindex
    void findCellRange();
    void findCellRangeAndHybridModeMiddleValue();
    void insertParticles();
    void arrangeBlockTasks();

	

    void CSInsertParticles();
    void CSCountingSortFull();
	void arrangeBlockTasksFixedM(int *hash, int *celloff, int *cellnum, BlockTask* d_task_array, int* d_cta_reqs, int* d_task_array_offset, int cta_size);
    void arrangeBlockTasksFixed(BlockTask* d_task_array, int* d_cta_reqs, int* d_task_array_offset, int cta_size);
    void arrangeBlockTasksFloat();

    void CSCalculateRequiredCTAsFixed(int *cat_offset, int* d_cta_reqs, int cta_size);


  

    ParticleBufferObject &buff_list_; // particle device buffer
    ParticleBufferObject &buff_temp_; // particle device buffer for replacement 
    unsigned int nump_;             // #particles
	unsigned int nump_capacity_;
    unsigned int numc_;             // #cells
    float cell_size_;
    ushort3 grid_size_;
    int middle_value_ = 0;

    int  h_num_cta_;

    int* d_num_cta_;
    int* d_cell_offset_;            // [numc] the offset in memory of the particles in each cell
    int* d_cell_nump_;              // [numc] the number of particles in each cell
    int* d_p_offset_;

    int* d_p_offset_p;

    int* d_cell_offset_data;

    int *d_start_index_;            // [numc]device buffer, cell start index
    int *d_end_index_;              // [numc]device buffer, cell end index
    int *d_hash_;                   // [nump]
    int *d_index_;                  // [nump]
    
    int *hashp;                     // [nump]
	int *d_hash_p;                  // [nump]
    int *cell_num_;
    int *cell_num_two;
    int *cell_type;
	int* d_cell_nump_M;
	int* d_cell_offset_M;


    int* d_task_array_offset_32_;   // [numc]result of prescan
    int *d_block_reqs_;             // [numc]for SMS Mode
    int *d_breqs_offset_;           // [numc]result of prescan
    int *d_num_block_;              // [1]
    int h_num_block_ = 0;
    BlockTask *d_block_task_;       // [numb]
    
    int *d_middle_value_;           // [1]for Hybrid Mode

    gpu_model::GPUModel *p_gpu_model_ = nullptr;
};

}

#endif/*_SPH_ARRANGEMENT_CUH*/
