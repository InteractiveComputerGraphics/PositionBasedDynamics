//
// particle.h
// SPH_SMS_Benchmark
//
// created by ruanjm on 2016/03/26
// Copyright (c) 2016 ruanjm. All rights reserved.
//

#ifndef _PARTICLE_H
#define _PARTICLE_H

//#include <windows.h>
#include "cuda_call_check.h.hip"

typedef unsigned int uint;

namespace sph
{
//struct Vlmfraction{
//    float a1;
//    float a2;
//};
//
//struct mixVelocity{
//    float3 v1;
//    float3 v2;
//    float3 mixv;
//};
//
//struct DriftVelocity{
//    float3 Vm1;
//    float3 Vm2;
//};
//
//struct mixMass{
//    float mass1;
//    float mass2;
//    float massMix;
//};
//
//
//struct mixDensity{
//    float den1;
//    float den2;
//    float mixDen;
//};
//
//struct mixPressure{
//    float pres1;
//    float pres2;
//    float mixPres;
//};

//sf add 粒子状态
enum condition { FLUID, SOLID,  };

struct BlockTask
{
	char isSame;
	int cellid;
	unsigned short p_offset;
	char xxi;
	char xxx;
	char yyi;
	char yyy;
	char zzi;
	char zzz;
	
};

enum BufferType
{
    kBuffTypeNone,
    kBuffTypeDevice,
    kBuffTypeHostPageable,
    kBuffTypeHostPinned
};

struct sumGrad{
	float3 sumGradW = make_float3(0.0, 0.0, 0.0);
	float sumGradWDot = 0.0f;
	uint num_neigh = 0;
};

struct ParticleBufferList
{
    float3* position_d;
    //float3* final_position;

 //   float3*	predicted_pos;  //sf

	////sf others-------------------------------
	//condition*	phase;             //sf 粒子种类

 //   __device__
 //   condition & getPhase(unsigned int idx) {
 //       return phase[idx];
 //   }
};

class ParticleBufferObject
{
public:
    explicit ParticleBufferObject();
    ParticleBufferObject(unsigned int cap, BufferType type);
    ParticleBufferObject(const ParticleBufferObject&) = delete;
    ParticleBufferObject& operator=(const ParticleBufferList&) = delete;
    ~ParticleBufferObject();

    void allocate(unsigned int nump, BufferType type);
    void allocateSubBuffer(const ParticleBufferObject* base_buffer);
    void reallocate(unsigned int new_nump);
    void free();
    void transfer(ParticleBufferObject &dst_buff_obj, unsigned int offset, unsigned int nump, hipMemcpyKind kind);

    inline BufferType get_type() { return type_; }
    inline unsigned int get_capacity() { return capacity_; }
    inline ParticleBufferList& get_buff_list() { return buff_list_; }

    void swapObj(ParticleBufferObject &obj);

private:
    ParticleBufferList buff_list_;
    BufferType type_ = kBuffTypeNone;
    unsigned int capacity_ = 0U;
    const ParticleBufferObject *base_buffer_ = nullptr;
};


}

#endif/*_PARTICLE_H*/