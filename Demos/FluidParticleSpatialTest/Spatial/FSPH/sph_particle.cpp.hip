//
// sph_particle.cpp
// Hybrid_Parallel_SPH
//
// created by ruanjm on 2016/05/07
// Copyright (c) 2016 ruanjm. All rights reserved.
//


#include <hip/hip_runtime.h>
#include "sph_particle.h.hip"

namespace sph
{

/****************************** Tools ******************************/

template<typename T>
inline void swap(T &a, T &b)
{
    T temp = a;
    a = b;
    b = temp;
}

template<typename T>
inline void reallocDeviceBuffer(T *&p, unsigned int old_nump, unsigned int new_nump)
{
    T *temp;
    CUDA_SAFE_CALL(hipMalloc(&temp, new_nump * sizeof(T)));
    CUDA_SAFE_CALL(hipMemcpy(temp, p, old_nump * sizeof(T), hipMemcpyDeviceToDevice));
    CUDA_SAFE_CALL(hipFree(p));
    p = temp;
}

template<typename T>
inline void reallocHostPinnedBuffer(T *&p, unsigned int old_nump, unsigned int new_nump)
{
    T *temp;
    CUDA_SAFE_CALL(hipHostMalloc(&temp, new_nump * sizeof(T)));
    std::memcpy(temp, p, old_nump * sizeof(T));
    CUDA_SAFE_CALL(hipHostFree(p));
    p = temp;
}

template<typename T>
inline void reallocHostPageableBuffer(T *&p, unsigned int old_nump, unsigned int new_nump)
{
    T *temp;
    temp = new T[new_nump];
    std::memcpy(temp, p, old_nump * sizeof(T));
    delete[]p;
    p = temp;
}

/****************************** Interface ******************************/

/****************************** ParticleBufferObject ******************************/
ParticleBufferObject::ParticleBufferObject()
{

}

ParticleBufferObject::ParticleBufferObject(unsigned int cap, BufferType type)
{
    allocate(cap, type);
}

ParticleBufferObject::~ParticleBufferObject()
{
    this->free();
}

void ParticleBufferObject::allocate(unsigned int nump, BufferType type)
{
    if (kBuffTypeNone != type_) return;

    capacity_ = nump;
    type_ = type;

    if (kBuffTypeDevice == type)
    {
        CUDA_SAFE_CALL(hipMalloc(&buff_list_.position_d, nump * sizeof(float3))); //Increase allocation size to force hip to actually allocate. What the hell HIP?
        /*CUDA_SAFE_CALL(hipMalloc(&buff_list_.velocity, nump * sizeof(float3)));
        CUDA_SAFE_CALL(hipMalloc(&buff_list_.acceleration, nump * sizeof(float3)));
        CUDA_SAFE_CALL(hipMalloc(&buff_list_.evaluated_velocity, nump * sizeof(float4)));*/
        //CUDA_SAFE_CALL(hipMalloc(&buff_list_.final_position, nump * sizeof(float3)));
		//CUDA_SAFE_CALL(hipMalloc(&buff_list_.color, nump * sizeof(unsigned int)));
    }
    if (kBuffTypeHostPinned == type)
    {
        CUDA_SAFE_CALL(hipHostMalloc(&buff_list_.position_d, nump * sizeof(float3)));// * 1.5 * 32)); //Increase allocation size to force hip to actually allocate. What the hell HIP?
        //CUDA_SAFE_CALL(hipHostMalloc(&buff_list_.velocity, nump * sizeof(float3)));
        //CUDA_SAFE_CALL(hipHostMalloc(&buff_list_.final_position, nump * sizeof(float3) * 1.5 * 32));
        //CUDA_SAFE_CALL(hipHostMalloc(&buff_list_.color, nump * sizeof(unsigned int)));
    }
    if (kBuffTypeHostPageable == type)
    {
        buff_list_.position_d = new float3[nump];
        /*buff_list_.velocity = new float3[nump];
        buff_list_.acceleration = new float3[nump];
        buff_list_.evaluated_velocity = new float4[nump];*/
        //buff_list_.final_position = new float3[nump];
        //buff_list_.color = new unsigned int[nump];
    }
}

void ParticleBufferObject::allocateSubBuffer(const ParticleBufferObject* base_buffer)
{
    if (kBuffTypeNone != type_) return;

    capacity_ = base_buffer->capacity_;
    type_ = base_buffer->type_;
    base_buffer_ = base_buffer;

    if (kBuffTypeDevice == type_)
    {
        CUDA_SAFE_CALL(hipMalloc(&buff_list_.position_d, capacity_ * sizeof(float3)));
        /*CUDA_SAFE_CALL(hipMalloc(&buff_list_.velocity, capacity_ * sizeof(float3)));
        CUDA_SAFE_CALL(hipMalloc(&buff_list_.evaluated_velocity, capacity_ * sizeof(float3)));
		CUDA_SAFE_CALL(hipMalloc(&buff_list_.color, capacity_ * sizeof(unsigned int)));*/

		//sf add
        //buff_list_.acceleration = base_buffer->buff_list_.acceleration;
        //buff_list_.final_position = base_buffer->buff_list_.final_position;
		
	
    }
    if (kBuffTypeHostPinned == type_)
    {
        CUDA_SAFE_CALL(hipHostMalloc(&buff_list_.position_d, capacity_ * sizeof(float3)));
        /*CUDA_SAFE_CALL(hipHostMalloc(&buff_list_.velocity, capacity_ * sizeof(float3)));
        CUDA_SAFE_CALL(hipHostMalloc(&buff_list_.color, capacity_ * sizeof(unsigned int)));*/
        
        //buff_list_.final_position = base_buffer->buff_list_.final_position;
    }
    if (kBuffTypeHostPageable == type_)
    {
        buff_list_.position_d = new float3[capacity_];
        /*buff_list_.velocity = new float3[capacity_];
        buff_list_.evaluated_velocity = new float4[capacity_];
        buff_list_.color = new unsigned int[capacity_];

        buff_list_.acceleration = base_buffer->buff_list_.acceleration;*/
        //buff_list_.final_position = base_buffer->buff_list_.final_position;
    }
}

void ParticleBufferObject::reallocate(unsigned int new_nump)
{
    if (new_nump <= capacity_) return;

    if (kBuffTypeDevice == type_)
    {
        reallocDeviceBuffer(buff_list_.position_d, capacity_, new_nump);
        /*reallocDeviceBuffer(buff_list_.velocity, capacity_, new_nump);
        reallocDeviceBuffer(buff_list_.evaluated_velocity, capacity_, new_nump);
        reallocDeviceBuffer(buff_list_.color, capacity_, new_nump);*/

	
        if (base_buffer_)
        {
            //buff_list_.acceleration = base_buffer_->buff_list_.acceleration;
            //buff_list_.final_position = base_buffer_->buff_list_.final_position;
        }
        else
        {
            //reallocDeviceBuffer(buff_list_.acceleration, capacity_, new_nump);
            //reallocDeviceBuffer(buff_list_.final_position, capacity_, new_nump);
        }
    }
    if (kBuffTypeHostPinned == type_)
    {
        reallocHostPinnedBuffer(buff_list_.position_d, capacity_, new_nump);
        /*reallocHostPinnedBuffer(buff_list_.velocity, capacity_, new_nump);
        reallocHostPinnedBuffer(buff_list_.color, capacity_, new_nump);*/

        if (base_buffer_)
        {
            //buff_list_.final_position = base_buffer_->buff_list_.final_position;
        }
        else
        {
            //reallocHostPinnedBuffer(buff_list_.final_position, capacity_, new_nump);
        }
    }
    if (kBuffTypeHostPageable == type_)
    {
        reallocHostPageableBuffer(buff_list_.position_d, capacity_, new_nump);
        /*reallocHostPageableBuffer(buff_list_.velocity, capacity_, new_nump);
        reallocHostPageableBuffer(buff_list_.evaluated_velocity, capacity_, new_nump);
        reallocHostPageableBuffer(buff_list_.color, capacity_, new_nump);*/

        if (base_buffer_)
        {
            //buff_list_.acceleration = base_buffer_->buff_list_.acceleration;
            //buff_list_.final_position = base_buffer_->buff_list_.final_position;
        }
        else
        {
            //reallocHostPageableBuffer(buff_list_.acceleration, capacity_, new_nump);
            //reallocHostPageableBuffer(buff_list_.final_position, capacity_, new_nump);
        }
    }

    capacity_ = new_nump;
}

void ParticleBufferObject::free()
{
    if (kBuffTypeDevice == type_)
    {
        CUDA_SAFE_CALL(hipFree(buff_list_.position_d));
        /*CUDA_SAFE_CALL(hipFree(buff_list_.velocity));
        CUDA_SAFE_CALL(hipFree(buff_list_.evaluated_velocity));
        CUDA_SAFE_CALL(hipFree(buff_list_.color));*/

        if (!base_buffer_)
        {
            //CUDA_SAFE_CALL(hipFree(buff_list_.acceleration));
            //CUDA_SAFE_CALL(hipFree(buff_list_.final_position));
        }
    }
    if (kBuffTypeHostPageable == type_)
    {
        delete[]buff_list_.position_d;
        /*delete[]buff_list_.velocity;
        delete[]buff_list_.evaluated_velocity;
        delete[]buff_list_.color;*/

        if (!base_buffer_)
        {
            //delete[]buff_list_.acceleration;
            //delete[]buff_list_.final_position;
        }
    }
    if (kBuffTypeHostPinned == type_)
    {
        CUDA_SAFE_CALL(hipHostFree(buff_list_.position_d));
        /*CUDA_SAFE_CALL(hipHostFree(buff_list_.velocity));
        CUDA_SAFE_CALL(hipHostFree(buff_list_.color));*/

        if (!base_buffer_)
        {
            //CUDA_SAFE_CALL(hipHostFree(buff_list_.final_position));
        }
    }

    type_ = kBuffTypeNone;
    capacity_ = 0U;
    base_buffer_ = nullptr;
}

void ParticleBufferObject::transfer(ParticleBufferObject &dst_buff_obj, unsigned int offset, unsigned int nump, hipMemcpyKind kind)
{
    CUDA_SAFE_CALL(hipMemcpy(dst_buff_obj.buff_list_.position_d + offset, buff_list_.position_d + offset, nump * sizeof(float3), kind));
    //CUDA_SAFE_CALL(hipMemcpy(dst_buff_obj.buff_list_.velocity + offset, buff_list_.velocity + offset, nump * sizeof(float3), kind));
    //CUDA_SAFE_CALL(hipMemcpy(dst_buff_obj.buff_list_.final_position + offset, buff_list_.final_position + offset, nump * sizeof(float3), kind));
    //CUDA_SAFE_CALL(hipMemcpy(dst_buff_obj.buff_list_.color + offset, buff_list_.color + offset, nump * sizeof(unsigned int), kind));
}

void ParticleBufferObject::swapObj(ParticleBufferObject &obj)
{
    swap(this->buff_list_, obj.buff_list_);
    swap(this->type_, obj.type_);
    swap(this->capacity_, obj.capacity_);
}

}
