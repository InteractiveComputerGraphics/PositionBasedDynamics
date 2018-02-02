#ifndef __OBJECTARRAY_H__
#define __OBJECTARRAY_H__

#include <memory.h>
#include "Common/Common.h"

namespace Utilities
{
	template <class T, int growBy = 100, bool linearGrow = true>
	class ObjectArray
	{
	private:
		unsigned int m_size;
		unsigned int m_capacity;
		T* m_data;

	protected:
		FORCE_INLINE void init()
		{
			m_size = 0u;
			m_capacity = 0u;
			m_data = 0;
		}

	public:
		FORCE_INLINE ObjectArray()
		{
			init();
		}

		FORCE_INLINE ObjectArray(const ObjectArray& other)
		{
            init();
			*this = other;
		}

		~ObjectArray()
		{
			clear();
		}

		/** Return the pointer of the data.
		 */
		FORCE_INLINE T* arrayPointer()
		{
			return m_data;
		}

		/** Return the pointer of the data.
		 */
		FORCE_INLINE const T* arrayPointer() const 
		{
			return m_data;
		}

		/** Set size to zero but do not change capacity.
		 */
		FORCE_INLINE void reset()
		{
			m_size = 0u;
		}

		FORCE_INLINE void clear()
		{
			delete [] m_data;
			init();
		}

		/** Return the number of elements
		 */
		FORCE_INLINE unsigned int size() const
		{	
			return m_size;
		}

		/** Return the capacity
		 */
		FORCE_INLINE unsigned int capacity() const
		{	
			return m_capacity;
		}

		FORCE_INLINE T* getData()
		{
			return m_data;
		}

		FORCE_INLINE const T* getData() const 
		{
			return m_data;
		}

		T& create()
		{
			if(m_size >= m_capacity)
				grow();
			return m_data[m_size++];
		}

		/** Insert element at the given index. 
		 */
		FORCE_INLINE void insert(const unsigned int index, const T& data)
		{
			if(m_size >= m_capacity)
				grow();
			for(unsigned int i=m_size; i > index; i--)
			{
				m_data[i] = m_data[i-1];
			}
			m_size++;
			m_data[index] = data;
		}

		/** Remove the element at the given index. 
		 */
		FORCE_INLINE void removeAt(const unsigned int index)
		{
			m_data[index].~T();
			for(unsigned int i=index+1u; i < m_size; i++)
			{
				m_data[i-1] = m_data[i];
			}
			m_size--;
		}

		/** Remove the given element and return true, if the element was found. 
		 */
		FORCE_INLINE bool remove(const T &element)
		{
			for (unsigned int i=0u; i < m_size; i++)
			{
				if (m_data[i] == element)
				{
					removeAt(i);
					return true;
				}
			}
			return false;
		}

		FORCE_INLINE void pop_back()
		{
			m_size--;
			m_data[m_size].~T();
		}

		FORCE_INLINE void push_back(const T& data)
		{
			if(m_size >= m_capacity)
				grow();
			m_data[m_size] = data;
			m_size++;
		}

		FORCE_INLINE void push_back(T& data)
		{
			if(m_size >= m_capacity)
				grow();
			m_data[m_size] = data;
			m_size++;
		}

		FORCE_INLINE T& operator[](const unsigned int i)
		{
			return m_data[i];
		}

		FORCE_INLINE const T& operator[](const unsigned int i) const
		{
			return m_data[i];
		}

		FORCE_INLINE ObjectArray& operator=(const ObjectArray &val)
		{
			this->resize(val.m_size);
			memcpy(this->m_data, val.m_data, sizeof(T)*val.m_size);
			return *this;
		}

		FORCE_INLINE void reserve(const unsigned int count)
		{
			if (!m_data)
			{
				unsigned int newSize = count;
				if(newSize < growBy)
					newSize = growBy;
				m_data = new T[newSize];
				m_size = 0;
				m_capacity = newSize;
			}
			else
			{
				if(count <= m_capacity)
					return;
				unsigned int newSize = m_capacity;
				newSize = (linearGrow) ? (newSize+growBy) : (newSize*2u);
				if (count < newSize)
				{
					grow();
					return;
				}
				T* tmp = new T[count];
				memcpy(tmp, m_data, sizeof(T)*m_size);
				delete [] m_data;
				m_data = tmp;
				m_capacity = count;
			}
		}

		FORCE_INLINE void resize(unsigned int count)
		{
			reserve(count);
			m_size = count;
		}

	private:
		FORCE_INLINE void grow()
		{
			if (!m_data)
			{
				m_data = new T[growBy];
				m_size = 0;
				m_capacity = growBy;
			}
			else if (linearGrow)
			{
				const unsigned int newSize = m_capacity + growBy;
				T* tmp = new T[newSize];
				memcpy(tmp, m_data, sizeof(T)*m_size);
				delete [] m_data;
				m_data = tmp;
				m_capacity = newSize;
			}
			else
			{
				const unsigned int newSize = m_capacity*2u;
				T* tmp = new T[newSize];
				memcpy(tmp, m_data, sizeof(T)*m_size);
				delete [] m_data;
				m_data = tmp;
				m_capacity = newSize;
			}
		}
	};
}

#endif
