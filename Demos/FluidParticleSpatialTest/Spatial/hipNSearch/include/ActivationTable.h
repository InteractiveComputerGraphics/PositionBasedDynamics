#pragma once
// This is a public header. Avoid references to cuda or other external references.

#include <vector>

namespace cuNSearch
{
class ActivationTable
{
private:
	std::vector<std::vector<unsigned char>> m_table;

public:

	bool operator==(ActivationTable const& other) const
	{
		return m_table == other.m_table;
	}

	bool operator!=(ActivationTable const& other) const
	{
		return !(m_table == other.m_table);
	}

	/** Add point set. If search_neighbors is true, neighbors in all other point sets are searched.
	* If find_neighbors is true, the new point set is activated in the neighborhood search of all other point sets.
	*/
	void add_point_set(bool search_neighbors = true, bool find_neighbors = true)
	{
		// add column to each row
		auto size = m_table.size();
		for (auto i = 0u; i < size; i++)
		{
			m_table[i].resize(size + 1);
			m_table[i][size] = static_cast<unsigned char>(find_neighbors);
		}

		// add new row
		m_table.resize(size + 1);
		m_table[size].resize(size + 1);
		for (auto i = 0u; i < size + 1; i++)
			m_table[size][i] = static_cast<unsigned char>(search_neighbors);
	}

	/** Activate/Deactivate that neighbors in point set index2 are found when searching for neighbors of point set index1.
	 */
	void set_active(unsigned int index1, unsigned int index2, bool active)
	{
		m_table[index1][index2] = static_cast<unsigned char>(active);
	}

	/** Activate/Deactivate all point set pairs containing the given index. If search_neighbors is true, neighbors in all other point sets are searched.
	* If find_neighbors is true, the new point set is activated in the neighborhood search of all other point sets.
	*/
	void set_active(unsigned int index, bool search_neighbors = true, bool find_neighbors = true)
	{
		auto size = m_table.size();
		for (auto i = 0u; i < size; i++)
		{
			m_table[i][index] = static_cast<unsigned char>(find_neighbors);
			m_table[index][i] = static_cast<unsigned char>(search_neighbors);
		}
		m_table[index][index] = static_cast<unsigned char>(search_neighbors && find_neighbors);
	}

	/** Activate/Deactivate all point set pairs.
	*/
	void set_active(bool active)
	{
		auto size = m_table.size();
		for (auto i = 0u; i < size; i++)
			for (auto j = 0u; j < size; j++)
				m_table[i][j] = static_cast<unsigned char>(active);
	}

	bool is_active(unsigned int index1, unsigned int index2) const
	{
		return m_table[index1][index2] != 0;
	}

	bool is_searching_neighbors(unsigned int const index) const
	{
		for (auto i = 0u; i < m_table[index].size(); i++)
		{
			if (m_table[index][i])
			{
				return true;
			}
		}
		return false;
	}
};
}
