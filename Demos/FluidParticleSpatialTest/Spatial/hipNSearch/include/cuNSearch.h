#pragma once
// This is a public header. Avoid references to cuda or other external references.

#include "Common.h"
#include "ActivationTable.h"
#include "PointSet.h"

namespace cuNSearch
{
	class cuNSearchDeviceData;

	/**
	* @class NeighborhoodSearch
	* Stores point data multiple set of points in which neighborhood information for a fixed
	* radius r should be generated.
	*/
	class NeighborhoodSearch
	{
	private:
		std::vector<PointSet> pointSets;

	public:

		/**
		* Constructor.
		* Creates a new instance of the neighborhood search class.
		* @param r Search radius. If two points are closer to each other than a distance r they are considered neighbors.
		*/
		NeighborhoodSearch(Real searchRadius);

		/**
		* Destructor.
		*/
		~NeighborhoodSearch();
		//Define descructor in cpp file to allow unique_ptr to incomplete type.
		//https://stackoverflow.com/questions/9954518/stdunique-ptr-with-an-incomplete-type-wont-compile


		/**
		* Get method to access a point set.
		* @param i Index of the point set to retrieve.
		*/
		PointSet const& point_set(unsigned int i) const { return pointSets[i]; }

		/**
		* Get method to access a point set.
		* @param i Index of the point set to retrieve.
		*/
		PointSet      & point_set(unsigned int i) { return pointSets[i]; }

		/**
		* Returns the number of point sets contained in the search.
		*/
		std::size_t  n_point_sets()               const { return pointSets.size(); }

		/**
		* Get method to access the list of point sets.
		*/
		std::vector<PointSet> const& point_sets() const { return pointSets; }

		/**
		* Get method to access the list of point sets.
		*/
		std::vector<PointSet>      & point_sets() { return pointSets; }

		/**
		* Increases the size of a point set under the assumption that the existing points remain at
		* the same position.
		* @param i Index of point set that will be resized.
		* @param x Pointer to the point position data. Must point to continguous data of 3 * n
		* real values.
		* @param n Number of points.
		*/
		void resize_point_set(unsigned int i, Real const* x, std::size_t n);


		/**
		* Creates and adds a new set of points.
		* @param x Pointer to the point position data. Must point to continguous data of 3 * n
		* real values.
		* @param n Number of points.
		* @param is_dynamic Specifies whether the point positions will change for future queries.
		* @param search_neighbors If true, neighbors in all other point sets are searched.
		* @param find_neighbors If true, the new point set is activated in the neighborhood search of all other point sets.
		* @returns Returns unique identifier in form of an index assigned to the newly created point
		* set.
		*/
		unsigned int add_point_set(Real const* x, std::size_t n, bool is_dynamic = true,
			bool search_neighbors = true, bool find_neighbors = true, void *user_data = nullptr);

		/**
		* Performs the actual query. This method will assign a list of neighboring points to each point
		* every added point set.
		*/
		void find_neighbors(bool points_changed = true);

		/**
		* Performs the actual query for a single point. This method return a list of neighboring points. Note: That points_changed() must be called each time
		* when the positions of a point set changed.
		*/
		void find_neighbors(unsigned int point_set_id, unsigned int point_index, std::vector<std::vector<unsigned int>> &neighbors);

		/**
		* Update neighborhood search data structures after a position change.
		* If general find_neighbors() function is called there is no requirement to manually update the point sets.
		* Otherwise, in case of using point-wise search (find_neighbors(i, j, neighbors)) the method must be called explicitly.
		*/
		void update_point_sets();

		/**
		* Update neighborhood search data structures after a position change.
		* Has to be called when the positions of a non-dynamic pointset change.
		* If general find_neighbors() function is called there is no requirement to manually update the point sets.
		* Otherwise, in case of using point-wise search (find_neighbors(i, j, neighbors)) the method must be called explicitly.
		*/
		void update_point_set(int i);

		/**
		* Update neighborhood search data structures after changing the activation table.
		* If general find_neighbors() function is called there is no requirement to manually update the point sets.
		* Otherwise, in case of using point-wise search (find_neighbors(i, j, neighbors)) the method must be called explicitly.
		*/
		void update_activation_table();

		/*
		* Generates a sort table according to a space-filling Z curve. Any array-based per point
		* information can then be reordered using the function sort_field of the PointSet class.
		* Please note that the position data will not be modified by this class, such that the user has
		* to invoke the sort_field function on the position array. Moreover, be aware the the grid has
		* be reinitialized after each sort. Therefore, the points should not be reordered too
		* frequently.
		*/
		void z_sort();

		/*
		* @returns Returns the radius in which point neighbors are searched.
		*/
		Real radius() const { return searchRadius; }

		/**
		* Sets the radius in which point point neighbors are searched.
		* Updates the hash table for all non-dynamic point sets
		* @param r Search radius.
		*/
		void set_radius(Real r);

		/** Activate/deactivate that neighbors in point set j are found when searching for neighbors of point set i.
		*   @param i Index of searching point set.
		*   @param j Index of point set of which points should/shouldn't be found by point set i.
		*   @param active Flag in order to (de)activate that points in i find point in j.
		*/
		void set_active(unsigned int i, unsigned int j, bool active)
		{
			m_activation_table.set_active(i, j, active);
		}

		/** Activate/Deactivate all point set pairs containing the given index. If search_neighbors is true, neighbors in all other point sets are searched.
		*   If find_neighbors is true, the new point set is activated in the neighborhood search of all other point sets.
		*   @param i Index of searching point set.
		*   @param search_neighbors If true/false enables/disables that point set i searches points in all other point sets.
		*   @param find_neighbors If true/false enable/disables that point set i is found by all other point sets.
		*/
		void set_active(unsigned int i, bool search_neighbors = true, bool find_neighbors = true)
		{
			m_activation_table.set_active(i, search_neighbors, find_neighbors);
		}

		/** Activate/Deactivate all point set pairs.
		*/
		void set_active(bool active)
		{
			m_activation_table.set_active(active);
		}

		/** Returns true if point set i searchs points in point set j.
		*   @param i Searching point set.
		*   @param j Set of points to be found by i.
		*/
		bool is_active(unsigned int i, unsigned int j) const
		{
			return m_activation_table.is_active(i, j);
		}

	private:
		bool isInitialized = false;
		Real searchRadius;
		ActivationTable m_activation_table;

		// Implementation and cuda data are hidden in the cuNSearchDeviceData class to avoid unnecessary dependencies in public headers.
		std::unique_ptr<cuNSearchDeviceData> deviceData;

		void updatePointSet(PointSet &pointSet);
	};
}
