#include "NearestFrontierPlanner.h"

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

NearestFrontierPlanner::NearestFrontierPlanner()
{
	
}

NearestFrontierPlanner::~NearestFrontierPlanner()
{
	
}

int NearestFrontierPlanner::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{
	// Create some workspace for the wavefront algorithm
	unsigned int mapSize = map->getSize();

	bool foundFrontier = false;
	// Search in right,down,up,left
	class Index {
	public:
		Index(GridMap *m, unsigned int index, unsigned int it):map_(m),index_(index),it_(it){};
		inline void operator++(void){
		int delta = 0;
		if (it_ == 0)
			delta = 1;
		if (it_ == 1)
			delta = -map_->getWidth();
		if (it_ == 2)
			delta = map_->getWidth();
		if (it_ == 3)
			delta = -1;

		index_ += delta*5;
		}

		unsigned int get() const
		{
			return index_;
		}

	private:
		unsigned int index_;
		unsigned int it_;
		GridMap* map_;
	};

	for (unsigned int it = 0; it < 4; it++)
	{
		for (Index index(map,start,it); ; ++index)
		{
			auto neighbors = map->getFreeNeighbors(index.get(), 2);
			ROS_WARN("index(%d), size(%d)", index.get(),neighbors.size());
			if (neighbors.size() != std::pow(5, 2))
				break;

			for(auto& neighbor : neighbors)
				if(map->getData(neighbor) != CLEANED){
					ROS_WARN("find goal(%d): start(%d)", index.get(), start);
					foundFrontier = true;
					goal = index.get();
					break;
				}

			if(foundFrontier)
				break;
		}
		if(foundFrontier)
			break;
	}

	// Initialize the queue with the robot position
	double distance;
	double linear = map->getResolution();

	double *plan = new double[mapSize];
	for (unsigned int i = 0; i < mapSize; i++)
	{
		plan[i] = -1;
	}
	plan[start] = 0;

	int cellCount = 0;
	if (!foundFrontier)
	{
		Queue queue;
		Entry startPoint(0.0, start);
		queue.insert(startPoint);
		Queue::iterator next;
		// Do full search with weightless Dijkstra-Algorithm
		ROS_INFO("Do full search with weightless Dijkstra-Algorithm");
		while (!queue.empty())
		{
			cellCount++;
			// Get the nearest cell from the queue
			next = queue.begin();
			distance = next->first;
			unsigned int index = next->second;
			queue.erase(next);

			// Add all adjacent cells
			ROS_WARN("index(%d),data(%d)", index, map->getData(index));
			if (map->isFree(index) && !map->isCleaned(index))
			{
				// We reached the border of the map, which is unexplored terrain as well:
				ROS_WARN("find goal index(%d),data(%d)", index, map->getData(index));
				foundFrontier = true;
				goal = index;
				break;
			} else
			{
				unsigned int ind[4];

				ind[0] = index + 1;               // right
				ind[1] = index - map->getWidth(); // up
				ind[2] = index + map->getWidth(); // down
				ind[3] = index - 1;               // left

				for (unsigned int it = 0; it < 4; it++)
				{
					unsigned int i = ind[it];

//					ROS_INFO("ind[%d],data(%d)", i, map->getData(i));
					if ((map->isFree(i) && !map->isCleaned(index)) || plan[i] == -1)
					{
						ROS_WARN("add to Queue:(%d)", map->getData(i));
						queue.insert(Entry(distance + linear, i));
						plan[i] = distance + linear;
//					foundFrontier = true;
//					goal = i;
//					break;
					}
				}

			}
		}
	}

	ROS_DEBUG("Checked %d cells.", cellCount);
	delete[] plan;

	if (foundFrontier)
	{
		return EXPL_TARGET_SET;
	} else
	{
		if (cellCount > 50)
			return EXPL_FINISHED;
		else
			return EXPL_FAILED;
	}
}
