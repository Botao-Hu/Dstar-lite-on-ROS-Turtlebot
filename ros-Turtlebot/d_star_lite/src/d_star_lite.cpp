#include <pluginlib/class_list_macros.h>
#include "d_star_lite.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(d_star_lite::D_star_lite, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace d_star_lite {

D_star_lite::D_star_lite (){

}

D_star_lite::D_star_lite(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	initialize(name, costmap_ros);
}


void D_star_lite::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

}

bool D_star_lite::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

	printf("***************\n");
	printf("Go straight!");
	plan.push_back(start);
	plan.push_back(goal);
	return true;
}
};

