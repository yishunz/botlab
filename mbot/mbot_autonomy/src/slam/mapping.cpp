#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
using namespace std::chrono; 

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}

void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    int rayStride = 1;
    MovingLaserScan movingScan(scan, previousPose_, pose, rayStride);
   
    for(auto& ray : movingScan){
        scoreEndpoint(ray, map);
    }

    for(auto& ray : movingScan){
        scoreRay(ray, map);
    }
    previousPose_ = pose;
    }

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your endpoint score ///////////////////////
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map);
    float dist = pow(ray.origin.x - f_end.x,2)*map.metersPerCell()+ pow(ray.origin.y - f_end.y,2)*map.metersPerCell();
    dist = sqrt(dist);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    CellOdds odd_cur = map.logOdds(end_cell.x, end_cell.y);
    std::cout<< "end point:" <<end_cell.x<<" "<<end_cell.y <<" "<<odd_cur <<std::endl;
    if (abs(dist-kMaxLaserDistance_) < 0.01) {
        map.setLogOdds(end_cell.x,  end_cell.y, overflowSub(odd_cur,kMissOdds_));
    }
    else{
        map.setLogOdds(end_cell.x,  end_cell.y, overflowAdd(odd_cur,kHitOdds_));
    }   
}


void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your ray score ///////////////////////
    std::vector<Point<int>> cells_touched = bresenham(ray, map);
    for (auto pt: cells_touched){
        CellOdds odd_cur = map.logOdds(pt.x, pt.y);
        map.setLogOdds(pt.x,  pt.y, overflowSub(odd_cur,kMissOdds_));
    }
}

CellOdds Mapping::overflowSub(CellOdds co, int8_t k)
{
//////////////// TODO: overflow case for addition ///////////////////////
    if ((k > 0) && (co < INT8_MIN + k)) {
        return INT8_MIN;
    }else{
        return co - k;
    }
}

CellOdds Mapping::overflowAdd(CellOdds co, int8_t k)
{
//////////////// TODO: overflow case for addition ///////////////////////
    if ((k > 0) && (co > INT8_MAX - k)) {
        return INT8_MAX;
    }else{
        return co + k;
    }
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get global positions 
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    std::vector<Point<int>> cells_touched;
    //////////////// TODO: Implement Bresenham's Algorithm ////////////////
    int dx = abs(end_cell.x-start_cell.x);
    int dy = abs(end_cell.y-start_cell.y);
    int sx = start_cell.x<end_cell.x ? 1 : -1;
    int sy = start_cell.y<end_cell.y ? 1 : -1;
    int err = dx-dy;
    int x = start_cell.x;
    int y = start_cell.y;
    while(x != end_cell.x || y != end_cell.y){
        // updateOdds(x,y);
        // double check this algorithm
        cells_touched.push_back(Point<int>(static_cast<int>(x),static_cast<int>(y)));
        int e2 = 2*err;
        if (e2 >= -dy){
            err -= dy;
            x += sx;
        }
        if (e2 <= dx){
            err += dx;
            y += sy;
        }
    }
    return cells_touched;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    auto end_cell = global_position_to_grid_cell(Point<double>(
        ray.origin.x + ray.range * std::cos(ray.theta),
        ray.origin.y + ray.range * std::sin(ray.theta)
        ), map);
    //////////////// TODO: Implement divide and step ////////////////
    std::vector<Point<int>> cells_touched;
    return cells_touched;
}
