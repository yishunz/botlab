#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(1)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    double likelihood = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    // TODO
    for(const auto& ray : movingScan)
        {
        double rayScore= scoreRay(ray, map);
        likelihood += rayScore;
        }
    return likelihood;
}

double SensorModel::scoreRay(const adjusted_ray_t ray,const OccupancyGrid& map)
{
    double rayScore = 0.0;
    Point<int> f_end = global_position_to_grid_cell(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map);
    
    if (map.logOdds(f_end.x,f_end.y) > 0.0){
        rayScore = map.logOdds(f_end.x,f_end.y);
        return rayScore;
    }else{
        // cell before 
        Point<int> cell_before;
        Point<int> start_cell;    
        Point<int> end_cell = global_position_to_grid_cell(ray.origin, map);

        start_cell.x = f_end.x;
        start_cell.y = f_end.y;
        int dx = abs(end_cell.x-start_cell.x);
        int dy = abs(end_cell.y-start_cell.y);
        int sx = start_cell.x<end_cell.x ? 1 : -1;
        int sy = start_cell.y<end_cell.y ? 1 : -1;
        int err = dx-dy;
        int x = start_cell.x;
        int y = start_cell.y;
        int e2 = 2*err;
        if (e2 >= -dy){
            x += sx;
        }
        if (e2 <= dx){
            y += sy;
        }
        cell_before.x = static_cast<int>(x);
        cell_before.y = static_cast<int>(y);

        // cell after
        Point<int> cell_after;
        x = start_cell.x;
        y = start_cell.y;
        if (e2 >= -dy){
            x -= sx;
        }
        if (e2 <= dx){
            y -= sy;
        }
        cell_after.x = static_cast<int>(x);
        cell_after.y = static_cast<int>(y);

        // check the cell before and after
        rayScore = 0.5*map.logOdds(cell_before.x,cell_before.y)+0.5*map.logOdds(cell_after.x,cell_after.y);
    
    }
    return rayScore;
}