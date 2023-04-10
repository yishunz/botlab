#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(5)
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
        // std::cout<<"rayscore:"<<rayScore<<std::endl;
        }
    double distance =  sqrt(pow(sample.pose.x - sample.parent_pose.x,2)+pow(sample.pose.y - sample.parent_pose.y,2));
    // std::cout<<"parent:"<<sample.parent_pose.x <<","<<sample.parent_pose.y<<","<<sample.parent_pose.theta <<std::endl;
    if (distance > 10){
        // std::cout<<"dist" << sqrt(pow(sample.pose.x - sample.parent_pose.x,2)+pow(sample.pose.y - sample.parent_pose.y,2))<<std::endl;
        // std::cout<<"particle:"<<sample.pose.x <<","<<sample.pose.y<<","<<sample.pose.theta <<std::endl;
        // std::cout<<"parent:"<<sample.parent_pose.x <<","<<sample.parent_pose.y<<","<<sample.parent_pose.theta <<std::endl;

    }
    // std::cout<<"ll:"<<likelihood<<std::endl;

    return likelihood;
}

double SensorModel::scoreRay(const adjusted_ray_t ray,const OccupancyGrid& map)
{
    // simplified liklihood mood
    double threshold = 100;
    double rayScore = 0.0;
    double maxScore = 127;
    Point<int> f_end = global_position_to_grid_cell(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map);
    // std::cout << f_end.x <<","<<f_end.y<<std::endl;
    
    if (map.logOdds(f_end.x,f_end.y) > threshold){
        rayScore = map.logOdds(f_end.x,f_end.y);
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


        // // check the cell before and after
        double rayScoreBefore = map.logOdds(cell_before.x,cell_before.y);
        double rayScoreAfter = map.logOdds(cell_after.x,cell_after.y);
        // // double rayScoreAlt = (0.5*rayScoreBefore+0.5*rayScoreAfter)/maxScore;
        // // if (rayScoreAlt > 0){
        //     // rayScore = rayScoreAlt;
        // // }
        if (rayScoreBefore > threshold) {rayScore +=rayScoreBefore*0.5;}
        if (rayScoreAfter > threshold) {rayScore +=rayScoreAfter*0.5;}
    }
    // for (int i = -1; i<=1; i++){
    //     for (int j = -1; j <=1;j++){
    //         if (map.logOdds(f_end.x+i,f_end.x+j) >threshold){
    //             rayScore+=map.logOdds(f_end.x+i,f_end.x+j); 
    //         }
    //     }
    // }
    rayScore /= static_cast<double>(maxScore);
    // std::cout<<"rayscore"<<rayScore<<std::endl;

    //liklihood model
    // double q = 1;
    // if ray.range < 
    //     q *= (z_hit* + z_random/z_max);
    return pow(rayScore,1);
}

