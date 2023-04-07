#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <common_utils/geometric/angle_functions.hpp>

#include <iostream>
#include <fstream>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double particleWeight= 1.0/kNumParticles_;
    posteriorPose_ = pose;

    for (auto & p:posterior_){
        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.parent_pose = p.pose;
        p.pose.utime = pose.utime;
        p.weight = particleWeight;
    }
    
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    randomPoseGen.update_map(&map);
    double particleWeight= 1.0/kNumParticles_;

    for (auto & p:posterior_){
        p = randomPoseGen.get_particle();
        p.weight = particleWeight;
    }
    posteriorPose_ = estimatePosteriorPose(posterior_);
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    auto prior = resamplePosteriorDistribution(&map);
    auto proposal = computeProposalDistribution(prior);
    posterior_ = computeNormalizedPosterior(proposal, laser, map);
    // OPTIONAL TODO: Add reinvigoration step
    posteriorPose_ = estimatePosteriorPose(posterior_);
    posteriorPose_.utime = odometry.utime;
    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid* map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior;
    // double particleWeights = 1.0/kNumParticles_;
    // std::random_device rd;
    // std::mt19937 generator(rd());
    // std::normal_distribution<> dist(0.0,0.01);
    // for (auto & p: prior){
    //     p.pose.x = posteriorPose_.x + dist(generator);
    //     p.pose.y = posteriorPose_.y + dist(generator);
    //     p.pose.theta = posteriorPose_.theta + dist(generator);
    //     p.pose.utime = posteriorPose_.utime;
    //     p.parent_pose = posteriorPose_;
    //     p.weight = particleWeights;
    // }

    // std::cout<<"num"<<kNumParticles_<<std::endl;
    double r = static_cast<double>(rand())/RAND_MAX/kNumParticles_;
    double c = posterior_.at(0).weight; 
    int i = 1;
    double u;
    double wavg = 0.0;
    std::ofstream weightfile;
    std::ofstream particleifile;
    weightfile.open("weightt.txt");
    particleifile.open("particle.txt");
    for (int m = 1; m<= kNumParticles_; m++){
        u = r+static_cast<double>((m-1))/kNumParticles_;
        while (u>c){
            i++;
            c+=posterior_.at(i-1).weight;
        } 
        
        prior.push_back(posterior_.at(i-1));
        particleifile<<i-1<<"\n";
        // wavg += 1/kNumParticles_*posterior_.at(i).weight;

        // prior.at(i-1).weight = 1.0/kNumParticles_;

    }
    
    for (auto & p:prior){
        weightfile<<p.weight<<"\n";
        p.weight = 1.0/kNumParticles_;
        // std::cout<<"weight:"<<p.weight<<std::endl;
    }
    weightfile.close();
    particleifile.close();
    // std::cout<<"numparticles:"<<kNumParticles_<<std::endl;


    // ParticleList priorMCL;
    // samplingAugmentation.insert_average_weight(wavg);
    // randomPoseGen.update_map(map);
    // // deal with losing diversity using MCL;
    // for (auto & p:prior){
    //     if (samplingAugmentation.sample_randomly()){
    //         priorMCL.push_back(randomPoseGen.get_particle());
    //     }else{
    //         priorMCL.push_back(p);
    //     }
    // }
    return prior;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
    for (auto & p: prior){
        proposal.push_back(actionModel_.applyAction(p));
        //proposal.push_back(p);
    }
    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    ParticleList posterior;
    double sumWeights = 0.0;
    for (auto & p:proposal){
        mbot_lcm_msgs::particle_t weighted = p;
        // std::cout<<"before:"<<p.weight<<std::endl;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        // plot particle position and its likelihood;
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }
    std::cout<<"sumw:"<<sumWeights<<std::endl;
    // for (auto & p : posterior){
    //         p.weight = 1.0/kNumParticles_;
            
    // }
    if (sumWeights == 0.0){
        for (auto & p : posterior){
            p.weight = 1.0/kNumParticles_;
            
        }
    }else{
        for (auto & p : posterior){
            // std::cout<<p.weight <<" "<<sumWeights <<std::endl;
            p.weight /= sumWeights;
            // std::cout<<p.weight <<std::endl;
        }
    }
    // posterior = proposal;
    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;
    ParticleList best_particles;
    double threshold  = 0.0;
    for (auto &p: posterior){
        // std::cout<<"w:"<<p.weight<<std::endl;
        // best_particles.push_back(p);
        if (p.weight > 1.0*1/kNumParticles_){
            best_particles.push_back(p);
        }
    }
    pose  = computeParticlesAverage(best_particles);
    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;
    for (auto &p : particles_to_average){
        // std::cout<<p.pose.x<<" "<<p.weight<<std::endl;
        avg_pose.x += p.pose.x * p.weight;
        avg_pose.y += p.pose.y * p.weight;
        cosThetaMean += std::cos(p.pose.theta) * p.weight;
        sinThetaMean += std::sin(p.pose.theta) * p.weight;
    }
    avg_pose.theta = std::atan2(sinThetaMean, cosThetaMean);
    return avg_pose;
}
