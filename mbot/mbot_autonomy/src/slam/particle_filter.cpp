#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <common_utils/geometric/angle_functions.hpp>

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
    double particleWeights = 1.0/kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0,0.01);
    // for (auto & p: prior){
    //     p.pose.x = posteriorPose_.x + dist(generator);
    //     p.pose.y = posteriorPose_.y + dist(generator);
    //     p.pose.theta = posteriorPose_.theta + dist(generator);
    //     p.pose.utime = posteriorPose_.utime;
    //     p.parent_pose = posteriorPose_;
    //     p.weight = particleWeights;
    // }

    // ParticleList prior;
    double r = rand()/RAND_MAX/kNumParticles_;
    double c = posterior_.at(0).weight; 
    int i = 1;
    double u;
    double wavg = 0.0;
    
    for (int m = 1; m<= kNumParticles_; m++){
        u = r+(m-1)/kNumParticles_;
        while (u>c){
            i++;
            c+=posterior_.at(i-1).weight;
        } 
        prior.push_back(posterior_.at(i-1));
        wavg += 1/kNumParticles_*posterior_.at(i).weight;
    }

    ParticleList priorMCL;
    samplingAugmentation.insert_average_weight(wavg);
    randomPoseGen.update_map(map);
    // deal with losing diversity using MCL;
    for (auto & p:prior){
        if (samplingAugmentation.sample_randomly()){
            priorMCL.push_back(randomPoseGen.get_particle());
        }else{
            priorMCL.push_back(p);
        }
    }
    return priorMCL;
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
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }
    for (auto & p : posterior){
        p.weight /= sumWeights;
    }
    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;
    ParticleList best_particles;
    //maybe do a kmeans?
    // for (auto &p: posterior){

    // }   
    pose  = computeParticlesAverage(posterior);
    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;
    for (auto &p : particles_to_average){
        avg_pose.x += p.pose.x * p.weight;
        avg_pose.y += p.pose.y * p.weight;
        cosThetaMean += std::cos(p.pose.theta) * p.weight;
        sinThetaMean += std::sin(p.pose.theta) * p.weight;
    }
    avg_pose.theta = std::atan2(sinThetaMean, cosThetaMean);
    return avg_pose;
}
