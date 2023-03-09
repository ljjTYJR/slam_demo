#include "slam_demo/mcl.h"
#include <cmath>
#include <random>

using namespace std;

void mcl::init(int n, double x, double y, double theta, double var[]) {
    n_particles = n;

    weights.resize(n_particles);
    particles.resize(n_particles);

    double var_x, var_y, var_theta;
    var_x = var[0];
    var_y = var[1];
    var_theta = var[2];

    // get normal distribution around x,y,theta
    normal_distribution<double> dist_x(x, var_x);
    normal_distribution<double> dist_y(y, var_y);
    normal_distribution<double> dist_theta(theta, var_theta);

    default_random_engine gen;

    // Create the random particles
    for (int i = 0; i < n_particles; ++i) {
        Particle p;
        p.idx = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1 / n_particles;
        p.score = 0;

        particles[i] = p;
        weights[i] = p.weight;
    }
    initialized = true;
}

void mcl::predict(double var[], double motion[]) {
    double var_x = var[0];
    double var_y = var[1];
    double var_theta = var[2];

    default_random_engine gen;

    for (int i = 0; i < n_particles; ++i) {
        Particle* p = &particles[i];

        // Predict
        double new_x = p->x + motion[0];
        double new_y = p->y + motion[1];
        double new_theta = p->theta + motion[2];

        // Add Gaussian noise
        normal_distribution<double> dist_x(new_x, var_x);
        normal_distribution<double> dist_y(new_y, var_y);
        normal_distribution<double> dist_theta(new_theta, var_theta);

        // Update the next pose
        p->x = dist_x(gen);
        p->y = dist_y(gen);
        p->theta = dist_theta(gen);
    }
}

void mcl::updateWeights() {
    double sum = 0;
    for (int i = 0; i < n_particles; ++i) {
        sum += exp(particles[i].score);
    }
    // Renew the weights
    for (int i = 0; i < n_particles; ++i) {
        particles[i].weight = exp(particles[i].score) / sum;
    }
    return;
}