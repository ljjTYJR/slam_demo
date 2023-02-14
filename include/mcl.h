#ifndef __MCL_H__
#define __MCL_H__

#include <vector>

struct Particle {
    int idx;
    double x;
    double y;
    double theta;
    double weight;
    double score;
};

class mcl
{
private:
    int n_particles;
    bool initialized;
    std::vector<double> weights;

public:
    std::vector<Particle> particles;
    mcl() : n_particles(0), initialized(false) {};
    ~mcl() {};

    void init(int n, double x,double y, double theta, double var[]);

    void predict(double var[3], double motion[3]);

    void updateWeights();

    void resample();

    bool isInitialized() {
        return initialized;
    }

    int getNumParticles() {
        return n_particles;
    }
};

#endif