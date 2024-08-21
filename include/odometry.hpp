#include <tuple>
#include <utility>

struct Odometry{
    private:
    std::pair<double, double> currentCoord {0, 0};
    double theta; 

    public:
    std::pair<double, double> getCoord();
    double getTheta(); 
    void updatePose();
};