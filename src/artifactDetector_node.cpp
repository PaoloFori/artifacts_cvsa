#include <ros/ros.h>
#include "artifacts_cvsa/ArtifactDetector.h"

int main(int argc, char** argv) {

    
    // ros initialization
    ros::init(argc, argv, "artifactDetector_node");

    ArtifactDetector cvsa;
    
    if(cvsa.configure("ArtifactCfg") == false) {
        std::cerr<<"SETUP ERROR"<<std::endl;
        return -1;
    }

    ROS_INFO("[INFO] Configuration done");
    
    cvsa.run();
    
    ros::shutdown();
    return 0;
}
