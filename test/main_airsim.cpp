//
// Created by larr-laptop on 24. 1. 6.
//

#include <analysis_mp_chaser/Analyzer.h>
int main(int argc, char** argv){
    ros::init(argc,argv,"analyzer");
    mp_chaser::Analyzer analyzer;
    analyzer.run();
    return 0;
}