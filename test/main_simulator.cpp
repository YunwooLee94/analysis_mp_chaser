//
// Created by larr-laptop on 24. 1. 11.
//
#include <analysis_los_keeper/Analyzer.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"analyzer_los_keeper");
    los_keeper::Analyzer analyzer;
    analyzer.run();
    return 0;
}