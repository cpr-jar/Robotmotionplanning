#include "motion/program/robot_program.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <fstream>
int main(int argc,char**argv) {
 using C=std::chrono::steady_clock; auto t=C::now();
 motion::PreparationContext ctx;
 auto source=motion::load_robot_program_source(argv[1],&ctx);
 auto profile=std::filesystem::absolute(argv[1]).parent_path()/source.robot_profile();
 auto robot=motion::load_serial_robot_files((profile.parent_path()/"robot.urdf").string(),profile.string(),&ctx);
 auto t1=C::now(); auto p=motion::compile_robot_program(source,std::move(robot),{},&ctx);auto t2=C::now();
 auto work=ctx.used(motion::PreparationResource::Work);
 std::ofstream csv("/tmp/motion-performance/phase-trace.csv");
 auto r=motion::simulate_robot_program(p,&csv,{100000,&ctx});auto t3=C::now();
 auto secs=[](auto a,auto b){return std::chrono::duration<double>(b-a).count();};
 std::cout<<"{\"import_s\":"<<secs(t,t1)<<",\"plan_s\":"<<secs(t1,t2)<<",\"simulate_csv_s\":"<<secs(t2,t3)<<",\"work\":"<<work<<",\"duration_s\":"<<r.duration_s<<",\"commands\":"<<r.commands<<"}\n";
}