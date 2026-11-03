// Independent cross-language serialized-message fixtures. No ROS initialization.
#include "alicia_d_driver/physical_reference_transport.hpp"
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
using namespace alicia_d_driver;

static control_msgs::FollowJointTrajectoryActionGoal load(const std::string& path) {
    std::ifstream in(path,std::ios::binary);
    if(!in)throw std::runtime_error("missing serialized fixture");
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),{});
    ros::serialization::IStream stream(bytes.data(),bytes.size());
    control_msgs::FollowJointTrajectoryActionGoal value;
    ros::serialization::deserialize(stream,value);
    if(stream.getLength()!=0)throw std::runtime_error("unconsumed fixture bytes");
    return value;
}

int main(int argc,char** argv) {
    if(argc!=2)return 2;
    const std::string root=argv[1];std::ifstream manifest(root+"/cases.tsv");
    if(!manifest)return 2;
    unsigned pass=0,fail=0;std::string name,left,right;int expected;
    while(manifest>>name>>expected>>left>>right) {
        bool ok=false;
        try {
            const auto a=load(root+"/"+left),b=load(root+"/"+right);
            ok=((reference_action_envelope(a)==reference_action_envelope(b))==bool(expected));
        }catch(const std::exception& e){std::cerr<<name<<": "<<e.what()<<"\n";}
        ok?++pass:++fail;
        std::cout<<"{\"case\":\""<<name<<"\",\"passed\":"<<(ok?"true":"false")<<"}\n";
    }
    std::cout<<"{\"summary\":{\"passed\":"<<pass<<",\"failed\":"<<fail<<"}}\n";
    return fail?1:0;
}
