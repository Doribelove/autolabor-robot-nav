#pragma once
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <dynamic_reconfigure/Config.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace autolabor_coverage {
inline std::string defaultTuningFile(const std::string& name) {
  const char* home = std::getenv("ROS_HOME");
  if (home && *home) return std::string(home)+"/autolabor_navigation/"+name;
  home = std::getenv("HOME");
  if (!home || !*home) throw std::runtime_error("ROS_HOME or HOME is required for persistent tuning");
  return std::string(home)+"/.ros/autolabor_navigation/"+name;
}
inline std::string tuningDirectory(const std::string& file) {
  if (file.empty() || file.front()!='/') throw std::runtime_error("tuning file must be absolute");
  return file.substr(0,file.find_last_of('/'));
}
inline void ensureTuningDirectory(const std::string& directory) {
  for (size_t i=1;i<=directory.size();++i) {
    if (i<directory.size() && directory[i]!='/') continue;
    const auto part=directory.substr(0,i);
    if (::mkdir(part.c_str(),0700)!=0 && errno!=EEXIST)
      throw std::runtime_error("cannot create tuning directory: "+part+": "+std::strerror(errno));
    struct stat info;
    if (::stat(part.c_str(),&info)!=0 || !S_ISDIR(info.st_mode))
      throw std::runtime_error("tuning parent is not a directory: "+part);
  }
}
template<class Config> bool sameTuning(const Config& a,const Config& b) {
  dynamic_reconfigure::Config ma,mb;a.__toMessage__(ma);b.__toMessage__(mb);
  if(ma.ints.size()!=mb.ints.size() || ma.doubles.size()!=mb.doubles.size()) return false;
  for(size_t i=0;i<ma.ints.size();++i) if(ma.ints[i].value!=mb.ints[i].value)return false;
  for(size_t i=0;i<ma.doubles.size();++i) if(ma.doubles[i].value!=mb.doubles[i].value)return false;
  return true;
}
template<class Config> Config loadTuning(const std::string& file,const Config& defaults) {
  tuningDirectory(file);
  struct stat info;
  if(::stat(file.c_str(),&info)!=0) {
    if(errno==ENOENT)return defaults;
    throw std::runtime_error("cannot stat tuning file: "+file);
  }
  boost::property_tree::ptree tree;boost::property_tree::read_json(file,tree);
  if(tree.get<int>("version")!=1)throw std::runtime_error("unsupported tuning schema");
  const auto& values=tree.get_child("values");
  dynamic_reconfigure::Config message;defaults.__toMessage__(message);
  if(values.size()!=message.ints.size()+message.doubles.size())
    throw std::runtime_error("saved tuning fields do not match this schema");
  for(auto& p:message.ints)p.value=values.get<int>(p.name);
  for(auto& p:message.doubles) {
    p.value=values.get<double>(p.name);
    if(!std::isfinite(p.value))throw std::runtime_error("nonfinite saved tuning: "+p.name);
  }
  Config result=defaults;
  if(!result.__fromMessage__(message))throw std::runtime_error("invalid saved tuning message");
  Config clamped=result;clamped.__clamp__();
  if(!sameTuning(result,clamped))throw std::runtime_error("saved tuning is out of range");
  return result;
}
template<class Config> void saveTuning(const std::string& file,const Config& config) {
  const std::string directory=tuningDirectory(file);ensureTuningDirectory(directory);
  dynamic_reconfigure::Config message;config.__toMessage__(message);
  boost::property_tree::ptree tree,values;tree.put("version",1);
  for(const auto& p:message.ints)values.put(p.name,p.value);
  for(const auto& p:message.doubles) {
    if(!std::isfinite(p.value))throw std::runtime_error("refusing nonfinite tuning");
    values.put(p.name,p.value);
  }
  tree.add_child("values",values);
  std::ostringstream output;boost::property_tree::write_json(output,tree);
  const auto bytes=output.str();
  std::string pattern=file+".tmp.XXXXXX";std::vector<char> staged(pattern.begin(),pattern.end());staged.push_back('\0');
  int descriptor=::mkstemp(staged.data());
  if(descriptor<0)throw std::runtime_error("cannot stage tuning: "+std::string(std::strerror(errno)));
  try {
    size_t done=0;
    while(done<bytes.size()) {
      const auto count=::write(descriptor,bytes.data()+done,bytes.size()-done);
      if(count<0 && errno==EINTR)continue;
      if(count<=0)throw std::runtime_error("cannot write tuning");
      done+=static_cast<size_t>(count);
    }
    if(::fsync(descriptor)!=0)throw std::runtime_error("cannot sync tuning");
    if(::close(descriptor)!=0) {descriptor=-1;throw std::runtime_error("cannot close staged tuning");}
    descriptor=-1;
    if(::rename(staged.data(),file.c_str())!=0)throw std::runtime_error("cannot commit tuning");
  } catch(...) {if(descriptor>=0)::close(descriptor);::unlink(staged.data());throw;}
  int parent=::open(directory.c_str(),O_RDONLY|O_DIRECTORY);
  if(parent>=0) {if(::fsync(parent)!=0)ROS_WARN("Tuning committed but directory sync failed: %s",directory.c_str());::close(parent);}
}
}
