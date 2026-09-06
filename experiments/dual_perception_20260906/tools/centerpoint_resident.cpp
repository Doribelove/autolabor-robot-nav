// Native NASH_M adapter. Model ABI is validated before allocating/submitting.
// Pillarization is CPU work; the model graph executes on the BPU.
#include "hobot/dnn/hb_dnn.h"
#include <array>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
constexpr int kPillars=40000, kPoints=20, kWidth=40064;
constexpr int kClasses[]={1,2,2,1,2,2};
constexpr size_t kOutput=128*128*(6*10+10);
using Clock=std::chrono::steady_clock;
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
void check(int code,const char* why){if(code)throw std::runtime_error(std::string(why)+": "+std::to_string(code));}
double elapsed(Clock::time_point start){return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
struct Session {
  hbDNNPackedHandle_t packed=nullptr;
  hbDNNHandle_t model=nullptr;
  hbUCPTaskHandle_t task=nullptr;
  std::array<hbDNNTensor,2> input{};
  std::array<hbDNNTensor,36> output{};
  std::vector<int> cells=std::vector<int>(512*512,-1);
  std::vector<int> counts=std::vector<int>(kPillars,0);
  bool failed=false;
  ~Session(){
    if(task)hbUCPReleaseTask(task);
    for(auto& t:output)if(t.sysMem.virAddr)hbUCPFree(&t.sysMem);
    for(auto& t:input)if(t.sysMem.virAddr)hbUCPFree(&t.sysMem);
    if(packed)hbDNNRelease(packed);
  }
  void load(const char* path){
    require(std::string(hbDNNGetVersion()).find("3.14.5")!=std::string::npos,"Unexpected private DNN ABI");
    check(hbDNNInitializeFromFiles(&packed,&path,1),"load");
    const char** names=nullptr;int count=0;
    check(hbDNNGetModelNameList(&names,&count,packed),"names");
    require(count==1 && std::string(names[0])=="centerpoint_pointpillar_nuscenes","Unexpected CenterPoint model");
    check(hbDNNGetModelHandle(&model,packed,names[0]),"handle");
    check(hbDNNGetInputCount(&count,model),"inputs");require(count==2,"Expected features and coors");
    check(hbDNNGetOutputCount(&count,model),"outputs");require(count==36,"Expected six CenterPoint heads");
    for(int i=0;i<2;++i){
      auto& p=input[i].properties;
      check(hbDNNGetInputTensorProperties(&p,model,i),"input properties");
      if(i==0){
        require(p.tensorType==HB_DNN_TENSOR_TYPE_S8 && p.quantiType==SCALE && p.validShape.numDimensions==4,
                "Unexpected feature type");
        const int shape[]={1,5,kPoints,kPillars};const int stride[]={5*kPoints*kWidth,kPoints*kWidth,kWidth,1};
        for(int d=0;d<4;++d)require(p.validShape.dimensionSize[d]==shape[d] && p.stride[d]==stride[d],"Feature layout mismatch");
        require(p.alignedByteSize==5*kPoints*kWidth && p.scale.scaleLen==1 && p.scale.scaleData &&
                std::isfinite(p.scale.scaleData[0]) && p.scale.scaleData[0]>0,"Feature scale/size mismatch");
        require(p.scale.zeroPointLen==0 || (p.scale.zeroPointLen==1 && p.scale.zeroPointData[0]==0),"Unsupported feature zero point");
      }else{
        require(p.tensorType==HB_DNN_TENSOR_TYPE_S32 && p.quantiType==NONE && p.validShape.numDimensions==2 &&
                p.validShape.dimensionSize[0]==kPillars && p.validShape.dimensionSize[1]==4 &&
                p.stride[0]==16 && p.stride[1]==4 && p.alignedByteSize==kPillars*16,"Coordinate layout mismatch");
      }
      check(hbUCPMallocCached(&input[i].sysMem,p.alignedByteSize,0),"allocate input");
    }
    for(int i=0;i<36;++i){
      auto& p=output[i].properties;check(hbDNNGetOutputTensorProperties(&p,model,i),"output properties");
      const int channels[]={2,1,3,2,2,kClasses[i/6]};int c=channels[i%6];
      require(p.tensorType==HB_DNN_TENSOR_TYPE_S32 && p.quantiType==SCALE && p.validShape.numDimensions==4 &&
              p.validShape.dimensionSize[0]==1 && p.validShape.dimensionSize[1]==128 &&
              p.validShape.dimensionSize[2]==128 && p.validShape.dimensionSize[3]==c,"Output shape/type mismatch");
      require(p.stride[3]==4 && p.stride[2]>=c*4 && p.stride[1]>=128*p.stride[2] &&
              p.stride[0]>=128*p.stride[1] && p.alignedByteSize>=p.stride[0] &&
              p.alignedByteSize<=1024*1024,"Unsafe output strides");
      require(p.scale.scaleData && (p.scale.scaleLen==1 || p.scale.scaleLen==c) &&
              (p.quantizeAxis==3 || p.scale.scaleLen==1),"Unsupported output quantization");
      require(p.scale.zeroPointLen==0 || (p.scale.zeroPointData && (p.scale.zeroPointLen==1 || p.scale.zeroPointLen==c)),"Invalid output zero points");
      for(int j=0;j<p.scale.scaleLen;++j)require(std::isfinite(p.scale.scaleData[j]) && p.scale.scaleData[j]>0,"Nonfinite output scale");
      check(hbUCPMallocCached(&output[i].sysMem,p.alignedByteSize,0),"allocate output");
    }
  }
  void infer(const float* points,size_t count,float* result,size_t result_count,double* timing){
    require(!failed && points && count>0 && count<=300000 && result && result_count==kOutput && timing,"Invalid inference request");
    auto started=Clock::now();
    std::fill(cells.begin(),cells.end(),-1);std::fill(counts.begin(),counts.end(),0);
    std::memset(input[0].sysMem.virAddr,0,input[0].properties.alignedByteSize);
    // Negative coordinates mark unused pillars for the scatter operation.
    std::memset(input[1].sysMem.virAddr,0xff,input[1].properties.alignedByteSize);
    auto* feature=static_cast<int8_t*>(input[0].sysMem.virAddr);
    auto* coors=static_cast<int32_t*>(input[1].sysMem.virAddr);
    const float scale=input[0].properties.scale.scaleData[0];
    int used=0,kept=0;
    for(size_t i=0;i<count;++i){
      const float* p=points+i*5;
      if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2])||!std::isfinite(p[3])||!std::isfinite(p[4]))continue;
      if(p[0]<=-51.2f||p[0]>=51.2f||p[1]<=-51.2f||p[1]>=51.2f||p[2]<=-5.f||p[2]>=3.f||p[3]<0||p[3]>255||p[4]<0||p[4]>1.0f)continue;
      int x=static_cast<int>(std::floor((p[0]+51.2f)/.2f));
      int y=static_cast<int>(std::floor((p[1]+51.2f)/.2f));
      if(x<0||x>=512||y<0||y>=512)continue;
      int& cell=cells[y*512+x];
      if(cell<0){
        if(used==kPillars)continue;
        cell=used++;coors[cell*4]=0;coors[cell*4+1]=0;coors[cell*4+2]=y;coors[cell*4+3]=x;
      }
      int slot=counts[cell];if(slot==kPoints)continue;
      const float normalized[]={(p[0]+51.2f)/102.4f,(p[1]+51.2f)/102.4f,(p[2]+5.f)/8.f,p[3]/255.f,p[4]};
      for(int c=0;c<5;++c){
        float q=std::nearbyint(normalized[c]/scale);
        feature[c*kPoints*kWidth+slot*kWidth+cell]=static_cast<int8_t>(std::max(-128.f,std::min(127.f,q)));
      }
      ++counts[cell];++kept;
    }
    require(used>0,"No valid pillars");
    for(auto& t:input)check(hbUCPMemFlush(&t.sysMem,HB_SYS_MEM_CACHE_CLEAN),"flush input");
    timing[0]=elapsed(started);timing[3]=used;timing[4]=kept;
    started=Clock::now();check(hbDNNInferV2(&task,output.data(),input.data(),model),"infer");
    hbUCPSchedParam schedule{};HB_UCP_INITIALIZE_SCHED_PARAM(&schedule);schedule.backend=HB_UCP_BPU_CORE_0;
    check(hbUCPSubmitTask(task,&schedule),"submit");check(hbUCPWaitTaskDone(task,2000),"wait");
    timing[1]=elapsed(started);check(hbUCPReleaseTask(task),"release");task=nullptr;started=Clock::now();
    for(auto& t:output){
      check(hbUCPMemFlush(&t.sysMem,HB_SYS_MEM_CACHE_INVALIDATE),"invalidate output");
      auto& p=t.properties;int channels=p.validShape.dimensionSize[3];
      for(int y=0;y<128;++y)for(int x=0;x<128;++x){
        auto* pixel=reinterpret_cast<const int32_t*>(static_cast<const char*>(t.sysMem.virAddr)+y*p.stride[1]+x*p.stride[2]);
        for(int c=0;c<channels;++c){
          int32_t zero=p.scale.zeroPointLen?p.scale.zeroPointData[p.scale.zeroPointLen==1?0:c]:0;
          *result++=static_cast<float>(static_cast<int64_t>(pixel[c])-zero)*p.scale.scaleData[p.scale.scaleLen==1?0:c];
        }
      }
    }
    timing[2]=elapsed(started);
  }
};
void error_text(char* p,size_t size,const std::exception& e){if(p&&size)std::snprintf(p,size,"%s",e.what());}
}
extern "C" {
void* centerpoint_create(const char* path,char* error,size_t size){try{auto s=std::make_unique<Session>();s->load(path);return s.release();}catch(const std::exception& e){error_text(error,size,e);return nullptr;}}
int centerpoint_infer(void* context,const float* p,size_t count,float* result,size_t n,double* time,char* error,size_t size){
  if(!context)return -1;
  auto* s=static_cast<Session*>(context);
  try{s->infer(p,count,result,n,time);return 0;}catch(const std::exception& e){
    s->failed=true;error_text(error,size,e);
    if(s->task){
      // A timed-out submitted task may still reference device memory. Exit this
      // private worker instead of freeing/reusing buffers under an active task.
      std::fprintf(stderr,"CenterPoint task state uncertain: %s; exiting private worker\n",e.what());
      std::fflush(stderr);std::_Exit(70);
    }
    return -1;
  }
}
void centerpoint_destroy(void* context){delete static_cast<Session*>(context);}
}
