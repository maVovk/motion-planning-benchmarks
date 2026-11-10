// macOS benchmark: one process per (manager, N, seed). Units: us and bytes.
#include <coal/broadphase/broadphase.h>
#include <coal/collision.h>
#include <coal/shape/geometric_shapes.h>
#include <malloc/malloc.h>
#include <mach/mach.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>
using namespace coal;
using Clock = std::chrono::steady_clock;
static double us(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::micro>(b-a).count();
}
static size_t heap() {
  malloc_statistics_t s{}; malloc_zone_statistics(nullptr, &s); return s.size_in_use;
}
static size_t rss() {
  mach_task_basic_info_data_t i{}; mach_msg_type_number_t n=MACH_TASK_BASIC_INFO_COUNT;
  if(task_info(mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&i, &n)!=KERN_SUCCESS)
    throw std::runtime_error("task_info failed");
  return i.resident_size;
}
static std::unique_ptr<BroadPhaseCollisionManager> manager(const std::string& s) {
  if(s=="naive") return std::make_unique<NaiveCollisionManager>();
  if(s=="sap") return std::make_unique<SaPCollisionManager>();
  if(s=="ssap") return std::make_unique<SSaPCollisionManager>();
  if(s=="interval") return std::make_unique<IntervalTreeCollisionManager>();
  if(s=="tree") return std::make_unique<DynamicAABBTreeCollisionManager>();
  if(s=="tree_array") return std::make_unique<DynamicAABBTreeArrayCollisionManager>();
  // Sphere diameter = 0.5. Fixed cell size and bounds for every N.
  if(s=="hash") return std::make_unique<SpatialHashingCollisionManager<>>(
      0.5, Vec3s::Constant(-32), Vec3s::Constant(32), 10007);
  throw std::runtime_error("unknown manager");
}
struct Callback : CollisionCallBackBase {
  bool stop=true; size_t hits=0, calls=0;
  CollisionRequest request; CollisionResult result;
  Callback() { request.num_max_contacts=1; request.enable_contact=false; }
  void init() override { hits=0; calls=0; }
  bool collide(CollisionObject* a, CollisionObject* b) override {
    ++calls; result.clear(); coal::collide(a,b,request,result);
    if(result.isCollision()) ++hits;
    return stop && hits>0;
  }
};
int main(int argc,char**argv) try {
  if(argc!=4) throw std::runtime_error("usage: coal_benchmark manager N seed");
  std::string name=argv[1]; int n=std::stoi(argv[2]); unsigned seed=std::stoul(argv[3]);
  if(n<1 || n>100000) throw std::runtime_error("invalid N");
  std::mt19937 rng(seed); std::uniform_real_distribution<double> d(-10,10);
  // Preallocate all measurement storage so snapshots exclude logging allocations.
  std::vector<std::unique_ptr<CollisionObject>> objects; objects.reserve(n);
  std::vector<CollisionObject*> pointers; pointers.reserve(n);
  std::vector<Vec3s> centers; centers.reserve(n);
  std::vector<std::unique_ptr<CollisionObject>> queries; queries.reserve(384);
  std::vector<size_t> expected; expected.reserve(384);
  for(int i=0;i<384;++i) {
    Vec3s p(d(rng),d(rng),d(rng));
    queries.push_back(std::make_unique<CollisionObject>(std::make_shared<Sphere>(0.25),Transform3s(Matrix3s::Identity(),p)));
  }
  Callback cb; cb.init();
  const auto h0=heap(); const auto r0=rss(); const auto t0=Clock::now();
  for(int i=0;i<n;++i) {
    Vec3s p(d(rng),d(rng),d(rng)); centers.push_back(p);
    objects.push_back(std::make_unique<CollisionObject>(std::make_shared<Sphere>(0.25),Transform3s(Matrix3s::Identity(),p)));
    pointers.push_back(objects.back().get());
  }
  const auto t1=Clock::now(); const auto h1=heap(); const auto r1=rss();
  const auto t2=Clock::now(); auto m=manager(name); m->registerObjects(pointers); m->setup();
  const auto t3=Clock::now(); const auto h2=heap(); const auto r2=rss();
  for(int i=0;i<384;++i) {
    Vec3s p=queries[i]->getTranslation();
    if(i<128) p[0]=30; // Guaranteed miss outside scene.
    else if(i<256) p=centers[(i-128)%n]; // Guaranteed hit.
    queries[i]->setTransform(Transform3s(Matrix3s::Identity(),p)); queries[i]->computeAABB();
    size_t count=0; for(auto& c:centers) count+=(c-p).squaredNorm()<=0.25;
    expected.push_back(count);
    cb.stop=false; cb.init(); m->collide(queries[i].get(),&cb);
    if(cb.hits!=count) throw std::runtime_error("all-hits oracle mismatch");
    cb.stop=true; cb.init(); m->collide(queries[i].get(),&cb);
    if((cb.hits>0)!=(count>0)) throw std::runtime_error("any-hit oracle mismatch");
  }
  const char* labels[]={"outside_miss","hit","random"};
  volatile size_t sink=0;
  for(int w=0;w<3;++w) {
    std::vector<double> times; times.reserve(9); size_t calls=0,hits=0;
    // Validation above doubles as cache/allocator warmup. Include callback reset;
    // target transforms/AABBs are prepared outside the timed query loop.
    for(int batch=0;batch<9;++batch) {
      auto a=Clock::now();
      for(int rep=0;rep<4;++rep) for(int j=0;j<128;++j) {
        cb.init(); m->collide(queries[w*128+j].get(),&cb); sink+=cb.hits;
      }
      times.push_back(us(a,Clock::now())/512);
    }
    for(int j=0;j<128;++j) {cb.init(); m->collide(queries[w*128+j].get(),&cb); calls+=cb.calls; hits+=cb.hits;}
    std::sort(times.begin(),times.end());
    std::cout<<name<<','<<n<<','<<seed<<','<<labels[w]<<','<<us(t0,t1)<<','<<us(t2,t3)<<','
      <<(static_cast<long long>(h1)-h0)<<','<<(static_cast<long long>(h2)-h1)<<','<<r0<<','<<r1<<','<<r2<<','
      <<times[4]<<','<<times[0]<<','<<times[8]<<','<<double(calls)/128<<','<<hits<<'\n';
  }
  return sink==static_cast<size_t>(-1);
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
