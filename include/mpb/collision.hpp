#pragma once

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/collision_data.h>

#include <cstddef>


namespace mpb {
struct CollisionCallback : coal::CollisionCallBackBase {
  bool stop_at_first = true;
  size_t hits = 0;
  size_t candidates = 0;
  coal::CollisionRequest request;
  coal::CollisionResult result;

  CollisionCallback();
  void init() override;
  bool collide(coal::CollisionObject* first, coal::CollisionObject* second) override;
};

}  // namespace mpb
