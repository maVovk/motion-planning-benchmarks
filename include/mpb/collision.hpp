#pragma once

#include "mpb/scene.hpp"

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/collision_data.h>

#include <cstddef>

namespace mpb {
// Reusable narrow-phase callback with optional early exit after a real collision.
struct CollisionCallback : coal::CollisionCallBackBase {
  bool stop_at_first = true;
  std::size_t hits = 0;
  std::size_t candidates = 0;
  coal::CollisionRequest request;
  coal::CollisionResult result;

  CollisionCallback();
  void init() override;
  bool collide(Object* first, Object* second) override;
};

}  // namespace mpb
