#include "mpb/collision.hpp"

#include <coal/collision.h>

namespace mpb {
CollisionCallback::CollisionCallback() {
  request.num_max_contacts = 1;
  request.enable_contact = false;
}

void CollisionCallback::init() {
  hits = 0;
  candidates = 0;
}

bool CollisionCallback::collide(coal::CollisionObject* first, coal::CollisionObject* second) {
  ++candidates;
  result.clear();
  coal::collide(first, second, request, result);
  hits += result.isCollision();
  return stop_at_first && hits > 0;
}

}  // namespace mpb
