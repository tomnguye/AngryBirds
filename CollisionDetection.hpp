#pragma once
#include <optional>
#include "CollisionEvent.hpp"
#include <memory>

std::vector<CollisionEvent> colliding_objects(std::vector<std::unique_ptr<PhysicsObject>>& objs);