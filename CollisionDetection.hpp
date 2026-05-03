#pragma once
#include "CollisionEvent.hpp"
#include <memory>
#include <optional>

std::vector<CollisionEvent>
colliding_objects(std::vector<std::unique_ptr<PhysicsObject>> &objs);