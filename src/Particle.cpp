// ======================================================================================
// Copyright 2017 State Key Laboratory of Remote Sensing Science, 
// Institute of Remote Sensing Science and Engineering, Beijing Normal University

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ======================================================================================

#include "Particle.h"

/* This is one of the important methods, where the time is progressed
*  a single step size (TIME_STEPSIZE) The method is called by
*  Cloth.time_step() Given the equation "force = mass * acceleration"
*  the next position is found through verlet integration*/
void Particle::timeStep() {
    if (movable) {
        Vec3 temp = pos;
        pos = pos + (pos - old_pos) * (1.0 - DAMPING) + acceleration * time_step2;
        old_pos = temp;
    }
}

void Particle::satisfyConstraintSelf(int constraintTimes) {
    // Immovable particles never need to move themselves.
    if (!movable) return;

    for (std::size_t i = 0; i < neighborsList.size(); i++) {
        Particle *p2 = neighborsList[i];
        // Only the Y axis is ever corrected by cloth constraints.
        double dy = p2->pos.f[1] - pos.f[1];

        if (p2->isMovable()) {
            // Both particles are movable. Apply only our half of the correction
            // here; p2 applies its symmetric half when it runs its own pass.
            // This makes the loop write-to-self-only and therefore race-free.
            pos.f[1] += dy * (constraintTimes > 14 ? 0.5 : doubleMove1[constraintTimes]);
        } else {
            // p2 is fixed — move ourselves the full correction.
            pos.f[1] += dy * (constraintTimes > 14 ? 1.0 : singleMove1[constraintTimes]);
        }
        // The former "!p1->isMovable() && p2->isMovable()" branch is gone:
        // p2 handles that case in its own pass, keeping all writes local to `this`.
    }
}
