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

#include "Rasterization.h"
#include <queue>


double Rasterization::findHeightValByScanline(Particle *p, Cloth& cloth) {
    int posX = p->pos_x;
    int posY = p->pos_y;

    for (int i = posX + 1; i < cloth.num_particles_width; i++) {
        double nearestHeight = cloth.getParticle(i, posY)->nearestPointHeight;

        if (nearestHeight > MIN_INF)
            return nearestHeight;
    }

    for (int i = posX - 1; i >= 0; i--) {
        double nearestHeight = cloth.getParticle(i, posY)->nearestPointHeight;

        if (nearestHeight > MIN_INF)
            return nearestHeight;
    }

    for (int j = posY - 1; j >= 0; j--) {
        double nearestHeight = cloth.getParticle(posX, j)->nearestPointHeight;

        if (nearestHeight > MIN_INF)
            return nearestHeight;
    }

    for (int j = posY + 1; j < cloth.num_particles_height; j++) {
        double nearestHeight = cloth.getParticle(posX, j)->nearestPointHeight;

        if (nearestHeight > MIN_INF)
            return nearestHeight;
    }

    return findHeightValByNeighbor(p);
}


double Rasterization::findHeightValByNeighbor(Particle *p) {
    std::queue<Particle *>  neighborQueue;
    std::vector<Particle *> visitedParticles;
    for (std::size_t i = 0; i < p->neighborsList.size(); i++) {
        p->isVisited = true;
        neighborQueue.push(p->neighborsList[i]);
    }

    // iterate over the neighborQueue
    while (!neighborQueue.empty()) {
        Particle *neighbor = neighborQueue.front();
        neighborQueue.pop();
        visitedParticles.push_back(neighbor);

        if (neighbor->nearestPointHeight > MIN_INF) {
            for (std::size_t i = 0; i < visitedParticles.size(); i++)
                visitedParticles[i]->isVisited = false;

            while (!neighborQueue.empty()) {
                Particle *pp = neighborQueue.front();
                pp->isVisited = false;
                neighborQueue.pop();
            }

            return neighbor->nearestPointHeight;
        } else {
            for (std::size_t i = 0; i < neighbor->neighborsList.size(); i++) {
                Particle *ptmp = neighbor->neighborsList[i];

                if (!ptmp->isVisited) {
                    ptmp->isVisited = true;
                    neighborQueue.push(ptmp);
                }
            }
        }
    }

    return MIN_INF;
}

void Rasterization::RasterTerrian(Cloth          & cloth,
                                  csf::PointCloud& pc,
                                  std::vector<double> & heightVal) {

    for (std::size_t i = 0; i < pc.size(); i++) {
        double pc_x = pc[i].x;
        double pc_z = pc[i].z;

        double deltaX = pc_x - cloth.origin_pos.f[0];
        double deltaZ = pc_z - cloth.origin_pos.f[2];
        int    col    = int(deltaX / cloth.step_x + 0.5);
        int    row    = int(deltaZ / cloth.step_y + 0.5);

        if ((col >= 0) && (row >= 0)) {
            Particle *pt = cloth.getParticle(col, row);
            pt->correspondingLidarPointList.push_back(i);
            double pc2particleDist = SQUARE_DIST(
                pc_x, pc_z,
                pt->getPos().f[0],
                pt->getPos().f[2]
            );

            if (pc2particleDist < pt->tmpDist) {
                pt->tmpDist            = pc2particleDist;
                pt->nearestPointHeight = pc[i].y;
                pt->nearestPointIndex  = i;
            }
        }
    }
    heightVal.resize(cloth.getSize());

    // #ifdef CSF_USE_OPENMP
    // #pragma omp parallel for
    // #endif
    for (int i = 0; i < cloth.getSize(); i++) {
        Particle *pcur          = cloth.getParticle1d(i);
        double    nearestHeight = pcur->nearestPointHeight;

        if (nearestHeight > MIN_INF) {
            heightVal[i] = nearestHeight;
        } else {
            heightVal[i] = findHeightValByScanline(pcur, cloth);
        }
    }
}
