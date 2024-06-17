/* fluid_simulation.cpp */

#include "fluid_simulation.h"

void FluidSimulator:: _populate_UV_neighbors() {
    for (int i = 0; i < size_of(UV_neighbors); ++i) {
        int x = i % Width;
        int y = floor(i / Width);
        int left = (2 * Width + 1) * y + x + Width + 1;
        int right = (2 * Width + 1) * y + x + Width + 2;
        int up = (2 * Width + 1) * y + x;
        int down = (2 * Width + 1) * y + x + 2 * Width + 1;
        
        UV_neighbors[i] = {left, right, up, down};
    }
}

void FluidSimulator:: _populate_XY_neighbors() {
    for (int i = 0; i < XY_neighbors.size(); ++i) {
        int left = (i % Width == 0) ? -1 : i - 1;
        int right = (i % Width == Width - 1) ? -1 : i + 1;
        int up = (i < Width) ? -1 : i - Width;
        int down = (i >= (Height - 1) * Width) ? -1 : i + Width;

        XY_neighbors[i] = {left, right, up, down};
    }
}

void FluidSimulator:: _solve_incompressibility(float delta) {
    for (int i = 0; i <= Physics_step; ++i) {
        for (int xy = 0; xy < XY.size(); ++xy) {
            // if there is no wall it will break
            if (CollisionMap[xy] == 0) {
                continue;
            }
            int collisionLeft = CollisionMap[XY_neighbors[xy].[0]] == -1 ? 0 : CollisionMap[XY_neighbors[xy].[0]];
            int collisionRight = CollisionMap[XY_neighbors[xy].[1]] == -1 ? 0 : CollisionMap[XY_neighbors[xy].[1]];
            int collisionUp = CollisionMap[XY_neighbors[xy].[2]] == -1 ? 0 : CollisionMap[XY_neighbors[xy].[2]];
            int collisionDown = CollisionMap[XY_neighbors[xy].[3]] == -1 ? 0 : CollisionMap[XY_neighbors[xy].[3]];


            if (collisionLeft == 0 && collisionRight == 0 && collisionUp == 0 && collisionDown == 0) {
                continue;
            }
            int collisionSum = collisionLeft + collisionRight + collisionUp + collisionDown;
            float div = - UV[UV_neighbors[xy].[0]] + UV[UV_neighbors[xy].[1]] + UV[UV_neighbors[xy].[2]] - UV[UV_neighbors[xy].[3]];
            float pressure = (- div / collisionSum) * 1.9;
            if (i == Physics_step) {
                float pressureQuotient = Density * GridSpacing / delta; 
                PressureMap[xy] = pressure * pressureQuotient;
            }
            UV[UV_neighbors[xy].[0]] += collisionLeft * pressure;
            UV[UV_neighbors[xy].[1]] -= collisionRight * pressure;
            UV[UV_neighbors[xy].[2]] -= collisionUp * pressure;
            UV[UV_neighbors[xy].[3]] += collisionDown * pressure;
        }
    }
}

void FluidSimulator::_make_box_around_collisionMap() {
    for (int i = 0; i < CollisionMap.size(); ++i) {
        if (i < Width) {
            CollisionMap[i] = 0;
            continue;
        }
        if ((i + 1) % Width == (Width -1)) {
            CollisionMap[i] = 0;
            continue;
        }
        if ((i + 1 ) % Width == 1) {
            CollisionMap[i] = 0;
            continue;
        }
        if (i >= (Width * Height - Width)) {
            CollisionMap[i] = 0;
            continue;
        }
    }
}

void FluidSimulator::delta_step(float delta) {
    _solve_incompressibility(delta);
}

void FluidSimulator::_bind_methods() {
	ClassDB::bind_method(D_METHOD("init", "width", "height"), &FluidSimulator::init);
    ClassDB::bind_method(D_METHOD("delta_step", "delta"), &FluidSimulator::init);
}

FluidSimulator::FluidSimulator(int width, int height, int step, float density, float gridSpacing) {
	Density(density);
    GridSpacing(gridSpacing);
    Width(width);
    Height(height);
    Physics_step(step);
    int XY_size = Width * Height;
    int UV_size = (2 * Width + 1) * Height / 2; 
    if (!Height % 2 ==0) {
        UV_size += Width + 1;
    };
    XY(XY_size, 0.0f);
    CollisionMap(XY_size,1);
    PressureMap(XY_size,0.0f);
    UV(UV_size, 0.0f);
    // pre-calculates the neighbor indexes of XY to XY
    XY_neighbors(XY_size, int arr[4]);
    // pre-calculates the neighbor indexes of UV to XY
    UV_neighbors(XY_size, int arr[4]);
    populate_XY_neighbors();
    populate_UV_neighbors();
}
