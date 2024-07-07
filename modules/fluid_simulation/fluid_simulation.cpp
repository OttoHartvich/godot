#include "fluid_simulation.h"
#include "core/object/ref_counted.h"
#include <Slot.h>
#include <array>
#include <cmath>
#include <pairpos-graph.hh>
#include <vector>

FluidSimulator::FluidSimulator(int width, int height, int step, float density, float gridSpacing) :
		Width(width),
		Height(height),
		Physics_step(step),
		Density(density),
		GridSpacing(gridSpacing),
		XY(width * height, 0.0f),
		CollisionMap(width * height, 1),
		PressureMap(width * height, 0.0f),
		UV((2 * width + 1) * (height / 2) + ((height % 2 == 0) ? 0 : width + 1), 0.0f),
		NewUV((2 * width + 1) * (height / 2) + ((height % 2 == 0) ? 0 : width + 1), 0.0f),
		XY_neighbors(width * height),
		UV_neighbors((2 * width + 1) * (height / 2) + ((height % 2 == 0) ? 0 : width + 1)),
		UV_to_UV_average_neighbors((2 * width + 1) * (height / 2) + ((height % 2 == 0) ? 0 : width + 1)) {
	_populate_XY_neighbors();
	_populate_UV_neighbors();
	_populate_UV_to_UV_average_neighbors(UV.size());
}

void FluidSimulator::_bind_methods() {
	ClassDB::bind_method(D_METHOD("_populate_UV_neighbors"), &FluidSimulator::_populate_UV_neighbors);
	ClassDB::bind_method(D_METHOD("_populate_XY_neighbors"), &FluidSimulator::_populate_XY_neighbors);
	ClassDB::bind_method(D_METHOD("_populate_UV_to_UV_average_neighbors", "UV_size"), &FluidSimulator::_populate_UV_to_UV_average_neighbors);
	ClassDB::bind_method(D_METHOD("_solve_incompressibility", "delta"), &FluidSimulator::_solve_incompressibility);
	ClassDB::bind_method(D_METHOD("delta_step", "delta"), &FluidSimulator::delta_step);
	ClassDB::bind_method(D_METHOD("_calculate_weighted_average","x","y"), &FluidSimulator::_calculate_weighted_average);
}
void FluidSimulator::_calculate_weighted_average(float x, float y) {
	
}

void FluidSimulator::_populate_UV_neighbors() {
	for (int i = 0; i < UV_neighbors.size(); ++i) {
		int x = i % Width;
		int y = std::floor(i / Width);
		int left = (2 * Width + 1) * y + x + Width + 1;
		int right = (2 * Width + 1) * y + x + Width + 2;
		int up = (2 * Width + 1) * y + x;
		int down = (2 * Width + 1) * y + x + 2 * Width + 1;

		UV_neighbors[i] = { left, right, up, down };
	}
}

void FluidSimulator::_populate_XY_neighbors() {
	for (int i = 0; i < XY_neighbors.size(); ++i) {
		int left = (i % Width == 0) ? -1 : i - 1;
		int right = (i % Width == Width - 1) ? -1 : i + 1;
		int up = (i < Width) ? -1 : i - Width;
		int down = (i >= (Height - 1) * Width) ? -1 : i + Width;

		XY_neighbors[i] = { left, right, up, down };
	}
}

void FluidSimulator::_populate_UV_to_UV_average_neighbors(int UV_size) {
	for (int i = 0; i < UV_to_UV_average_neighbors.size(); ++i) {
			int leftUp = (i - (Width + 1)) < 0 ? 0 : (i - (Width + 1));
			int rightUp = i - Width;
			int leftDown = (i < Width) ? -1 : i - Width;
			int rightDown = (i + Width + 1) > UV_size ? 0 : (i + Width + 1);

			UV_to_UV_average_neighbors[i] = { leftUp, rightUp, leftDown, rightDown };
	}
}

void FluidSimulator::_solve_incompressibility(float delta) {
    for (int i = 0; i <= Physics_step; ++i) {
        for (int xy = 0; xy < XY.size(); ++xy) {
            if (CollisionMap[xy] == 0) {
                continue;
            }
            int collisionLeft = (XY_neighbors[xy][0] == -1) ? 0 : CollisionMap[XY_neighbors[xy][0]];
            int collisionRight = (XY_neighbors[xy][1] == -1) ? 0 : CollisionMap[XY_neighbors[xy][1]];
            int collisionUp = (XY_neighbors[xy][2] == -1) ? 0 : CollisionMap[XY_neighbors[xy][2]];
            int collisionDown = (XY_neighbors[xy][3] == -1) ? 0 : CollisionMap[XY_neighbors[xy][3]];

            if (collisionLeft == 0 && collisionRight == 0 && collisionUp == 0 && collisionDown == 0) {
                continue;
            }
            int collisionSum = collisionLeft + collisionRight + collisionUp + collisionDown;
            float div = -UV[UV_neighbors[xy][0]] + UV[UV_neighbors[xy][1]] + UV[UV_neighbors[xy][2]] - UV[UV_neighbors[xy][3]];
            float pressure = (-div / collisionSum) * 1.9f;

            if (i == Physics_step) {
                float pressureQuotient = Density * GridSpacing / delta; 
                PressureMap[xy] = pressure * pressureQuotient;
            }
            UV[UV_neighbors[xy][0]] = UV[UV_neighbors[xy][0]] + collisionLeft * pressure;
            UV[UV_neighbors[xy][1]] = UV[UV_neighbors[xy][1]] - collisionRight * pressure;
            UV[UV_neighbors[xy][2]] = UV[UV_neighbors[xy][2]] - collisionUp * pressure;
            UV[UV_neighbors[xy][3]] = UV[UV_neighbors[xy][3]] + collisionDown * pressure;
        }
    }
}

void FluidSimulator::_make_box_around_collisionMap() {
	for (int i = 0; i < CollisionMap.size(); ++i) {
		if (i < Width) {
			CollisionMap[i] = 0;
			continue;
		}
		if ((i + 1) % Width == (Width - 1)) {
			CollisionMap[i] = 0;
			continue;
		}
		if ((i + 1) % Width == 1) {
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
