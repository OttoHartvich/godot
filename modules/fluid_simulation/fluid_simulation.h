#ifndef FLUID_SIMULATOR_H
#define FLUID_SIMULATOR_H

#include "core/object/ref_counted.h"
#include <iostream>
#include <vector>
#include <array>

class FluidSimulator : public RefCounted {
    GDCLASS(FluidSimulator, RefCounted);

    int Width;
    int Height;
    int Physics_step;
    float Density;
    float GridSpacing;
    std::vector<float> XY;
    std::vector<std::array<int, 4>> XY_neighbors;
    std::vector<float> UV;
    std::vector<float> NewUV;
    std::vector<std::array<int, 4>> UV_neighbors;
    std::vector<std::array<int, 4>> UV_to_UV_average_neighbors;
    std::vector<int> CollisionMap;
    std::vector<float> PressureMap;

protected:
    static void _bind_methods();
    void _populate_UV_neighbors();
    void _populate_XY_neighbors();
    void _populate_UV_to_UV_average_neighbors(int UV_size);
    void _solve_incompressibility(float delta);
    void _make_box_around_collisionMap();

public:
    FluidSimulator(int width, int height, int step, float density, float gridSpacing);
    void delta_step(float delta);
};

#endif // FLUID_SIMULATOR_H
