// Loads the lumen scene format (JSON) into a ready-to-render scene.
//
// Top-level keys: settings, camera, environment, textures, materials, objects.
// Unknown keys are rejected so typos surface as errors instead of silently
// changing the picture. Every error names the JSON path of the problem.
#pragma once

#include <filesystem>
#include <string>

#include "lumen/camera.hpp"
#include "lumen/json.hpp"
#include "lumen/renderer.hpp"
#include "lumen/scene.hpp"

namespace lumen {

struct SceneDescription {
    Scene scene;
    CameraParams camera_params;  // kept so the CLI can re-derive the camera when the resolution changes
    Camera camera;
    RenderSettings render;
    Real exposure = 1;
    bool denoise = false;
};

/// `base_dir` resolves relative asset paths (meshes, textures).
SceneDescription build_scene(const Json& document, const std::filesystem::path& base_dir);

/// Reads and parses `path`. Throws InputError with file name and location on failure.
SceneDescription load_scene_file(const std::string& path);

}  // namespace lumen
