file(GLOB LUMEN_TEST_SOURCES CONFIGURE_DEPENDS test_*.cpp)
add_executable(lumen_tests main.cpp ${LUMEN_TEST_SOURCES})
target_link_libraries(lumen_tests PRIVATE lumen_core lumen_options)
target_compile_definitions(lumen_tests PRIVATE LUMEN_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# One ctest entry per suite so failures are attributed precisely.
set(LUMEN_SUITES
    math
    rng
    sampling
    ggx
    bvh
    deflate
    png
    json
    obj
    bsdf
    camera
    texture
    scene
    light
    furnace
    direct
    path
    scheduler
    render
    denoise
    tonemap
    loader
    pipeline)
foreach(suite IN LISTS LUMEN_SUITES)
  add_test(NAME ${suite} COMMAND lumen_tests --suite ${suite})
endforeach()

# End-to-end checks of the command line tool.
set(LUMEN_CLI $<TARGET_FILE:lumen>)
add_test(NAME cli_version COMMAND ${LUMEN_CLI} --version)
set_tests_properties(cli_version PROPERTIES PASS_REGULAR_EXPRESSION "^lumen [0-9]+\\.[0-9]+\\.[0-9]+")
add_test(NAME cli_info COMMAND ${LUMEN_CLI} info ${PROJECT_SOURCE_DIR}/scenes/glass-atrium.json)
set_tests_properties(cli_info PROPERTIES PASS_REGULAR_EXPRESSION "lights: 2")
add_test(NAME cli_render
         COMMAND ${LUMEN_CLI} render ${PROJECT_SOURCE_DIR}/scenes/knot-study.json --width 64 --height 36 --spp 2
                 --threads 2 --quiet -o ${CMAKE_CURRENT_BINARY_DIR}/cli_render.png)
set_tests_properties(cli_render PROPERTIES PASS_REGULAR_EXPRESSION "rendered 64x36 @ 2 spp")
add_test(NAME cli_rejects_missing_scene COMMAND ${LUMEN_CLI} render /nonexistent/scene.json)
set_tests_properties(cli_rejects_missing_scene PROPERTIES WILL_FAIL TRUE)
add_test(NAME cli_rejects_bad_option COMMAND ${LUMEN_CLI} render x.json --spp zero)
set_tests_properties(cli_rejects_bad_option PROPERTIES PASS_REGULAR_EXPRESSION "expects a whole number")
