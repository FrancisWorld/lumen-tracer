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
    texture)
foreach(suite IN LISTS LUMEN_SUITES)
  add_test(NAME ${suite} COMMAND lumen_tests --suite ${suite})
endforeach()
